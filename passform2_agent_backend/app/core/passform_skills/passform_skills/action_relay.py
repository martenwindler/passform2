import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.server import ActionServer, GoalStatus

from collections import OrderedDict

from passform_msgs.action import Passform

class ActionRelay:
    """
    """

    def __init__(self, node: Node):

        self._node = node
        self._relay_clients: OrderedDict[str, ActionClient] = OrderedDict()  # id_short, client()

        self._relay_goal_handle_client = None # goal handle of driver modules
        self._relay_goal_handle_server = None # goal handle of local action server

        self._result: Passform.Result() = None

    def create_action_client(
        self,
        key: str, #operation.id_short
        action_type,
        action_name,
        **kwargs
    ):
        self._relay_clients[key] = ActionClient(self._node, action_type, action_name, **kwargs)

    def relay_goal(self, goal_handle, server_key:str) -> None:
        """
        Send goal to specified action server (e.g. driver node).
        The goal itself will not be modified.

        :param server_key: key under which the ActionServer was stored
        """
        self._node.get_logger().info(f'Relaying goal to "{server_key}"')
        temp_client = self._relay_clients[server_key]
        temp_client.wait_for_server()

        self._relay_goal_future = temp_client.send_goal_async(
            goal_handle.request,
            feedback_callback=self.relay_feedback_callback)

        self._relay_goal_future.add_done_callback(self.relay_goal_response_callback)
        self._relay_goal_handle_server = goal_handle

    def is_active(self) -> bool:
        """True, if the action relay is currently relaying actions"""
        return self._relay_goal_handle_server is not None

    def cancel(self):
        """Cancel the action on both server and client side.
        Sets goal to `canceled`.
        """
        if self._relay_goal_handle_client is not None and self._relay_goal_handle_client.is_active:
            self._relay_goal_handle_client.cancel_goal_async()
            self._node.get_logger().info('Canceled goal.')

    def __del__(self):
        self.destroy_all_clients()

    def destroy_all_clients(self):
        """Destroy all action clients and remove from internal dict.
        This disconnects the relay from all ActionServer.
        """
        for key in list(self._relay_clients.keys()):
            self._destroy_action_client(key)

    def _destroy_action_client(self, server_key: str):
        """Destroy action client and remove from internal dict.
        This disconnects the relay from the specified ActionServer.

        :param server_key: key under which the ActionServer was stored
        """
        if server_key not in self._relay_clients.keys():
            return
        self._relay_clients[server_key].destroy()
        del self._relay_clients[server_key]

    ## CLIENT SIDE CALLBACKS
    def relay_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn('Goal rejected.')
            return
        self._relay_goal_handle_client = goal_handle
        self._node.get_logger().debug('Goal accepted :)')

        self._get_result_future = self._relay_goal_handle_client.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def relay_feedback_callback(self, feedback:Passform.Feedback) -> None:
        """This node will publish all received feedback up to its own Client.
        Request: ActionClient -> THIS_NODE -> ActionServer (e.g. driver)
        Feedback: ActionClient <- THIS_NODE <- ActionServer (e.g. driver)
        """
        self._relay_goal_handle_server.publish_feedback(feedback.feedback)

    def get_result_callback(self, future) -> None:
        self._relay_goal_handle_server = None
        self._result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().debug(f'Goal succeeded! Result: {self._result}')
        else:
            self._node.get_logger().warn(f'Goal failed with status: {status}')

    def get_result(self) -> Passform.Result:
        """Getter for locally stored result"""
        return self._result
