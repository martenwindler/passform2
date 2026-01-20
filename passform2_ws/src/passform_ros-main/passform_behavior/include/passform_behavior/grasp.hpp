#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_action/client_goal_handle.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"

#include "passform_msgs/action/passform.hpp"
#include "passform_msgs/msg/task.hpp"
#include "passform_msgs/msg/skill_type.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

using namespace BT;

class Grasp: public RosActionNode<passform_msgs::action::Passform>
{
public:
  Grasp(const std::string& name,
              const BT::NodeConfiguration& conf,
              const ActionNodeParams& params,
              typename std::shared_ptr<ActionClient> action_client)
    : RosActionNode<passform_msgs::action::Passform>(name, conf, params, action_client)
  {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports_list{};
    for(const std::string& port : float_ports){ports_list.insert(InputPort<float>(port));}
    for(const std::string& port : string_ports){ports_list.insert(InputPort<std::string>(port));}
    for(const std::string& port : int_ports){ports_list.insert(InputPort<int>(port));}
    for(const std::string& port : bool_ports){ports_list.insert(InputPort<bool>(port));}
    return ports_list;
  }

  bool setGoal(Goal& goal) override
  {
    goal.task = passform_msgs::msg::Task();
    goal.task.type.skill_type = skill_type;

    for(const std::string& port : float_ports){ add_parameter<float>(port, goal.task.optional_parameter); }
    for(const std::string& port : string_ports){ add_parameter<std::string>(port, goal.task.optional_parameter); }
    for(const std::string& port : int_ports){ add_parameter<int>(port, goal.task.optional_parameter); }
    for(const std::string& port : bool_ports){ add_parameter<bool>(port, goal.task.optional_parameter); }

    RCLCPP_INFO( node_->get_logger(), "Start skill type %i", goal.task.type.skill_type);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    RCLCPP_INFO( node_->get_logger(), "Skill succeeded" );
    return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "Skill failed: %d", error );
    return NodeStatus::FAILURE;
  }

private:
  const static std::vector<std::string> float_ports;
  const static std::vector<std::string> string_ports;
  const static std::vector<std::string> int_ports;
  const static std::vector<std::string> bool_ports;
  const static uint skill_type;

  template <typename T>
  void add_parameter(std::string port, std::vector<rcl_interfaces::msg::Parameter>& param_list)
  /* 
  Add parameter with name port to passed vector. Typename specifies the parameter type.
  */
  {
    try
    {
      auto parameter = rclcpp::Parameter(port,rclcpp::ParameterValue(getInput<T>(port).value()));
      param_list.push_back(parameter.to_parameter_msg());
    }
    catch(const std::exception& e)  // TODO: this is to catch nonstd::expected_lite::bad_expected_access
    {
      RCLCPP_INFO( node_->get_logger(), "Input '%s' not defined. Skipped.", port.c_str() );
    }
  }
};

const std::vector<std::string> Grasp::float_ports{"a_max", "v_max", "F_max"};
const std::vector<std::string> Grasp::string_ports{"uuid"};
const std::vector<std::string> Grasp::int_ports{"task_id"};
const std::vector<std::string> Grasp::bool_ports{};
const uint Grasp::skill_type{passform_msgs::msg::SkillType::GRASP};