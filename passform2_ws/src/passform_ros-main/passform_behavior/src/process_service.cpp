#include "passform_behavior/process_node.hpp"

/* THIS NODE IS A WORK IN PROGRESS AND CURRENTLY FAILS AFTER THE FIRST TICK */


using namespace std::chrono_literals;

BehaviorNode::BehaviorNode() : Node("process_node")
{
    this->declare_parameter("behavior_file", "/home/jasper/ros2_ws/install/passform_behavior/share/passform_behavior/config/external.xml");
    this->declare_parameter("action_topic", "/perform_action");
    this->declare_parameter("xml_topic", "/passform_process");
    this->declare_parameter("trigger_service", "/start_process");

    behavior_service_ = create_service<passform_msgs::srv::SetString>(
        this->get_parameter("trigger_service").as_string(),
        std::bind(&BehaviorNode::start_service_cb_, this, std::placeholders::_1, std::placeholders::_2));
    tree_publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("xml_topic").as_string(), 10);
    tick_timer_ = this->create_wall_timer(20ms, std::bind(&BehaviorNode::tick_tree, this));    
    timer_ = this->create_wall_timer(1000ms, std::bind(&BehaviorNode::publish_tree, this));
}

void BehaviorNode::prepare_behavior()
/* 
Read xml file and register all actions.
 */
{
    ActionNodeParams params = {
        shared_from_this(), // nodehandle
        this->get_parameter("action_topic").as_string(),    // action topic to be called
        2000ms // action_server timeout
    };    

    // create actions
    factory.registerNodeType<PrintString>("PrintString");
    RegisterRosAction<PassformAction>(factory, "Passform", params);
    RegisterRosAction<MoveBody>(factory, "MoveBody", params);
}

void BehaviorNode::load_tree(std::string& str)
/*
Start publisher (ZeroMQ and ROS) and tree ticking
*/
{
    // if(tree.rootNode()){
    //     throw std::runtime_error( "behavior already active" ); 
    // }
    // init tree from file path stored in ROS parameter
    tree = factory.createTreeFromText(str);

    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree); 
}

void BehaviorNode::publish_tree()
/* 
Publish xml tree description as ROS String
*/
{
    if( status != NodeStatus::RUNNING ){
        return;
    }
    auto message = std_msgs::msg::String();
    message.data = tree_xml;
    tree_publisher_->publish(message);
}

void BehaviorNode::start_service_cb_(
    const std::shared_ptr<passform_msgs::srv::SetString::Request> request,
          std::shared_ptr<passform_msgs::srv::SetString::Response> response)
/*
Service to start the tree. Load the tree from the sent XML and sets status to running. 
From here on, tick_tree will perform the ticking.
*/
{
    try
    {
        load_tree(request->data);
        status = NodeStatus::RUNNING; // set status to running

        response->success = true;
        response->message = "succeeded";
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN( this->get_logger(), "Failed to start tree from XML. %s", e.what());
        response->message = e.what();
    }
    
}

void BehaviorNode::tick_tree()
/* 
Tick if tree is already running
*/
{
    if (status == NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
    }
}

NodeStatus BehaviorNode::stop_tree()
/* 
Stop tree and set status IDLE
*/
{
    tree.haltTree();
    RCLCPP_INFO( this->get_logger(), "Stopped tree.");
    return NodeStatus::IDLE;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>();
  node->prepare_behavior();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}