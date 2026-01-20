#include "passform_behavior/process_node.hpp"

using namespace std::chrono_literals;

BehaviorNode::BehaviorNode() : Node("process_node")
{
    this->declare_parameter("behavior_file", "/home/jasper/ros2_ws/install/passform_behavior/share/passform_behavior/config/external.xml");
    this->declare_parameter("action_topic", "/perform_action");
    this->declare_parameter("xml_topic", "/passform_process");
    tree_publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("xml_topic").as_string(), 10);
}

void BehaviorNode::create_passform_tree()
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
    RegisterRosAction<Grasp>(factory, "Grasp", params);
    RegisterRosAction<Move>(factory, "Move", params);
    RegisterRosAction<Position>(factory, "Position", params);
    RegisterRosAction<Pressure>(factory, "Pressure", params);
    RegisterRosAction<Release>(factory, "Release", params);
    RegisterRosAction<ToolGet>(factory, "ToolGet", params);
    RegisterRosAction<ToolPut>(factory, "ToolPut", params);
    RegisterRosAction<ToolUse>(factory, "ToolUse", params);
    RegisterRosAction<AssemblyMount>(factory, "AssemblyMount", params);
    RegisterRosAction<AssemblyPickplace>(factory, "AssemblyPickplace", params);
    RegisterRosAction<AssemblyPlug>(factory, "AssemblyPlug", params);
    RegisterRosAction<AssemblyScrew>(factory, "AssemblyScrew", params);


    // init tree from file path stored in ROS parameter
    tree_xml = read_file(this->get_parameter("behavior_file").as_string());
    tree = factory.createTreeFromText(tree_xml);
}

void BehaviorNode::start_tree()
/*
Start publisher (ZeroMQ and ROS) and tree ticking
*/
{
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
    // periodix XML tree publisher (ROS)
    timer_ = this->create_wall_timer(1000ms, std::bind(&BehaviorNode::publish_tree, this));
    // start tree
    status = tree.tickRootWhileRunning();
}

void BehaviorNode::publish_tree()
/* 
Publish xml tree description as ROS String
*/
{
    auto message = std_msgs::msg::String();
    message.data = tree_xml;
    tree_publisher_->publish(message);
}

std::string BehaviorNode::read_file(std::string path)
/* 
Read file into string.
Source: https://stackoverflow.com/a/2912614/21859283
*/
{
    std::ifstream ifs(path);
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                        (std::istreambuf_iterator<char>()    ) );
    return content;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>();
  node->create_passform_tree();
  node->start_tree();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}