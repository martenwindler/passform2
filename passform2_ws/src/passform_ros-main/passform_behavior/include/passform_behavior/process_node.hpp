#ifndef PROCESS_NODE_HPP
#define PROCESS_NODE_HPP

#include <chrono>
#include <functional>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "std_msgs/msg/string.hpp"

#include "passform_behavior/actions.hpp"
#include "passform_behavior/print.hpp"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


using namespace std::chrono_literals;

class BehaviorNode : public rclcpp::Node
{
public:
    BehaviorNode();
    void create_passform_tree();
    void start_tree();

private:
    std::string tree_xml;
    BehaviorTreeFactory factory;
    Tree tree;
    NodeStatus status = NodeStatus::IDLE;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tree_publisher_;

    void publish_tree();
    std::string read_file(std::string path);
};

#endif