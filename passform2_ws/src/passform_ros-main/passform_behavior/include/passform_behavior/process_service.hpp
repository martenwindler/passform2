#ifndef PROCESS_SERVICE_HPP
#define PROCESS_SERVICE_HPP

#include <chrono>
#include <functional>
#include <string>
#include <fstream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <passform_msgs/srv/set_string.hpp>

#include "passform_behavior/actions.hpp"
#include "passform_behavior/print.hpp"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


using namespace std::chrono_literals;

class BehaviorNode : public rclcpp::Node
{
public:
    BehaviorNode();
    void prepare_behavior();
    void load_tree(std::string&);

private:
    std::string tree_xml;
    BehaviorTreeFactory factory;
    Tree tree;
    NodeStatus status = NodeStatus::IDLE;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tree_publisher_;
    rclcpp::Service<passform_msgs::srv::SetString>::SharedPtr behavior_service_;

    void publish_tree();
    void tick_tree();
    NodeStatus stop_tree();
    void start_service_cb_(
        const std::shared_ptr<passform_msgs::srv::SetString::Request> request,
        std::shared_ptr<passform_msgs::srv::SetString::Response> response
    );
};

#endif