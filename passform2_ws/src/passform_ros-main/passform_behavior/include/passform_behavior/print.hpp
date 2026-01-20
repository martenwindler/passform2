#ifndef PASSFORM_BEHAVIOR__PRINT_HPP_
#define PASSFORM_BEHAVIOR__PRINT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class PrintString : public BT::SyncActionNode
{
public:
  PrintString(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintString: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintString FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<std::string>("message") };
  }
};


class PrintInt : public BT::SyncActionNode
{
public:
  PrintInt(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    int msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintInt: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintInt FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<int>("message") };
  }
};

#endif // PASSFORM_BEHAVIOR__PRINT_HPP_
