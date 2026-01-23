/**
 * Author: https://github.com/emrecanbulut
 * https://github.com/BehaviorTree/BehaviorTree.CPP/issues/268#issuecomment-891103810
 */

#include "passform_behavior/nodes.hpp"

BT::PortsList RangeLoop::providedPorts() {
  return {
    BT::InputPort<int>("start"),
    BT::InputPort<int>("stop"),
    BT::InputPort<int>("step"),
    BT::OutputPort<int>("current")
  };
}

BT::NodeStatus RangeLoop::tick() {
  if (status() == BT::NodeStatus::IDLE) {
    start_ = getInputValue<int>(*this, "start");
    stop_ = getInputValue<int>(*this, "stop");
    step_ = getInputValue<int>(*this, "step");

    if (step_ == 0) {
      throw BT::RuntimeError(name() + ": [step] cannot be zero");
    }
  }

  this->setStatus(BT::NodeStatus::RUNNING);

  for (; step_ >= 0 ? (start_ < stop_) : (start_ > stop_); start_ += step_) {
    setOutput("current", start_);

    switch (child()->executeTick()) {
      case BT::NodeStatus::SUCCESS:
        haltChild();
        break;
      case BT::NodeStatus::FAILURE:
        haltChild();
        return BT::NodeStatus::FAILURE;
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      default:
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  return BT::NodeStatus::SUCCESS;
}
