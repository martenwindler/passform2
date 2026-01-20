/**
 * The decorator node providing a loop functionality with an output to track
 * the loop's current iteration.
 * Author: https://github.com/emrecanbulut
 * https://github.com/BehaviorTree/BehaviorTree.CPP/issues/268#issuecomment-891103810
 */

#ifndef PASSFORM_BEHAVIOR__NODES_HPP_
#define PASSFORM_BEHAVIOR__NODES_HPP_

#include "behaviortree_cpp_v3/bt_factory.h"

class RangeLoop : public BT::DecoratorNode {
 public:
  using DecoratorNode::DecoratorNode;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  int start_{0};
  int stop_{0};
  int step_{1};
};


// The input ports in our scenario are mostly mandatory ports. So, this part has nothing to do with
// the RangeLoop. I think this would also be a very good feature to add to the BT.
template <class T>
inline T getInputValue(const BT::TreeNode& node, const std::string& key) {
  T value{};
  auto result = node.getInput(key, value);
  if (!result) {
    throw BT::RuntimeError(
        node.name() + ": missing required input [" + key + "]: ",
        result.error());
  }
  return value;
}

#endif //PASSFORM_BEHAVIOR__NODES_HPP_
