#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "passform_behavior/nodes.hpp"
#include "passform_behavior/print.hpp"

int main(int argc, char **argv)
{
  std::string config_path{ament_index_cpp::get_package_share_directory("passform_behavior")+"/config/"};
  std::string xml_file_name{"forloop.xml"};
  std::string xml_path = config_path + xml_file_name;

  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_passform");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RangeLoop>("RangeLoop");
  factory.registerNodeType<PrintInt>("PrintInt");

  auto tree = factory.createTreeFromFile(xml_path);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while( rclcpp::ok() && status == BT::NodeStatus::RUNNING )
  {
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(100));
  }

  return 0;
}
