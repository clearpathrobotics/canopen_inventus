#include "canopen_inventus_driver/inventus_driver.hpp"

using namespace ros2_canopen;


InventusDriver::InventusDriver(rclcpp::NodeOptions node_options) : CanopenDriver(node_options)
{
  node_canopen_inventus_driver_ =
    std::make_shared<node_interfaces::NodeCanopenInventusDriver<rclcpp::Node>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(node_canopen_inventus_driver_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::InventusDriver)
