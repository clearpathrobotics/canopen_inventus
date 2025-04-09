#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver.hpp"
#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver_impl.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopenInventusDriver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>;
