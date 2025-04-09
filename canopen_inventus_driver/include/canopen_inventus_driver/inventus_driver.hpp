#ifndef CANOPEN_INVENTUS_DRIVER__INVENTUS_DRIVER_HPP_
#define CANOPEN_INVENTUS_DRIVER__INVENTUS_DRIVER_HPP_

#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{

class InventusDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenInventusDriver<rclcpp::Node>> node_canopen_inventus_driver_;
public:
  InventusDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
};

} // namespace ros2_canopen

#endif // CANOPEN_INVENTUS_DRIVER__INVENTUS_DRIVER_HPP_
