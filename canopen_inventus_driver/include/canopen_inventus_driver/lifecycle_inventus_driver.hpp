#ifndef CANOPEN_INVENTUS_DRIVER__CANOPEN_LIFECYCLE_INVENTUS_DRIVER_HPP_
#define CANOPEN_INVENTUS_DRIVER__CANOPEN_LIFECYCLE_INVENTUS_DRIVER_HPP_

#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver.hpp"
#include "canopen_core/driver_node.hpp"
namespace ros2_canopen
{
/**
   * @brief Lifecycle Proxy Driver
   *
   * A very basic driver without any functionality.
   *
   */
class LifecycleInventusDriver : public ros2_canopen::LifecycleCanopenDriver
{
   std::shared_ptr<node_interfaces::NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>> node_canopen_inventus_driver_;
public:
   LifecycleInventusDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
};

} // namespace ros2_canopen

#endif
