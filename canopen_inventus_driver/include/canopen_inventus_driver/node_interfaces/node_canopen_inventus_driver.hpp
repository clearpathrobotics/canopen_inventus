#ifndef NODE_CANOPEN_INVENTUS_DRIVER
#define NODE_CANOPEN_INVENTUS_DRIVER

#include "canopen_inventus_driver/battery.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "inventus_bmu/msg/multi_status.hpp"
#include "inventus_bmu/msg/status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopenInventusDriver : public NodeCanopenProxyDriver<NODETYPE>
{
   static_assert(
         std::is_base_of<rclcpp::Node, NODETYPE>::value ||
            std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
         "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");
public:
   NodeCanopenInventusDriver(NODETYPE *node);

   virtual void init(bool called_from_base) override;
   virtual void configure(bool called_from_base) override;
   virtual void activate(bool called_from_base) override;
   virtual void deactivate(bool called_from_base) override;
   virtual void add_to_master() override;

protected:
   std::shared_ptr<Battery> battery_;
   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publish_battery_state_;
   rclcpp::Publisher<inventus_bmu::msg::Status>::SharedPtr publish_status_;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publish_soc_;

   void publish();
   virtual void poll_timer_callback() override;
};
} // namespace canopen_inventus_driver
} // namespace node_interfaces


#endif
