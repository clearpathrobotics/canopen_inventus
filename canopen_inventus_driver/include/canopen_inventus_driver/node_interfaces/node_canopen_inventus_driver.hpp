#ifndef NODE_CANOPEN_INVENTUS_DRIVER
#define NODE_CANOPEN_INVENTUS_DRIVER

#include "canopen_inventus_driver/battery.hpp"
#include "canopen_inventus_interfaces/msg/virtual_battery.hpp"
#include "canopen_inventus_interfaces/msg/status.hpp"
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
   void test();
   bool readSDO(ros2_canopen::COData & data);

   template <typename T>
   bool readTypedSDO(ros2_canopen::COTypedData<T> & data);

   template <typename T>
   void readTypedPDO(ros2_canopen::COTypedData<T> & data);

   void readAllSDO();
   void readAllPDO();

protected:
   std::shared_ptr<Battery> battery_;
   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publish_battery_state_;
   rclcpp::Publisher<canopen_inventus_interfaces::msg::Status>::SharedPtr publish_battery_status_;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publish_soc_;

   rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
   publish_virtual_battery_state_;
   rclcpp::Publisher<canopen_inventus_interfaces::msg::VirtualBattery>::SharedPtr publish_virtual_battery_status_;

   void publish();
   virtual void poll_timer_callback() override;

   bool is_master_;
   std::string location_;
};
} // namespace canopen_inventus_driver
} // namespace node_interfaces


#endif
