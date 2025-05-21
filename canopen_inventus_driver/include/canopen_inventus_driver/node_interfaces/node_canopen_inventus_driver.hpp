/**
 *
 *  \file       node_canopen_inventus_driver.hpp
 *  \brief      Inventus battery node ros2_canopen
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2025, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#ifndef CANOPEN_INVENTUS_DRIVER__NODE_CANOPEN_INVENTUS_DRIVER_HPP_
#define CANOPEN_INVENTUS_DRIVER__NODE_CANOPEN_INVENTUS_DRIVER_HPP_

#include "canopen_inventus_driver/battery.hpp"
#include "canopen_inventus_interfaces/msg/virtual_battery.hpp"
#include "canopen_inventus_interfaces/msg/status.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
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
   rclcpp::Publisher<canopen_inventus_interfaces::msg::Status>::SharedPtr publish_battery_status_;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publish_soc_;

   rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
   publish_virtual_battery_state_;
   rclcpp::Publisher<canopen_inventus_interfaces::msg::VirtualBattery>::SharedPtr publish_virtual_battery_status_;

   rclcpp::TimerBase::SharedPtr publish_timer_;

   void publish();
   virtual void poll_timer_callback() override;
   virtual void on_rpdo(COData data) override;

   void publish_timer_callback();

   bool is_master_;
   std::string location_;
   uint32_t publish_ms_;
   int delay_count_;
};
} // namespace canopen_inventus_driver
} // namespace node_interfaces


#endif
