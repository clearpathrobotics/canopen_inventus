#ifndef NODE_CANOPEN_INVENTUS_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_INVENTUS_DRIVER_IMPL_HPP_

#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver.hpp"

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

/**
 * @brief Constructor for templated Node Canopen Inventus Driver<NODETYPE>:: Node Canopen Inventus Driver object
 *
 * @tparam NODETYPE
 * @param node
 */
template <class NODETYPE>
NodeCanopenInventusDriver<NODETYPE>::NodeCanopenInventusDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

/**
 * @brief Templated initialization, not utilized.
 *
 * @tparam NODETYPE
 * @param called_from_base
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::init(bool called_from_base)
{
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

/**
 * @brief Initialization of publishers for rclcpp::Node
 *
 * @tparam
 * @param called_from_base
 */
template <>
void NodeCanopenInventusDriver<rclcpp::Node>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  // Publisher: Battery State
  publish_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 1);

  // Publisher: Battery Status
  publish_battery_status_ = this->node_->create_publisher<canopen_inventus_interfaces::msg::Status>("~/battery_status", 1);
}

/**
 * @brief Initialization of publishers for rclcpp_lifecycle::LifecycleNode
 *
 * @tparam
 * @param called_from_base
 */
template <>
void NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  // Publisher: Battery State
  publish_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 1);

  // Publisher: Battery Status
  publish_battery_status_ = this->node_->create_publisher<canopen_inventus_interfaces::msg::Status>("~/battery_status", 1);
}

/**
 * @brief Configuration of parameters and registration of required publishers
 * for class type rclcpp::Node
 *
 * @tparam
 * @param called_from_base
 */
template <>
void NodeCanopenInventusDriver<rclcpp::Node>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
  // Initialize internal variables
  try
  {
    is_master_ = this->config_["is_master"].as<bool>();
    RCLCPP_INFO(this->node_->get_logger(), "Master configuration toggle set. This is the master battery.");
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Master configuration toggle, 'is_master', not set, assuming this is not the master battery.");
    is_master_ = false;
  }
  try
  {
    location_ = this->config_["location"].as<std::string>();
    RCLCPP_INFO(this->node_->get_logger(), "Battery 'location' set to '%s'", location_.c_str());
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Battery 'location' not defined. Defaulting to 'unknown'.");
    location_ = "unknown";
  }
  try
  {
    publish_ms_ = this->config_["publish_ms"].as<uint32_t>();
    RCLCPP_INFO(this->node_->get_logger(), "Publish loop timer period 'publish_ms' set to '%d'", publish_ms_);
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Publish loop timer period 'publish_ms' not set, defaulting to 1000 ms");
    publish_ms_ = 1000;
  }
  // Create extra publishers for virtual battery
  if (is_master_)
  {
    // Publisher: Visual Battery BatteryState
    publish_virtual_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/virtual_battery_state", 1);
    // Publisher: Visual Battery Inventus Status
    publish_virtual_battery_status_ = this->node_->create_publisher<canopen_inventus_interfaces::msg::VirtualBattery>("~/virtual_battery_status", 1);
  }
}

/**
 * @brief Configuration of parameters and registration of required publishers
 * for class type rclcpp_lifecycle::LifecycleNode
 *
 * @tparam
 * @param called_from_base
 */
template <>
void NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::configure(false);
  // Initialize internal variables
  try
  {
    is_master_ = this->config_["is_master"].as<bool>();
    RCLCPP_INFO(this->node_->get_logger(), "Master configuration toggle set. This is the master battery.");
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Master configuration toggle, 'is_master', not set, assuming this is not the master battery.");
    is_master_ = false;
  }
  try
  {
    location_ = this->config_["location"].as<std::string>();
    RCLCPP_INFO(this->node_->get_logger(), "Battery 'location' set to '%s'", location_.c_str());
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Battery 'location' not defined. Defaulting to 'unknown'.");
    location_ = "unknown";
  }
  try
  {
    publish_ms_ = this->config_["publish_ms"].as<uint32_t>();
    RCLCPP_INFO(this->node_->get_logger(), "Publish loop timer period 'publish_ms' set to '%d'", publish_ms_);
  }
  catch(...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Publish loop timer period 'publish_ms' not set, defaulting to 1000 ms");
    publish_ms_ = 1000;
  }
  // Create extra publishers for virtual battery
  if (is_master_)
  {
    // Publisher: Visual Battery BatteryState
    publish_virtual_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/virtual_battery_state", 1);
    // Publisher: Visual Battery Inventus Status
    publish_virtual_battery_status_ = this->node_->create_publisher<canopen_inventus_interfaces::msg::VirtualBattery>("~/virtual_battery_status", 1);
  }
}

/**
 * @brief Activation function. Start timers.
 *
 * @tparam NODETYPE
 * @param called_from_base
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  delay_count_ = 0;
  // Activate controller
  publish_timer_ = this->node_->create_wall_timer(
    std::chrono::milliseconds(publish_ms_),
    std::bind(&NodeCanopenInventusDriver<NODETYPE>::publish_timer_callback, this), this->timer_cbg_);
}

/**
 * @brief Deactivation function. Stop timer.
 *
 * @tparam NODETYPE
 * @param called_from_base
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::deactivate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  timer_->cancel();
  publish_timer_->cancel();
}

/**
 * @brief Timer callback where SDO and PDO reads are executed.
 *
 * @tparam NODETYPE
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();

  if (this->activated_.load())
  {
    delay_count_++;
    if(delay_count_ < int(2000 / this->period_ms_))
    {
      return;
    }
    // SDO Read
    battery_->readAllSDO();

    // PDO Read
    if(is_master_)
    {
      battery_->readAllPDO();
    }
  }
}

/**
 * @brief Timer callback where topics are published.
 *
 * @tparam NODETYPE
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::publish_timer_callback()
{
  if (this->activated_.load())
  {
    // Publish
    publish();
  }
}

/**
 * @brief Publish state and status messages.
 *
 * @tparam NODETYPE
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::publish()
{
  // Stamp
  std_msgs::msg::Header header;
  header.stamp = this->node_->now();

  // Battery State Message
  sensor_msgs::msg::BatteryState battery_state_msg = battery_->getBatteryState();
  battery_state_msg.header.stamp = header.stamp;
  battery_state_msg.location = location_;
  publish_battery_state_->publish(battery_state_msg);

  // Battery Status Message
  canopen_inventus_interfaces::msg::Status battery_status_msg = battery_->getBatteryStatus();
  battery_status_msg.header.stamp = header.stamp;
  publish_battery_status_->publish(battery_status_msg);

  if(is_master_)
  {
    // Virtual Battery State
    sensor_msgs::msg::BatteryState virtual_battery_state_msg = battery_->getVirtualBatteryState();
    virtual_battery_state_msg.header.stamp = header.stamp;
    publish_virtual_battery_state_->publish(virtual_battery_state_msg);

    // Virtual Battery Status
    canopen_inventus_interfaces::msg::VirtualBattery virtual_battery_status_msg = battery_->getVirtualBatteryStatus();
    virtual_battery_status_msg.header.stamp = header.stamp;
    publish_virtual_battery_status_->publish(virtual_battery_status_msg);
  }

}

/**
 * @brief Register node with the master node.
 *
 * @tparam NODETYPE
 */
template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  battery_ = std::make_shared<Battery>(this->lely_driver_, this->sdo_mtex, this->node_->get_logger());
}

#endif
