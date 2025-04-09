#ifndef NODE_CANOPEN_INVENTUS_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_INVENTUS_DRIVER_IMPL_HPP_

#include "canopen_inventus_driver/node_interfaces/node_canopen_inventus_driver.hpp"

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template <class NODETYPE>
NodeCanopenInventusDriver<NODETYPE>::NodeCanopenInventusDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::init(bool called_from_base)
{
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template <>
void NodeCanopenInventusDriver<rclcpp::Node>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  // Publisher: Battery State
  publish_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/bms/battery_state", 1);
  // Publisher: Multi. Status
  publish_status_ = this->node_->create_publisher<inventus_bmu::msg::Status>("~/modules", 1);
  // Publisher: SOC
  publish_soc_ = this->node_->create_publisher<std_msgs::msg::Float32>("~/bms/soc", 1);
}

template <>
void NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  // Publisher: Battery State
  publish_battery_state_ = this->node_->create_publisher<sensor_msgs::msg::BatteryState>("~/bms/battery_state", 1);
  // Publisher: Multi. Status
  publish_status_ = this->node_->create_publisher<inventus_bmu::msg::Status>("~/modules", 1);
  // Publisher: SOC
  publish_soc_ = this->node_->create_publisher<std_msgs::msg::Float32>("~/bms/soc", 1);
}

template <>
void NodeCanopenInventusDriver<rclcpp::Node>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
  // Initialize internal variables
}

template <>
void NodeCanopenInventusDriver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::configure(false);
  // Initialize internal variables
}

template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  // Activate controller
}

template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::deactivate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  timer_->cancel();
}

template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  battery_->readState();
  publish();
}


template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::publish()
{
  RCLCPP_INFO(this->node_->get_logger(), "PUBLISH");
  inventus_bmu::msg::Status status_msg;
  sensor_msgs::msg::BatteryState battery_state_msg;
  std_msgs::msg::Float32 soc_msg;
  // Status Message
  // status_msg.header.stamp = this->node_->now();
  status_msg.node_id = battery_->get_id();
  status_msg.firmware_version = battery_->firmware_version_;
  status_msg.serial_number = std::to_string(battery_->serial_number_);
  status_msg.soc = battery_->battery_state_of_charge_;
  for(int i = 0; i < 8; i++)
  {
    status_msg.cell_voltages[i] = battery_->cell_voltage_[i] / 1000.0;
  }
  status_msg.number_of_batteries = battery_->vb_number_of_batteries_;
  status_msg.sum_capacity = battery_->vb_current_stored_;
  status_msg.remaining_runtime = battery_->vb_remaining_run_time_;
  status_msg.remaining_chargetime = battery_->vb_remaining_charge_time_;
  status_msg.avg_voltage = battery_->vb_pack_voltage_ / 1000.0;
  status_msg.sum_current = battery_->vb_current_ / 10.0;
  status_msg.discharge_current_limit = battery_->vb_discharge_current_limit_ / 10.0;
  status_msg.charge_cutoff_current = battery_->vb_charge_cut_off_current_ / 10.0;
  status_msg.fully_charged = battery_->vb_fully_charged_;
  status_msg.avg_temp = battery_->vb_temperature_ * 0.125;
  status_msg.discharge_cutoff_voltage = battery_->vb_discharge_cut_off_voltage_ / 1000.0;
  status_msg.charge_current_limit = battery_->vb_charge_current_limit_ / 10.0;
  status_msg.max_voltage_allowed = battery_->vb_max_allowed_charge_voltage_ / 1000.0;
  status_msg.avg_health = battery_->vb_soh_;
  status_msg.num_faulted_batteries = battery_->vb_number_of_batteries_fault_;
  status_msg.num_active_batteries = battery_->vb_number_of_active_batteries_;
  status_msg.op_mode = battery_->vb_operational_mode_;
  status_msg.charge_fault = battery_->vb_charge_fault_;
  status_msg.discharge_fault = battery_->vb_discharge_fault_;
  status_msg.regen_current_limit = battery_->vb_regen_current_limit_ / 10.0;
  status_msg.min_cell_voltage = battery_->vb_min_cell_voltage_ / 1000.0;
  status_msg.max_cell_voltage = battery_->vb_max_cell_voltage_ / 1000.0;
  status_msg.cell_balance_status = battery_->vb_cell_balance_status_all_;
  status_msg.all_avg_voltage = battery_->vb_pack_voltage_all_ / 1000.0;
  status_msg.all_soc = battery_->vb_soc_all_;
  status_msg.all_avg_temp = battery_->vb_temperature_all_ * 0.125;
  status_msg.master_node_id = battery_->master_node_id_;
  // Battery State Message
  battery_state_msg.header.stamp = this->node_->now();
  battery_state_msg.power_supply_technology = battery_state_msg.POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery_state_msg.power_supply_health = battery_state_msg.POWER_SUPPLY_HEALTH_GOOD;
  // battery_state_msg.design_capacity = std::nanf;
  battery_state_msg.location = "internal";
  battery_state_msg.temperature = battery_->vb_temperature_ * 0.125;
  // battery_state_msg.design_capacity = battery_->design_capacity_ * 0.005;
  battery_state_msg.voltage = battery_->vb_pack_voltage_ / 1000.0;
  battery_state_msg.current = battery_->current_ / 10.0;
  battery_state_msg.charge = battery_->vb_soc_all_ * battery_->vb_current_stored_ / 100.0;
  battery_state_msg.capacity = battery_->vb_current_stored_;
  battery_state_msg.percentage = battery_->vb_soc_all_ / 100.0;
  if(status_msg.fully_charged)
  {
    battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_FULL;
  }
  else if (status_msg.sum_current > 0.0)
  {
    battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_CHARGING;
  }
  else
  {
    battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_DISCHARGING;
  }
  // SOC Message
  soc_msg.data = battery_state_msg.percentage;
  // Publish
  publish_battery_state_->publish(battery_state_msg);
  publish_status_->publish(status_msg);
  publish_soc_->publish(soc_msg);
}

template <class NODETYPE>
void NodeCanopenInventusDriver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  battery_ = std::make_shared<Battery>(this->lely_driver_);
}

#endif
