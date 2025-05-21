/**
 *
 *  \file       battery.hpp
 *  \brief      Inventus battery ros2_canopen implementation
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
#ifndef CANOPEN_INVENTUS_DRIVER__BATTERY_HPP_
#define CANOPEN_INVENTUS_DRIVER__BATTERY_HPP_

#include <lely/co/type.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "canopen_inventus_interfaces/msg/status.hpp"
#include "canopen_inventus_interfaces/msg/virtual_battery.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"


namespace ros2_canopen
{

struct COIndex
{
public:
  uint16_t index_;
  uint8_t subindex_;
  uint16_t type_;

  uint32_t getUniqueIndex()
  {
    return index_ << 8 | subindex_;
  };
};

class Battery
{
public:
  Battery(std::shared_ptr<LelyDriverBridge> driver, std::mutex &node_mutex, rclcpp::Logger logger):
  sdo_mutex(node_mutex), read_static_sdo_(true), logger_(logger)
  {
    this->driver_ = driver;
    begin = std::chrono::steady_clock::now();
  }

  // SDO: Visible Strings
  COIndex sdo_hardware_version_ = {0x1009, 0x00, CO_DEFTYPE_VISIBLE_STRING};
  COIndex sdo_software_version_ = {0x100A, 0x00, CO_DEFTYPE_VISIBLE_STRING};
  COIndex sdo_firmware_version_ = {0xD000, 0x20, CO_DEFTYPE_VISIBLE_STRING};

  // SDO: Unsigned 8 Bit
  COIndex sdo_state_of_charge_ = {0x6081, 0x00, CO_DEFTYPE_UNSIGNED8};

  // SDO: Unsigned 16 Bit
  COIndex sdo_operational_mode_ = {0x4801, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_charge_fault_ = {0x4802, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_discharge_fault_ = {0x4803, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_min_cell_temperature_ = {0x4808, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_max_cell_temperature_ = {0x4809, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_rem_capacity_ = {0x4900, 0x0F, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_fcc_ = {0x4900, 0x10, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_design_capacity_ = {0x4900, 0x18, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_serial_number_ = {0x4900, 0x1C, CO_DEFTYPE_UNSIGNED16};
  COIndex sdo_cell_voltages_[8] = {
    COIndex({0x4900, 0x32, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x33, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x34, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x35, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x36, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x37, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x38, CO_DEFTYPE_UNSIGNED16}),
    COIndex({0x4900, 0x39, CO_DEFTYPE_UNSIGNED16})
  };

  // SDO: Unsigned 32 Bit
  COIndex sdo_battery_voltage_ = {0x6060, 0x00, CO_DEFTYPE_UNSIGNED32};

  // SDO: Signed 16 Bit
  COIndex sdo_current_ = {0x4804, 0x00, CO_DEFTYPE_INTEGER16};
  COIndex sdo_temperature_ = {0x6010, 0x00, CO_DEFTYPE_INTEGER16};

  // SDO List: Static Reads
  COIndex sdo_static_list_[6] = {
    sdo_hardware_version_,
    sdo_software_version_,
    sdo_firmware_version_,
    sdo_fcc_,
    sdo_design_capacity_,
    sdo_serial_number_
  };

  // SDO List: Dynamic Reads
  COIndex sdo_dynamic_list_[18] = {
    sdo_state_of_charge_,
    sdo_operational_mode_,
    sdo_charge_fault_,
    sdo_discharge_fault_,
    sdo_min_cell_temperature_,
    sdo_max_cell_temperature_,
    sdo_rem_capacity_,
    sdo_battery_voltage_,
    sdo_current_,
    sdo_temperature_,
    sdo_cell_voltages_[0],
    sdo_cell_voltages_[1],
    sdo_cell_voltages_[2],
    sdo_cell_voltages_[3],
    sdo_cell_voltages_[4],
    sdo_cell_voltages_[5],
    sdo_cell_voltages_[6],
    sdo_cell_voltages_[7],
  };

  // PDO: Unsigned 8 Bit
  COIndex pdo_number_of_batteries_ = {0x4850, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_soc_ = {0x4851, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_charge_cut_off_current_ = {0x4858, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_fully_charged_ = {0x4859, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_soh_ = {0x485E, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_number_of_batteries_fault_ = {0x485F, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_number_of_active_batteries_ = {0x4860, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_operational_mode_ = {0x4861, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_soc_all_ = {0x4869, 0x00, CO_DEFTYPE_UNSIGNED8};
  COIndex pdo_master_node_id_ = {0x486C, 0x00, CO_DEFTYPE_UNSIGNED8};

  // PDO: Unsigned 16 Bit
  COIndex pdo_current_stored_ = {0x4852, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_remaining_run_time_ = {0x4853, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_remaining_charge_time_ = {0x4854, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_pack_voltage_ = {0x4855, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_discharge_current_limit_ = {0x4857, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_discharge_cut_off_voltage_ = {0x485B, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_charge_current_limit_ = {0x485C, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_max_allowed_charge_voltage_ = {0x485D, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_charge_fault_ = {0x4862, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_discharge_fault_ = {0x4863, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_regen_current_limit_ = {0x4864, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_min_cell_voltage_ = {0x4865, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_max_cell_voltage_ = {0x4866, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_cell_balance_status_all_ = {0x4867, 0x00, CO_DEFTYPE_UNSIGNED16};
  COIndex pdo_pack_voltage_all_ = {0x4868, 0x00, CO_DEFTYPE_UNSIGNED16};

  // PDO: Signed 16 Bit
  COIndex pdo_current_ = {0x4856, 0x00, CO_DEFTYPE_INTEGER16};
  COIndex pdo_temperature_ = {0x485A, 0x00, CO_DEFTYPE_INTEGER16};
  COIndex pdo_temperature_all_ = {0x486A, 0x00, CO_DEFTYPE_INTEGER16};

  // PDO List
  COIndex pdo_list_[28] = {
    pdo_number_of_batteries_,
    pdo_soc_,
    pdo_charge_cut_off_current_,
    pdo_fully_charged_,
    pdo_soh_,
    pdo_number_of_batteries_fault_,
    pdo_number_of_active_batteries_,
    pdo_operational_mode_,
    pdo_soc_all_,
    pdo_master_node_id_,
    pdo_current_stored_,
    pdo_remaining_run_time_,
    pdo_remaining_charge_time_,
    pdo_pack_voltage_,
    pdo_discharge_current_limit_,
    pdo_discharge_cut_off_voltage_,
    pdo_charge_current_limit_,
    pdo_max_allowed_charge_voltage_,
    pdo_charge_fault_,
    pdo_discharge_fault_,
    pdo_regen_current_limit_,
    pdo_min_cell_voltage_,
    pdo_max_cell_voltage_,
    pdo_cell_balance_status_all_,
    pdo_pack_voltage_all_,
    pdo_current_,
    pdo_temperature_,
    pdo_temperature_all_
  };

  // Data Structures
  std::map<uint32_t, std::string> string_data_map_;
  std::map<uint32_t, uint8_t> uint8_data_map_;
  std::map<uint32_t, uint16_t> uint16_data_map_;
  std::map<uint32_t, uint32_t> uint32_data_map_;
  std::map<uint32_t, int16_t> int16_data_map_;

  // Availability
  std::map<uint32_t, bool> available_data_map_;

  // Methods
  bool getBatteryState(sensor_msgs::msg::BatteryState &msg);
  bool getBatteryStatus(canopen_inventus_interfaces::msg::Status &msg);
  bool getVirtualBatteryState(sensor_msgs::msg::BatteryState &msg);
  bool getVirtualBatteryStatus(canopen_inventus_interfaces::msg::VirtualBattery &msg);

  bool readState();
  bool faultHighTemp(uint16_t data);
  bool faultLowTemp(uint16_t data);
  bool faultOverCurrent(uint16_t data);
  bool faultOverVoltage(uint16_t data);
  bool faultAny(uint16_t data);

  uint8_t modeBatteryStatus(uint16_t data);

  bool readSDO(COIndex index);
  bool readPDO(COIndex index);
  void readAllSDO();
  void readAllPDO();

  // Data Methods
  void initializeDataMaps();
  void initializeData(COIndex index);

  template<typename T>
  T getData(COIndex index);
  std::string getStringData(COIndex index);

  template<typename T>
  void setData(COIndex index, T data, bool update_availability = true);
  void setStringData(COIndex index, std::string data, bool update_availability = true);

  bool isAvailable(COIndex index);
  bool isAllSDOAvailable();
  bool isAllPDOAvailable();
  void clearAvailable(COIndex index);
  void clearAllAvailable();
  void clearAllSDOAvailable();
  void clearAllPDOAvailable();
  void setAvailable(COIndex index);

  // Data Conversion
  std::string getSerialNumber();
  double getScaledDouble(COIndex index, double factor);
  double getVoltage(COIndex index);
  double getTemperature(COIndex index);
  double getCurrent(COIndex index);
  double getCapacity(COIndex index);
  double getPercentage(COIndex index);

private:
  std::shared_ptr<LelyDriverBridge> driver_;
  std::mutex &sdo_mutex;
  bool read_static_sdo_;

  uint32_t index_;

  rclcpp::Logger logger_;
  std::chrono::steady_clock::time_point begin;
  std::chrono::steady_clock::time_point end;
};

} // namespace ros2_canopen

#endif // CANOPEN_INVENTUS_DRIVER__BATTERY_HPP_
