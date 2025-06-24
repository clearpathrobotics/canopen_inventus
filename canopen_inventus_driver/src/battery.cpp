/**
Software License Agreement (BSD)

\file      battery.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "canopen_inventus_driver/battery.hpp"

using namespace ros2_canopen;

/**
 * @brief Read SDO using either typed or COData read from the LelyBridge
 *
 * @param index
 * @return true
 * @return false
 */
bool Battery::readSDO(COIndex index)
{
  // Only allow one SDO request concurrently
  std::scoped_lock<std::mutex> lk(this->sdo_mutex);

  // Strings use typed read.
  if (index.type_ == CO_DEFTYPE_VISIBLE_STRING)
  {
    try
    {
      auto f = this->driver_->template async_sdo_read_typed<std::string>(index.index_, index.subindex_);
      f.wait();
      std::string data = f.get();
      setStringData(index, data);
    }
    catch (std::exception & e)
    {
      RCLCPP_DEBUG(logger_, e.what());
      clearAvailable(index);
      return false;
    }
  }
  // Integers use COData read.
  else
  {
    COData data = {index.index_, index.subindex_, 0U};
    auto f = this->driver_->async_sdo_read(data);
    try
    {
      data.data_ = f.get().data_;
      switch(index.type_)
      {
        case CO_DEFTYPE_INTEGER16:
          setData<int16_t>(index, data.data_);
          break;
        case CO_DEFTYPE_UNSIGNED8:
          setData<uint8_t>(index, data.data_);
          break;
        case CO_DEFTYPE_UNSIGNED16:
          setData<uint16_t>(index, data.data_);
          break;
        case CO_DEFTYPE_UNSIGNED32:
          setData<uint32_t>(index, data.data_);
          break;
        default:
          RCLCPP_DEBUG(logger_, "readSDO: type not known.");
          return false;
      }
    }
    catch (std::exception & e)
    {
      RCLCPP_DEBUG(logger_, e.what());
      clearAvailable(index);
      return false;
    }
  }
  return true;
}

/**
 * @brief Read PDO and store based on type.
 *
 * @param index
 * @return true
 * @return false
 */
bool Battery::readPDO(COIndex index)
{
  try
  {
    switch(index.type_)
    {
      case CO_DEFTYPE_INTEGER16:
        setData(index, this->driver_->template universal_get_value<int16_t>(index.index_, index.subindex_), false);
        break;
      case CO_DEFTYPE_UNSIGNED8:
        setData(index, this->driver_->template universal_get_value<uint8_t>(index.index_, index.subindex_), false);
        break;
      case CO_DEFTYPE_UNSIGNED16:
        setData(index, this->driver_->template universal_get_value<uint16_t>(index.index_, index.subindex_), false);
        break;
      case CO_DEFTYPE_UNSIGNED32:
        setData(index, this->driver_->template universal_get_value<uint32_t>(index.index_, index.subindex_), false);
        break;
      case CO_DEFTYPE_VISIBLE_STRING:
        setStringData(index, this->driver_->template universal_get_value<std::string>(index.index_, index.subindex_), false);
        break;
      default:
        return false;
    }
  }
  catch (std::exception & e)
  {
    RCLCPP_DEBUG(logger_, e.what());
    clearAvailable(index);
    return false;
  }
  return true;
}

/**
 * @brief Read all SDO indices. Skip those that do not need to be read on
 * every iteration.
 *
 */
void Battery::readAllSDO()
{
  // Initialization: Read SDO that only need to be read once.
  if (read_static_sdo_)
  {
    for(auto i : sdo_static_list_)
    {
      if (!isAvailable(i))
      {
        readSDO(i);
        return;
      }
    }
    // Done
    read_static_sdo_ = false;
  }

  for(auto i : sdo_dynamic_list_)
  {
    if (!isAvailable(i))
    {
      readSDO(i);
      return;
    }
  }
}

/**
 * @brief Read all PDO.
 *
 */
void Battery::readAllPDO()
{
  for (auto i : pdo_list_)
  {
    readPDO(i);
  }
}

/**
 * @brief Create BatteryState message for this battery.
 *
 * @return bool
 */
bool Battery::getBatteryState(sensor_msgs::msg::BatteryState &msg)
{
  // Serial Number
  msg.serial_number = getSerialNumber();

  // Location, set by node
  msg.location = "";

  // Battery Voltage (V)
  msg.voltage = getScaledDouble(sdo_battery_voltage_, 1.0/1024);

  // Temperature (Celsius)
  msg.temperature = getTemperature(sdo_temperature_);

  // Current (A)
  msg.current = getCurrent(sdo_current_);

  // Charge (Ah)
  msg.charge = getCapacity(sdo_rem_capacity_);

  // Capacity (Ah)
  msg.capacity = getCapacity(sdo_fcc_);

  // Design Capacity (Ah)
  msg.design_capacity = getCapacity(sdo_design_capacity_);

  // Percentage
  msg.percentage = getPercentage(sdo_state_of_charge_);

  // Power Supply Status
  if (msg.percentage == 1.0)
  {
    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    msg.power_supply_status = modeBatteryStatus(getData<uint16_t>(sdo_operational_mode_));
  }

  // Power Supply Health
  msg.power_supply_health = 0;
  if (faultHighTemp(getData<uint16_t>(sdo_charge_fault_)) || faultHighTemp(getData<uint16_t>(sdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERHEAT;
  }
  else if (faultLowTemp(getData<uint16_t>(sdo_charge_fault_)) || faultLowTemp(getData<uint16_t>(sdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_COLD;
  }
  else if (faultOverVoltage(getData<uint16_t>(sdo_charge_fault_)) || faultOverVoltage(getData<uint16_t>(sdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  }
  else if (faultAny(getData<uint16_t>(sdo_charge_fault_)) || faultAny(getData<uint16_t>(sdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
  }
  else
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_GOOD;
  }

  // Power Supply Technology
  msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_LIFE;

  // Present
  msg.present = true;

  // Cell Voltages
  msg.cell_voltage.clear();
  for (int i = 0; i < 8; i ++)
  {
    msg.cell_voltage.push_back(getVoltage(sdo_cell_voltages_[i]));
  }

  return isAllSDOAvailable();
}

/**
 * @brief Create BatteryState message for the Virtual Battery. Only the
 * master battery needs to create and publish this message.
 *
 * @return sensor_msgs::msg::BatteryState
 */
bool Battery::getVirtualBatteryState(sensor_msgs::msg::BatteryState & msg)
{
  // Serial Number
  msg.serial_number = getSerialNumber();

  // Location, set by node
  msg.location = "";

  // Battery Voltage (V)
  msg.voltage = getVoltage(pdo_pack_voltage_all_);

  // Temperature (Celsius)
  msg.temperature = getTemperature(pdo_temperature_all_);

  // Current (A)
  msg.current = getCurrent(pdo_current_);

  // Charge (Ah):
  msg.charge = double(getData<uint16_t>(pdo_current_stored_));

  // Percentage
  msg.percentage = getPercentage(pdo_soc_all_);

  // Capacity (Ah): Missing for Virtual Battery
  msg.capacity = msg.charge / msg.percentage;

  // Design Capacity (Ah): Missing for Virtual Battery
  msg.design_capacity = msg.charge / msg.percentage;


  // Power Supply Status
  if (getData<uint8_t>(pdo_fully_charged_))
  {
    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    msg.power_supply_status = modeBatteryStatus(getData<uint8_t>(pdo_operational_mode_));
  }

  // Power Supply Health
  msg.power_supply_health = 0;
  if (faultHighTemp(getData<uint16_t>(pdo_charge_fault_)) || faultHighTemp(getData<uint16_t>(pdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERHEAT;
  }
  else if (faultLowTemp(getData<uint16_t>(pdo_charge_fault_)) || faultLowTemp(getData<uint16_t>(pdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_COLD;
  }
  else if (faultOverVoltage(getData<uint16_t>(pdo_charge_fault_)) || faultOverVoltage(getData<uint16_t>(pdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  }
  else if (faultAny(getData<uint16_t>(pdo_charge_fault_)) || faultAny(getData<uint16_t>(pdo_discharge_fault_)))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
  }
  else
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_GOOD;
  }

  // Power Supply Technology
  msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_LIFE;

  // Present
  msg.present = true;

  return isAllPDOAvailable();
}

/**
 * @brief Battery Status custom message for the Inventus battery.
 * Publishes data retrieved from SDO indices for this battery.
 *
 * @return canopen_inventus_interfaces::msg::Status
 */
bool Battery::getBatteryStatus(canopen_inventus_interfaces::msg::Status &msg)
{
  // Versions and Serial Number
  msg.hardware_version = getStringData(sdo_hardware_version_);
  msg.software_version = getStringData(sdo_software_version_);
  msg.firmware_version = getStringData(sdo_firmware_version_);
  msg.serial_number = getSerialNumber();

  // Current
  msg.current = getCurrent(sdo_current_);

  // Voltage
  msg.voltage = getScaledDouble(sdo_battery_voltage_, 1.0/1024);

  // Temperature
  msg.temperature = getTemperature(sdo_temperature_);

  // Max. and Min. Cell Temperatures
  msg.min_cell_temperature = getTemperature(sdo_min_cell_temperature_);
  msg.max_cell_temperature = getTemperature(sdo_max_cell_temperature_);

  // State of Charge
  msg.state_of_charge = getPercentage(sdo_state_of_charge_);

  // Full Charge Capcacity
  msg.full_charge_capacity = getCapacity(sdo_fcc_);

  // Design Capacity
  msg.design_capacity = getCapacity(sdo_design_capacity_);

  // Remaining Capacity
  msg.remaining_capacity = getCapacity(sdo_rem_capacity_);

  // Cell Voltages
  msg.cell_voltage.clear();
  for (int i = 0; i < 8; i ++)
  {
    msg.cell_voltage.push_back(getVoltage(sdo_cell_voltages_[i]));
  }

  // Operation Mode and Faults
  msg.op_mode = getData<uint16_t>(sdo_operational_mode_);
  msg.charge_fault = getData<uint16_t>(sdo_charge_fault_);
  msg.discharge_fault = getData<uint16_t>(sdo_discharge_fault_);

  return isAllSDOAvailable();
}

/**
 * @brief Create VirtualBattery message which contains all data from PDO
 * indices for the virtual battery pack that includes all batteries in
 * the system.
 *
 * @return canopen_inventus_interfaces::msg::VirtualBattery
 */
bool Battery::getVirtualBatteryStatus(canopen_inventus_interfaces::msg::VirtualBattery &msg)
{
  // TPDO1: Number of Batteries
  msg.number_of_batteries = getData<uint8_t>(pdo_number_of_batteries_);
  // TPDO1: State of Charge
  msg.soc = getPercentage(pdo_soc_);
  // TPDO1: Sum Capacity
  msg.sum_capacity = getScaledDouble(pdo_current_stored_, 1.0);
  // TPDO1: Remaining Run Time in Minutes
  msg.remaining_runtime = getScaledDouble(pdo_remaining_run_time_, 1.0);
  // TPDO1: Remaining Charge Time in Minutes
  msg.remaining_chargetime = getScaledDouble(pdo_remaining_charge_time_, 1.0);
  if(msg.remaining_chargetime == 0xFFFF)
  {
    msg.remaining_chargetime = -1;
  }

  // TPDO2: Average Voltage
  msg.avg_voltage = getVoltage(pdo_pack_voltage_);
  // TPDO2: Sum Current
  msg.sum_current = getCurrent(pdo_current_);
  // TPDO2: Discharge Current Limit
  msg.discharge_current_limit = getCurrent(pdo_discharge_current_limit_);
  // TPDO2: Charge Cutoff Current
  msg.charge_cutoff_current = getCurrent(pdo_charge_cut_off_current_);
  // TPDO2: Fully Charged
  msg.fully_charged = bool(getData<uint8_t>(pdo_fully_charged_));

  // TPDO3: Average Temperature
  msg.avg_temp = getTemperature(pdo_temperature_);
  // TPDO3: Discharge Cutoff Voltage
  msg.discharge_cutoff_voltage = getVoltage(pdo_discharge_cut_off_voltage_);
  // TPDO3: Charge Current Limit
  msg.charge_current_limit = getCurrent(pdo_charge_current_limit_);
  // TPDO3: Max Voltage Allowed
  msg.max_voltage_allowed = getVoltage(pdo_max_allowed_charge_voltage_);
  // TPDO4: Average Health
  msg.avg_health = getData<uint8_t>(pdo_soh_);
  // TPDO4: Number of Faulted Batteries
  msg.num_faulted_batteries = getData<uint8_t>(pdo_number_of_batteries_fault_);
  // TPDO4: Number of Active Batteries
  msg.num_active_batteries = getData<uint8_t>(pdo_number_of_active_batteries_);
  // TPDO4: Operation Mode
  msg.op_mode = getData<uint8_t>(pdo_operational_mode_);
  // TPDO4: Charge Fault
  msg.charge_fault = getData<uint16_t>(pdo_charge_fault_);
  // TPDO4: Discharge Fault
  msg.discharge_fault = getData<uint16_t>(pdo_discharge_fault_);

  // TPDO5: Regen Current Limit
  msg.regen_current_limit = getCurrent(pdo_regen_current_limit_);
  // TPDO5: Min. Cell Voltage
  msg.min_cell_voltage = getVoltage(pdo_min_cell_voltage_);
  // TPDO5: Max. Cell Voltage
  msg.max_cell_voltage = getVoltage(pdo_max_cell_voltage_);
  // TPDO5: Cell Balance Status
  msg.cell_balance_status = getData<uint16_t>(pdo_cell_balance_status_all_);

  // TPDO6: All Average Voltage
  msg.all_avg_voltage = getVoltage(pdo_pack_voltage_all_);
  // TPDO6: All State of Charge
  msg.all_soc = getPercentage(pdo_soc_all_);
  // TPDO6: All Average Temperature
  msg.all_avg_temp = getTemperature(pdo_temperature_all_);
  // TPDO6: Node ID of Master Pack
  msg.master_node_id = getData<uint8_t>(pdo_master_node_id_);

  return isAllPDOAvailable();
}

/**
 * @brief Check if Charge/Discharge fault was triggered by high
 * temperatures.
 *
 * @param data
 * @return true
 * @return false
 */
bool Battery::faultHighTemp(uint16_t data)
{
  return data & 0x01;
}

/**
 * @brief Check if Charge/Discharge fault was triggered by low
 * temperatures.
 *
 * @param data
 * @return true
 * @return false
 */
bool Battery::faultLowTemp(uint16_t data)
{
  return data & 0x02;
}

/**
 * @brief Check if Charge/Discharge fault was triggered by over current.
 *
 * @param data
 * @return true
 * @return false
 */
bool Battery::faultOverCurrent(uint16_t data)
{
  return data & 0x04;
}

/**
 * @brief Check if Charge/Discharge fault was triggered by over voltage.
 *
 * @param data
 * @return true
 * @return false
 */
bool Battery::faultOverVoltage(uint16_t data)
{
  return data & 0x08;
}

/**
 * @brief Check if Charge/Discharge fault was triggered for any reason.
 *
 * @param data
 * @return true
 * @return false
 */
bool Battery::faultAny(uint16_t data)
{
  bool any = false;
  // Check all faults, ignore reserved
  for (int i = 0; i < 16; i++)
  {
    // Reserved range
    if ((i > 8) | (i < 15))
    {
      continue;
    }
    // Check fault
    any &= data & (0x01 << i);
  }
  return any;
}

/**
 * @brief Convert Inventus Operation Mode to BatteryState status.
 *
 * @param data
 * @return uint8_t
 */
uint8_t Battery::modeBatteryStatus(uint16_t data)
{
  // Standby
  if (data == 0x04)
  {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  // Discharge or Pre-Discharge
  else if (data == 0x05 or data == 0x03)
  {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  // Charge or Pre-Charge
  else if (data == 0x06 or data == 0x08)
  {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
  // None
  else
  {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }
}

/**
 * @brief Initialize data maps for the three main data indices.
 *
 */
void Battery::initializeDataMaps()
{
  // SDO: Static Reads
  for(int i = 0; i < 6; i++)
  {
    initializeData(sdo_static_list_[i]);
  }
  // SDO: Dynamic Reads
  for(int i = 0; i < 18; i++)
  {
    initializeData(sdo_dynamic_list_[i]);
  }
  // PDO
  for(int i = 0; i < 27; i++)
  {
    initializeData(pdo_list_[i]);
  }
}

/**
 * @brief Initialize default entry in typed map for given index.
 *
 * @param index
 */
void Battery::initializeData(COIndex index)
{
  switch(index.type_)
  {
    case CO_DEFTYPE_INTEGER16:
      int16_data_map_[index.getUniqueIndex()] = 0;
      break;
    case CO_DEFTYPE_UNSIGNED8:
      uint8_data_map_[index.getUniqueIndex()] = 0;
      break;
    case CO_DEFTYPE_UNSIGNED16:
      uint16_data_map_[index.getUniqueIndex()] = 0;
      break;
    case CO_DEFTYPE_UNSIGNED32:
      uint32_data_map_[index.getUniqueIndex()] = 0;
      break;
    case CO_DEFTYPE_VISIBLE_STRING:
      string_data_map_[index.getUniqueIndex()] = "";
      break;
    default:
      return;
  }
  available_data_map_[index.getUniqueIndex()] = false;
}

/**
 * @brief Retrieve data corresponding to index and type
 *
 * @tparam T
 * @param index
 * @return T
 */
template<typename T>
T Battery::getData(COIndex index)
{
  switch(index.type_)
  {
    case CO_DEFTYPE_INTEGER16:
      return int16_data_map_[index.getUniqueIndex()];
    case CO_DEFTYPE_UNSIGNED8:
      return uint8_data_map_[index.getUniqueIndex()];
    case CO_DEFTYPE_UNSIGNED16:
      return uint16_data_map_[index.getUniqueIndex()];
    case CO_DEFTYPE_UNSIGNED32:
      return uint32_data_map_[index.getUniqueIndex()];
    default:
      return -1;
  }
}

/**
 * @brief Retrieve string data. Cutout specific for special type.
 *
 * @param index
 * @return std::string
 */
std::string Battery::getStringData(COIndex index)
{
  return string_data_map_[index.getUniqueIndex()];
}

/**
 * @brief Set data corresponding to index and type.
 *
 * @tparam T
 * @param index
 * @param data
 */
template<typename T>
void Battery::setData(COIndex index, T data, bool update_availability)
{
  switch(index.type_)
  {
    case CO_DEFTYPE_INTEGER16:
      int16_data_map_[index.getUniqueIndex()] = int16_t(data);
      break;
    case CO_DEFTYPE_UNSIGNED8:
      uint8_data_map_[index.getUniqueIndex()] = uint8_t(data);
      break;
    case CO_DEFTYPE_UNSIGNED16:
      uint16_data_map_[index.getUniqueIndex()] = uint16_t(data);
      break;
    case CO_DEFTYPE_UNSIGNED32:
      uint32_data_map_[index.getUniqueIndex()] = uint32_t(data);
      break;
    default:
      return;
  }
  if (update_availability)
  {
    setAvailable(index);
  }
}

/**
 * @brief Set string data. Cutout for special type.
 *
 * @param index
 * @param data
 */
void Battery::setStringData(COIndex index, std::string data, bool update_availability)
{
  string_data_map_[index.getUniqueIndex()] = data;
  if (update_availability)
  {
    setAvailable(index);;
  }
}

/**
 * @brief Check if data for given index is available.
 *
 * @param index
 * @return true
 * @return false
 */
bool Battery::isAvailable(COIndex index)
{
  return available_data_map_[index.getUniqueIndex()];
}

/**
 * @brief Check if all SDO data is available
 *
 */
bool Battery::isAllSDOAvailable()
{
  bool available = true;
  for (COIndex it : sdo_dynamic_list_)
  {
    available = available && isAvailable(it);
  }
  return available;
}

/**
 * @brief Check if all PDO data is available
 *
 */
bool Battery::isAllPDOAvailable()
{
  bool available = true;
  for (COIndex it : pdo_list_)
  {
    available = available && isAvailable(it);
  }
  return available;
}

/**
 * @brief Lower available flag for given index.
 *
 * @param index
 */
void Battery::clearAvailable(COIndex index)
{
  available_data_map_[index.getUniqueIndex()] = false;
}

/**
 * @brief Lower available flag for all indices.
 *
 */
void Battery::clearAllAvailable()
{
  for (auto it = available_data_map_.begin(); it != available_data_map_.end(); it++)
  {
    it->second = false;
  }
}

/**
 * @brief Lower available flag for all SDOs
 */
void Battery::clearAllSDOAvailable()
{
  for (COIndex it : sdo_dynamic_list_)
  {
    clearAvailable(it);
  }
}

/**
 * @brief Lower available flag for all PDOs
 *
 */
void Battery::clearAllPDOAvailable()
{
  for (COIndex it : pdo_list_)
  {
    clearAvailable(it);
  }
}

/**
 * @brief Raise available flag for index
 *
 * @param index
 */
void Battery::setAvailable(COIndex index)
{
  available_data_map_[index.getUniqueIndex()] = true;
}

/**
 * @brief Convert integer data to floating point data and scale it
 * by given value
 *
 * @param index
 * @param factor
 * @return double
 */
double Battery::getScaledDouble(COIndex index, double factor)
{
  double value = 0.0;
  if (index.type_ == CO_DEFTYPE_INTEGER16)
  {
    value = double(getData<int16_t>(index));
  }
  else if (index.type_ == CO_DEFTYPE_UNSIGNED8)
  {
    value = double(getData<uint8_t>(index));
  }
  else if (index.type_ == CO_DEFTYPE_UNSIGNED16)
  {
    value = double(getData<uint16_t>(index));
  }
  else if (index.type_ == CO_DEFTYPE_UNSIGNED32)
  {
    value = double(getData<uint32_t>(index));
  }
  return value * factor;
}

/**
 * @brief Get voltage (V) by scaling from given mV.
 *
 * @param index
 * @return double
 */
double Battery::getVoltage(COIndex index)
{
  return getScaledDouble(index, 0.001);
}

/**
 * @brief Get temperature (Celsius) by scaling from given +/- 0.125C
 *
 * @param index
 * @return double
 */
double Battery::getTemperature(COIndex index)
{
  return getScaledDouble(index, 0.125);
}

/**
 * @brief Get current (A) by scaling given 100mA
 *
 * @param index
 * @return double
 */
double Battery::getCurrent(COIndex index)
{
  return getScaledDouble(index, 0.1);
}

/**
 * @brief Get charge capacity (Ah) by scaling given 5mAh
 *
 * @param index
 * @return double
 */
double Battery::getCapacity(COIndex index)
{
  return getScaledDouble(index, 0.005);
}

/**
 * @brief Get percentage by converting given value into decimal
 *
 * @param index
 * @return double
 */
double Battery::getPercentage(COIndex index)
{
  return getScaledDouble(index, 0.01);
}

/**
 * @brief Convert integer serial number to string.
 *
 * @return std::string
 */
std::string Battery::getSerialNumber()
{
  return std::to_string(getData<uint16_t>(sdo_serial_number_));
};


