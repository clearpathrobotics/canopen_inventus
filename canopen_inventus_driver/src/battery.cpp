#include "canopen_inventus_driver/battery.hpp"

using namespace ros2_canopen;

/**
 * @brief Read SDO using the typed COData structure.
 *
 * @tparam T : Type of data in SDO index
 * @param data : COTypedData
 * @return true
 * @return false
 */
template <typename T>
bool Battery::readTypedSDO(ros2_canopen::COTypedData<T> & data)
{
  // Only allow one SDO request concurrently
  std::scoped_lock<std::mutex> lk(this->sdo_mutex);
  // Send read request
  auto f = this->driver_->template async_sdo_read_typed<T>(data.index_, data.subindex_);
  // Wait for response
  f.wait();
  // Process response
  try
  {
    data.data_ = f.get();
  }
  catch (std::exception & e)
  {
    return false;
  }
  return true;
}

/**
 * @brief Read PDO using the typed COData structure.
 *
 * @tparam T
 * @param data
 * @return true
 * @return false
 */
template <typename T>
bool Battery::readTypedPDO(ros2_canopen::COTypedData<T> & data)
{
  try
  {
    data.data_ = this->driver_->template universal_get_value<T>(data.index_, data.subindex_);
  }
  catch (std::exception & e)
  {
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

  // SDO: Read Once
  if (read_static_sdo_)
  {
    // Strings
    readTypedSDO<std::string>(sdo_hardware_version_);
    readTypedSDO<std::string>(sdo_software_version_);
    readTypedSDO<std::string>(sdo_firmware_version_);
    // Unsigned 16 Bit
    readTypedSDO<uint16_t>(sdo_serial_number_);
    readTypedSDO<uint16_t>(sdo_fcc_);
    readTypedSDO<uint16_t>(sdo_design_capacity_);

    read_static_sdo_ = false;
  }

  // SDO: Read Unsigned 8 Bit
  readTypedSDO<uint8_t>(sdo_state_of_charge_);

  // SDO: Read Unsigned 16 Bit
  readTypedSDO<uint16_t>(sdo_operational_mode_);
  readTypedSDO<uint16_t>(sdo_charge_fault_);
  readTypedSDO<uint16_t>(sdo_discharge_fault_);
  readTypedSDO<uint16_t>(sdo_min_cell_temperature_);
  readTypedSDO<uint16_t>(sdo_max_cell_temperature_);
  readTypedSDO<uint16_t>(sdo_rem_capacity_);
  for (int i = 0; i < 8; i++)
  {
    readTypedSDO<uint16_t>(sdo_cell_voltages_[i]);
  }

  // SDO: Read Unsigned 32 Bit
  readTypedSDO<uint32_t>(sdo_battery_voltage_);

  // SDO: Read Signed 16 Bit
  readTypedSDO<int16_t>(sdo_current_);
  readTypedSDO<int16_t>(sdo_temperature_);
}

/**
 * @brief Read all PDO.
 *
 */
void Battery::readAllPDO()
{
  // PDO: Read Unsigned 8 Bit
  readTypedPDO<uint8_t>(pdo_number_of_batteries_);
  readTypedPDO<uint8_t>(pdo_charge_cut_off_current_);
  readTypedPDO<uint8_t>(pdo_fully_charged_);
  readTypedPDO<uint8_t>(pdo_soh_);
  readTypedPDO<uint8_t>(pdo_number_of_batteries_fault_);
  readTypedPDO<uint8_t>(pdo_number_of_active_batteries_);
  readTypedPDO<uint8_t>(pdo_operational_mode_);
  readTypedPDO<uint8_t>(pdo_soc_all_);
  readTypedPDO<uint8_t>(pdo_master_node_id_);

  // PDO: Read Unsigned 16 Bit
  readTypedPDO<uint16_t>(pdo_current_stored_);
  readTypedPDO<uint16_t>(pdo_remaining_run_time_);
  readTypedPDO<uint16_t>(pdo_remaining_charge_time_);
  readTypedPDO<uint16_t>(pdo_pack_voltage_);
  readTypedPDO<uint16_t>(pdo_discharge_current_limit_);
  readTypedPDO<uint16_t>(pdo_discharge_cut_off_voltage_);
  readTypedPDO<uint16_t>(pdo_charge_current_limit_);
  readTypedPDO<uint16_t>(pdo_max_allowed_charge_voltage_);
  readTypedPDO<uint16_t>(pdo_charge_fault_);
  readTypedPDO<uint16_t>(pdo_discharge_fault_);
  readTypedPDO<uint16_t>(pdo_regen_current_limit_);
  readTypedPDO<uint16_t>(pdo_min_cell_voltage_);
  readTypedPDO<uint16_t>(pdo_max_cell_voltage_);
  readTypedPDO<uint16_t>(pdo_cell_balance_status_all_);
  readTypedPDO<uint16_t>(pdo_pack_voltage_all_);

  // PDO: Read Signed 16 Bit
  readTypedPDO<int16_t>(pdo_current_);
  readTypedPDO<int16_t>(pdo_temperature_);
  readTypedPDO<int16_t>(pdo_temperature_all_);
}

/**
 * @brief Create BatteryState message for this battery.
 *
 * @return sensor_msgs::msg::BatteryState
 */
sensor_msgs::msg::BatteryState Battery::getBatteryState()
{
  sensor_msgs::msg::BatteryState msg;

  // Serial Number
  msg.serial_number = std::to_string(sdo_serial_number_.data_);

  // Location, set by node
  msg.location = "";

  // Battery Voltage (V): Given 1/1024 V
  msg.voltage = double(sdo_battery_voltage_.data_) / 1024;

  // Temperature (Celsius): Given 0.125 Celsius
  msg.temperature = double(sdo_temperature_.data_) * 0.125;

  // Current (A): Given +/- 5 mA
  msg.current = double(sdo_current_.data_) * 0.005;

  // Charge (Ah): Given +/- 5 mA
  msg.charge = double(sdo_rem_capacity_.data_) * 0.005;

  // Capacity (Ah): Given +/- 5 mA
  msg.capacity = double(sdo_fcc_.data_) * 0.005;

  // Design Capacity (Ah): Given +/- 5 mA
  msg.design_capacity = double(sdo_design_capacity_.data_) * 0.005;

  // Percentage: Given percentage
  msg.percentage = double(sdo_state_of_charge_.data_) / 100;

  // Power Supply Status
  if (sdo_state_of_charge_.data_ == 100)
  {
    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    msg.power_supply_status = modeBatteryStatus(sdo_operational_mode_.data_);
  }

  // Power Supply Health
  msg.power_supply_health = 0;
  if (faultHighTemp(sdo_charge_fault_.data_) || faultHighTemp(sdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERHEAT;
  }
  else if (faultLowTemp(sdo_charge_fault_.data_) || faultLowTemp(sdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_COLD;
  }
  else if (faultOverVoltage(sdo_charge_fault_.data_) || faultOverVoltage(sdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  }
  else if (faultAny(sdo_charge_fault_.data_) || faultAny(sdo_discharge_fault_.data_))
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
  for (int i = 0; i < 8; i ++)
  {
    msg.cell_voltage.push_back(double(sdo_cell_voltages_[i].data_) / 1024);
  }

  return msg;
}

/**
 * @brief Create BatteryState message for the Virtual Battery. Only the
 * master battery needs to create and publish this message.
 *
 * @return sensor_msgs::msg::BatteryState
 */
sensor_msgs::msg::BatteryState Battery::getVirtualBatteryState()
{
  sensor_msgs::msg::BatteryState msg;

  // Serial Number
  msg.serial_number = std::to_string(sdo_serial_number_.data_);

  // Location, set by node
  msg.location = "";

  // Battery Voltage (V): Given 1/1024 V
  msg.voltage = double(pdo_pack_voltage_all_.data_) / 1024;

  // Temperature (Celsius): Given 0.125 Celsius
  msg.temperature = double(pdo_temperature_all_.data_) * 0.125;

  // Current (A): Given +/- 5 mA
  msg.current = double(pdo_current_.data_) * 0.1;

  // Charge (Ah):
  msg.charge = double(pdo_current_stored_.data_);

  // Capacity (Ah): Given +/- 5 mA
  // msg.capacity = double(sdo_fcc_.data_) * 0.005;

  // Design Capacity (Ah): Given +/- 5 mA
  // msg.design_capacity = double(sdo_design_capacity_.data_) * 0.005;

  // Percentage: Given percentage
  msg.percentage = double(pdo_soc_all_.data_) / 100;

  // Power Supply Status
  if (pdo_fully_charged_.data_)
  {
    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    msg.power_supply_status = modeBatteryStatus(pdo_operational_mode_.data_);
  }

  // Power Supply Health
  msg.power_supply_health = 0;
  if (faultHighTemp(pdo_charge_fault_.data_) || faultHighTemp(pdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERHEAT;
  }
  else if (faultLowTemp(pdo_charge_fault_.data_) || faultLowTemp(pdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_COLD;
  }
  else if (faultOverVoltage(pdo_charge_fault_.data_) || faultOverVoltage(pdo_discharge_fault_.data_))
  {
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  }
  else if (faultAny(pdo_charge_fault_.data_) || faultAny(pdo_discharge_fault_.data_))
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

  return msg;
}

/**
 * @brief Battery Status custom message for the Inventus battery.
 * Publishes data retrieved from SDO indices for this battery.
 *
 * @return canopen_inventus_interfaces::msg::Status
 */
canopen_inventus_interfaces::msg::Status Battery::getBatteryStatus()
{
  canopen_inventus_interfaces::msg::Status msg;

  // Versions and Serial Number
  msg.hardware_version = sdo_hardware_version_.data_;
  msg.software_version = sdo_software_version_.data_;
  msg.firmware_version = sdo_firmware_version_.data_;
  msg.serial_number = std::to_string(sdo_serial_number_.data_);

  // Current
  msg.current = double(sdo_current_.data_) * 0.005;
  // Voltage
  msg.voltage = double(sdo_battery_voltage_.data_) / 1024;
  // Temperature
  msg.temperature = double(sdo_temperature_.data_) * 0.125;
  // Max. and Min. Cell Temperatures
  msg.min_cell_temperature = double(sdo_min_cell_temperature_.data_) * 0.125;
  msg.max_cell_temperature = double(sdo_max_cell_temperature_.data_) * 0.125;
  // State of Charge
  msg.state_of_charge = double(sdo_state_of_charge_.data_) / 100;
  // Full Charge Capcacity
  msg.full_charge_capacity = double(sdo_fcc_.data_) * 0.05;
  // Design Capacity
  msg.design_capacity = double(sdo_design_capacity_.data_) * 0.05;
  // Remaining Capacity
  msg.remaining_capacity = double(sdo_rem_capacity_.data_) * 0.05;
  // Cell Voltages
  for (int i = 0; i < 8; i ++)
  {
    msg.cell_voltage.push_back(double(sdo_cell_voltages_[i].data_) / 1024);
  }
  // Operation Mode and Faults
  msg.op_mode = sdo_operational_mode_.data_;
  msg.charge_fault = sdo_charge_fault_.data_;
  msg.discharge_fault = sdo_discharge_fault_.data_;

  return msg;
}

/**
 * @brief Create VirtualBattery message which contains all data from PDO
 * indices for the virtual battery pack that includes all batteries in
 * the system.
 *
 * @return canopen_inventus_interfaces::msg::VirtualBattery
 */
canopen_inventus_interfaces::msg::VirtualBattery Battery::getVirtualBatteryStatus()
{
  canopen_inventus_interfaces::msg::VirtualBattery msg;

  // TPDO1: Number of Batteries
  msg.number_of_batteries = pdo_number_of_batteries_.data_;
  // TPDO1: State of Charge
  // msg.soc = typed_
  // TPDO1: Sum Capacity
  msg.sum_capacity = double(pdo_current_stored_.data_);
  // TPDO1: Remaining Run Time
  msg.remaining_runtime = double(pdo_remaining_run_time_.data_);
  // TPDO1: Remaining Charge Time
  msg.remaining_chargetime = double(pdo_remaining_charge_time_.data_);

  // TPDO2: Average Voltage
  msg.avg_voltage = double(pdo_pack_voltage_.data_) / 1024;
  // TPDO2: Sum Current
  msg.sum_current = double(pdo_current_.data_) / 10.0;
  // TPDO2: Discharge Current Limit
  msg.discharge_current_limit = double(pdo_discharge_current_limit_.data_) / 10.0;
  // TPDO2: Charge Cutoff Current
  msg.charge_cutoff_current = double(pdo_charge_cut_off_current_.data_) / 10.0;
  // TPDO2: Fully Charged
  msg.fully_charged = pdo_fully_charged_.data_;

  // TPDO3: Average Temperature
  msg.avg_temp = double(pdo_temperature_.data_) * 0.125;
  // TPDO3: Discharge Cutoff Voltage
  msg.discharge_cutoff_voltage = double(pdo_discharge_cut_off_voltage_.data_) / 1024;
  // TPDO3: Charge Current Limit
  msg.charge_current_limit = double(pdo_charge_current_limit_.data_) / 10.0;
  // TPDO3: Max Voltage Allowed
  msg.max_voltage_allowed = double(pdo_max_allowed_charge_voltage_.data_) / 1024;

  // TPDO4: Average Health
  msg.avg_health = pdo_soh_.data_;
  // TPDO4: Number of Faulted Batteries
  msg.num_faulted_batteries = pdo_number_of_batteries_fault_.data_;
  // TPDO4: Number of Active Batteries
  msg.num_active_batteries = pdo_number_of_active_batteries_.data_;
  // TPDO4: Operation Mode
  msg.op_mode = sdo_operational_mode_.data_;
  // TPDO4: Charge Fault
  msg.charge_fault = sdo_charge_fault_.data_;
  // TPDO4: Discharge Fault
  msg.discharge_fault = sdo_discharge_fault_.data_;

  // TPDO5: Regen Current Limit
  msg.regen_current_limit = double(pdo_regen_current_limit_.data_)/ 10.0;
  // TPDO5: Min. Cell Voltage
  msg.min_cell_voltage = double(pdo_min_cell_voltage_.data_) / 1024;
  // TPDO5: Max. Cell Voltage
  msg.max_cell_voltage = double(pdo_max_cell_voltage_.data_) / 1024;
  // TPDO5: Cell Balance Status
  msg.cell_balance_status = double(pdo_cell_balance_status_all_.data_);

  // TPDO6: All Average Voltage
  msg.all_avg_voltage = double(pdo_pack_voltage_all_.data_) / 1024;
  // TPDO6: All State of Charge
  msg.all_soc = double(pdo_soc_all_.data_);
  // TPDO6: All Average Temperature
  msg.all_avg_temp = double(pdo_temperature_all_.data_) * 0.125;
  // TPDO6: Node ID of Master Pack
  msg.master_node_id = pdo_master_node_id_.data_;

  return msg;
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
  return data | 0x00;
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
