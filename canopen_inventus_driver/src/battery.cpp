#include "canopen_inventus_driver/battery.hpp"

using namespace ros2_canopen;

bool Battery::readState()
{
  // SDO: Software Version
  // driver_->sync_sdo_read_typed<decltype(software_version_)>(SDO_SOFTWARE_VERSION, SUB_INDEX_ZERO, software_version_, std::chrono::milliseconds(20ms));

  // SDO: Hardware Version
  // driver_->sync_sdo_read_typed<decltype(hardware_version_)>(SDO_HARDWARE_VERSION, SUB_INDEX_ZERO, hardware_version_, std::chrono::milliseconds(20ms));

  // SDO: Firmware Version
  // driver_->sync_sdo_read_typed<decltype(firmware_version_)>(SDO_FIRMWARE_VERSION, SDO_FIRMWARE_VERSION_SUB, firmware_version_, std::chrono::milliseconds(20ms));

  // SDO: BQ8050 Serial Number
  // serial_number_ = driver_->universal_get_value<decltype(serial_number_)>(SDO_BQ8050_DATA, SDO_BQ8050_SUB_SN);

  // // SDO: BQ8050 Design Capacity
  // design_capacity_ = driver_->universal_get_value<decltype(design_capacity_)>(SDO_BQ8050_DATA, SDO_BQ8050_SUB_DESIGN_CAPACITY);

  // // SDO: BQ8050 Rem Cap
  // rem_cap_ = driver_->universal_get_value<decltype(rem_cap_)>(SDO_BQ8050_DATA, SDO_BQ8050_SUB_REM_CAP);

  // // SDO: BQ8050 FCC
  // fcc_ = driver_->universal_get_value<decltype(fcc_)>(SDO_BQ8050_DATA, SDO_BQ8050_SUB_FCC);

  // // SDO: BQ8050 Cell Voltage
  // for(int i = 0; i < 8; i++)
  // {
  //   cell_voltage_[i] = driver_->universal_get_value<uint16_t>(SDO_BQ8050_DATA, SDO_BQ8050_SUB_CELL + i);
  // }

  // // SDO: Operational Mode
  // operational_mode_ = driver_->universal_get_value<decltype(operational_mode_)>(SDO_OPERATIONAL_MODE, SUB_INDEX_ZERO);

  // // SDO: Charge Fault
  // charge_fault_ = driver_->universal_get_value<decltype(charge_fault_)>(SDO_CHARGE_FAULT, SUB_INDEX_ZERO);

  // // SDO: Discharge Fault
  // discharge_fault_ = driver_->universal_get_value<decltype(discharge_fault_)>(SDO_DISCHARGE_FAULT, SUB_INDEX_ZERO);

  // // SDO: Battery Voltage
  // battery_voltage_ = driver_->universal_get_value<decltype(discharge_fault_)>(SDO_BATTERY_VOLTAGE, SUB_INDEX_ZERO);

  // // SDO: Temperature
  // temperature_ = driver_->universal_get_value<decltype(temperature_)>(SDO_TEMPERATURE, SUB_INDEX_ZERO);

  // // SDO: Current
  // current_ = driver_->universal_get_value<decltype(current_)>(SDO_CURRENT, SUB_INDEX_ZERO);

  // // SDO: Min. Cell Temperature
  // min_cell_temperature_ = driver_->universal_get_value<decltype(min_cell_temperature_)>(SDO_MIN_CELL_TEMPERATURE, SUB_INDEX_ZERO);

  // // SDO: Max. Cell Temperature
  // max_cell_temperature_ = driver_->universal_get_value<decltype(max_cell_temperature_)>(SDO_MAX_CELL_TEMPERATURE, SUB_INDEX_ZERO);

  // SDO: Battery State of Charge
  // battery_state_of_charge_ = driver_->universal_get_value<decltype(battery_state_of_charge_)>(SDO_BATTERY_STATE_OF_CHARGE, SUB_INDEX_ZERO);

  // PDO: VB Number of Batteries
  vb_number_of_batteries_ = driver_->universal_get_value<decltype(vb_number_of_batteries_)>(PDO_NUMBER_OF_BATTERIES, SUB_INDEX_ZERO);

  // PDO: VB Current Stored
  vb_current_stored_ = driver_->universal_get_value<decltype(vb_current_stored_)>(PDO_CURRENT_STORED, SUB_INDEX_ZERO);

  // PDO: VB Remaining Run Time
  vb_remaining_run_time_ = driver_->universal_get_value<decltype(vb_remaining_run_time_)>(PDO_REMAINING_RUN_TIME, SUB_INDEX_ZERO);

  // PDO: VB Remaining Charge Time
  vb_remaining_charge_time_ = driver_->universal_get_value<decltype(vb_remaining_charge_time_)>(PDO_REMAINING_CHARGE_TIME, SUB_INDEX_ZERO);

  // PDO: VB Pack Voltage
  vb_pack_voltage_ = driver_->universal_get_value<decltype(vb_pack_voltage_)>(PDO_PACK_VOLTAGE, SUB_INDEX_ZERO);

  // PDO: VB Current
  vb_current_ = driver_->universal_get_value<decltype(vb_current_)>(PDO_CURRENT, SUB_INDEX_ZERO);

  // PDO: VB Discharge Current Limit
  vb_discharge_current_limit_ = driver_->universal_get_value<decltype(vb_discharge_current_limit_)>(PDO_DISCHARGE_CURRENT_LIMIT, SUB_INDEX_ZERO);

  // PDO: VB Charge Cut Off Current
  vb_charge_cut_off_current_ = driver_->universal_get_value<decltype(vb_charge_cut_off_current_)>(PDO_CHARGE_CUT_OFF_CURRENT, SUB_INDEX_ZERO);

  // PDO: VB Fully Charged
  vb_fully_charged_ = driver_->universal_get_value<decltype(vb_fully_charged_)>(PDO_FULLY_CHARGED, SUB_INDEX_ZERO);

  // PDO: VB Temperature
  vb_temperature_ = driver_->universal_get_value<decltype(vb_temperature_)>(PDO_TEMPERATURE, SUB_INDEX_ZERO);

  // PDO: VB Discharge Cut Off Voltage
  vb_discharge_cut_off_voltage_ = driver_->universal_get_value<decltype(vb_discharge_cut_off_voltage_)>(PDO_DISCHARGE_CUT_OFF_VOLTAGE, SUB_INDEX_ZERO);

  // PDO: VB Charge Current Limit
  vb_charge_current_limit_ = driver_->universal_get_value<decltype(vb_charge_current_limit_)>(PDO_CHARGE_CURRENT_LIMIT, SUB_INDEX_ZERO);

  // PDO: VB Max Allowed Charge Voltage
  vb_max_allowed_charge_voltage_ = driver_->universal_get_value<decltype(vb_max_allowed_charge_voltage_)>(PDO_MAX_ALLOWED_CHARGE_VOLTAGE, SUB_INDEX_ZERO);

  // PDO: VB SOH
  vb_soh_ = driver_->universal_get_value<decltype(vb_soh_)>(PDO_SOH, SUB_INDEX_ZERO);

  // PDO: VB Number of Batteries Fault
  vb_number_of_batteries_fault_ = driver_->universal_get_value<decltype(vb_number_of_batteries_fault_)>(PDO_NUMBER_OF_BATTERIES_FAULT, SUB_INDEX_ZERO);

  // PDO: VB Number of Active Batteries
  vb_number_of_active_batteries_ = driver_->universal_get_value<decltype(vb_number_of_active_batteries_)>(PDO_NUMBER_OF_ACTIVE_BATTERIES, SUB_INDEX_ZERO);

  // PDO: VB Operational Mode
  vb_operational_mode_ = driver_->universal_get_value<decltype(vb_operational_mode_)>(PDO_OPERATIONAL_MODE, SUB_INDEX_ZERO);

  // PDO: VB Charge Fault
  vb_charge_fault_ = driver_->universal_get_value<decltype(vb_charge_fault_)>(PDO_CHARGE_FAULT, SUB_INDEX_ZERO);

  // PDO: VB Discharge Fault
  vb_discharge_fault_ = driver_->universal_get_value<decltype(vb_discharge_fault_)>(PDO_DISCHARGE_FAULT, SUB_INDEX_ZERO);

  // PDO: VB Regen. Current Limit
  vb_regen_current_limit_ = driver_->universal_get_value<decltype(vb_regen_current_limit_)>(PDO_REGEN_CURRENT_LIMIT, SUB_INDEX_ZERO);

  // PDO: VB Min. Cell Voltage
  vb_min_cell_voltage_ = driver_->universal_get_value<decltype(vb_min_cell_voltage_)>(PDO_MIN_CELL_VOLTAGE, SUB_INDEX_ZERO);

  // PDO: VB Max. Cell Voltage
  vb_max_cell_voltage_ = driver_->universal_get_value<decltype(vb_max_cell_voltage_)>(PDO_MAX_CELL_VOLTAGE, SUB_INDEX_ZERO);

  // PDO: VB Cell Balance Status All
  vb_cell_balance_status_all_ = driver_->universal_get_value<decltype(vb_cell_balance_status_all_)>(PDO_CELL_BALANCE_STATUS_ALL, SUB_INDEX_ZERO);

  // PDO: VB Pack Voltage All
  vb_pack_voltage_all_ = driver_->universal_get_value<decltype(vb_pack_voltage_all_)>(PDO_PACK_VOLTAGE_ALL, SUB_INDEX_ZERO);

  // PDO: VB SOC All
  vb_soc_all_ = driver_->universal_get_value<decltype(vb_soc_all_)>(PDO_SOC_ALL, SUB_INDEX_ZERO);

  // PDO: VB Temperature All
  vb_temperature_all_ = driver_->universal_get_value<decltype(vb_temperature_all_)>(PDO_TEMPERATURE_ALL, SUB_INDEX_ZERO);
  master_node_id_ = driver_->universal_get_value<decltype(master_node_id_)>(PDO_MASTER_NODE_ID, SUB_INDEX_ZERO);

  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO serial_number_: %d", serial_number_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO design_capacity_: %d", design_capacity_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO operational_mode_: %d", operational_mode_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO charge_fault_: %d", charge_fault_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO discharge_fault_: %d", discharge_fault_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO battery_voltage_: %d", battery_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO temperature_: %d", temperature_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO current_: %d", current_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO rem_cap_: %d", rem_cap_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO fcc_: %d", fcc_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO min_cell_temperature_: %d", min_cell_temperature_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO max_cell_temperature_: %d", max_cell_temperature_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO battery_state_of_charge_: %d", battery_state_of_charge_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO firmware_version_: %s", firmware_version_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO software_version_: %s", software_version_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO hardware_version_: %s", hardware_version_.c_str());
  for(int i = 0; i < 8; i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "SDO cell_voltage_%d: %d", i, cell_voltage_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_number_of_batteries_: %d", vb_number_of_batteries_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_current_stored_: %d", vb_current_stored_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_remaining_run_time_: %d", vb_remaining_run_time_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_remaining_charge_time_: %d", vb_remaining_charge_time_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_pack_voltage_: %d", vb_pack_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_current_: %d", vb_current_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_discharge_current_limit_: %d", vb_discharge_current_limit_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_charge_cut_off_current_: %d", vb_charge_cut_off_current_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_fully_charged_: %d", vb_fully_charged_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_temperature_: %d", vb_temperature_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_discharge_cut_off_voltage_: %d", vb_discharge_cut_off_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_charge_current_limit_: %d", vb_charge_current_limit_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_max_allowed_charge_voltage_: %d", vb_max_allowed_charge_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_soh_: %d", vb_soh_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_number_of_batteries_fault_: %d", vb_number_of_batteries_fault_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_number_of_active_batteries_: %d", vb_number_of_active_batteries_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_operational_mode_: %d", vb_operational_mode_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_charge_fault_: %d", vb_charge_fault_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_discharge_fault_: %d", vb_discharge_fault_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_regen_current_limit_: %d", vb_regen_current_limit_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_min_cell_voltage_: %d", vb_min_cell_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_max_cell_voltage_: %d", vb_max_cell_voltage_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_cell_balance_status_all_: %d", vb_cell_balance_status_all_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_pack_voltage_all_: %d", vb_pack_voltage_all_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_soc_all_: %d", vb_soc_all_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO vb_temperature_all_: %d", vb_temperature_all_);
  RCLCPP_INFO(rclcpp::get_logger("canopen_inventus_driver"), "PDO master_node_id_: %d", master_node_id_);


  return true;
}
