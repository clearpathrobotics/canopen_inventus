#ifndef BATTERY_HPP
#define BATTERY_HPP

#include "rclcpp/rclcpp.hpp"


#include "canopen_base_driver/lely_driver_bridge.hpp"

#define SUB_INDEX_ZERO 0x00
#define SDO_HARDWARE_VERSION 0x1009
#define SDO_SOFTWARE_VERSION 0x100A
#define SDO_OPERATIONAL_MODE 0x4801
#define SDO_CHARGE_FAULT 0x4802
#define SDO_DISCHARGE_FAULT 0x4803
#define SDO_CURRENT 0x4804
#define SDO_MIN_CELL_TEMPERATURE 0x4808
#define SDO_MAX_CELL_TEMPERATURE 0x4809

#define SDO_BQ8050_DATA 0x4900
#define SDO_BQ8050_SUB_SN 0x1C
#define SDO_BQ8050_SUB_DESIGN_CAPACITY 0x18
#define SDO_BQ8050_SUB_REM_CAP 0x0F
#define SDO_BQ8050_SUB_FCC 0x10
#define SDO_BQ8050_SUB_CELL 0x32

#define SDO_TEMPERATURE 0x6010
#define SDO_BATTERY_VOLTAGE 0x6060
#define SDO_BATTERY_STATE_OF_CHARGE 0x6081
#define SDO_FIRMWARE_VERSION 0xD000
#define SDO_FIRMWARE_VERSION_SUB 0x20

#define PDO_NUMBER_OF_BATTERIES 0x4850
#define PDO_CURRENT_STORED 0x4852
#define PDO_REMAINING_RUN_TIME 0x4853
#define PDO_REMAINING_CHARGE_TIME 0x4854
#define PDO_PACK_VOLTAGE 0x4855
#define PDO_CURRENT 0x4856
#define PDO_DISCHARGE_CURRENT_LIMIT 0x4857
#define PDO_CHARGE_CUT_OFF_CURRENT 0x4858
#define PDO_FULLY_CHARGED 0x4859
#define PDO_TEMPERATURE 0x485A
#define PDO_DISCHARGE_CUT_OFF_VOLTAGE 0x485B
#define PDO_CHARGE_CURRENT_LIMIT 0x485C
#define PDO_MAX_ALLOWED_CHARGE_VOLTAGE 0x485D
#define PDO_SOH 0x485E
#define PDO_NUMBER_OF_BATTERIES_FAULT 0x485F
#define PDO_NUMBER_OF_ACTIVE_BATTERIES 0x4860
#define PDO_OPERATIONAL_MODE 0x4861
#define PDO_CHARGE_FAULT 0x4862
#define PDO_DISCHARGE_FAULT 0x4863
#define PDO_REGEN_CURRENT_LIMIT 0x4864
#define PDO_MIN_CELL_VOLTAGE 0x4865
#define PDO_MAX_CELL_VOLTAGE 0x4866
#define PDO_CELL_BALANCE_STATUS_ALL 0x4867
#define PDO_PACK_VOLTAGE_ALL 0x4868
#define PDO_SOC_ALL 0x4869
#define PDO_TEMPERATURE_ALL 0x486A
#define PDO_MASTER_NODE_ID 0x486C


namespace ros2_canopen
{

class Battery
{
public:
  Battery(std::shared_ptr<LelyDriverBridge> driver)
  {
    this->driver_ = driver;
  }

  bool readState();
  int get_id() {return driver_->get_id();};

  uint16_t serial_number_;
  uint16_t design_capacity_;
  uint16_t operational_mode_;
  uint16_t charge_fault_;
  uint16_t discharge_fault_;
  uint32_t battery_voltage_;
  int16_t temperature_;
  int16_t current_;
  uint16_t rem_cap_;
  uint16_t fcc_;
  uint16_t min_cell_temperature_;
  uint16_t max_cell_temperature_;
  uint8_t battery_state_of_charge_;
  std::string hardware_version_;
  std::string firmware_version_;
  std::string software_version_;
  uint16_t cell_voltage_[8];

  uint8_t vb_number_of_batteries_;
  uint16_t vb_current_stored_;
  uint16_t vb_remaining_run_time_;
  uint16_t vb_remaining_charge_time_;
  uint16_t vb_pack_voltage_;
  int16_t vb_current_;
  uint16_t vb_discharge_current_limit_;
  uint8_t vb_charge_cut_off_current_;
  uint8_t vb_fully_charged_;
  int16_t vb_temperature_;
  uint16_t vb_discharge_cut_off_voltage_;
  uint16_t vb_charge_current_limit_;
  uint16_t vb_max_allowed_charge_voltage_;
  uint8_t vb_soh_;
  uint8_t vb_number_of_batteries_fault_;
  uint8_t vb_number_of_active_batteries_;
  uint8_t vb_operational_mode_;
  uint16_t vb_charge_fault_;
  uint16_t vb_discharge_fault_;
  uint16_t vb_regen_current_limit_;
  uint16_t vb_min_cell_voltage_;
  uint16_t vb_max_cell_voltage_;
  uint16_t vb_cell_balance_status_all_;
  uint16_t vb_pack_voltage_all_;
  uint8_t vb_soc_all_;
  int16_t vb_temperature_all_;
  uint8_t master_node_id_;
private:
  std::shared_ptr<LelyDriverBridge> driver_;


};

} // namespace ros2_canopen

#endif // BATTERY_HPP
