#ifndef BATTERY_HPP
#define BATTERY_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "canopen_inventus_interfaces/msg/status.hpp"
#include "canopen_inventus_interfaces/msg/virtual_battery.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"


namespace ros2_canopen
{

template <typename T>
struct COTypedData
{
public:
  uint16_t index_;
  uint8_t subindex_;
  T data_;
};

class Battery
{
public:
  Battery(std::shared_ptr<LelyDriverBridge> driver, std::mutex &node_mutex):
  sdo_mutex(node_mutex), read_static_sdo_(true)
  {
    this->driver_ = driver;
  }

  // SDO: Visible Strings
  ros2_canopen::COTypedData<std::string> typed_sdo_hardware_version_ = {0x1009, 0x00, ""};
  ros2_canopen::COTypedData<std::string> typed_sdo_software_version_ = {0x100A, 0x00, ""};
  ros2_canopen::COTypedData<std::string> typed_sdo_firmware_version_ = {0xD000, 0x20, ""};

  // SDO: Unsigned 8 Bit
  ros2_canopen::COTypedData<uint8_t> typed_sdo_state_of_charge_ = {0x6081, 0x00, 0U};

  // SDO: Unsigned 16 Bit
  ros2_canopen::COTypedData<uint16_t> typed_sdo_operational_mode_ = {0x4801, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_charge_fault_ = {0x4802, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_discharge_fault_ = {0x4803, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_min_cell_temperature_ = {0x4808, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_max_cell_temperature_ = {0x4809, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_rem_capacity_ = {0x4900, 0x0F, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_fcc_ = {0x4900, 0x10, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_design_capacity_ = {0x4900, 0x18, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_serial_number_ = {0x4900, 0x1C, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_sdo_cell_voltages_[8] = {
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x32, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x33, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x34, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x35, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x36, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x37, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x38, 0U}),
    ros2_canopen::COTypedData<uint16_t>({0x4900, 0x39, 0U})
  };

  // SDO: Unsigned 32 Bit
  ros2_canopen::COTypedData<uint32_t> typed_sdo_battery_voltage_ = {0x6060, 0x00, 0U};

  // SDO: Signed 16 Bit
  ros2_canopen::COTypedData<int16_t> typed_sdo_current_ = {0x4804, 0x00, 0U};
  ros2_canopen::COTypedData<int16_t> typed_sdo_temperature_ = {0x6010, 0x00, 0U};

  // PDO: Unsigned 8 Bit
  ros2_canopen::COTypedData<uint8_t> typed_pdo_number_of_batteries_ = {0x4850, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_charge_cut_off_current_ = {0x4858, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_fully_charged_ = {0x4859, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_soh_ = {0x485E, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_number_of_batteries_fault_ = {0x485F, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_number_of_active_batteries_ = {0x4860, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_operational_mode_ = {0x4861, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_soc_all_ = {0x4869, 0x00, 0U};
  ros2_canopen::COTypedData<uint8_t> typed_pdo_master_node_id_ = {0x486C, 0x00, 0U};

  // PDO: Unsigned 16 Bit
  ros2_canopen::COTypedData<uint16_t> typed_pdo_current_stored_ = {0x4852, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_remaining_run_time_ = {0x4853, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_remaining_charge_time_ = {0x4854, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_pack_voltage_ = {0x4855, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_discharge_current_limit_ = {0x4857, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_discharge_cut_off_voltage_ = {0x485B, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_charge_current_limit_ = {0x485C, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_max_allowed_charge_voltage_ = {0x485D, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_charge_fault_ = {0x4862, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_discharge_fault_ = {0x4863, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_regen_current_limit_ = {0x4864, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_min_cell_voltage_ = {0x4865, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_max_cell_voltage_ = {0x4866, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_cell_balance_status_all_ = {0x4867, 0x00, 0U};
  ros2_canopen::COTypedData<uint16_t> typed_pdo_pack_voltage_all_ = {0x4868, 0x00, 0U};

  // PDO: Signed 16 Bit
  ros2_canopen::COTypedData<int16_t> typed_pdo_current_ = {0x4856, 0x00, 0U};
  ros2_canopen::COTypedData<int16_t> typed_pdo_temperature_ = {0x485A, 0x00, 0U};
  ros2_canopen::COTypedData<int16_t> typed_pdo_temperature_all_ = {0x486A, 0x00, 0U};

  // Methods
  sensor_msgs::msg::BatteryState getBatteryState();
  canopen_inventus_interfaces::msg::Status getBatteryStatus();
  sensor_msgs::msg::BatteryState getVirtualBatteryState();
  canopen_inventus_interfaces::msg::VirtualBattery getVirtualBatteryStatus();

  bool readState();
  bool faultHighTemp(uint16_t data);
  bool faultLowTemp(uint16_t data);
  bool faultOverCurrent(uint16_t data);
  bool faultOverVoltage(uint16_t data);
  bool faultAny(uint16_t data);

  uint8_t modeBatteryStatus(uint16_t data);

  template <typename T>
  bool readTypedSDO(ros2_canopen::COTypedData<T> & data);

  template <typename T>
  bool readTypedPDO(ros2_canopen::COTypedData<T> & data);

  void readAllSDO();
  void readAllPDO();

private:
  std::shared_ptr<LelyDriverBridge> driver_;
  std::mutex &sdo_mutex;
  bool read_static_sdo_;
};

} // namespace ros2_canopen

#endif // BATTERY_HPP
