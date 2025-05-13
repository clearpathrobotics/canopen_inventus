# canopen_inventus

Implementation of a BMS driver for the Inventus battery using the `ros2_canopen` library.

## Configuration
If an EDS file is available, follow the `Configuration Package` instructions in the [`ros2_canopen` tutorials](https://ros-industrial.github.io/ros2_canopen/manual/rolling/) to setup a configuration package for your system. In the bus configuration, use this package's driver.

For example, the following `bus.yml` configuration can be used to launch a two battery system:
```yaml
options:
  dcf_path: '@BUS_CONFIG_PATH@'

master:
  node_id: 1
  driver: 'ros2_canopen::MasterDriver'
  package: 'canopen_master_driver'
  baudrate: 250

battery_0:
  node_id: 49
  is_master: true # Battery reporting Virtual Battery PDOs
  dcf: &dcf inventus.eds
  driver: &driver ros2_canopen::InventusDriver
  package: &package canopen_inventus_driver
  sdo_timeout_ms: &sdo_timeout_ms 100 # Timeout on SDO
  period: &period 100 # Read PDO and SDO period
  publish_ms: &publish_ms 100 # Publish ROS messages period

battery_1:
  node_id: 50
  is_master: false
  dcf: *dcf
  driver: *driver
  package: *package
  sdo_timeout_ms: *sdo_timeout_ms
  period: *period
  publish_ms: *publish_ms
```

Parameters:
- **dcf:** The EDS file must be under the same configuration directory as the `bus.yml`.
- **period:** The period at which PDOs and SDOs are read. Note, only one SDO is read per period.
- **publish_ms:** The rate at which topics are published.
- **is_master:** The master battery aggregates data from the other batteries in the system and publishes a Virtual Battery message that averages data from the entire system.

## Topics
Each battery publishes a **sensor_msgs/BatteryState** (e.g. `battery_0/state`) and **canopen_inventus_interfaces/Status** (e.g. `battery_0/status`) topic. The master battery will publish a **sensor_msgs/BatteryState** and **canopen_inventus_interfaces/VirtualBattery** topic to non-node namespaced topic: `state` and `status`.
