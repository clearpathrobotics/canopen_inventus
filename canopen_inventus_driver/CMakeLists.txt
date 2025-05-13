cmake_minimum_required(VERSION 3.8)
project(canopen_inventus_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_inventus_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
rclcpp
rclcpp_components
rclcpp_lifecycle
lifecycle_msgs
canopen_core
canopen_interfaces
canopen_inventus_interfaces
canopen_base_driver
canopen_proxy_driver
lely_core_libraries
std_msgs
std_srvs
sensor_msgs
)

# Battery Library
add_library(lely_inventus_controller_bridge
  src/battery.cpp
)
target_compile_features(lely_inventus_controller_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lely_inventus_controller_bridge PUBLIC -Wl,--no-undefined)
target_include_directories(lely_inventus_controller_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lely_inventus_controller_bridge
  ${dependencies}
)

# Functionality library
add_library(node_canopen_inventus_driver
src/node_interfaces/node_canopen_inventus_driver.cpp
)
target_compile_features(node_canopen_inventus_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_inventus_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_inventus_driver PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_inventus_driver
  lely_inventus_controller_bridge
)
ament_target_dependencies(
node_canopen_inventus_driver
${dependencies}
)

# Lifecycle driver
add_library(lifecycle_inventus_driver
src/lifecycle_inventus_driver.cpp
)
target_compile_features(lifecycle_inventus_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_inventus_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_inventus_driver PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_inventus_driver
  node_canopen_inventus_driver
  lely_inventus_controller_bridge
)
ament_target_dependencies(
lifecycle_inventus_driver
${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_inventus_driver PRIVATE "CANOPEN_Inventus_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_inventus_driver "ros2_canopen::LifecycleInventusDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleInventusDriver;$<TARGET_FILE:lifecycle_inventus_driver >\n")


# Non lifecycle driver
add_library(inventus_driver
src/inventus_driver.cpp
)
target_compile_features(inventus_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(inventus_driver PUBLIC -Wl,--no-undefined)
target_include_directories(inventus_driver PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(inventus_driver
  node_canopen_inventus_driver
  lely_inventus_controller_bridge
)

ament_target_dependencies(
inventus_driver
${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(inventus_driver PRIVATE "CANOPEN_Inventus_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(inventus_driver "ros2_canopen::InventusDriver")
set(node_plugins "${node_plugins}ros2_canopen::InventusDriver;$<TARGET_FILE:inventus_driver >\n")

install(
DIRECTORY include/
DESTINATION include
)

install(
TARGETS lifecycle_inventus_driver inventus_driver node_canopen_inventus_driver lely_inventus_controller_bridge
EXPORT export_${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
include
)
ament_export_libraries(
  lifecycle_inventus_driver
  inventus_driver
  node_canopen_inventus_driver
  lely_inventus_controller_bridge
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
