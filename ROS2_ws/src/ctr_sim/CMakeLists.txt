cmake_minimum_required(VERSION 3.5)
project(ctr_sim)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(launch_ros)

# find_package(ament_index_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorPos.msg"
  "msg/RobotPos.msg"
  "msg/KeyInput.msg"
  "srv/MotorSpeed.srv"
  "srv/MotorConnect.srv"
  "srv/MotorJog.srv"
  "action/MotorCalib.action"
)
ament_export_dependencies(rosidl_default_runtime)

set(ctr_SRCS 
  #src/galil_node.cpp
  src/galil_controller.cpp
  src/motors.cpp
  #src/kinematics_node.cpp
)
set(ctr_HDRS 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include/ctr_sim
  #src
)
set(gclib_ICLD "C:/Program\ Files\ (x86)/Galil/gclib/include")
set(gclib_LIB 
  "C:/Program\ Files\ (x86)/Galil/gclib/lib/dynamic/x64/gclib.lib"
  "C:/Program\ Files\ (x86)/Galil/gclib/lib/dynamic/x64/gclibo.lib"
)
set(gclib_DLL
  "C:/Program\ Files\ (x86)/Galil/gclib/dll/x64/gclib.dll"
  "C:/Program\ Files\ (x86)/Galil/gclib/dll/x64/gclibo.dll"
  "C:/Program\ Files\ (x86)/Galil/gclib/dll/x64/libcrypto-3.dll"
  "C:/Program\ Files\ (x86)/Galil/gclib/dll/x64/libssl-3.dll"
)
file(COPY ${gclib_DLL} DESTINATION "C:/Users/lwang848/Desktop/CTR_ws/install/bin")
  
add_executable(galil_node src/galil_node.cpp ${ctr_SRCS})
target_include_directories(galil_node PUBLIC ${ctr_HDRS})
rosidl_target_interfaces(galil_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(kinematics_node src/kinematics_node.cpp)
target_include_directories(kinematics_node PUBLIC ${ctr_HDRS})
rosidl_target_interfaces(kinematics_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(teleop_node src/teleop_node.cpp)
target_include_directories(teleop_node PUBLIC ${ctr_HDRS})
rosidl_target_interfaces(teleop_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(motor_setup_node src/motor_setup_node.cpp)
target_include_directories(motor_setup_node PUBLIC ${ctr_HDRS})
rosidl_target_interfaces(motor_setup_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(key_input_node src/key_input_node.cpp)
target_include_directories(key_input_node PUBLIC ${ctr_HDRS})
rosidl_target_interfaces(key_input_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(gclib INTERFACE)
set_target_properties(gclib PROPERTIES 
  INTERFACE_INCLUDE_DIRECTORIES "${gclib_ICLD}"
  INTERFACE_LINK_DIRECTORIES "C:/Program\ Files\ (x86)/Galil/gclib/lib/dynamic/x64"
  INTERFACE_LINK_LIBRARIES "${gclib_LIB}"
)
target_link_libraries(galil_node gclib)
# target_link_libraries(motor_setup_node gclib)
ament_target_dependencies(galil_node rclcpp std_msgs rclcpp_action)
ament_target_dependencies(kinematics_node rclcpp std_msgs)
ament_target_dependencies(teleop_node rclcpp std_msgs)
ament_target_dependencies(motor_setup_node rclcpp std_msgs rclcpp_action)
ament_target_dependencies(key_input_node rclcpp)

install(TARGETS 
  galil_node
  kinematics_node
  teleop_node
  motor_setup_node
  key_input_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()