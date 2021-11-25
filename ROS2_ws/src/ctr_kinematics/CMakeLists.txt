cmake_minimum_required(VERSION 3.5)
project(ctr_kinematics)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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

set(blaze_DLL
  "C:/src/vcpkg/installed/x64-windows/bin/liblapack.dll"
  "C:/src/vcpkg/installed/x64-windows/bin/libgfortran-5.dll"
  "C:/src/vcpkg/installed/x64-windows/bin/libgcc_s_seh-1.dll"
  "C:/src/vcpkg/installed/x64-windows/bin/libwinpthread-1.dll"
  "C:/src/vcpkg/installed/x64-windows/bin/libquadmath-0.dll"
)
file(COPY ${blaze_DLL} DESTINATION "C:/Users/lwang848/Desktop/CTR/ROS2_ws/install/bin")

add_library(blaze INTERFACE)
set_target_properties(blaze PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/include"
  INTERFACE_LINK_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/lib"
)

set(ctr_SRCS
  src/Tube.cpp
  src/BC.cpp
  src/BVP.cpp
  src/CTR.cpp
  src/Kinematics.cpp
  src/Controller.cpp
  # src/MyTest.cpp
)
add_library(CTRLib ${ctr_SRCS})
target_include_directories(CTRLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include/ctr_kinematics)
target_link_libraries(CTRLib INTERFACE blaze)

add_executable(ctr_node src/ctr_node.cpp)
target_link_libraries(ctr_node CTRLib)
ament_target_dependencies(ctr_node rclcpp)

install(TARGETS 
  ctr_node
  DESTINATION lib/${PROJECT_NAME})

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