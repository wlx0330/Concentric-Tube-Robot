cmake_minimum_required(VERSION 3.5)

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
endif()

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

# find_package(blaze REQUIRED PATHS "C:/src/vcpkg/packages/blaze_x64-windows")
# if (blaze_FOUND)
#   message(STATUS "Blaze has been found!")
# endif()
# find_package(blaze REQUIRED)
find_package(LAPACK REQUIRED)
# find_package(BLAS REQUIRED)



# add_library(lapack)
# target_link_directories()
# target_link_directories(lapack PUBLIC $<C:/src/vcpkg/installed/x64-windows/lib>)
# target_include_directories(lapack C:/src/vcpkg/installed/x64-windows/include)

# set_target_properties(lapack PROPERTIES
#   INTERFACE_INCLUDE_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/include"
#   INTERFACE_LINK_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/lib"
#   INTERFACE_LINK_LIBRARIES "C:/src/vcpkg/installed/x64-windows/lib/lapack.lib"
# )

set(blaze_LIBS
  "C:/src/vcpkg/packages/lapack-reference_x64-windows/lib/lapack.lib"
  # "C:/src/vcpkg/installed/x64-windows/lib/lapack.lib"
  # "C:/src/vcpkg/packages/openblas_x64-windows/lib/openblas.lib"
  # "C:/Users/lwang848/Desktop/lapack_build/liblapack.lib"
  # "C:/Users/lwang848/Desktop/lapack_build/liblapack.dll"
)

set(blaze_LINK
  "C:/src/vcpkg/installed/x64-windows/lib"
  "C:/src/vcpkg/installed/x64-windows/bin"
)

add_library(blaze INTERFACE)
set_target_properties(blaze PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/include"
  # INTERFACE_LINK_DIRECTORIES ${blaze_LINK}
  INTERFACE_LINK_DIRECTORIES "C:/src/vcpkg/installed/x64-windows/lib"
  # INTERFACE_LINK_LIBRARIES ${blaze_LIBS}
) 
# target_include_directories(blaze INTERFACE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
#   $<INSTALL_INTERFACE:include>) 
# target_link_libraries(blaze INTERFACE ${LAPACK_LIBRARIES})
# target_compile_options(blaze INTERFACE ${LAPACK_LINKER_FLAGS})   


set(ctr_SRCS
  # src/Tube.cpp
  # src/BC.cpp
  # src/BVP.cpp
  # src/CTR.cpp
  # src/Kinematics.cpp
  # src/Controller.cpp
  src/MyTest.cpp
)

add_library(CTRLib ${ctr_SRCS})
target_include_directories(CTRLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include/ctr_kinematics)
target_link_libraries(CTRLib INTERFACE blaze) 

add_executable(ctr_node src/ctr_node.cpp)
# target_include_directories(ctr_node PUBLIC ${LAPACK_INCLUDE_DIRS})
# include_directories(${LAPACK_INCLUDE_DIRS})
target_link_libraries(ctr_node CTRLib)
# target_link_directories(ctr_node "C:/src/vcpkg/installed/x64-windows/bin")

# target_link_libraries(ctr_node INTERFACE ${LAPACK_LIBRARIES})
# target_compile_options(ctr_node INTERFACE ${LAPACK_LINKER_FLAGS})  
# target_link_libraries(ctr_node LAPACK::LAPACK)
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