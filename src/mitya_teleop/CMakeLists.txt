cmake_minimum_required(VERSION 3.5)
project(mitya_teleop)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mitya_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

#add_executable(talker src/publisher_member_function.cpp)
#ament_target_dependencies(talker rclcpp std_msgs mitya_interfaces)

#add_executable(listener src/subscriber_member_function.cpp)
#ament_target_dependencies(listener rclcpp std_msgs mitya_interfaces)

add_executable(arduino_node src/arduino_node.cpp src/robo_com.cpp src/consts.hpp)
ament_target_dependencies(arduino_node rclcpp std_msgs mitya_interfaces)

add_executable(joystick_node src/joystick_node.cpp src/robo_com.cpp src/consts.hpp src/button_event.cpp)
ament_target_dependencies(joystick_node rclcpp std_msgs sensor_msgs mitya_interfaces yaml-cpp)
target_link_libraries(joystick_node yaml-cpp)

add_executable(herkulex_node src/herkulex_node.cpp src/herkulex.cpp src/robo_com.cpp src/consts.hpp)
ament_target_dependencies(herkulex_node rclcpp std_msgs sensor_msgs mitya_interfaces yaml-cpp)
target_link_libraries(herkulex_node yaml-cpp)

install(TARGETS
        arduino_node
        joystick_node
        herkulex_node
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
