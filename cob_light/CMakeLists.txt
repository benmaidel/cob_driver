cmake_minimum_required(VERSION 3.5)
project(cob_light)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(HEADERS
    ${PROJECT_SOURCE_DIR}/common/include/breathColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/breathMode.h
    ${PROJECT_SOURCE_DIR}/common/include/circleColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/colorUtils.h
    ${PROJECT_SOURCE_DIR}/common/include/concurrentQueue.h
    ${PROJECT_SOURCE_DIR}/common/include/fadeColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/flashMode.h
    ${PROJECT_SOURCE_DIR}/common/include/mode.h
    ${PROJECT_SOURCE_DIR}/common/include/modeExecutor.h
    ${PROJECT_SOURCE_DIR}/common/include/modeFactory.h
    ${PROJECT_SOURCE_DIR}/common/include/modeRunner.h
    ${PROJECT_SOURCE_DIR}/common/include/sequenceMode.h
    ${PROJECT_SOURCE_DIR}/common/include/serialIO.h
    ${PROJECT_SOURCE_DIR}/common/include/staticMode.h
    ${PROJECT_SOURCE_DIR}/common/include/sweepColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/distApproxMode.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorOSim.h
    ${PROJECT_SOURCE_DIR}/ros/include/iColorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/ms35.h
    ${PROJECT_SOURCE_DIR}/ros/include/stageprofi.h
)

### Message Generation ###

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/LightModes.msg
    msg/ColorRGBAArray.msg
    msg/LightMode.msg
    msg/Sequence.msg
    srv/SetLightMode.srv
    srv/StopLightMode.srv
    action/SetLightMode.action
    DEPENDENCIES std_msgs 
)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include)

add_executable(light_node common/src/modeExecutor.cpp common/src/modeFactory.cpp common/src/serialIO.cpp ros/src/light_node.cpp ros/src/colorO.cpp ros/src/colorOSim.cpp ros/src/ms35.cpp ros/src/stageprofi.cpp)
ament_target_dependencies(light_node "std_msgs" "rclcpp" "rclcpp_action" "sensor_msgs" "visualization_msgs" "diagnostic_msgs")
rosidl_target_interfaces(light_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

### INSTALL ###
install(TARGETS
  light_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
