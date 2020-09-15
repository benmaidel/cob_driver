cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS libphidgets message_generation roscpp sensor_msgs std_msgs)

### Message Generation ###
add_message_files(
  FILES
  AnalogSensor.msg
  DigitalSensor.msg
  Encoder.msg
  Motor.msg
  Velocity.msg
)

add_service_files(
  FILES
  SetAcceleration.srv
  SetBackEMFSensingState.srv
  SetBraking.srv
  SetDataRate.srv
  SetDigitalSensor.srv
  SetEncoderPosition.srv
  SetTriggerValue.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
  INCLUDE_DIRS common/include ros/include
  LIBRARIES phidget_sensors_lib
)

### BUILD ###
include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS})

add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)
add_dependencies(range_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_sensors ${catkin_LIBRARIES})

add_executable(device_name ros/src/set_device_name.cpp)
add_dependencies(device_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(device_name ${catkin_LIBRARIES})

add_executable(phidget_sensors
  common/src/phidget.cpp
  common/src/phidget_motor.cpp
  common/src/phidgetik.cpp
  ros/src/phidget_manager.cpp
  ros/src/phidget_motor_ros.cpp
  ros/src/phidget_sensors_node.cpp
  ros/src/phidgetik_ros.cpp)
add_dependencies(phidget_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phidget_sensors ${catkin_LIBRARIES})

add_library(phidget_sensors_lib
  common/src/phidget.cpp
  common/src/phidget_motor.cpp
  common/src/phidgetik.cpp)
add_dependencies(phidget_sensors_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(phidget_sensors_lib ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS device_name phidget_sensors phidget_sensors_lib range_sensors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS ros/src/simulated_range_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
