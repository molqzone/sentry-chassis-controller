cmake_minimum_required(VERSION 3.10)
project(sentry_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  control_toolbox
  dynamic_reconfigure
  ddynamic_reconfigure
  realtime_tools
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
if(TARGET Sophus::Sophus)
  set(Sophus_LIBRARIES Sophus::Sophus)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs controller_interface controller_manager hardware_interface pluginlib control_toolbox dynamic_reconfigure ddynamic_reconfigure realtime_tools nav_msgs tf tf2 tf2_ros tf2_geometry_msgs
  DEPENDS Eigen3 Sophus
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/kinematics.cpp
)

add_library(${PROJECT_NAME}
  src/sentry_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_sentry_chassis_controller.cpp
    test/KinematicsTest.cpp
    test/SentryChassisControllerPidTest.cpp
    test/SentryChassisControllerOdomTest.cpp
    test/SentryChassisControllerRuntimeParamTest.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
      ${PROJECT_NAME}
      ${PROJECT_NAME}_core
      ${Sophus_LIBRARIES}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES
  sentry_chassis_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  doc
  config
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/sentry_chassis_controller.launch
  launch/sentry_sim.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/teleop_keyboard.sh
  scripts/serve_rm_description_meshes.py
  scripts/evaluate_teleop_behavior.py
  scripts/evaluate_teleop_keys.py
  scripts/auto_calibrate_signs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
