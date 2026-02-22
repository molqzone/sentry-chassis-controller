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
  realtime_tools
  nav_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs controller_interface controller_manager hardware_interface pluginlib control_toolbox dynamic_reconfigure realtime_tools nav_msgs tf tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/kinematics.cpp
)

add_library(${PROJECT_NAME}
  src/sentry_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
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
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
      ${PROJECT_NAME}
      ${PROJECT_NAME}_core
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/sentry_chassis_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
