cmake_minimum_required(VERSION 3.0.2)
project(dcd_timeref)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# Create src directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Full PPS version (requires pps-tools)
add_executable(dcd_timeref src/dcd_timeref.cpp)
target_link_libraries(dcd_timeref 
  ${catkin_LIBRARIES}
  rt
)

# Simplified version for testing (no PPS dependencies)
add_executable(dcd_timeref_simple src/dcd_timeref_simple.cpp)
target_link_libraries(dcd_timeref_simple 
  ${catkin_LIBRARIES}
)

# Install executables
install(TARGETS dcd_timeref dcd_timeref_simple
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
