cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

find_package(catkin REQUIRED COMPONENTS roscpp serial message_generation std_msgs roboteq_msgs)

catkin_package()

# Build and install binary driver.
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(mbs)
add_subdirectory(src)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

