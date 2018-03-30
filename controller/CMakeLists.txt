cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)
find_package(Boost REQUIRED)

add_message_files(
	DIRECTORY msg
	FILES AckermannDriveStamped.msg	AckermannDrive.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
	message_runtime
  DEPENDS 
    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(write_serial src/write_serial.cpp)
add_executable(read_serial src/read_serial.cpp)
add_executable(teleop_kuuve_key src/teleop_kuuve_key.cpp)
add_executable(main src/main.cpp)

target_link_libraries(write_serial ${catkin_LIBRARIES})
target_link_libraries(read_serial ${catkin_LIBRARIES})
target_link_libraries(teleop_kuuve_key ${catkin_LIBRARIES})
target_link_libraries(main ${catkin_LIBRARIES})
