cmake_minimum_required(VERSION 2.8.3)
project(razor_imu_9dof)

find_package(catkin REQUIRED roscpp message_generation sensor_msgs COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/imu.cfg)

catkin_package(DEPENDS CATKIN DEPENDS dynamic_reconfigure roscpp)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY magnetometer_calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(test_node src/imutest.cpp)
##add_executable(main_node nodes/test.cpp)

target_link_libraries(test_node
	${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS 
	nodes/imu_node.py
	nodes/display_3D_visualization.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
