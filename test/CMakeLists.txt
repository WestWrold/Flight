add_executable(config_test config_test.cpp)
target_link_libraries(config_test libFlight)

add_executable(ros_test ros_test.cpp)
target_link_libraries(ros_test libFlight ${catkin_LIBRARIES})