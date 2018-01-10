add_executable(config_test config_test.cpp)
target_link_libraries(config_test libFlight)

add_executable(ros_test ros_test.cpp)
target_link_libraries(ros_test libFlight ${catkin_LIBRARIES})

### 
add_executable(setting_destination
set_destination.cpp)

target_link_libraries(setting_destination
${catkin_LIBRARIES})
###

add_executable(trajplan_test
trajplan_test.cpp
)
target_link_libraries(trajplan_test
libFlight ${catkin_LIBRARIES}
)