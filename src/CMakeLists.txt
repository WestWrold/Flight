add_library(Flight SHARED
    Config.cpp
    Frame.cpp 
    ImgPro.cpp 
    StereoMap.cpp
    trajplanner/uavtrans.cpp
    trajplanner/csvparser.c
    trajplanner/searchdistance.cpp
    trajplanner/Trajectory.cpp
    trajplanner/TrajectoryLibrary.cpp
    )    
target_link_libraries( Flight ${THIRD_LIBS} ${catkin_LIBRARIES})
#add_dependencies(libFlight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
