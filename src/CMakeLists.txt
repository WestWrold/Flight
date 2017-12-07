add_library(libFlight SHARED
    Config.cpp
    Frame.cpp 
    ImgPro.cpp 
    StereoMap.cpp)
target_link_libraries( libFlight ${THIRD_LIBS})