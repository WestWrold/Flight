#include <flight/Config.h>
using namespace myslam;
int main()
{
     Config::setParameterFile("default.yaml");
     float fx = Config::get<float>("camera.fx");
    std::cout<< fx << std::endl;
}