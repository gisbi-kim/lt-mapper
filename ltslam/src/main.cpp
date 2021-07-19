#include "ltslam/LTslam.h"

using namespace gtsam;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ROS_INFO("\033[1;32m----> LTslam starts.\033[0m");

    LTslam ltslam3d;
    ltslam3d.run();

    ROS_INFO("\033[1;32m----> LTslam done.\033[0m");
    ros::spin();
    return 0;
}
