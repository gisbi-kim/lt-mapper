#pragma once

#include "ltslam/utility.h"

class RosNodeHandle
{
public:
    ros::NodeHandle nh_super;
}; // class: RosNodeHandle


class RosParamServer: public RosNodeHandle
{
public:
    ros::NodeHandle & nh;

    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;

    std::string save_directory_;
    
    bool is_display_debug_msgs_;

    int kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound;
    int numberOfCores;

    float loopFitnessScoreThreshold;

public:
    RosParamServer();

}; // class: RosParamServer
