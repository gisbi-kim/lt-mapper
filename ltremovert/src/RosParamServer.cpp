#include "removert/RosParamServer.h"


RosParamServer::RosParamServer()
: nh(nh_super)
{
    nh.param<bool>("removert/isScanFileKITTIFormat", isScanFileKITTIFormat_, true);

    nh.param<float>("removert/rimg_color_min", rimg_color_min_, 0.0);
    nh.param<float>("removert/rimg_color_max", rimg_color_max_, 10.0);
    kRangeColorAxis = std::pair<float, float> {rimg_color_min_, rimg_color_max_}; // meter
    kRangeColorAxisForDiff = std::pair<float, float>{0.0, 0.5}; // meter 

    // fov 
    nh.param<float>("removert/sequence_vfov", kVFOV, 50.0);
    nh.param<float>("removert/sequence_hfov", kHFOV, 360.0);
    kFOV = std::pair<float, float>(kVFOV, kHFOV);

    // resolution 
    nh.param<std::vector<float>>("removert/remove_resolution_list", remove_resolution_list_, std::vector<float>());
    nh.param<std::vector<float>>("removert/revert_resolution_list", revert_resolution_list_, std::vector<float>());

    // knn 
    nh.param<int>("removert/num_nn_points_within", kNumKnnPointsToCompare, 3);                 // using higher, more strict static
    nh.param<float>("removert/dist_nn_points_within", kScanKnnAndMapKnnAvgDiffThreshold, 0.1); // using smaller, more strict static

    // sequcne system info 
    nh.param<std::vector<double>>("removert/ExtrinsicLiDARtoPoseBase", kVecExtrinsicLiDARtoPoseBase, std::vector<double>());
    kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);
    kSE3MatExtrinsicPoseBasetoLiDAR = kSE3MatExtrinsicLiDARtoPoseBase.inverse();

    // config: point cloud pre-processing
    nh.param<float>("removert/downsample_voxel_size", kDownsampleVoxelSize, 0.05);

    // sessions' paths 
    nh.param<std::string>("removert/central_sess_scan_dir", central_sess_scan_dir_, "/use/your/directory/having/*.bin");
    nh.param<std::string>("removert/central_sess_pose_path", central_sess_pose_path_, "/use/your/path/having/pose.txt");
    nh.param<std::string>("removert/query_sess_scan_dir", query_sess_scan_dir_, "/use/your/directory/having/*.bin");
    nh.param<std::string>("removert/query_sess_pose_path", query_sess_pose_path_, "/use/your/path/having/pose.txt");


    // target scan index range (used in Removert.cpp)
    nh.param<int>("removert/start_idx", start_idx_, 1);
    nh.param<int>("removert/end_idx", end_idx_, 100);

    nh.param<bool>("removert/use_keyframe_gap", use_keyframe_gap_, true);
    nh.param<bool>("removert/use_keyframe_meter", use_keyframe_meter_, false);
    nh.param<int>("removert/keyframe_gap", keyframe_gap_, 10);
    nh.param<float>("removert/keyframe_meter", keyframe_gap_meter_, 2.0);

    nh.param<int>("removert/repeat_removert_iter", repeat_removert_iter_, 1);

    // faster
    nh.param<int>("removert/num_omp_cores", kNumOmpCores, 4);

    // save info
    nh.param<bool>("removert/saveMapPCD", kFlagSaveMapPointcloud, false);
    nh.param<bool>("removert/saveCleanScansPCD", kFlagSaveCleanScans, false);
    nh.param<std::string>("removert/save_pcd_directory", save_pcd_directory_, "/");


    usleep(100);
} // ctor RosParamServer

