#include "removert/Removerter.h"

namespace ltremovert
{

void fsmkdir(std::string _path)
{
    if (!fs::is_directory(_path) || !fs::exists(_path))
        fs::create_directories(_path); // create src folder
}                                      //fsmkdir

void Removerter::cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    cout << "TODO" << endl;
}

Removerter::Removerter()
    : ROSimg_transporter_(nh)
{
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os1_points", 5, &Removerter::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    // voxelgrid generates warnings frequently, so verbose off + ps. recommend to use octree (see makeGlobalMap)
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    if (save_pcd_directory_.substr(save_pcd_directory_.size() - 1, 1) != std::string("/"))
        save_pcd_directory_ = save_pcd_directory_ + "/";
    fsmkdir(save_pcd_directory_);

    updated_scans_save_dir_ = save_pcd_directory_ + "scans_updated";
    fsmkdir(updated_scans_save_dir_);
    updated_strong_scans_save_dir_ = save_pcd_directory_ + "scans_updated_strong";
    fsmkdir(updated_strong_scans_save_dir_);
    pd_scans_save_dir = save_pcd_directory_ + "scans_pd";
    fsmkdir(pd_scans_save_dir);
    strong_pd_scans_save_dir = save_pcd_directory_ + "scans_pd_strong";
    fsmkdir(strong_pd_scans_save_dir);
    strong_nd_scans_save_dir = save_pcd_directory_ + "scans_nd_strong";
    fsmkdir(strong_nd_scans_save_dir);
    // central_scan_static_save_dir_ = save_pcd_directory_ + "scans_updated";
    // fsmkdir(central_scan_static_save_dir_);
    // central_scan_dynamic_save_dir_ = save_pcd_directory_ + "scans_pd";
    // fsmkdir(central_scan_dynamic_save_dir_);
    // central_scan_dynamic_save_dir_ = save_pcd_directory_ + "scans_nd";
    // fsmkdir(central_scan_dynamic_save_dir_);

    central_map_static_save_dir_ = save_pcd_directory_ + "map_static";
    fsmkdir(central_map_static_save_dir_);
    central_map_dynamic_save_dir_ = save_pcd_directory_ + "map_dynamic";
    fsmkdir(central_map_dynamic_save_dir_);

    allocateMemory();

    // ros pub for visual debug
    scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/scan_single_local", 1);
    global_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/scan_single_global", 1);

    original_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/original_map", 10);

    curr_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/curr_map", 1);
    static_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/static_map", 1);
    dynamic_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/dynamic_map", 1);

    static_curr_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/scan_single_local_static", 1);
    dynamic_curr_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removert/scan_single_local_dynamic", 1);

    // for visualization  
    scan_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/scan_rimg_single", 10);
    map_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_single", 10);
    diff_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/diff_rimg_single", 10);
    map_rimg_ptidx_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_ptidx_single", 10);

} // ctor

void Removerter::allocateMemory()
{
} // allocateMemory

Removerter::~Removerter() {}

void Removerter::loadSessionInfo(void)
{ // note: first load all poses
    central_sess_.loadSessionInfo("Central", central_sess_scan_dir_, central_sess_pose_path_);
    query_sess_.loadSessionInfo("Query", query_sess_scan_dir_, query_sess_pose_path_);

    central_sess_.setDownsampleSize(kDownsampleVoxelSize);
    query_sess_.setDownsampleSize(kDownsampleVoxelSize);
} // loadSessionInfo

void Removerter::parseKeyframes(void)
{ // note: parse subsets of the loaded all poses
    central_sess_.parseKeyframes({start_idx_, end_idx_}, keyframe_gap_);
    query_sess_.parseKeyframesInROI(central_sess_.keyframe_poses_, keyframe_gap_);
} // parseKeyframes

void Removerter::loadKeyframes(void)
{
    central_sess_.loadKeyframes();
    query_sess_.loadKeyframes();
} // loadKeyframes

void Removerter::precleaningKeyframes(float _radius)
{
    central_sess_.precleaningKeyframes(_radius);
    query_sess_.precleaningKeyframes(_radius);
} // loadKeyframes


cv::Mat Removerter::scan2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                                    const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                                    const std::pair<int, int> _rimg_size)
{
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;

    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;
    // cout << "rimg size is: [" << _rimg_size.first << ", " << _rimg_size.second << "]." << endl;

    // @ range image initizliation
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(kFlagNoPOINT)); // float matrix

    // @ points to range img
    int num_points = _scan->points.size();
    #pragma omp parallel for num_threads(kNumOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx)
    {
        PointType this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks.
        int lower_bound_row_idx{0};
        int lower_bound_col_idx{0};
        int upper_bound_row_idx{kNumRimgRow - 1};
        int upper_bound_col_idx{kNumRimgCol - 1};
        int pixel_idx_row = int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))), float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section),
        // @ But, the resulting range image is acceptable (watching via Rviz),
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them,
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel,
        // #               so we can assume practically there are few race conditions.
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        if (curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
        {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
        }
    }

    return rimg;
} // scan2RangeImg

void Removerter::mergeScansWithinGlobalCoord(
    const std::vector<pcl::PointCloud<PointType>::Ptr> &_scans,
    const std::vector<Eigen::Matrix4d> &_scans_poses,
    pcl::PointCloud<PointType>::Ptr &_ptcloud_to_save)
{
    // NOTE: _scans must be in local coord
    for (std::size_t scan_idx = 0; scan_idx < _scans.size(); scan_idx++)
    {
        auto ii_scan = _scans.at(scan_idx);       // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = _scans_poses.at(scan_idx); // Eigen::Matrix4d

        // local to global (local2global)
        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
        pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);

        // merge the scan into the global map
        *_ptcloud_to_save += *scan_global_coord;
    }
} // mergeScansWithinGlobalCoord

// void Removerter::octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &_src, pcl::PointCloud<PointType>::Ptr &_to_save)
// {
//     pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(kDownsampleVoxelSize);
//     octree.setInputCloud(_src);
//     octree.defineBoundingBox();
//     octree.addPointsFromInputCloud();
//     pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
//     octree.getVoxelCentroids(centroids);

//     // init current map with the downsampled full cloud
//     _to_save->points.assign(centroids.begin(), centroids.end());
//     _to_save->width = 1;
//     _to_save->height = _to_save->points.size(); // make sure again the format of the downsampled point cloud
//     ROS_INFO_STREAM("\033[1;32m Octree Downsampled pointcloud have: " << _to_save->points.size() << " points.\033[0m");
// } // octreeDownsampling

// void Removerter::octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &_src, pcl::PointCloud<PointType>::Ptr &_to_save,
//                                     const float _kDownsampleVoxelSize)
// {
//     pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(_kDownsampleVoxelSize);
//     octree.setInputCloud(_src);
//     octree.defineBoundingBox();
//     octree.addPointsFromInputCloud();
//     pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
//     octree.getVoxelCentroids(centroids);

//     // init current map with the downsampled full cloud
//     _to_save->points.assign(centroids.begin(), centroids.end());
//     _to_save->width = 1;
//     _to_save->height = _to_save->points.size(); // make sure again the format of the downsampled point cloud
//     ROS_INFO_STREAM("\033[1;32m Octree Downsampled pointcloud have: " << _to_save->points.size() << " points.\033[0m");
// } // octreeDownsampling


void Removerter::makeGlobalMap(Session& _sess)
{
        // transform local to global and merging the scans
    _sess.map_global_orig_->clear();
    _sess.map_global_curr_->clear();
    _sess.mergeScansWithinGlobalCoord();
    ROS_INFO_STREAM("\033[1;32m Map pointcloud (having redundant points) have: " << _sess.map_global_orig_->points.size() << " points.\033[0m");
    ROS_INFO_STREAM("\033[1;32m Downsampling leaf size is " << kDownsampleVoxelSize << " m.\033[0m");

    // remove repeated (redundant) points
    // - using OctreePointCloudVoxelCentroid for  downsampling
    // - For a large-size point cloud should use OctreePointCloudVoxelCentroid rather VoxelGrid
    octreeDownsampling(_sess.map_global_orig_, _sess.map_global_curr_, kDownsampleVoxelSize);

    // save the original cloud
    if (kFlagSaveMapPointcloud)
    {
        // - in global coord
        std::string static_global_file_name = save_pcd_directory_ + "OriginalNoisy" + _sess.sess_type_ + "MapGlobal.pcd";
        pcl::io::savePCDFileBinary(static_global_file_name, *_sess.map_global_curr_);
        ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (global coord): " << static_global_file_name << "\033[0m");

        // - in local coord (i.e., base_node_idx == 0 means a start idx is the identity pose)
        // int base_node_idx = base_node_idx_;
        // Eigen::Matrix4d base_pose_inverse = sess.keyframe_inverse_poses_.at(base_node_idx);

        // pcl::PointCloud<PointType>::Ptr map_local_curr(new pcl::PointCloud<PointType>);
        // transformGlobalMapToLocal(central_map_global_curr_, base_pose_inverse, map_local_curr);
        // std::string static_local_file_name = save_pcd_directory_ + "OriginalNoisyMapLocal.pcd";
        // pcl::io::savePCDFileBinary(static_local_file_name, *map_local_curr);
        // ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (local coord): " << static_local_file_name << "\033[0m");
    }   
} // makeGlobalMap


void Removerter::makeGlobalMap(void)
{
    makeGlobalMap(central_sess_);
    makeGlobalMap(query_sess_);
} // makeGlobalMap

// void Removerter::transformGlobalMapToLocal(Eigen::Matrix4d _base_pose_inverse)
// {
//     // global to local (global2local)
//     central_map_local_curr_->clear();
//     pcl::transformPointCloud(*central_map_global_curr_, *central_map_local_curr_, _base_pose_inverse);
//     pcl::transformPointCloud(*central_map_local_curr_, *central_map_local_curr_, kSE3MatExtrinsicPoseBasetoLiDAR);
// } // transformGlobalMapToLocal

// void Removerter::transformGlobalMapToLocal(Eigen::Matrix4d _base_pose_inverse, pcl::PointCloud<PointType>::Ptr &_map_local)
// {
//     // global to local (global2local)
//     _map_local->clear();
//     pcl::transformPointCloud(*central_map_global_curr_, *_map_local, _base_pose_inverse);
//     pcl::transformPointCloud(*_map_local, *_map_local, kSE3MatExtrinsicPoseBasetoLiDAR);

// } // transformGlobalMapToLocal


// void Removerter::parseMapPointcloudSubsetUsingPtIdx( std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save )
// {
//     // extractor
//     pcl::ExtractIndices<PointType> extractor;
//     boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
//     extractor.setInputCloud(central_map_global_curr_);
//     extractor.setIndices(index_ptr);
//     extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

//     // parse
//     _ptcloud_to_save->clear();
//     extractor.filter(*_ptcloud_to_save);
// } // parseMapPointcloudSubsetUsingPtIdx


// void Removerter::parsePointsUsingPtIdx(
//     const pcl::PointCloud<PointType>::Ptr& _map_global_curr,
//     pcl::PointCloud<PointType>::Ptr& _map_global_curr_subset,
//     std::vector<int> &_point_indexes)
// {
//     // extractor
//     pcl::ExtractIndices<PointType> extractor;
//     boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
//     extractor.setInputCloud(_map_global_curr);
//     extractor.setIndices(index_ptr);
//     extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

//     // parse
//     _map_global_curr_subset->clear();
//     extractor.filter(*_map_global_curr_subset);
// } // parsePointsUsingPtIdx

// void Removerter::parseDynamicMapPointcloudUsingPtIdx(std::vector<int> &_point_indexes)
// {
//     // extractor
//     pcl::ExtractIndices<PointType> extractor;
//     boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
//     extractor.setInputCloud(central_map_global_curr_);
//     extractor.setIndices(index_ptr);
//     extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

//     // parse
//     central_map_global_curr_dynamic_->clear();
//     extractor.filter(*central_map_global_curr_dynamic_);
// } // parseDynamicMapPointcloudUsingPtIdx

void Removerter::saveCurrentStaticAndDynamicPointCloudGlobal(const Session& _sess, std::string _postfix)
{
    if (!kFlagSaveMapPointcloud)
        return;

    std::string sess_type = _sess.sess_type_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _sess.map_global_curr_static_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _sess.map_global_curr_dynamic_;

    std::string curr_res_alpha_str = std::to_string(curr_res_alpha_);

    // dynamic
    std::string dyna_file_name = central_map_dynamic_save_dir_ + "/" + sess_type + "DynamicMapMapsideGlobal" + _postfix + "ResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(dyna_file_name, *map_global_curr_dynamic);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << dyna_file_name << "\033[0m");

    // static
    std::string static_file_name = central_map_static_save_dir_ + "/" + sess_type + "StaticMapMapsideGlobalResX" + _postfix + "ResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(static_file_name, *map_global_curr_static);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << static_file_name << "\033[0m");
} // saveCurrentStaticAndDynamicPointCloudGlobal

// void Removerter::saveCurrentStaticAndDynamicPointCloudGlobal(void)
// {
//     if (!kFlagSaveMapPointcloud)
//         return;

//     std::string curr_res_alpha_str = std::to_string(curr_res_alpha_);

//     // dynamic
//     std::string dyna_file_name = central_map_dynamic_save_dir_ + "/DynamicMapMapsideGlobalResX" + curr_res_alpha_str + ".pcd";
//     pcl::io::savePCDFileBinary(dyna_file_name, *central_map_global_curr_dynamic_);
//     ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << dyna_file_name << "\033[0m");

//     // static
//     std::string static_file_name = central_map_static_save_dir_ + "/StaticMapMapsideGlobalResX" + curr_res_alpha_str + ".pcd";
//     pcl::io::savePCDFileBinary(static_file_name, *central_map_global_curr_static_);
//     ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << static_file_name << "\033[0m");
// } // saveCurrentStaticAndDynamicPointCloudGlobal

// void Removerter::saveCurrentStaticAndDynamicPointCloudLocal( int _base_node_idx )
// {
//     if( ! kFlagSaveMapPointcloud )
//         return;

//     std::string curr_res_alpha_str = std::to_string(curr_res_alpha_);

//     // dynamic
//     pcl::PointCloud<PointType>::Ptr map_local_curr_dynamic (new pcl::PointCloud<PointType>);
//     transformGlobalMapToLocal(central_map_global_curr_dynamic_, _base_node_idx, map_local_curr_dynamic);
//     std::string dyna_file_name = central_map_dynamic_save_dir_ + "/DynamicMapMapsideLocalResX" + curr_res_alpha_str + ".pcd";
//     pcl::io::savePCDFileBinary(dyna_file_name, *map_local_curr_dynamic);
//     ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << dyna_file_name << "\033[0m");

//     // static
//     pcl::PointCloud<PointType>::Ptr map_local_curr_static (new pcl::PointCloud<PointType>);
//     transformGlobalMapToLocal(central_map_global_curr_static_, _base_node_idx, map_local_curr_static);
//     std::string static_file_name = central_map_static_save_dir_ + "/StaticMapMapsideLocalResX" + curr_res_alpha_str + ".pcd";
//     pcl::io::savePCDFileBinary(static_file_name, *map_local_curr_static);
//     ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << static_file_name << "\033[0m");

// } // saveCurrentStaticAndDynamicPointCloudLocal

std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdx(
    const cv::Mat &_scan_rimg, const cv::Mat &_diff_rimg, const cv::Mat &_map_rimg_ptidx, 
    float _diff_thres)
{
    int num_dyna_points{0}; // TODO: tracking the number of dynamic-assigned points and decide when to stop removing (currently just fixed iteration e.g., [2.5, 2.0, 1.5])

    std::vector<int> dynamic_point_indexes;
    for (int row_idx = 0; row_idx < _diff_rimg.rows; row_idx++)
    {
        for (int col_idx = 0; col_idx < _diff_rimg.cols; col_idx++)
        {
            float this_diff = _diff_rimg.at<float>(row_idx, col_idx);
            float this_range = _scan_rimg.at<float>(row_idx, col_idx);

            // float adaptive_coeff = 0.05; // meter, // i.e., if 4m apart point, it should be 0.4m be diff (nearer) wrt the query
            // float adaptive_dynamic_descrepancy_threshold = adaptive_coeff * this_range; // adaptive descrepancy threshold
            // float adaptive_dynamic_descrepancy_threshold = 0.1;
            float adaptive_dynamic_descrepancy_threshold = _diff_thres; // default: 0.1

            // cout << this_diff << endl;
            if (this_diff < kValidDiffUpperBound // exclude no-point pixels either on scan img or map img (100 is roughly 100 meter)
                && this_diff > adaptive_dynamic_descrepancy_threshold /* dynamic */)
            { // dynamic
                int this_point_idx_in_global_map = _map_rimg_ptidx.at<int>(row_idx, col_idx);
                dynamic_point_indexes.emplace_back(this_point_idx_in_global_map);

                // num_dyna_points++; // TODO
            }
        }
    }

    return dynamic_point_indexes;
} // calcDescrepancyAndParseDynamicPointIdx

// void Removerter::takeGlobalMapSubsetWithinBall( int _center_scan_idx )
// {
//     Eigen::Matrix4d center_pose_se3 = scan_poses_.at(_center_scan_idx);
//     PointType center_pose;
//     center_pose.x = float(center_pose_se3(0, 3));
//     center_pose.y = float(center_pose_se3(1, 3));
//     center_pose.z = float(center_pose_se3(2, 3));

//     std::vector<int> subset_indexes;
//     std::vector<float> pointSearchSqDisGlobalMap;
//     kdtree_central_map_global_curr_->radiusSearch(center_pose, kBallSize, subset_indexes, pointSearchSqDisGlobalMap, 0);
//     parseMapPointcloudSubsetUsingPtIdx(subset_indexes, map_subset_global_curr_);
// } // takeMapSubsetWithinBall

std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdxForEachScanForPD(
    const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_pd_;
    pcl::PointCloud<PointType>::Ptr map_local_curr = _target_sess.map_local_pd_;
    
    std::vector<int> dynamic_point_indexes;
    // dynamic_point_indexes.reserve(100000); // TODO

    // NOTE: if the query_session == target_session, that is the self-removert
    //       if the query_session != target_session, that is the cross-removert
    const auto &scans = _source_sess.keyframe_scans_static_projected_; // this is important
    const auto &inverse_poses = _source_sess.keyframe_inverse_poses_;
    for (std::size_t idx_scan = 0; idx_scan < scans.size(); ++idx_scan)
    {
        // curr scan
        pcl::PointCloud<PointType>::Ptr _scan = scans.at(idx_scan);
        // cout << _target_sess.scan_dir_ << ": " << _scan->size() << endl;

        // scan's pointcloud to range img
        cv::Mat scan_rimg = scan2RangeImg(_scan, kFOV, _rimg_shape); // openMP inside
        Eigen::Matrix4d base_pose_inverse = inverse_poses.at(idx_scan);
        transformGlobalMapToLocal(map_global_curr, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local_curr);
        auto [map_rimg, map_rimg_ptidx] = map2RangeImg(map_local_curr, kFOV, _rimg_shape); // the most time comsuming part 2 -> so openMP applied inside

        // diff range img
        const int kNumRimgRow = _rimg_shape.first;
        const int kNumRimgCol = _rimg_shape.second;
        cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0)); // float matrix, save range value
        // cv::absdiff(scan_rimg, map_rimg, diff_rimg); // was wrong, the sign is required.
        diff_rimg = scan_rimg - map_rimg; // orig 

        // parse dynamic points' indexes: rule: If a pixel value of diff_rimg is larger, scan is the further - means that pixel of submap is likely dynamic.
        float diff_thres_for_check_valid_nd = 0.1;
        std::vector<int> dynamic_map_point_indexes_by_this_scan 
            = calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx, diff_thres_for_check_valid_nd);
        dynamic_point_indexes.insert(dynamic_point_indexes.end(),
                                        dynamic_map_point_indexes_by_this_scan.begin(), dynamic_map_point_indexes_by_this_scan.end());

        // visualization (optional)
        pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(map_rimg, map_rimg_msg_, map_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(diff_rimg, diff_rimg_msg_, diff_rimg_msg_publisher_, kRangeColorAxisForDiff);
        std::pair<float, float> kRangeColorAxisForPtIdx{0.0, float(map_global_curr->points.size())};
        pubRangeImg(map_rimg_ptidx, map_rimg_ptidx_msg_, map_rimg_ptidx_msg_publisher_, kRangeColorAxisForPtIdx);
        publishPointcloud2FromPCLptr(scan_publisher_, _scan);
    } // for_each scan Done

    // remove repeated indexes
    std::set<int> dynamic_point_indexes_set(dynamic_point_indexes.begin(), dynamic_point_indexes.end());
    std::vector<int> dynamic_point_indexes_unique(dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());

    return dynamic_point_indexes_unique;
}


std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdxForEachScanForND(
    const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_nd_;
    pcl::PointCloud<PointType>::Ptr map_local_curr = _target_sess.map_local_nd_;
    
    std::vector<int> dynamic_point_indexes;
    // dynamic_point_indexes.reserve(100000); // TODO

    // NOTE: if the query_session == target_session, that is the self-removert
    //       if the query_session != target_session, that is the cross-removert
    const auto &scans = _source_sess.keyframe_scans_static_projected_;
    const auto &inverse_poses = _source_sess.keyframe_inverse_poses_;
    for (std::size_t idx_scan = 0; idx_scan < scans.size(); ++idx_scan)
    {
        // curr scan
        pcl::PointCloud<PointType>::Ptr _scan = scans.at(idx_scan);
        // cout << _target_sess.scan_dir_ << ": " << _scan->size() << endl;

        // scan's pointcloud to range img
        cv::Mat scan_rimg = scan2RangeImg(_scan, kFOV, _rimg_shape); // openMP inside
        Eigen::Matrix4d base_pose_inverse = inverse_poses.at(idx_scan);
        transformGlobalMapToLocal(map_global_curr, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local_curr);
        auto [map_rimg, map_rimg_ptidx] = map2RangeImg(map_local_curr, kFOV, _rimg_shape); // the most time comsuming part 2 -> so openMP applied inside

        // diff range img
        const int kNumRimgRow = _rimg_shape.first;
        const int kNumRimgCol = _rimg_shape.second;
        cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0)); // float matrix, save range value
        // cv::absdiff(scan_rimg, map_rimg, diff_rimg); // was wrong, the sign is required.
        // diff_rimg = scan_rimg - map_rimg; // orig 
        diff_rimg = map_rimg - scan_rimg; // reversed diff for ND validity check 

        // parse dynamic points' indexes: rule: If a pixel value of diff_rimg is larger, scan is the further - means that pixel of submap is likely dynamic.
        float diff_thres_for_check_valid_nd = 0.1;
        // float diff_thres_for_check_valid_nd = 1.0; // more larger value for between-sess diff
        std::vector<int> dynamic_map_point_indexes_by_this_scan 
            = calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx, diff_thres_for_check_valid_nd);
        dynamic_point_indexes.insert(dynamic_point_indexes.end(),
                                        dynamic_map_point_indexes_by_this_scan.begin(), dynamic_map_point_indexes_by_this_scan.end());

        // visualization (optional)
        pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(map_rimg, map_rimg_msg_, map_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(diff_rimg, diff_rimg_msg_, diff_rimg_msg_publisher_, kRangeColorAxisForDiff);
        std::pair<float, float> kRangeColorAxisForPtIdx{0.0, float(map_global_curr->points.size())};
        pubRangeImg(map_rimg_ptidx, map_rimg_ptidx_msg_, map_rimg_ptidx_msg_publisher_, kRangeColorAxisForPtIdx);
        publishPointcloud2FromPCLptr(scan_publisher_, _scan);
    } // for_each scan Done

    // remove repeated indexes
    std::set<int> dynamic_point_indexes_set(dynamic_point_indexes.begin(), dynamic_point_indexes.end());
    std::vector<int> dynamic_point_indexes_unique(dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());

    return dynamic_point_indexes_unique;
}

std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdxForEachScan(
    const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_curr_;
    pcl::PointCloud<PointType>::Ptr map_local_curr = _target_sess.map_local_curr_;
    
    std::vector<int> dynamic_point_indexes;
    // dynamic_point_indexes.reserve(100000); // TODO

    // NOTE: if the query_session == target_session, that is the self-removert
    //       if the query_session != target_session, that is the cross-removert
    const auto &scans = _source_sess.keyframe_scans_;
    const auto &inverse_poses = _source_sess.keyframe_inverse_poses_;
    for (std::size_t idx_scan = 0; idx_scan < scans.size(); ++idx_scan)
    {
        // curr scan
        pcl::PointCloud<PointType>::Ptr _scan = scans.at(idx_scan);
        // cout << _target_sess.scan_dir_ << ": " << _scan->size() << endl;

        // scan's pointcloud to range img
        cv::Mat scan_rimg = scan2RangeImg(_scan, kFOV, _rimg_shape); // openMP inside
        Eigen::Matrix4d base_pose_inverse = inverse_poses.at(idx_scan);
        transformGlobalMapToLocal(map_global_curr, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local_curr);
        auto [map_rimg, map_rimg_ptidx] = map2RangeImg(map_local_curr, kFOV, _rimg_shape); // the most time comsuming part 2 -> so openMP applied inside

        // diff range img
        const int kNumRimgRow = _rimg_shape.first;
        const int kNumRimgCol = _rimg_shape.second;
        cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0)); // float matrix, save range value
        // cv::absdiff(scan_rimg, map_rimg, diff_rimg); // was wrong, the sign is required.
        diff_rimg = scan_rimg - map_rimg;

        // parse dynamic points' indexes: rule: If a pixel value of diff_rimg is larger, scan is the further - means that pixel of submap is likely dynamic.
        std::vector<int> dynamic_map_point_indexes_by_this_scan = calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx);
        dynamic_point_indexes.insert(dynamic_point_indexes.end(),
                                        dynamic_map_point_indexes_by_this_scan.begin(), dynamic_map_point_indexes_by_this_scan.end());

        // visualization (optional)
        pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(map_rimg, map_rimg_msg_, map_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(diff_rimg, diff_rimg_msg_, diff_rimg_msg_publisher_, kRangeColorAxisForDiff);
        std::pair<float, float> kRangeColorAxisForPtIdx{0.0, float(map_global_curr->points.size())};
        pubRangeImg(map_rimg_ptidx, map_rimg_ptidx_msg_, map_rimg_ptidx_msg_publisher_, kRangeColorAxisForPtIdx);
        publishPointcloud2FromPCLptr(scan_publisher_, _scan);
    } // for_each scan Done

    // remove repeated indexes
    std::set<int> dynamic_point_indexes_set(dynamic_point_indexes.begin(), dynamic_point_indexes.end());
    std::vector<int> dynamic_point_indexes_unique(dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());

    return dynamic_point_indexes_unique;
} // calcDescrepancyForEachScan


// std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdxForEachScan(std::pair<int, int> _rimg_shape)
// {
//     std::vector<int> dynamic_point_indexes;
//     // dynamic_point_indexes.reserve(100000);

//     const auto &sess = query_sess_;
//     // NOTE: if the query_session == map_session, that is the self-removert
//     //       if the query_session != map_session, that is the cross-removert

//     const auto &scans = sess.keyframe_scans_;
//     for (std::size_t idx_scan = 0; idx_scan < scans.size(); ++idx_scan)
//     {
//         // curr scan
//         pcl::PointCloud<PointType>::Ptr _scan = scans.at(idx_scan);
//         cout << _scan->size() << endl;

//         // scan's pointcloud to range img
//         cv::Mat scan_rimg = scan2RangeImg(_scan, kFOV, _rimg_shape); // openMP inside

//         // NOT recommend to use for under 5 million points map input
//         // map's pointcloud to range img
//         // if( kUseSubsetMapCloud ) {
//         //     takeGlobalMapSubsetWithinBall(idx_scan);
//         //     transformGlobalMapSubsetToLocal(idx_scan); // the most time comsuming part 1
//         // } else {
//         //     // if the input map size (of a batch) is short, just using this line is more fast.
//         //     // - e.g., 100-1000m or ~5 million points are ok, empirically more than 10Hz
//         //     transformGlobalMapToLocal(idx_scan);
//         // }
//         Eigen::Matrix4d base_pose_inverse = sess.keyframe_inverse_poses_.at(idx_scan);
//         transformGlobalMapToLocal(base_pose_inverse);
//         auto [map_rimg, map_rimg_ptidx] = map2RangeImg(central_map_local_curr_, kFOV, _rimg_shape); // the most time comsuming part 2 -> so openMP applied inside

//         // diff range img
//         const int kNumRimgRow = _rimg_shape.first;
//         const int kNumRimgCol = _rimg_shape.second;
//         cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0)); // float matrix, save range value
//         // cv::absdiff(scan_rimg, map_rimg, diff_rimg); // was wrong, the sign is required.
//         diff_rimg = scan_rimg - map_rimg;

//         // parse dynamic points' indexes: rule: If a pixel value of diff_rimg is larger, scan is the further - means that pixel of submap is likely dynamic.
//         std::vector<int> dynamic_map_point_indexes_by_this_scan = calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx);
//         dynamic_point_indexes.insert(dynamic_point_indexes.end(),
//                                         dynamic_map_point_indexes_by_this_scan.begin(), dynamic_map_point_indexes_by_this_scan.end());

//         // visualization
//         pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
//         pubRangeImg(map_rimg, map_rimg_msg_, map_rimg_msg_publisher_, kRangeColorAxis);
//         pubRangeImg(diff_rimg, diff_rimg_msg_, diff_rimg_msg_publisher_, kRangeColorAxisForDiff);

//         std::pair<float, float> kRangeColorAxisForPtIdx{0.0, float(central_map_global_curr_->points.size())};
//         pubRangeImg(map_rimg_ptidx, map_rimg_ptidx_msg_, map_rimg_ptidx_msg_publisher_, kRangeColorAxisForPtIdx);

//         publishPointcloud2FromPCLptr(scan_publisher_, _scan);

//         // fast removal (20210116)
//         if (0)
//         {
//             ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points: " << dynamic_map_point_indexes_by_this_scan.size() << "\033[0m");
//             // parseDynamicMapPointcloudUsingPtIdx(dynamic_map_point_indexes_by_this_scan);

//             std::vector<int> static_point_indexes = getGlobalMapStaticIdxFromDynamicIdx(dynamic_map_point_indexes_by_this_scan);
//             ROS_INFO_STREAM("\033[1;32m -- The number of static points: " << static_point_indexes.size() << "\033[0m");
//             // parseStaticMapPointcloudUsingPtIdx(static_point_indexes);

//             // Update the current map and reset the tree
//             central_map_global_curr_->clear();
//             *central_map_global_curr_ = *central_map_global_curr_static_;
//         }
//     } // for_each scan Done

//     // remove repeated indexes
//     std::set<int> dynamic_point_indexes_set(dynamic_point_indexes.begin(), dynamic_point_indexes.end());
//     std::vector<int> dynamic_point_indexes_unique(dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());
//     // std::vector<int> dynamic_point_indexes_unique {0};

//     return dynamic_point_indexes_unique;
// } // calcDescrepancyForEachScan

std::vector<int> Removerter::getStaticIdxFromDynamicIdx(const std::vector<int> &_dynamic_point_indexes, int _num_all_points)
{
    std::vector<int> pt_idx_all = linspace<int>(0, _num_all_points, _num_all_points);

    std::set<int> pt_idx_all_set(pt_idx_all.begin(), pt_idx_all.end());
    for (auto &_dyna_pt_idx : _dynamic_point_indexes)
    {
        pt_idx_all_set.erase(_dyna_pt_idx);
    }

    std::vector<int> static_point_indexes(pt_idx_all_set.begin(), pt_idx_all_set.end());
    return static_point_indexes;
} // getStaticIdxFromDynamicIdx

std::vector<int> Removerter::getGlobalMapStaticIdxFromDynamicIdx(
    const pcl::PointCloud<PointType>::Ptr& _map_global_curr, 
    std::vector<int> &_dynamic_point_indexes)
{
    int num_all_points = _map_global_curr->points.size();
    return getStaticIdxFromDynamicIdx(_dynamic_point_indexes, num_all_points);
} // getGlobalMapStaticIdxFromDynamicIdx


// std::vector<int> Removerter::getGlobalMapStaticIdxFromDynamicIdx(const std::vector<int> &_dynamic_point_indexes)
// {
//     int num_all_points = central_map_global_curr_->points.size();
//     return getStaticIdxFromDynamicIdx(_dynamic_point_indexes, num_all_points);
// } // getGlobalMapStaticIdxFromDynamicIdx

// void Removerter::saveCurrentStaticMapHistory(void)
// {
//     // deep copy
//     pcl::PointCloud<PointType>::Ptr map_global_curr_static (new pcl::PointCloud<PointType>);
//     *map_global_curr_static = *central_map_global_curr_;

//     // save
//     static_map_global_history_.emplace_back(map_global_curr_static);
// } // saveCurrentStaticMapHistory

void Removerter::resetCurrrentMapAsDynamic(const Session& _sess, bool _as_dynamic)
{
    // filter spec (i.e., a shape of the range image)
    pcl::PointCloud<PointType>::Ptr map_global_curr = _sess.map_global_curr_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _sess.map_global_curr_static_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _sess.map_global_curr_dynamic_;

    map_global_curr->clear();
    if( _as_dynamic ) {
        *map_global_curr = *map_global_curr_dynamic;
    } else {
        *map_global_curr = *map_global_curr_static;
    }
}

void Removerter::resetCurrrentMapAsDynamic(const Session& _sess)
{
    resetCurrrentMapAsDynamic(_sess, true);   
}

void Removerter::resetCurrrentMapAsStatic(const Session& _sess)
{
    resetCurrrentMapAsDynamic(_sess, false);   
}


std::pair<pcPtr, pcPtr> Removerter::partitionCurrentMapForPD(
    const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_pd_;

    curr_res_alpha_ = _res_alpha;
    curr_rimg_shape_ = resetRimgSize(kFOV, _res_alpha);
    float deg_per_pixel = 1.0 / _res_alpha;
    ROS_INFO_STREAM("\033[1;32m with resolution: x" << _res_alpha << " (" << deg_per_pixel << " deg/pixel)\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The range image size is: [" << curr_rimg_shape_.first << ", " << curr_rimg_shape_.second << "].\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The number of " << _target_sess.sess_type_ << " PD map points: " << map_global_curr->points.size() << "\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- ... starts to clean non-volume-extending PD ... " << "\033[0m");

    // map-side removal: remove and get dynamic (will be removed) points' index set
    std::vector<int> dynamic_point_indexes 
        = calcDescrepancyAndParseDynamicPointIdxForEachScanForPD(_target_sess, _source_sess, curr_rimg_shape_);
    ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points: " << dynamic_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, dynamic_point_indexes, map_global_curr_dynamic_this_turn);

    // static_point_indexes == complemently indexing dynamic_point_indexes
    std::vector<int> static_point_indexes = getGlobalMapStaticIdxFromDynamicIdx(map_global_curr, dynamic_point_indexes);
    ROS_INFO_STREAM("\033[1;32m -- The number of static points: " << static_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_static_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, static_point_indexes, map_global_curr_static_this_turn);

    return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(
        map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn);
} 


std::pair<pcPtr, pcPtr> Removerter::partitionCurrentMapForND(
    const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_nd_;

    curr_res_alpha_ = _res_alpha;
    curr_rimg_shape_ = resetRimgSize(kFOV, _res_alpha);
    float deg_per_pixel = 1.0 / _res_alpha;
    ROS_INFO_STREAM("\033[1;32m with resolution: x" << _res_alpha << " (" << deg_per_pixel << " deg/pixel)\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The range image size is: [" << curr_rimg_shape_.first << ", " << curr_rimg_shape_.second << "].\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The number of " << _target_sess.sess_type_ << " ND map points: " << map_global_curr->points.size() << "\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- ... starts to clean ambiguous ND ... " << "\033[0m");

    // map-side removal: remove and get dynamic (will be removed) points' index set
    std::vector<int> dynamic_point_indexes 
        = calcDescrepancyAndParseDynamicPointIdxForEachScanForND(_target_sess, _source_sess, curr_rimg_shape_);
    ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points: " << dynamic_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, dynamic_point_indexes, map_global_curr_dynamic_this_turn);

    // static_point_indexes == complemently indexing dynamic_point_indexes
    std::vector<int> static_point_indexes = getGlobalMapStaticIdxFromDynamicIdx(map_global_curr, dynamic_point_indexes);
    ROS_INFO_STREAM("\033[1;32m -- The number of static points: " << static_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_static_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, static_point_indexes, map_global_curr_static_this_turn);

    return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(
        map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn);
} 

std::pair<pcPtr, pcPtr> Removerter::partitionCurrentMap(
    const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_curr_;

    curr_res_alpha_ = _res_alpha;
    curr_rimg_shape_ = resetRimgSize(kFOV, _res_alpha);
    float deg_per_pixel = 1.0 / _res_alpha;
    ROS_INFO_STREAM("\033[1;32m with resolution: x" << _res_alpha << " (" << deg_per_pixel << " deg/pixel)\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The range image size is: [" << curr_rimg_shape_.first << ", " << curr_rimg_shape_.second << "].\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- The number of " << _target_sess.sess_type_ << " map points: " << map_global_curr->points.size() << "\033[0m");
    ROS_INFO_STREAM("\033[1;32m -- ... starts cleaning ... " << "\033[0m");

    // map-side removal: remove and get dynamic (will be removed) points' index set
    std::vector<int> dynamic_point_indexes = calcDescrepancyAndParseDynamicPointIdxForEachScan(_target_sess, _source_sess, curr_rimg_shape_);
    ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points: " << dynamic_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, dynamic_point_indexes, map_global_curr_dynamic_this_turn);

    // static_point_indexes == complemently indexing dynamic_point_indexes
    std::vector<int> static_point_indexes = getGlobalMapStaticIdxFromDynamicIdx(map_global_curr, dynamic_point_indexes);
    ROS_INFO_STREAM("\033[1;32m -- The number of static points: " << static_point_indexes.size() << "\033[0m");
    pcl::PointCloud<PointType>::Ptr map_global_curr_static_this_turn(new pcl::PointCloud<PointType>());
    parsePointcloudSubsetUsingPtIdx(map_global_curr, static_point_indexes, map_global_curr_static_this_turn);

    return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(
        map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn);
}


void Removerter::iremoveOnceForND(
    const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_nd_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _target_sess.map_global_nd_strong_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _target_sess.map_global_nd_weak_;

    ROS_INFO_STREAM("\033[1;32m" << "\nIdentifying Strong/Weak ND points starts \033[0m");
    auto [map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn] 
        = partitionCurrentMapForND( _target_sess, _source_sess, _res_alpha ); 

    // Update the current map and reset the tree
    map_global_curr_static->clear();
    *map_global_curr_static = *map_global_curr_static_this_turn; // update (i.e., remove)
    octreeDownsampling(map_global_curr_static, map_global_curr_static, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Static pointcloud have: " << map_global_curr_static->points.size() << " points.\033[0m");

    map_global_curr->clear();
    *map_global_curr = *map_global_curr_static; // update

    *map_global_curr_dynamic += *map_global_curr_dynamic_this_turn; // append (for later reverts)
    octreeDownsampling(map_global_curr_dynamic, map_global_curr_dynamic, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Dynamic pointcloud have: " << map_global_curr_dynamic->points.size() << " points.\033[0m");
}

void Removerter::removeOnceForPD(
    const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_pd_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _target_sess.map_global_pd_strong_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _target_sess.map_global_pd_weak_;

    ROS_INFO_STREAM("\033[1;32m" << "\nIdentifying Strong/Weak PD points starts \033[0m");
    auto [map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn] 
        = partitionCurrentMapForPD( _target_sess, _source_sess, _res_alpha ); 

    // Update the current map and reset the tree
    map_global_curr_static->clear();
    *map_global_curr_static = *map_global_curr_static_this_turn; // update (i.e., remove)
    octreeDownsampling(map_global_curr_static, map_global_curr_static, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Static pointcloud have: " << map_global_curr_static->points.size() << " points.\033[0m");

    // renewal the current pd for next removing initialization point
    map_global_curr->clear();
    *map_global_curr = *map_global_curr_static; // update

    *map_global_curr_dynamic += *map_global_curr_dynamic_this_turn; // append (for later reverts)
    octreeDownsampling(map_global_curr_dynamic, map_global_curr_dynamic, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Dynamic pointcloud have: " << map_global_curr_dynamic->points.size() << " points.\033[0m");
}

void Removerter::removeOnce(const Session& _target_sess, const Session& _source_sess, float _res_alpha)
{
    // filter spec (i.e., a shape of the range image)
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_curr_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _target_sess.map_global_curr_static_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _target_sess.map_global_curr_dynamic_;

    ROS_INFO_STREAM("\033[1;32m" << "\nSelf-removing starts \033[0m");
    auto [map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn] 
        = partitionCurrentMap( _target_sess, _source_sess, _res_alpha ); 

    // Update the current map and reset the tree
    map_global_curr_static->clear();
    *map_global_curr_static = *map_global_curr_static_this_turn; // update (i.e., remove)
    octreeDownsampling(map_global_curr_static, map_global_curr_static, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Static pointcloud have: " << map_global_curr_static->points.size() << " points.\033[0m");

    map_global_curr->clear();
    *map_global_curr = *map_global_curr_static; // update

    *map_global_curr_dynamic += *map_global_curr_dynamic_this_turn; // append (for later reverts)
    octreeDownsampling(map_global_curr_dynamic, map_global_curr_dynamic, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Dynamic pointcloud have: " << map_global_curr_dynamic->points.size() << " points.\033[0m");
}


void Removerter::revertOnce( const Session& _target_sess, const Session& _source_sess, float _res_alpha )
{
    pcl::PointCloud<PointType>::Ptr map_global_curr = _target_sess.map_global_curr_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static = _target_sess.map_global_curr_static_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic = _target_sess.map_global_curr_dynamic_;

    ROS_INFO_STREAM("\033[1;32m" << "\nSelf-reverting starts \033[0m");
    auto [map_global_curr_static_this_turn, map_global_curr_dynamic_this_turn] 
        = partitionCurrentMap( _target_sess, _source_sess, _res_alpha ); 

    // Update the current map and reset the tree
    map_global_curr_dynamic->clear();
    *map_global_curr_dynamic = *map_global_curr_dynamic_this_turn; // update
    octreeDownsampling(map_global_curr_dynamic, map_global_curr_dynamic, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Dynamic pointcloud have: " << map_global_curr_dynamic->points.size() << " points.\033[0m");

    map_global_curr->clear();
    *map_global_curr = *map_global_curr_dynamic; // update

    *map_global_curr_static += *map_global_curr_static_this_turn; // append (i.e., revert)
    octreeDownsampling(map_global_curr_static, map_global_curr_static, 0.05);
    ROS_INFO_STREAM("\033[1;32m Current Static pointcloud have: " << map_global_curr_static->points.size() << " points.\033[0m");

} // revertOnce

void Removerter::parsePointcloudSubsetUsingPtIdx( const pcl::PointCloud<PointType>::Ptr& _ptcloud_orig,
            std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save )
{
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(_ptcloud_orig);
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse
    _ptcloud_to_save->clear();
    extractor.filter(*_ptcloud_to_save);
} // parsePointcloudSubsetUsingPtIdx


// std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>
// Removerter::removeDynamicPointsOfScanByKnnBtnSession(int _scan_idx)
// {
//     // const auto& sess = central_sess_;
//     const auto &sess = query_sess_;
//     const auto &scans_ = sess.keyframe_scans_;
//     const auto &scan_poses_ = sess.keyframe_poses_;
//     const auto &scan_inverse_poses = sess.keyframe_inverse_poses_;
//     const auto &scan_paths_ = sess.keyframe_paths_;

//     // curr scan (in local coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig = scans_.at(_scan_idx);
//     auto scan_pose = scan_poses_.at(_scan_idx);

//     // curr scan (in global coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig_global = local2global(scan_orig, scan_poses_.at(_scan_idx));

//     // fine reg to map
//     pcl::PointCloud<PointType>::Ptr scan_orig_global_down(new pcl::PointCloud<PointType>());
//     float icpVoxelSize = 0.4; // meter
//     octreeDownsampling(scan_orig_global, scan_orig_global_down, icpVoxelSize);
//     icp_.setInputSource(scan_orig_global_down);
//     pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
//     icp_.align(*unused_result);
//     pcl::PointCloud<PointType>::Ptr scan_orig_global_reg(new pcl::PointCloud<PointType>());
//     pcl::transformPointCloud(*scan_orig_global, *scan_orig_global_reg, icp_.getFinalTransformation());

//     cout << icp_.getFitnessScore() << endl;
//     pcl::PointCloud<PointType>::Ptr scan_eff(new pcl::PointCloud<PointType>());
//     if (icp_.getFitnessScore() < 1.0)
//     {
//         *scan_eff = *scan_orig_global_reg;
//     }
//     else
//     {
//         *scan_eff = *scan_orig_global;
//     }

//     //
//     int num_points_of_a_scan = scan_orig_global->points.size();
//     pcl::PointCloud<PointType>::Ptr scan_static_global(new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_global(new pcl::PointCloud<PointType>);
//     for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
//     {
//         std::vector<int> topk_indexes_map;
//         std::vector<float> topk_L2dists_map;
//         // // kdtree_query_map_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
//         // kdtree_query_map_global_curr_->nearestKSearch(scan_orig_global_reg->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
//         kdtree_central_map_global_curr_->nearestKSearch(scan_orig_global_reg->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
//         float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
//         float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

//         if (std::abs(avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold)
//         {
//             scan_static_global->push_back(scan_eff->points[pt_idx]);
//         }
//         else
//         {
//             scan_dynamic_global->push_back(scan_eff->points[pt_idx]);
//         }
//     }

//     // again global2local because later in the merging global map function, which requires scans within each local coord.
//     pcl::PointCloud<PointType>::Ptr scan_static_local = global2local(scan_static_global, scan_inverse_poses.at(_scan_idx));
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_local = global2local(scan_dynamic_global, scan_inverse_poses.at(_scan_idx));

//     return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(scan_static_local, scan_dynamic_local);

// } // removeDynamicPointsOfScanByKnnBtnSession

// std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>
// Removerter::removeDynamicPointsOfScanByKnn(
//     int _scan_idx,
//     const Session& _sess,
//     const pcl::KdTreeFLANN<PointType>::Ptr& _kdtree_map_global_curr)
// {
//     const auto &scans_ = _sess.keyframe_scans_;
//     const auto &scan_poses_ = _sess.keyframe_poses_;
//     const auto &scan_inverse_poses = _sess.keyframe_inverse_poses_;
//     const auto &scan_paths_ = _sess.keyframe_paths_;

//     // curr scan (in local coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig = scans_.at(_scan_idx);
//     auto scan_pose = scan_poses_.at(_scan_idx);

//     // curr scan (in global coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig_global = local2global(scan_orig, scan_poses_.at(_scan_idx));
//     kdtree_scan_global_curr_->setInputCloud(scan_orig_global);
//     int num_points_of_a_scan = scan_orig_global->points.size();

//     pcl::PointCloud<PointType>::Ptr scan_static_global(new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_global(new pcl::PointCloud<PointType>);
//     for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
//     {
//         // std::vector<int> topk_indexes_scan;
//         // std::vector<float> topk_L2dists_scan;
//         // kdtree_scan_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_scan, topk_L2dists_scan);
//         // float sum_topknn_dists_in_scan = accumulate(topk_L2dists_scan.begin(), topk_L2dists_scan.end(), 0.0);
//         // float avg_topknn_dists_in_scan = sum_topknn_dists_in_scan / float(kNumKnnPointsToCompare);

//         std::vector<int> topk_indexes_map;
//         std::vector<float> topk_L2dists_map;
//         _kdtree_map_global_curr->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
//         float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
//         float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

//         // if (std::abs(avg_topknn_dists_in_scan - avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold) {
//         if (std::abs(avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold) {
//             scan_static_global->push_back(scan_orig_global->points[pt_idx]);
//         } else {
//             scan_dynamic_global->push_back(scan_orig_global->points[pt_idx]);
//         }
//     }

//     // again global2local because later in the merging global map function, which requires scans within each local coord.
//     pcl::PointCloud<PointType>::Ptr scan_static_local = global2local(scan_static_global, scan_inverse_poses.at(_scan_idx));
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_local = global2local(scan_dynamic_global, scan_inverse_poses.at(_scan_idx));

//     return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(scan_static_local, scan_dynamic_local);

// } // removeDynamicPointsOfScanByKnn

// std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>
// Removerter::removeDynamicPointsOfScanByKnn(int _scan_idx)
// {
//     const auto &sess = central_sess_;
//     const auto &scans_ = sess.keyframe_scans_;
//     const auto &scan_poses_ = sess.keyframe_poses_;
//     const auto &scan_inverse_poses = sess.keyframe_inverse_poses_;
//     const auto &scan_paths_ = sess.keyframe_paths_;

//     // curr scan (in local coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig = scans_.at(_scan_idx);
//     auto scan_pose = scan_poses_.at(_scan_idx);

//     // curr scan (in global coord)
//     pcl::PointCloud<PointType>::Ptr scan_orig_global = local2global(scan_orig, scan_poses_.at(_scan_idx));
//     kdtree_scan_global_curr_->setInputCloud(scan_orig_global);
//     int num_points_of_a_scan = scan_orig_global->points.size();

//     //
//     pcl::PointCloud<PointType>::Ptr scan_static_global(new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_global(new pcl::PointCloud<PointType>);
//     for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
//     {
//         std::vector<int> topk_indexes_scan;
//         std::vector<float> topk_L2dists_scan;
//         kdtree_scan_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_scan, topk_L2dists_scan);
//         float sum_topknn_dists_in_scan = accumulate(topk_L2dists_scan.begin(), topk_L2dists_scan.end(), 0.0);
//         float avg_topknn_dists_in_scan = sum_topknn_dists_in_scan / float(kNumKnnPointsToCompare);

//         std::vector<int> topk_indexes_map;
//         std::vector<float> topk_L2dists_map;
//         kdtree_central_map_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
//         float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
//         float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

//         //
//         if (std::abs(avg_topknn_dists_in_scan - avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold)
//         {
//             scan_static_global->push_back(scan_orig_global->points[pt_idx]);
//         }
//         else
//         {
//             scan_dynamic_global->push_back(scan_orig_global->points[pt_idx]);
//         }
//     }

//     // again global2local because later in the merging global map function, which requires scans within each local coord.
//     pcl::PointCloud<PointType>::Ptr scan_static_local = global2local(scan_static_global, scan_inverse_poses.at(_scan_idx));
//     pcl::PointCloud<PointType>::Ptr scan_dynamic_local = global2local(scan_dynamic_global, scan_inverse_poses.at(_scan_idx));

//     return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(scan_static_local, scan_dynamic_local);

// } // removeDynamicPointsOfScanByKnn

// void Removerter::saveStaticScan(int _scan_idx, const pcl::PointCloud<PointType>::Ptr &_ptcloud)
// {
//     std::string file_name_orig = central_sess_.keyframe_names_.at(_scan_idx);
//     // std::string file_name = central_scan_static_save_dir_ + "/" + file_name_orig + ".pcd";
//     std::string file_name = central_scan_static_save_dir_ + "/" + file_name_orig;
//     ROS_INFO_STREAM("\033[1;32m Scan " << _scan_idx << "'s static points is saved (" << file_name << ")\033[0m");
//     pcl::io::savePCDFileBinary(file_name, *_ptcloud);
// } // saveStaticScan

// void Removerter::saveDynamicScan(int _scan_idx, const pcl::PointCloud<PointType>::Ptr &_ptcloud)
// {
//     std::string file_name_orig = central_sess_.keyframe_names_.at(_scan_idx);
//     // std::string file_name = central_scan_dynamic_save_dir_ + "/" + file_name_orig + ".pcd";
//     std::string file_name = central_scan_dynamic_save_dir_ + "/" + file_name_orig;
//     ROS_INFO_STREAM("\033[1;32m Scan " << _scan_idx << "'s dynamic points is saved (" << file_name << ")\033[0m");
//     pcl::io::savePCDFileBinary(file_name, *_ptcloud);
// } // saveDynamicScan

// void Removerter::saveCleanedScans(void)
// {
//     if (!kFlagSaveCleanScans)
//         return;

//     for (std::size_t idx_scan = 0; idx_scan < central_scans_static_.size(); idx_scan++)
//     {

//         auto scan_static_local = central_scans_static_.at(idx_scan);
//         auto scan_dynamic_local = central_scans_dynamic_.at(idx_scan);

//         saveStaticScan(idx_scan, scan_static_local);
//         saveDynamicScan(idx_scan, scan_dynamic_local);

//         // for debug
//         const auto &sess = central_sess_;
//         const auto &scan_paths_ = sess.keyframe_paths_;

//         ROS_INFO_STREAM("\033[1;32m The scan " << scan_paths_.at(idx_scan) << "\033[0m");
//         ROS_INFO_STREAM("\033[1;32m -- The number of static points in a scan: " << scan_static_local->points.size() << "\033[0m");
//         ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points in a scan: " << scan_dynamic_local->points.size() << "\033[0m");
//         publishPointcloud2FromPCLptr(static_curr_scan_publisher_, scan_static_local);
//         publishPointcloud2FromPCLptr(dynamic_curr_scan_publisher_, scan_dynamic_local);
//         usleep(kPauseTimeForClearStaticScanVisualization);
//     }
// } // saveCleanedScans

// void Removerter::saveCleanedScans(const Session& _sess)
// {
//     if (!kFlagSaveCleanScans)
//         return;

//     for (std::size_t idx_scan = 0; idx_scan < central_scans_static_.size(); idx_scan++)
//     {

//         auto scan_static_local = central_scans_static_.at(idx_scan);
//         auto scan_dynamic_local = central_scans_dynamic_.at(idx_scan);

//         saveStaticScan(idx_scan, scan_static_local);
//         saveDynamicScan(idx_scan, scan_dynamic_local);

//         // for debug
//         const auto &sess = central_sess_;
//         const auto &scan_paths_ = sess.keyframe_paths_;

//         ROS_INFO_STREAM("\033[1;32m The scan " << scan_paths_.at(idx_scan) << "\033[0m");
//         ROS_INFO_STREAM("\033[1;32m -- The number of static points in a scan: " << scan_static_local->points.size() << "\033[0m");
//         ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points in a scan: " << scan_dynamic_local->points.size() << "\033[0m");
//         publishPointcloud2FromPCLptr(static_curr_scan_publisher_, scan_static_local);
//         publishPointcloud2FromPCLptr(dynamic_curr_scan_publisher_, scan_dynamic_local);
//         usleep(kPauseTimeForClearStaticScanVisualization);
//     }
// } // saveCleanedScans

// void Removerter::saveMapPointcloudByMergingCleanedScans( const Session& _sess, std::string _postfix )
// {
//     // merge for verification
//     if (!kFlagSaveMapPointcloud)
//         return;

//     std::vector<pcl::PointCloud<PointType>::Ptr>* scans_static;
//     std::vector<pcl::PointCloud<PointType>::Ptr>* scans_dynamic;
//     std::string sess_name;
//     if( _sess.scan_dir_ ==  central_sess_scan_dir_) {
//         scans_static = &central_scans_static_;
//         scans_dynamic = &central_scans_dynamic_;
//         sess_name = "Central";
//     } else {        
//         scans_static = &query_scans_static_;
//         scans_dynamic = &query_scans_dynamic_;
//         sess_name = "Query";
//     }

//     const auto &scan_poses_ = _sess.keyframe_poses_;

//     // static map
//     {
//         pcl::PointCloud<PointType>::Ptr map_global_static_scans_merged_to_verify_full(new pcl::PointCloud<PointType>);
//         pcl::PointCloud<PointType>::Ptr map_global_static_scans_merged_to_verify(new pcl::PointCloud<PointType>);
//         mergeScansWithinGlobalCoord(*scans_static, scan_poses_, map_global_static_scans_merged_to_verify_full);
//         octreeDownsampling(map_global_static_scans_merged_to_verify_full, map_global_static_scans_merged_to_verify);

//         // global
//         std::string local_file_name = central_map_static_save_dir_ + "/" + _postfix + sess_name + "StaticScansMergedMapGlobal.pcd";
//         pcl::io::savePCDFileBinary(local_file_name, *map_global_static_scans_merged_to_verify);
//         ROS_INFO_STREAM("\033[1;32m  [For verification] A static pointcloud (cleaned scans mscan_static_localved (global coord): " << local_file_name << "\033[0m");
//     }

//     // dynamic map
//     {
//         pcl::PointCloud<PointType>::Ptr map_global_dynamic_scans_merged_to_verify_full(new pcl::PointCloud<PointType>);
//         pcl::PointCloud<PointType>::Ptr map_global_dynamic_scans_merged_to_verify(new pcl::PointCloud<PointType>);
//         mergeScansWithinGlobalCoord(*scans_dynamic, scan_poses_, map_global_dynamic_scans_merged_to_verify_full);
//         octreeDownsampling(map_global_dynamic_scans_merged_to_verify_full, map_global_dynamic_scans_merged_to_verify);

//         // global
//         std::string local_file_name = central_map_dynamic_save_dir_ + "/" + _postfix + sess_name + "DynamicScansMergedMapGlobal.pcd";
//         pcl::io::savePCDFileBinary(local_file_name, *map_global_dynamic_scans_merged_to_verify);
//         ROS_INFO_STREAM("\033[1;32m  [For verification] A dynamic pointcloud (cleaned scans merged) is saved (global coord): " << local_file_name << "\033[0m");

//         // local - unused 
//         // pcl::PointCloud<PointType>::Ptr map_local_dynamic_scans_merged_to_verify (new pcl::PointCloud<PointType>);
//         // int base_node_idx = base_node_idx_;
//         // transformGlobalMapToLocal(map_global_dynamic_scans_merged_to_verify, base_node_idx, map_local_dynamic_scans_merged_to_verify);
//         // std::string global_file_name = central_map_dynamic_save_dir_ + "/DynamicMapScansideMapLocal.pcd";
//         // pcl::io::savePCDFileBinary(global_file_name, *map_local_dynamic_scans_merged_to_verify);
//         // ROS_INFO_STREAM("\033[1;32m  [For verification] A dynamic pointcloud (cleaned scans merged) is saved (local coord): " << global_file_name << "\033[0m");
//     }
// } // saveMapPointcloudByMergingCleanedScans

// void Removerter::scansideRemovalForEachScan(const Session& _map_sess, const Session& _scan_sess)
// {
//     // NOTE:
//     /* (map_sess, scan_sess)
//         - (C, C) -> using C map, scanwise remove C: self-removing of C
//         - (Q, Q) -> using Q map, scanwise remove Q: self-removing of Q
//         - (C, Q) -> using C map, scanwise remove Q: here dynamic map is, positive diff
//         - (Q, C) -> using Q map, scanwise remove C: here dynamic map is, negative diff
//     */

//     float icpVoxelSize = 0.4; // meter
//     icp_.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
//     icp_.setMaximumIterations(100);
//     icp_.setTransformationEpsilon(1e-6);
//     icp_.setEuclideanFitnessEpsilon(1e-6);
//     icp_.setRANSACIterations(0);

//     pcl::PointCloud<PointType>::Ptr map_global_curr;
//     std::string sess_type = _scan_sess.sess_type_;
//     auto scans_static = 
//     std::vector<pcl::PointCloud<PointType>::Ptr>* scans_static;
//     std::vector<pcl::PointCloud<PointType>::Ptr>* scans_dynamic;
//     if( _scan_sess.scan_dir_ ==  central_sess_scan_dir_) {
//         sess_type = std::string{"central"};
//         map_global_curr = central_map_global_curr_; // in here, already it was static-ed 
//         scans_static = &central_scans_static_;
//         scans_dynamic = &central_scans_dynamic_;
//         kdtree_central_map_global_curr_->setInputCloud(central_map_global_curr_);
//         kdtree_map_global_curr_ = kdtree_central_map_global_curr_;
//         octreeDownsampling(central_map_global_curr_, central_map_global_curr_down_for_icp_, icpVoxelSize);
//         icp_.setInputTarget(central_map_global_curr_down_for_icp_); 
//     } else {        
//         sess_type = std::string{"query"};
//         map_global_curr = query_map_global_curr_;  // in here, already it was static-ed 
//         scans_static = &query_scans_static_;
//         scans_dynamic = &query_scans_dynamic_;
//         kdtree_query_map_global_curr_->setInputCloud(query_map_global_curr_);
//         kdtree_map_global_curr_ = kdtree_query_map_global_curr_;
//         octreeDownsampling(query_map_global_curr_, query_map_global_curr_down_for_icp_, icpVoxelSize);
//         icp_.setInputTarget(query_map_global_curr_down_for_icp_); 
//     }

//     bool isFineRegistration {false};
//     if (_map_sess.scan_dir_ != _scan_sess.scan_dir_)
//         isFineRegistration = true; 

//     scans_static->clear();
//     scans_dynamic->clear();
//     const auto &scans_ = _scan_sess.keyframe_scans_;
//     for (std::size_t idx_scan = 0; idx_scan < scans_.size(); idx_scan++) {
//         // cout << sess_type << endl;
//         // auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnn(idx_scan);
//         // auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnnBtnSession(idx_scan);
//         auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnn(idx_scan, _scan_sess, kdtree_map_global_curr_);
//         scans_static->emplace_back(this_scan_static);
//         scans_dynamic->emplace_back(this_scan_dynamic);
//     }
// } // scansideRemovalForEachScan // TODO: change the name 

// void Removerter::scansideRemovalForEachScan(void)
// {
//     // for fast scan-side neighbor search
//     kdtree_central_map_global_curr_->setInputCloud(central_map_global_curr_);

//     float knnVoxelSize = 0.05; // meter
//     pcl::PointCloud<PointType>::Ptr query_map_global_curr_down(new pcl::PointCloud<PointType>());
//     octreeDownsampling(query_map_global_curr_, query_map_global_curr_down, knnVoxelSize);
//     kdtree_query_map_global_curr_->setInputCloud(query_map_global_curr_down);

//     // icp setup
//     float icpVoxelSize = 0.4; // meter
//     // octreeDownsampling(query_map_global_curr_, query_map_global_curr_down_for_icp_, icpVoxelSize);
//     octreeDownsampling(central_map_global_curr_, query_map_global_curr_down_for_icp_, icpVoxelSize);
//     icp_.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
//     icp_.setMaximumIterations(100);
//     icp_.setTransformationEpsilon(1e-6);
//     icp_.setEuclideanFitnessEpsilon(1e-6);
//     icp_.setRANSACIterations(0);
//     icp_.setInputTarget(query_map_global_curr_down_for_icp_); // query is bigger here so set the query map as target is safe.

//     const auto &sess = central_sess_;
//     const auto &scans_ = central_sess_.keyframe_scans_;
//     for (std::size_t idx_scan = 0; idx_scan < scans_.size(); idx_scan++)
//     {
//         // auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnn(idx_scan);
//         auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnnBtnSession(idx_scan);
//         central_scans_static_.emplace_back(this_scan_static);
//         central_scans_dynamic_.emplace_back(this_scan_dynamic);
//     }
// } // scansideRemovalForEachScan

// void Removerter::scansideRemovalViaMapReprojectionForEachScan(void)
// {
//     // for fast scan-side neighbor search
//     kdtree_central_map_global_curr_->setInputCloud(central_map_global_curr_);

//     // for each scan
//     const auto &sess = central_sess_;
//     const auto &scans_ = sess.keyframe_scans_;
//     for (std::size_t idx_scan = 0; idx_scan < scans_.size(); idx_scan++)
//     {
//         Eigen::Matrix4d base_pose_inverse = sess.keyframe_inverse_poses_.at(idx_scan);
//         transformGlobalMapToLocal(base_pose_inverse);
//         auto this_scan_static = parseProjectedPoints(central_map_local_curr_, kFOV, resetRimgSize(kFOV, kReprojectionAlpha)); // the most time comsuming part 2 -> so openMP applied inside
//         auto [unused, this_scan_dynamic] = removeDynamicPointsOfScanByKnn(idx_scan);

//         central_scans_static_.emplace_back(this_scan_static);
//         central_scans_dynamic_.emplace_back(this_scan_dynamic);
//     }
// } // scansideRemovalViaMapReprojectionForEachScan

// void Removerter::scansideRemovalForEachScanAndSaveThem(void)
// {
//     scansideRemovalForEachScan();
//     saveCleanedScans();
//     // saveMapPointcloudByMergingCleanedScans();
// } // scansideRemovalForEachScanAndSaveThem

// void Removerter::scansideRemovalViaMapReprojectionForEachScanAndSaveThem(void)
// {
//     scansideRemovalViaMapReprojectionForEachScan();
//     saveCleanedScans();
//     // saveMapPointcloudByMergingCleanedScans();
// } // scansideRemovalViaMapReprojectionForEachScanAndSaveThem

void Removerter::selfRemovert(const Session& _sess, int _repeat = 1)
{
    for (float _res : remove_resolution_list_) {
        for(int i=0; i<_repeat; i++) {
            removeOnce(_sess, _sess, _res); 
            
            resetCurrrentMapAsDynamic(_sess);
            revertOnce(_sess, _sess, 0.95*_res );
            resetCurrrentMapAsStatic(_sess);    
            
            removeOnce(_sess, _sess, _res);
        }
    }   

    saveCurrentStaticAndDynamicPointCloudGlobal(_sess, "_MVM");
}

void Removerter::filterStrongPD(Session& _sess_target, Session& _sess_source)
{
    float res = 2.5;
    removeOnceForPD(_sess_target, _sess_source, res); // hard coding for fast test
    removeOnceForPD(_sess_target, _sess_source, res); // 
    removeOnceForPD(_sess_target, _sess_source, res); // 
}

void Removerter::filterStrongND(Session& _sess_target, Session& _sess_source)
{
    // _sess_target: to be cleaned 
    // _sess_source: a cleaner 
    float res = 2.5;
    iremoveOnceForND(_sess_target, _sess_source, res); // hard coding for fast test
    iremoveOnceForND(_sess_target, _sess_source, res); // hard coding for fast test
    iremoveOnceForND(_sess_target, _sess_source, res); // hard coding for fast test
}

void Removerter::detectLowDynamicPoints(void)
{
    ROS_INFO_STREAM("\033[1;32m parse low dynamic diff via knn: " << central_sess_.sess_type_ << " to " << query_sess_.sess_type_ << "\033[0m");
    central_sess_.extractLowDynPointsViaKnnDiff(query_sess_.map_global_curr_static_);
    ROS_INFO_STREAM("\033[1;32m parse low dynamic diff via knn: " << query_sess_.sess_type_ << " to " << central_sess_.sess_type_ << "\033[0m");
    query_sess_.extractLowDynPointsViaKnnDiff(central_sess_.map_global_curr_static_);

    // strong ND
    central_sess_.constructGlobalNDMap();
    // *central_sess_.map_global_nd_ = *central_sess_.map_global_curr_;
    filterStrongND(central_sess_, query_sess_); // filtering central_sess_.scans_knn_diff_
    central_sess_.removeWeakNDMapPointsHavingStrongNDInNear(); // propagation"

    // strong PD
    query_sess_.constructGlobalPDMap();
    // *query_sess_.map_global_pd_ = *query_sess_.map_global_curr_static_;
    filterStrongPD(query_sess_, central_sess_); // filtering central_sess_.scans_knn_diff_
    query_sess_.revertStrongPDMapPointsHavingWeakPDInNear(); // propagation"

    // save pd map (in query sess instance) into the central sess instance
    // the map is already in global (central) coord, but resave into the central sess because we'll later using the central sess's poses, reproject this map.
    *central_sess_.map_global_pd_ = *query_sess_.map_global_pd_; 
    *central_sess_.map_global_pd_orig_ = *query_sess_.map_global_pd_orig_; 
    *central_sess_.map_global_pd_strong_ = *query_sess_.map_global_pd_strong_;


    /*
        save merged maps for visual debug
    */
    bool flagSaveMergedMapForViz = true;
    if(flagSaveMergedMapForViz) {
        auto map_global_union_queryside = mergeScansWithinGlobalCoordUtil(query_sess_.scans_knn_coexist_, query_sess_.keyframe_poses_, query_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
        octreeDownsampling(map_global_union_queryside, map_global_union_queryside, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "union_map_queryside.pcd", *map_global_union_queryside);

        auto map_global_union_centralside = mergeScansWithinGlobalCoordUtil(central_sess_.scans_knn_coexist_, central_sess_.keyframe_poses_, central_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
        octreeDownsampling(map_global_union_centralside, map_global_union_centralside, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "union_map_centralside.pcd", *map_global_union_centralside);

        auto map_global_positive_diff = mergeScansWithinGlobalCoordUtil(query_sess_.scans_knn_diff_, query_sess_.keyframe_poses_, query_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
        octreeDownsampling(map_global_positive_diff, map_global_positive_diff, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "pd_map.pcd", *map_global_positive_diff);

        auto map_global_negative_diff = mergeScansWithinGlobalCoordUtil(central_sess_.scans_knn_diff_, central_sess_.keyframe_poses_, central_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
        octreeDownsampling(map_global_negative_diff, map_global_negative_diff, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "nd_map.pcd", *map_global_negative_diff);

        ROS_INFO_STREAM("\033[1;32m Union, PD, and ND map saved \033[0m");

        // debug for ND
        if( central_sess_.map_global_nd_strong_->points.size() != 0 ) {
            octreeDownsampling(central_sess_.map_global_nd_strong_, central_sess_.map_global_nd_strong_, 0.05);
            pcl::io::savePCDFileBinary(save_pcd_directory_ + "strong_nd_map.pcd", *central_sess_.map_global_nd_strong_);
        }
        octreeDownsampling(central_sess_.map_global_nd_weak_, central_sess_.map_global_nd_weak_, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "weak_nd_map.pcd", *central_sess_.map_global_nd_weak_);

        ROS_INFO_STREAM("\033[1;32m Strong/Weak ND map saved \033[0m");

        // debug for PD (2021.02.12)
        octreeDownsampling(query_sess_.map_global_pd_strong_, query_sess_.map_global_pd_strong_, 0.05);    
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "strong_pd_map.pcd", *query_sess_.map_global_pd_strong_);

        octreeDownsampling(query_sess_.map_global_pd_weak_, query_sess_.map_global_pd_weak_, 0.05);
        pcl::io::savePCDFileBinary(save_pcd_directory_ + "weak_pd_map.pcd", *query_sess_.map_global_pd_weak_);

        ROS_INFO_STREAM("\033[1;32m Strong/Weak PD map saved \033[0m");
    }
} // detectLowDynamicPoints

void Removerter::updateCurrentMap(void)
{
    pcl::PointCloud<PointType>::Ptr map_global_updated(new pcl::PointCloud<PointType>());

    // 1: removing ND points from current session 
    // - NOTE: removing strong ND is implicitly done while making union map 
    auto map_global_union_queryside = mergeScansWithinGlobalCoordUtil(query_sess_.scans_knn_coexist_, query_sess_.keyframe_poses_, query_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
    octreeDownsampling(map_global_union_queryside, map_global_union_queryside, 0.05);

    auto map_global_union_centralside = mergeScansWithinGlobalCoordUtil(central_sess_.scans_knn_coexist_, central_sess_.keyframe_poses_, central_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
    octreeDownsampling(map_global_union_centralside, map_global_union_centralside, 0.05);

    *map_global_updated = *map_global_union_queryside; // init
    *map_global_updated += *map_global_union_centralside; // init
    ROS_INFO_STREAM("\033[1;32m -- The number of map points (updating ...): " << map_global_updated->points.size() << "\033[0m");
    
    // 1-2: revert weak ND points to static 
    *map_global_updated += *central_sess_.map_global_nd_weak_; // append
    ROS_INFO_STREAM("\033[1;32m -- The number of map points (updating ...): " << map_global_updated->points.size() << "\033[0m");

    // 2-1: add strong PD ver. 
    pcl::PointCloud<PointType>::Ptr map_global_updated_strong_(new pcl::PointCloud<PointType>());
    *map_global_updated_strong_ = *map_global_updated; // ==     pcl::copyPointCloud(*map_global_updated, *map_global_updated_strong_);
    *map_global_updated_strong_ += *central_sess_.map_global_pd_strong_;
    octreeDownsampling(map_global_updated_strong_, map_global_updated_strong_, 0.05);
    ROS_INFO_STREAM("\033[1;32m -- The number of strong map points (updating ...): " << map_global_updated_strong_->points.size() << "\033[0m");

    // 2-2: add PD ver. 
    *map_global_updated += *central_sess_.map_global_pd_orig_; // append
    octreeDownsampling(map_global_updated, map_global_updated, 0.05);
    ROS_INFO_STREAM("\033[1;32m -- The number of map points (updating ...): " << map_global_updated->points.size() << "\033[0m");

    // 3: write 
    *central_sess_.map_global_updated_ = *map_global_updated;
    pcl::io::savePCDFileBinary(save_pcd_directory_ + "updated_map.pcd", *central_sess_.map_global_updated_);

    *central_sess_.map_global_updated_strong_ = *map_global_updated_strong_;
    pcl::io::savePCDFileBinary(save_pcd_directory_ + "updated_map_strong.pcd", *central_sess_.map_global_updated_strong_);

    ROS_INFO_STREAM("\033[1;32m -- The updated map is saved \033[0m");

} // updateCurrentMap


void Removerter::parseStaticScansViaProjection(Session& _sess)
{
    ROS_INFO_STREAM("\033[1;32m parse static scans via projection: " << _sess.sess_type_ << "\033[0m");
    _sess.parseStaticScansViaProjection();
} // parseStaticScansViaProjection


void Removerter::parseStaticScansViaProjection(void)
{
    parseStaticScansViaProjection(central_sess_);
    parseStaticScansViaProjection(query_sess_);
} // parseStaticScansViaProjection

void Removerter::updateScansScanwise()
{
    updateScansScanwise(central_sess_);
}
void Removerter::updateScansScanwise(Session& _sess)
{
    _sess.updateScansScanwise();
    ROS_INFO_STREAM("\033[1;32m final update scans: " << _sess.sess_type_ << "\033[0m");
}


void Removerter::parseUpdatedStaticScansViaProjection()
{
    parseUpdatedStaticScansViaProjection(central_sess_);
}
void Removerter::parseUpdatedStaticScansViaProjection(Session& _sess)
{
    _sess.parseUpdatedStaticScansViaProjection();
    _sess.parseUpdatedStrongStaticScansViaProjection();

    ROS_INFO_STREAM("\033[1;32m parse updated scans via projection: " << _sess.sess_type_ << "\033[0m");
} // parseStaticScansViaProjection


void Removerter::parseLDScansViaProjection()
{
    parseLDScansViaProjection(central_sess_);
}
void Removerter::parseLDScansViaProjection(Session& _sess)
{
    // NOTE: this function is expected to use only central sess 
    _sess.parsePDScansViaProjection();
    _sess.parseStrongPDScansViaProjection();
    _sess.parseWeakNDScansViaProjection();
    _sess.parseStrongNDScansViaProjection();

    ROS_INFO_STREAM("\033[1;32m parse LD scans via projection: " << _sess.sess_type_ << "\033[0m");
} // parseStaticScansViaProjection


void Removerter::removeHighDynamicPoints(void)
{   
    // selfRemovert(central_sess_, repeat_removert_iter_); // use for the very initial map (where central map is also noisy)
    // removeOnce(central_sess_, central_sess_, 1.0); // use after the map init
    removeOnce(central_sess_, central_sess_, 2.5); // use after the map init

    // selfRemovert(query_sess_, repeat_removert_iter_); 
    removeOnce(query_sess_, query_sess_, 2.5); // use after the map init


    // dyn map save (for paper viz)
    central_sess_.extractHighDynPointsViaKnnDiff(central_sess_.map_global_curr_static_);
    query_sess_.extractHighDynPointsViaKnnDiff(query_sess_.map_global_curr_static_);

    auto map_central_high_dyn = mergeScansWithinGlobalCoordUtil(central_sess_.keyframe_scans_dynamic_, central_sess_.keyframe_poses_, central_sess_.kSE3MatExtrinsicLiDARtoPoseBase);
    auto map_query_high_dyn = mergeScansWithinGlobalCoordUtil(query_sess_.keyframe_scans_dynamic_, query_sess_.keyframe_poses_, query_sess_.kSE3MatExtrinsicLiDARtoPoseBase);

    octreeDownsampling(map_central_high_dyn, map_central_high_dyn, 0.05);
    octreeDownsampling(map_query_high_dyn, map_query_high_dyn, 0.05);

    pcl::io::savePCDFileBinary(save_pcd_directory_ + "central_sess_high_dyn.pcd", *map_central_high_dyn);
    pcl::io::savePCDFileBinary(save_pcd_directory_ + "query_sess_high_dyn.pcd", *map_query_high_dyn);
    ROS_INFO_STREAM("\033[1;32m high dynamic maps are saved. \033[0m");

} // removeHighDynamicPoints


void Removerter::saveAllTypeOfScans()
{
    saveUpdatedScans(central_sess_);
    saveLDScans(central_sess_);
}

void Removerter::saveLDScans(Session& _sess)
{
    savePDScans(_sess);
    saveStrongPDScans(_sess);
    saveStrongNDScans(_sess);
}

void Removerter::saveUpdatedScans(Session& _sess) {
    saveScans(_sess, _sess.keyframe_scans_updated_, updated_scans_save_dir_);
    saveScans(_sess, _sess.keyframe_scans_updated_strong_, updated_strong_scans_save_dir_);
}

void Removerter::savePDScans(Session& _sess) {
    saveScans(_sess, _sess.keyframe_scans_pd_, pd_scans_save_dir);
}

void Removerter::saveStrongPDScans(Session& _sess) {
    saveScans(_sess, _sess.keyframe_scans_strong_pd_, strong_pd_scans_save_dir);
}

void Removerter::saveStrongNDScans(Session& _sess) {
    saveScans(_sess, _sess.keyframe_scans_strong_nd_, strong_nd_scans_save_dir);
}

void Removerter::saveScans(Session& _sess, std::vector<pcl::PointCloud<PointType>::Ptr> _scans, std::string _save_dir)
{
    for (std::size_t idx_scan = 0; idx_scan < _scans.size(); idx_scan++) 
    {
        auto keyframe_scan_local = _scans.at(idx_scan);
        std::string file_name_orig = _sess.keyframe_names_.at(idx_scan);
        std::string file_name = _save_dir + "/" + file_name_orig;
        ROS_INFO_STREAM("\033[1;32m Updated Scan " << idx_scan << "'s static points is saved (" << file_name << ")\033[0m");
        pcl::io::savePCDFileBinary(file_name, *keyframe_scan_local);

        // publishPointcloud2FromPCLptr(static_curr_scan_publisher_, scan_static_local);
        // usleep(kPauseTimeForClearStaticScanVisualization);
    }
}


void Removerter::run(void)
{
    // # Step 0: Preparations 
    loadSessionInfo();
 
    parseKeyframes();
    loadKeyframes();
    precleaningKeyframes(2.5); // optional. remove points within near radius from the lidar 
 
    makeGlobalMap();

    // # Step 1: HD noise removal 
    removeHighDynamicPoints();
    parseStaticScansViaProjection();

    // # Step 2: LD change detection
    detectLowDynamicPoints();

    // # Step 3: LT-map
    updateCurrentMap();
    parseUpdatedStaticScansViaProjection();
    parseLDScansViaProjection(); // TODO
    updateScansScanwise(); // == eq(4) in the paper
    saveAllTypeOfScans(); // TODO

} // Removerter::run

} // namespace ltremovert
