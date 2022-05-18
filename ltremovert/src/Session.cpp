#include "removert/Session.h"

using std::cout;
using std::endl;

namespace ltremovert 
{

Session::Session( void )
{
    allocateMemory();
}


void Session::allocateMemory(void)
{
    //
    kdtree_map_global_curr_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_target_map_global_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_scan_global_curr_.reset(new pcl::KdTreeFLANN<PointType>());

    kdtree_map_global_nd_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_map_global_pd_.reset(new pcl::KdTreeFLANN<PointType>());

    map_global_orig_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_.reset(new pcl::PointCloud<PointType>());
    map_local_curr_.reset(new pcl::PointCloud<PointType>());

    map_global_updated_.reset(new pcl::PointCloud<PointType>());
    map_global_updated_strong_.reset(new pcl::PointCloud<PointType>());

    map_global_curr_static_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_dynamic_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_down_for_icp_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_diff_positive_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_diff_negative_.reset(new pcl::PointCloud<PointType>());

    map_global_nd_.reset(new pcl::PointCloud<PointType>());
    map_local_nd_.reset(new pcl::PointCloud<PointType>());
    map_global_nd_strong_.reset(new pcl::PointCloud<PointType>());
    map_global_nd_weak_.reset(new pcl::PointCloud<PointType>());

    map_global_pd_.reset(new pcl::PointCloud<PointType>());
    map_global_pd_orig_.reset(new pcl::PointCloud<PointType>());
    map_local_pd_.reset(new pcl::PointCloud<PointType>());
    map_global_pd_strong_.reset(new pcl::PointCloud<PointType>());
    map_global_pd_weak_.reset(new pcl::PointCloud<PointType>());

    // 
    target_map_down_for_knn_.reset(new pcl::PointCloud<PointType>());

    map_global_orig_->clear();
    map_global_curr_->clear();
    map_local_curr_->clear();
    map_global_curr_static_->clear();
    map_global_curr_dynamic_->clear();

    map_global_updated_->clear();
    map_global_updated_strong_->clear();

    map_global_curr_down_for_icp_->clear();
    map_global_curr_diff_positive_->clear();
    map_global_curr_diff_negative_->clear();

    map_global_nd_->clear();
    map_local_nd_->clear();
    map_global_nd_strong_->clear();
    map_global_nd_weak_->clear();

    map_global_pd_->clear();
    map_global_pd_orig_->clear();
    map_local_pd_->clear();
    map_global_pd_strong_->clear();
    map_global_pd_weak_->clear();

    target_map_down_for_knn_->clear();
} // allocateMemory


void Session::loadSessionInfo( std::string _sess_type, std::string _scan_dir, std::string _pose_path )
{
    sess_type_ = _sess_type;
    scan_dir_ = _scan_dir;
    pose_path_ = _pose_path;

    // load scan paths 
    for(auto& _entry : fs::directory_iterator(_scan_dir)) {
        scan_names_.emplace_back(_entry.path().filename());
        scan_paths_.emplace_back(_entry.path());
    }
    std::sort(scan_names_.begin(), scan_names_.end());
    std::sort(scan_paths_.begin(), scan_paths_.end());

    num_scans_ = scan_paths_.size();
    ROS_INFO_STREAM("\033[1;32m Total : " << num_scans_ << " scans in the directory.\033[0m");
    
    // debug 
    // for(auto& _elm: scan_paths_)
    //     cout << _elm << endl;

    // load the trajectory poses
    std::ifstream pose_file_handle (_pose_path);
    std::string strOneLine;
    while (getline(pose_file_handle, strOneLine)) {
        std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' '); // str to vec
        if(ith_pose_vec.size() == 12) { // append a row [0,0,0,1]
            ith_pose_vec.emplace_back(double(0.0)); ith_pose_vec.emplace_back(double(0.0)); ith_pose_vec.emplace_back(double(0.0)); ith_pose_vec.emplace_back(double(1.0));
        }
        Eigen::Matrix4d ith_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ith_pose_vec.data(), 4, 4);
        Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();

        scan_poses_.emplace_back(ith_pose);
        scan_inverse_poses_.emplace_back(ith_pose_inverse);
    }

    // check the number of scans and the number of poses are equivalent
    assert(scan_paths_.size() == scan_poses_.size());
} // loadSessionInfo 


void Session::setDownsampleSize( float _voxel_size )
{
    kDownsampleVoxelSize = _voxel_size;
} // setDownsampleSize


void Session::clearKeyframes( void )
{
    int num_keyframes_ = 0; 

    keyframe_names_.clear();
    keyframe_paths_.clear();
    keyframe_poses_.clear();
    keyframe_inverse_poses_.clear();
} // clearKeyframes


void Session::parseKeyframes( std::pair<int, int> _range, int _gap )
{
    clearKeyframes();

    auto start_idx = _range.first;
    auto end_idx = _range.second;

    int num_valid_parsed {0};
    for(int curr_idx=0; curr_idx < int(scan_paths_.size()); curr_idx++) 
    {
        // check the scan idx within the target idx range 
        if(curr_idx > end_idx || curr_idx < start_idx) {
            curr_idx++;
            continue;
        }

        if( remainder(num_valid_parsed, _gap) != 0 ) {
            num_valid_parsed++;
            continue;
        }

        // save the info (reading scan bin is in makeGlobalMap) 
        keyframe_paths_.emplace_back(scan_paths_.at(curr_idx));
        keyframe_names_.emplace_back(scan_names_.at(curr_idx));

        keyframe_poses_.emplace_back(scan_poses_.at(curr_idx)); // used for local2global
        keyframe_inverse_poses_.emplace_back(scan_inverse_poses_.at(curr_idx)); // used for global2local

        // 
        num_valid_parsed++;
    }

    ROS_INFO_STREAM("\033[1;32m Total " << keyframe_paths_.size()
        << " nodes are used from the index range [" << start_idx << ", " << end_idx << "]" 
        << " (every " << _gap << " frames parsed)\033[0m");

} // parseKeyframes


void Session::parseKeyframes( int _gap )
{
    clearKeyframes();

    std::pair<int, int> _range { 0, int(scan_paths_.size()) };
    parseKeyframes(_range, _gap);
} // parseKeyframes for all range 


void Session::mergeScansWithinGlobalCoord(void)
{
    // NOTE: _scans must be in local coord
    for (std::size_t scan_idx = 0; scan_idx < keyframe_scans_.size(); scan_idx++)
    {
        auto ii_scan = keyframe_scans_.at(scan_idx);       // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = keyframe_poses_.at(scan_idx); // Eigen::Matrix4d

        // local to global (local2global)
        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
        pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);

        // merge the scan into the global map
        *map_global_orig_ += *scan_global_coord;
    }
} // mergeScansWithinGlobalCoord


double xyzDist (const Eigen::Matrix4d& _pose1, const Eigen::Matrix4d& _pose2)
{
    // cout << _pose1(0,0) << ", " << _pose1(0,1) << ", " << _pose1(0,2) << ", " << _pose1(0,3) << endl;
    // cout << _pose1(1,0) << ", " << _pose1(1,1) << ", " << _pose1(1,2) << ", " << _pose1(1,3) << endl;
    // cout << _pose1(2,0) << ", " << _pose1(2,1) << ", " << _pose1(2,2) << ", " << _pose1(2,3) << endl;
    // cout << _pose1(3,0) << ", " << _pose1(3,1) << ", " << _pose1(3,2) << ", " << _pose1(3,3) << endl;

    return std::sqrt(   (_pose1(0,3)-_pose2(0,3))*(_pose1(0,3)-_pose2(0,3)) 
                      + (_pose1(1,3)-_pose2(1,3))*(_pose1(1,3)-_pose2(1,3)) 
                      + (_pose1(2,3)-_pose2(2,3))*(_pose1(2,3)-_pose2(2,3)) );
} // xyzDist

double nnDist( const Eigen::Matrix4d& _pose, const std::vector<Eigen::Matrix4d>& _roi_poses)
{
    double nn_dist = 10000000000.0;
    for (auto _roi_pose: _roi_poses) {
        double dist = xyzDist(_pose, _roi_pose);        
        if( dist < nn_dist )
            nn_dist = dist;
    }

    return nn_dist;
} // nnDist


void Session::parseKeyframesInROI( const std::vector<Eigen::Matrix4d>& _roi_poses, int _gap )
{
    clearKeyframes();
    
    double inplace_thres = 10.0; // e.g., 10, 30 meter, user param  

    int num_valid_parsed {0};
    for(int curr_idx=0; curr_idx < int(scan_paths_.size()); curr_idx++) 
    {
        auto curr_pose = scan_poses_.at(curr_idx);
        double dist = nnDist(curr_pose, _roi_poses);
        if(dist > inplace_thres)
            continue;

        if( remainder(num_valid_parsed, _gap) != 0 ) {
            num_valid_parsed++;
            continue;
        }

        // save the info (reading scan bin is in makeGlobalMap) 
        keyframe_paths_.emplace_back(scan_paths_.at(curr_idx));
        keyframe_names_.emplace_back(scan_names_.at(curr_idx));

        keyframe_poses_.emplace_back(scan_poses_.at(curr_idx)); // used for local2global
        keyframe_inverse_poses_.emplace_back(scan_inverse_poses_.at(curr_idx)); // used for global2local

        // 
        num_valid_parsed++;
    }

    ROS_INFO_STREAM("\033[1;32m Total " << keyframe_paths_.size()
        << " keyframes parsed in the map's ROI\033[0m");

} // parseKeyframesInROI


void Session::loadKeyframes( void )
{
    const int cout_interval {10}; 
    int cout_counter {0};
    cout << endl;
    cout << " ... (display every " << cout_interval << " readings) ..." << endl;
    for(auto& _scan_path : keyframe_paths_) {
        // read bin files and save  
        pcl::PointCloud<PointType>::Ptr points (new pcl::PointCloud<PointType>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).
        pcl::io::loadPCDFile<PointType> (_scan_path, *points); // saved from SC-LIO-SAM's pcd binary (.pcd)
        // Deprecated: PLEASE use the binary pcd format scans saved by using SC-LIO-SAM's save utility 
        // if( isScanFileKITTIFormat_ ) {
        //     readBin(_scan_path, points); // For KITTI (.bin)
        // } else {
        //     pcl::io::loadPCDFile<PointType> (_scan_path, *points); // saved from SC-LIO-SAM's pcd binary (.pcd)
        // }

        // pcdown
        pcl::VoxelGrid<PointType> downsize_filter;
        downsize_filter.setLeafSize(kDownsampleVoxelSize, kDownsampleVoxelSize, kDownsampleVoxelSize);
        downsize_filter.setInputCloud(points);

        pcl::PointCloud<PointType>::Ptr downsampled_points (new pcl::PointCloud<PointType>);
        downsize_filter.filter(*downsampled_points);

        // save downsampled pointcloud
        keyframe_scans_.emplace_back(downsampled_points);

        // cout for debug  
        cout_counter++;
        if (remainder(cout_counter, cout_interval) == 0) {
            cout << _scan_path << endl;
            cout << "Read a pointcloud with " << points->points.size() << " points (" 
                 << "downsampled size: " << downsampled_points->points.size() << " points)" << endl;
        }
    }
} // loadKeyframes


void Session::parseStaticScansViaProjection(void)
{
    parseScansViaProjection(map_global_curr_, keyframe_scans_static_projected_);

    // keyframe_scans_static_projected_.clear();
    // for (std::size_t idx_scan = 0; idx_scan < keyframe_scans_.size(); idx_scan++) {
    //     Eigen::Matrix4d base_pose_inverse = keyframe_inverse_poses_.at(idx_scan);
    //     transformGlobalMapToLocal(map_global_curr_, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local_curr_);
    //     auto this_scan_static = parseProjectedPoints(map_local_curr_, kFOV, resetRimgSize(kFOV, kReprojectionAlpha)); // the most time comsuming part 2 -> so openMP applied inside
    //     keyframe_scans_static_projected_.emplace_back(this_scan_static);
    // }   
} // parseStaticScansViaProjection

void Session::parseUpdatedStaticScansViaProjection(void)
{
    parseScansViaProjection(map_global_updated_, keyframe_scans_updated_);
}

void Session::parseUpdatedStrongStaticScansViaProjection(void)
{
    parseScansViaProjection(map_global_updated_strong_, keyframe_scans_updated_strong_);
}

void Session::parsePDScansViaProjection(void)
{
    parseScansViaProjection(map_global_pd_orig_, keyframe_scans_pd_);
}

void Session::parseStrongPDScansViaProjection(void)
{
    parseScansViaProjection(map_global_pd_strong_, keyframe_scans_strong_pd_); // ?
}

void Session::parseWeakNDScansViaProjection(void)
{
    parseScansViaProjection(map_global_nd_weak_, keyframe_scans_weak_nd_);
}

void Session::parseStrongNDScansViaProjection(void)
{
    parseScansViaProjection(map_global_nd_strong_, keyframe_scans_strong_nd_);
}

void Session::parseScansViaProjection(
    pcl::PointCloud<PointType>::Ptr _map, // to_be_parsed
    std::vector<pcl::PointCloud<PointType>::Ptr>& _vec_to_store )
{
    _vec_to_store.clear();
    pcl::PointCloud<PointType>::Ptr map_local(new pcl::PointCloud<PointType>());       
    for (std::size_t idx_scan = 0; idx_scan < keyframe_scans_.size(); idx_scan++) {
        Eigen::Matrix4d base_pose_inverse = keyframe_inverse_poses_.at(idx_scan);
        transformGlobalMapToLocal(_map, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local);
        auto this_scan_static = parseProjectedPoints(map_local, kFOV, resetRimgSize(kFOV, kReprojectionAlpha)); // the most time comsuming part 2 -> so openMP applied inside
        _vec_to_store.emplace_back(this_scan_static);
    }   
} // parseStaticScansViaProjection

void Session::updateScansScanwise()
{
    for(std::size_t ii = 0; ii < keyframe_scans_updated_.size(); ii++)
    {
        // init 
        pcl::PointCloud<PointType>::Ptr final_updated_scan(new pcl::PointCloud<PointType>());
        *final_updated_scan = *keyframe_scans_updated_.at(ii);

        // append
        *final_updated_scan += *keyframe_scans_weak_nd_.at(ii);
        *final_updated_scan += *keyframe_scans_pd_.at(ii);
        
        // downsampling (to remove repeated points)
        octreeDownsampling(final_updated_scan, final_updated_scan, 0.05);

        // renewal
        *keyframe_scans_updated_[ii] = *final_updated_scan;
    }
} // updateScansScanwise

// void Session::parseDynamicScansViaProjection(void)
// {
//     scans_dynamic_projected_.clear();
//     for (std::size_t idx_scan = 0; idx_scan < keyframe_scans_.size(); idx_scan++) {
//         Eigen::Matrix4d base_pose_inverse = keyframe_inverse_poses_.at(idx_scan);
//         transformGlobalMapToLocal(map_global_curr_, base_pose_inverse, kSE3MatExtrinsicPoseBasetoLiDAR, map_local_curr_);
//         auto this_scan_static = parseProjectedPoints(map_local_curr_, kFOV, resetRimgSize(kFOV, kReprojectionAlpha)); // the most time comsuming part 2 -> so openMP applied inside
//         scans_dynamic_projected_.emplace_back(this_scan_static);
//     }   
// } // parseDynamicScansViaProjection

void Session::extractLowDynPointsViaKnnDiff(const pcl::PointCloud<PointType>::Ptr _target_map)
{
    float icpVoxelSize = 0.4; // meter
    icp_for_knn_.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp_for_knn_.setMaximumIterations(100);
    icp_for_knn_.setTransformationEpsilon(1e-6);
    icp_for_knn_.setEuclideanFitnessEpsilon(1e-6);
    icp_for_knn_.setRANSACIterations(0);
    octreeDownsampling(_target_map, target_map_down_for_knn_, icpVoxelSize);
    icp_for_knn_.setInputTarget(target_map_down_for_knn_); 

    kdtree_target_map_global_->setInputCloud(_target_map);

    std::map<int, pcl::PointCloud<PointType>::Ptr> scans_knn_coexist_map;
    std::map<int, pcl::PointCloud<PointType>::Ptr> scans_knn_diff_map;
    #pragma omp parallel for num_threads(kNumOmpCores) // if using icp refinement (in current version, not recommended), do not use the this omp  
    for (std::size_t idx_scan = 0; idx_scan < keyframe_scans_static_projected_.size(); idx_scan++) {
        auto [this_scan_coexist, this_scan_diff] = partitionLowDynamicPointsOfScanByKnn(idx_scan);
        mtx.lock();
        scans_knn_coexist_map[int(idx_scan)] = this_scan_coexist;
        scans_knn_diff_map[int(idx_scan)] = this_scan_diff;
        mtx.unlock();
    }

    scans_knn_coexist_.clear();
    scans_knn_diff_.clear();
    scans_knn_coexist_.reserve(keyframe_scans_static_projected_.size());
    scans_knn_diff_.reserve(keyframe_scans_static_projected_.size());

    for(auto& _elm: scans_knn_coexist_map) 
        scans_knn_coexist_.emplace_back(_elm.second);

    for(auto& _elm: scans_knn_diff_map) 
        scans_knn_diff_.emplace_back(_elm.second);
}


void Session::constructGlobalNDMap()
{
    auto map_global_nd = mergeScansWithinGlobalCoordUtil(scans_knn_diff_, keyframe_poses_, kSE3MatExtrinsicLiDARtoPoseBase);
    *map_global_nd_ = *map_global_nd;
    octreeDownsampling(map_global_nd_, map_global_nd_, 0.05);
} 

void Session::constructGlobalPDMap()
{
    auto map_global_pd = mergeScansWithinGlobalCoordUtil(scans_knn_diff_, keyframe_poses_, kSE3MatExtrinsicLiDARtoPoseBase);
    *map_global_pd_ = *map_global_pd;
    octreeDownsampling(map_global_pd_, map_global_pd_, 0.05);

    // backup (for save)
    *map_global_pd_orig_ = *map_global_pd_;
} 

void Session::revertStrongPDMapPointsHavingWeakPDInNear()
{
    // TODO
}

void Session::removeWeakNDMapPointsHavingStrongNDInNear()
{
    if( map_global_nd_strong_->points.size() == 0 )
        return; 
    
    kdtree_map_global_nd_->setInputCloud(map_global_nd_strong_);
    
    int num_points_of_weak_nd = map_global_nd_weak_->points.size();
    pcl::PointCloud<PointType>::Ptr points_added_to_strong(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr points_to_be_new_weak(new pcl::PointCloud<PointType>);
    for (std::size_t pt_idx = 0; pt_idx < num_points_of_weak_nd; pt_idx++)
    {
        std::vector<int> topk_indexes_map;
        std::vector<float> topk_L2dists_map;

        // hard coding for fast test 
        int kNumKnnPointsToCompare = 2;
        float kScanKnnAndMapKnnAvgDiffThreshold = 1.0; // meter, should be larger than the knn diff's threshold

        kdtree_map_global_nd_->nearestKSearch(map_global_nd_weak_->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);   
        float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
        float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

        if (std::abs(avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold) // if this weak point have strong friend
            points_added_to_strong->push_back(map_global_nd_weak_->points[pt_idx]);
        else
            points_to_be_new_weak->push_back(map_global_nd_weak_->points[pt_idx]);
    }

    // update 
    *map_global_nd_strong_ += *points_added_to_strong; // note that not = but += (append)
    *map_global_nd_weak_ = *points_to_be_new_weak; // note that not += but = (remake)
}


void Session::extractHighDynPointsViaKnnDiff(const pcl::PointCloud<PointType>::Ptr _target_map)
{
    kdtree_target_map_global_->setInputCloud(_target_map);
    std::map<int, pcl::PointCloud<PointType>::Ptr> keyframe_scans_dynamic_map;
    #pragma omp parallel for num_threads(kNumOmpCores)
    for (std::size_t idx_scan = 0; idx_scan < keyframe_scans_.size(); idx_scan++) {
        auto [unused, this_scan_diff] = partitionHighDynamicPointsOfScanByKnn(idx_scan);
        mtx.lock();
        keyframe_scans_dynamic_map[int(idx_scan)] = this_scan_diff;
        mtx.unlock();
    }
  
    keyframe_scans_dynamic_.clear();
    keyframe_scans_dynamic_.reserve(keyframe_scans_.size());

    for(auto& _elm: keyframe_scans_dynamic_map) 
        keyframe_scans_dynamic_.emplace_back(_elm.second);
}

void Session::precleaningKeyframes ( float _radius )
{
    for (std::size_t idx_kf = 0; idx_kf < keyframe_scans_.size(); idx_kf++) 
    {
        auto scan = keyframe_scans_.at(idx_kf); // pcl::PointCloud<PointType>::Ptr

        int num_points = scan->points.size();
        pcl::PointCloud<PointType>::Ptr scan_cleaned(new pcl::PointCloud<PointType>());
        for (int pt_idx = 0; pt_idx < num_points; ++pt_idx) 
        {

            PointType this_point = scan->points[pt_idx];
            SphericalPoint sph_point = cart2sph(this_point);
            float this_point_range = sph_point.r;
            float this_point_z = this_point.z;

            if( this_point_range < _radius 
                & this_point_z < 0.5 
                & -0.5 < this_point_z ) {
                continue;
            }

            scan_cleaned->push_back(this_point);
        }

        *scan = *scan_cleaned;
    }
} // precleaningKeyframes


// note: using pcPtr = pcl::PointCloud<PointType>::Ptr; // in utility.h
std::pair<pcPtr, pcPtr> Session::partitionLowDynamicPointsOfScanByKnn(int _scan_idx)
{
    // curr scan (in local coord)
    // pcl::PointCloud<PointType>::Ptr scan = keyframe_scans_static_projected_.at(_scan_idx);
    auto scan = keyframe_scans_static_projected_.at(_scan_idx);
    auto scan_pose = keyframe_poses_.at(_scan_idx);

    // curr scan (in global coord)
    pcl::PointCloud<PointType>::Ptr scan_global = local2global(scan, scan_pose, kSE3MatExtrinsicPoseBasetoLiDAR);
    // kdtree_scan_global_curr_->setInputCloud(scan_global);
    int num_points_of_a_scan = scan_global->points.size();

    pcl::PointCloud<PointType>::Ptr scan_eff_for_knn_comp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scan_eff_to_parse_in_cetral_frame(new pcl::PointCloud<PointType>());
    bool useICPrefinement { false }; // TODO: need to be tested more ... in current version not using it is better.  
    if(useICPrefinement) {
        mtx.lock();
        cout << "Knn diff of " << keyframe_names_.at(_scan_idx) << " of " << sess_type_ << " (with ICP refinement) ..." << endl;
        mtx.unlock();

        pcl::PointCloud<PointType>::Ptr scan_global_down(new pcl::PointCloud<PointType>());
        float icpVoxelSize = 0.4; // meter
        octreeDownsampling(scan_global, scan_global_down, icpVoxelSize);
        icp_for_knn_.setInputSource(scan_global_down);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp_for_knn_.align(*unused_result);

        pcl::PointCloud<PointType>::Ptr scan_global_reg(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*scan_global, *scan_global_reg, icp_for_knn_.getFinalTransformation());

        // pcl::io::savePCDFileBinary("/home/user/Desktop/data/rss21-ltmapper/mulran/dcc/01/removert/0102refactor/map.pcd", *target_map_down_for_knn_);
        // pcl::io::savePCDFileBinary("/home/user/Desktop/data/rss21-ltmapper/mulran/dcc/01/removert/0102refactor/scan.pcd", *scan_global);

        cout << icp_for_knn_.getFitnessScore() << endl;
        if (icp_for_knn_.getFitnessScore() < 1.0) {
            *scan_eff_for_knn_comp = *scan_global_reg;
            *scan_eff_to_parse_in_cetral_frame = *scan_global_reg;
        } else {
            *scan_eff_for_knn_comp = *scan_global;
            *scan_eff_to_parse_in_cetral_frame = *scan_global;
        }
    } else {
        mtx.lock();
        cout << "Knn diff of " << keyframe_names_.at(_scan_idx) << " of " << sess_type_ << " (without ICP refinement) ..." << endl;
        mtx.unlock();
        *scan_eff_for_knn_comp = *scan_global;
        *scan_eff_to_parse_in_cetral_frame = *scan_global;
    }

    pcl::PointCloud<PointType>::Ptr scan_knn_coexist(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr scan_knn_diff(new pcl::PointCloud<PointType>);
    for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
    {
        std::vector<int> topk_indexes_map;
        std::vector<float> topk_L2dists_map;
        kdtree_target_map_global_->nearestKSearch(scan_eff_for_knn_comp->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
        float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
        float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

        if (std::abs(avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold)
            scan_knn_coexist->push_back(scan_eff_to_parse_in_cetral_frame->points[pt_idx]);
        else
            scan_knn_diff->push_back(scan_eff_to_parse_in_cetral_frame->points[pt_idx]);
    }

    // again global2local because later in the merging global map function, which requires scans within each local coord.
    pcl::PointCloud<PointType>::Ptr scan_knn_coexist_local = global2local(scan_knn_coexist, keyframe_inverse_poses_.at(_scan_idx), kSE3MatExtrinsicPoseBasetoLiDAR);
    pcl::PointCloud<PointType>::Ptr scan_knn_diff_local = global2local(scan_knn_diff, keyframe_inverse_poses_.at(_scan_idx), kSE3MatExtrinsicPoseBasetoLiDAR);

    return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>(scan_knn_coexist_local, scan_knn_diff_local);
} // partitionLowDynamicPointsOfScanByKnn


std::pair<pcPtr, pcPtr> Session::partitionHighDynamicPointsOfScanByKnn(int _scan_idx)
{
    // curr scan (in local coord)
    // pcl::PointCloud<PointType>::Ptr scan = keyframe_scans_static_projected_.at(_scan_idx);
    auto scan = keyframe_scans_.at(_scan_idx);
    auto scan_pose = keyframe_poses_.at(_scan_idx);

    // curr scan (in global coord)
    pcl::PointCloud<PointType>::Ptr scan_global = local2global(scan, scan_pose, kSE3MatExtrinsicPoseBasetoLiDAR);
    // kdtree_scan_global_curr_->setInputCloud(scan_global);
    int num_points_of_a_scan = scan_global->points.size();
    pcl::PointCloud<PointType>::Ptr scan_knn_coexist(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr scan_knn_diff(new pcl::PointCloud<PointType>);
    for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
    {
        std::vector<int> topk_indexes_map;
        std::vector<float> topk_L2dists_map;
        kdtree_target_map_global_->nearestKSearch(scan_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
        float sum_topknn_dists_in_map = accumulate(topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
        float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

        if (std::abs(avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold)
            scan_knn_coexist->push_back(scan_global->points[pt_idx]);
        else
            scan_knn_diff->push_back(scan_global->points[pt_idx]);
    }

    // again global2local because later in the merging global map function, which requires scans within each local coord.
    pcl::PointCloud<PointType>::Ptr scan_knn_coexist_local = global2local(scan_knn_coexist, keyframe_inverse_poses_.at(_scan_idx), kSE3MatExtrinsicPoseBasetoLiDAR);
    pcl::PointCloud<PointType>::Ptr scan_knn_diff_local = global2local(scan_knn_diff, keyframe_inverse_poses_.at(_scan_idx), kSE3MatExtrinsicPoseBasetoLiDAR);

    return std::pair<pcPtr, pcPtr>(scan_knn_coexist_local, scan_knn_diff_local);
} // partitionHighDynamicPointsOfScanByKnn




} // namespace ltremovert