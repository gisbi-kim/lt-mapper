#pragma once

#include "removert/RosParamServer.h"
#include "removert/utility.h"

namespace ltremovert 
{

class Session : public RosParamServer
{
public:

    const float kReprojectionAlpha = 3.0;

    std::string sess_type_;

    // config
    float kDownsampleVoxelSize;
    
    // all 
    std::string scan_dir_;
    std::string pose_path_;
    
    // including all scans inforamtion in the session
    std::vector<std::string> scan_names_;
    std::vector<std::string> scan_paths_;
    std::vector<Eigen::Matrix4d> scan_poses_; // including all scans inforamtion in the session
    std::vector<Eigen::Matrix4d> scan_inverse_poses_;
    int num_scans_; 

    // keyframes (for efficient processing)
    int keyframe_gap_;
    std::vector<std::string> keyframe_names_;
    std::vector<std::string> keyframe_paths_;
    std::vector<Eigen::Matrix4d> keyframe_poses_;
    std::vector<Eigen::Matrix4d> keyframe_inverse_poses_;
    int num_keyframes_; 

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_;
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_static_;

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_static_projected_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_dynamic_; 

    pcl::PointCloud<PointType>::Ptr target_map_down_for_knn_;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_knn_coexist_; // keyframes, between session
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_knn_diff_; // keyframes, between session

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_updated_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_updated_strong_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_pd_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_strong_pd_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_strong_nd_; //keyframes
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_scans_weak_nd_; //keyframes

    // 
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_global_curr_; 

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_target_map_global_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_scan_global_curr_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_global_nd_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_global_pd_;

    pcl::PointCloud<PointType>::Ptr map_global_orig_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_; // the M_i. i.e., removert is: M1 -> S1 + D1, D1 -> M2 , M2 -> S2 + D2 ... repeat ... 
    pcl::PointCloud<PointType>::Ptr map_local_curr_;
    pcl::PointCloud<PointType>::Ptr map_global_curr_static_; // the S_i
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic_;  // the D_i

    pcl::PointCloud<PointType>::Ptr map_global_updated_;
    pcl::PointCloud<PointType>::Ptr map_global_updated_strong_;

    pcl::PointCloud<PointType>::Ptr map_global_curr_diff_positive_; // the S_i
    pcl::PointCloud<PointType>::Ptr map_global_curr_diff_negative_;  // the D_i
    pcl::PointCloud<PointType>::Ptr map_global_curr_down_for_icp_;

    pcl::PointCloud<PointType>::Ptr map_global_nd_;
    pcl::PointCloud<PointType>::Ptr map_local_nd_;
    pcl::PointCloud<PointType>::Ptr map_global_nd_strong_;
    pcl::PointCloud<PointType>::Ptr map_global_nd_weak_; // == ambigous 

    pcl::PointCloud<PointType>::Ptr map_global_pd_;
    pcl::PointCloud<PointType>::Ptr map_global_pd_orig_;
    pcl::PointCloud<PointType>::Ptr map_local_pd_;
    pcl::PointCloud<PointType>::Ptr map_global_pd_strong_;
    pcl::PointCloud<PointType>::Ptr map_global_pd_weak_;  

    pcl::IterativeClosestPoint<PointType, PointType> icp_for_knn_;

    std::mutex mtx;

public:
    Session( void );

    void allocateMemory();

    void loadSessionInfo( std::string _sess_type, std::string _scan_dir, std::string _pose_path );
    void setDownsampleSize( float _voxel_size );

    void clearKeyframes( void );

    void parseKeyframes( int _gap = 1 );
    void parseKeyframes( std::pair<int, int> _range, int _gap = 1 );
    void parseKeyframesInROI( const std::vector<Eigen::Matrix4d>& _roi_poses, int _gap = 1 );

    void loadKeyframes( void );
    void precleaningKeyframes ( float _radius );
    void mergeScansWithinGlobalCoord(void);

    void parseStaticScansViaProjection(void);
    void parseUpdatedStaticScansViaProjection(void);
    void parseUpdatedStrongStaticScansViaProjection(void);
    void parsePDScansViaProjection(void);
    void parseStrongPDScansViaProjection(void);
    void parseWeakNDScansViaProjection(void);
    void parseStrongNDScansViaProjection(void);    
    void parseScansViaProjection(
        pcl::PointCloud<PointType>::Ptr _map, // to_be_parsed
        std::vector<pcl::PointCloud<PointType>::Ptr>& _vec_to_store );

    void updateScansScanwise();

    void extractLowDynPointsViaKnnDiff(const pcl::PointCloud<PointType>::Ptr _target_map);
    std::pair<pcPtr, pcPtr> partitionLowDynamicPointsOfScanByKnn(int _scan_idx);

    void extractHighDynPointsViaKnnDiff(const pcl::PointCloud<PointType>::Ptr _target_map);
    std::pair<pcPtr, pcPtr> partitionHighDynamicPointsOfScanByKnn(int _scan_idx);

    void constructGlobalNDMap();
    void removeWeakNDMapPointsHavingStrongNDInNear();

    void constructGlobalPDMap();
    void revertStrongPDMapPointsHavingWeakPDInNear();

};



} // namespace ltremovert