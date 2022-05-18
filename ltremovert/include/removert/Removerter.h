#pragma once

#include "removert/RosParamServer.h"
#include "removert/Session.h"

namespace ltremovert 
{

class Removerter : public RosParamServer
{
private:
    
    ros::Subscriber subLaserCloud; // TODO, for real-time integration (later)

    Session central_sess_;
    Session query_sess_;

    std::string updated_scans_save_dir_;
    std::string updated_strong_scans_save_dir_;
    std::string pd_scans_save_dir;
    std::string strong_pd_scans_save_dir;
    std::string strong_nd_scans_save_dir;

    std::string central_map_static_save_dir_;
    std::string central_map_dynamic_save_dir_;

    pcl::IterativeClosestPoint<PointType, PointType> icp_;

    // unuseds
    pcl::PointCloud<PointType>::Ptr map_subset_global_curr_; // unused 
    pcl::PointCloud<PointType>::Ptr map_global_accumulated_static_; // TODO, the S_i after reverted
    pcl::PointCloud<PointType>::Ptr map_global_accumulated_dynamic_;  // TODO, the D_i after reverted 

    std::vector<pcl::PointCloud<PointType>::Ptr> static_map_global_history_; // TODO
    std::vector<pcl::PointCloud<PointType>::Ptr> dynamic_map_global_history_; // TODO

    // configs 
    std::pair<int, int> curr_rimg_shape_;

    float curr_res_alpha_; // just for tracking current status

    const int base_node_idx_ = 0;

    unsigned long kPauseTimeForClearStaticScanVisualization = 1000; // microsec

    // NOT recommend to use for under 5 million points map input (becausing not-using is just faster)
    const bool kUseSubsetMapCloud = false; 
    const float kBallSize = 80.0; // meter

    // for viz
    image_transport::ImageTransport ROSimg_transporter_;

    sensor_msgs::ImagePtr scan_rimg_msg_;
    image_transport::Publisher scan_rimg_msg_publisher_;

    sensor_msgs::ImagePtr map_rimg_msg_;
    image_transport::Publisher map_rimg_msg_publisher_;

    sensor_msgs::ImagePtr diff_rimg_msg_;
    image_transport::Publisher diff_rimg_msg_publisher_;

    sensor_msgs::ImagePtr map_rimg_ptidx_msg_;
    image_transport::Publisher map_rimg_ptidx_msg_publisher_;

 
public:
    Removerter();
    ~Removerter();

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg); // TODO (if removert run online)

    void allocateMemory();

    void loadSessionInfo( void );
    void parseKeyframes( void );
    void loadKeyframes( void );
    void precleaningKeyframes(float _radius);

    // void readValidScans();

    void selfRemovert(const Session& _sess, int _repeat);
    void removeHighDynamicPoints(void);
    void detectLowDynamicPoints(void);

    void mergeScansWithinGlobalCoord( 
            const std::vector<pcl::PointCloud<PointType>::Ptr>& _scans, 
            const std::vector<Eigen::Matrix4d>& _scans_poses,
            pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );

    void makeGlobalMap(Session& _sess);
    void makeGlobalMap();

    void updateCurrentMap(void); // the updated one is saved in the central session's class

    void run(void);

    void parseStaticScansViaProjection(Session& _sess);
    void parseStaticScansViaProjection(void);

    std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>
        partitionCurrentMap(const Session& _target_sess, const Session& _source_sess, float _res_alpha);

    void removeOnce(const Session& _target_sess, const Session& _source_sess, float _res_alpha);
    void removeOnce(float _res);

    void revertOnce( const Session& _target_sess, const Session& _source_sess, float _res_alpha );
    void revertOnce(float _res);

    void resetCurrrentMapAsDynamic(const Session& _sess, bool _as_dynamic);
    void resetCurrrentMapAsDynamic(const Session& _sess);
    void resetCurrrentMapAsStatic(const Session& _sess);

    void saveCurrentStaticMapHistory(void); // the 0th element is a noisy (original input) (actually not static) map.
    void saveCurrentDynamicMapHistory(void);

    void takeGlobalMapSubsetWithinBall( int _center_scan_idx );
    void transformGlobalMapSubsetToLocal(int _base_scan_idx);

    cv::Mat scan2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                        const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                        const std::pair<int, int> _rimg_size);
    // std::pair<cv::Mat, cv::Mat> map2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
    //                     const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
    //                     const std::pair<int, int> _rimg_size);
    pcl::PointCloud<PointType>::Ptr parseProjectedPoints(const pcl::PointCloud<PointType>::Ptr& _scan, 
                        const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                        const std::pair<int, int> _rimg_size);

    std::vector<int> calcDescrepancyAndParseDynamicPointIdx(
        const cv::Mat& _scan_rimg, const cv::Mat& _diff_rimg, const cv::Mat& _map_rimg_ptidx, float _diff_thres = 0.1);

    std::vector<int> calcDescrepancyAndParseDynamicPointIdxForEachScan( std::pair<int, int> _rimg_shape );
    std::vector<int> calcDescrepancyAndParseDynamicPointIdxForEachScan( 
        const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape);

    std::vector<int> getStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes, int _num_all_points);
    std::vector<int> getGlobalMapStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes);
    std::vector<int> getGlobalMapStaticIdxFromDynamicIdx(const pcl::PointCloud<PointType>::Ptr& _map_global_curr, 
                                                                    std::vector<int> &_dynamic_point_indexes);
   
    void parsePointcloudSubsetUsingPtIdx( const pcl::PointCloud<PointType>::Ptr& _ptcloud_orig,
            std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );
    void parseMapPointcloudSubsetUsingPtIdx( std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );
    void parseStaticMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes );
    void parseDynamicMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes );

    void saveCurrentStaticAndDynamicPointCloudGlobal( void );
    void saveCurrentStaticAndDynamicPointCloudLocal( int _base_pose_idx  = 0);
    void saveCurrentStaticAndDynamicPointCloudGlobal(const Session& _sess, std::string _postfix);

    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& _scan_local, const Eigen::Matrix4d& _scan_pose);
    pcl::PointCloud<PointType>::Ptr global2local(const pcl::PointCloud<PointType>::Ptr& _scan_global, const Eigen::Matrix4d& _scan_pose_inverse);

    // scan-side removal
    std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> removeDynamicPointsOfScanByKnn ( int _scan_idx );
    std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> removeDynamicPointsOfScanByKnn( int _scan_idx, 
                                            const Session& _sess, const pcl::KdTreeFLANN<PointType>::Ptr& _kdtree_map_global_curr);
    std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> removeDynamicPointsOfScanByKnnBtnSession ( int _scan_idx );
    void removeDynamicPointsAndSaveStaticScanForEachScan( void );

    void scansideRemovalForEachScan(void);
    void scansideRemovalForEachScan(const Session& _map_sess, const Session& _scan_sess);
    void scansideRemovalViaMapReprojectionForEachScan(void);
    void saveCleanedScans(void);
    void saveCleanedScans(const Session& _sess);
    // void saveMapPointcloudByMergingCleanedScans(void);
    void saveMapPointcloudByMergingCleanedScans( const Session& _sess, std::string _postfix );
    void scansideRemovalForEachScanAndSaveThem(void);
    void scansideRemovalViaMapReprojectionForEachScanAndSaveThem(void);

    void saveStaticScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud );
    void saveDynamicScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud );

    void filterStrongND(Session& _sess_src, Session& _sess_cleaner);
    void iremoveOnceForND(const Session& _target_sess, const Session& _source_sess, float _res_alpha);
    std::pair<pcPtr, pcPtr> partitionCurrentMapForND(const Session& _target_sess, const Session& _source_sess, float _res_alpha);
    std::vector<int> calcDescrepancyAndParseDynamicPointIdxForEachScanForND(
        const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape);

    void filterStrongPD(Session& _sess_src, Session& _sess_cleaner);
    void removeOnceForPD(const Session& _target_sess, const Session& _source_sess, float _res_alpha);
    std::pair<pcPtr, pcPtr> partitionCurrentMapForPD(const Session& _target_sess, const Session& _source_sess, float _res_alpha);
    std::vector<int> calcDescrepancyAndParseDynamicPointIdxForEachScanForPD(
        const Session& _target_sess, const Session& _source_sess, std::pair<int, int> _rimg_shape);

    void parseUpdatedStaticScansViaProjection( void );
    void parseUpdatedStaticScansViaProjection(Session& _sess);

    void parseLDScansViaProjection();
    void parseLDScansViaProjection(Session& _sess);

    void updateScansScanwise(Session& _sess);
    void updateScansScanwise();

    void saveAllTypeOfScans();
    void saveUpdatedScans( Session& _sess );
    void saveLDScans( Session& _sess );
    void savePDScans( Session& _sess );
    void saveStrongPDScans( Session& _sess );
    void saveStrongNDScans( Session& _sess );
    void saveScans(Session& _sess, std::vector<pcl::PointCloud<PointType>::Ptr> _scans, std::string _save_dir);


}; // Removerter

} // namespace ltremovert 
