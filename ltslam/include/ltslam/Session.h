#pragma once

#include "ltslam/utility.h"
#include "ltslam/Scancontext.h"

using namespace LTslamParam;


struct Edge {
    int from_idx;
    int to_idx;
    gtsam::Pose3 relative;
};

struct Node {
    int idx;
    gtsam::Pose3 initial;
};

using SessionNodes = std::multimap<int, Node>; // from_idx, Edge
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge


class Session 
{
public:

    int index_;

    std::string name_;
    std::string session_dir_path_;

    bool is_base_session_;

    SessionNodes nodes_;
    SessionEdges edges_;

    int anchor_node_idx_;

    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D; // used for parsing submap represented in central coord
    pcl::PointCloud<PointTypePose>::Ptr originPoses6D;

    std::vector<pcl::PointCloud<PointType>::Ptr> cloudKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterICP;

    SCManager scManager;

public:
    Session();
    Session(int _idx, std::string _name, std::string _session_dir, bool _is_base_session);

    void loadSessionGraph();
    void loadSessionScanContextDescriptors();
    void loadSessionKeyframePointclouds();

    void initKeyPoses(void);
    void updateKeyPoses(const gtsam::ISAM2 * isam, const gtsam::Pose3& _anchor_transform);

    void loopFindNearKeyframesCentralCoord(
        pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
        const int& key, const int& searchNum);
    void loopFindNearKeyframesLocalCoord(
        pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
        const int& key, const int& searchNum);

    void allocateMemory();

}; // Session


// using Sessions = std::vector<Session>;
using Sessions = std::map<int, Session>;
using SessionsDict = std::map<std::string, Session>; // session name, session information

