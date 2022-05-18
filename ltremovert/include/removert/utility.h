#pragma once
#ifndef _UTILITY_REMOVERT_H_
#define _UTILITY_REMOVERT_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <utility>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <filesystem> // requires gcc version >= 8

namespace fs = std::filesystem;
using std::ios;
using std::cout;
using std::cerr;
using std::endl;

// struct PointXYZIS
// {
//     PCL_ADD_POINT4D
//     float intensity;
//     float score;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIS,  
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, score, score)
// )

using PointType = pcl::PointXYZI;
using pcPtr = pcl::PointCloud<PointType>::Ptr;

const float kFlagNoPOINT = 10000.0; // no point constant, 10000 has no meaning, but must be larger than the maximum scan range (e.g., 200 meters)
const float kValidDiffUpperBound = 200.0; // must smaller than kFlagNoPOINT

struct SphericalPoint
{
    float az; // azimuth 
    float el; // elevation
    float r; // radius
};

float rad2deg(float radians);
float deg2rad(float degrees);

void readBin(std::string _bin_path, pcl::PointCloud<PointType>::Ptr _pcd_ptr);

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter);

SphericalPoint cart2sph(const PointType & _cp);

std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio);

template<typename T>
cv::Mat convertColorMappedImg (const cv::Mat &_src, std::pair<T, T> _caxis)
{
  T min_color_val = _caxis.first;
  T max_color_val = _caxis.second;

  cv::Mat image_dst;
  image_dst = 255 * (_src - min_color_val) / (max_color_val - min_color_val);
  image_dst.convertTo(image_dst, CV_8UC1);
  
  cv::applyColorMap(image_dst, image_dst, cv::COLORMAP_JET);

  return image_dst;
}

pcl::PointCloud<PointType>::Ptr parseProjectedPoints(const pcl::PointCloud<PointType>::Ptr &_scan,
                                                                    const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                                                                    const std::pair<int, int> _rimg_size);

std::pair<cv::Mat, cv::Mat> map2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                                                        const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                                                        const std::pair<int, int> _rimg_size);

void transformGlobalMapToLocal(
    const pcl::PointCloud<PointType>::Ptr &_map_global,
    Eigen::Matrix4d _base_pose_inverse, Eigen::Matrix4d _base2lidar, 
    pcl::PointCloud<PointType>::Ptr &_map_local);

void octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &_src, pcl::PointCloud<PointType>::Ptr &_to_save);
void octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &_src, pcl::PointCloud<PointType>::Ptr &_to_save,
                                    const float _kDownsampleVoxelSize);

std::set<int> convertIntVecToSet(const std::vector<int> & v);

pcl::PointCloud<PointType>::Ptr mergeScansWithinGlobalCoordUtil(
    const std::vector<pcl::PointCloud<PointType>::Ptr> &_scans,
    const std::vector<Eigen::Matrix4d> &_scans_poses, Eigen::Matrix4d _lidar2base);

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &_scan_local, 
                                const Eigen::Matrix4d &_scan_pose, const Eigen::Matrix4d &_base2lidar);

pcl::PointCloud<PointType>::Ptr global2local(const pcl::PointCloud<PointType>::Ptr &_scan_global, 
                                const Eigen::Matrix4d &_scan_pose_inverse, const Eigen::Matrix4d &_base2lidar);

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img);

void pubRangeImg(cv::Mat& _rimg, sensor_msgs::ImagePtr& _msg, image_transport::Publisher& _publiser, std::pair<float, float> _caxis);
void publishPointcloud2FromPCLptr(const ros::Publisher& _scan_publisher, const pcl::PointCloud<PointType>::Ptr _scan);
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

float pointDistance(PointType p);
float pointDistance(PointType p1, PointType p2);

#endif