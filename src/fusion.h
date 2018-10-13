#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
#include <memory>
#include <tr1/memory>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include "tracker_object.h"

struct object
{
    cv::Point3f corners[8]; /**< 8 corners of boundingbox */
    cv::Point3f size;       /**< x:length, y:width, z:height, notice that length >= width*/
    Eigen::Vector3f center; /**<box geometry center*/
    int track_id = 0;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    int class_type = 0;
};

struct radar_object : object
{
    
};

struct fusion_object:object
{
    int num_points;
    std::map<std::string,std::shared_ptr<radar_object> > fusion_radar_object;
};

class Fusion{
    public:
  Fusion(ros::NodeHandle nh, std::string file_path);
  ~Fusion() {}
  void getLidarTrackers(std::vector<tracker> trackers);

private:
  void radarCallback(const sensor_msgs::PointCloud2ConstPtr &radar_msg);
  void draw_box(const fusion_object &tmp_tracker, const int &marker_id, visualization_msgs::Marker &marker);
  void AssignObjectsToTracks(
      std::vector<int> tracks_idx,
      std::vector<int> objects_idx,
      std::vector<std::vector<double> > cost_value,
      std::vector<std::pair<int, int> > *assignments,
      std::vector<int> *unassigned_tracks, std::vector<int> *unassigned_objects);
  void computeAssociateMatrix(
      const std::vector<fusion_object> &fusion_objects,
      const std::vector<radar_object> &radar_objects,
      std::vector<std::vector<double> > &cost);
  void updateFusions(std::vector<fusion_object> &fusion_objects,
                     const std::vector<radar_object> &radar_objects, std::vector<std::pair<int, int> > local_assignments);
  ros::Subscriber radar_sub;
  ros::Publisher marker_pub;
  std::vector<radar_object> radar_objects_;
  std::deque<std::vector<radar_object> > que_radar_objects_;
  std::vector<fusion_object> fusion_objects_;
};