#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
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
#include "ground_detect.h"
#include "cluster.h"
#include "boxer.h"
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>
#include "tracker.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "fusion.h"

ros::NodeHandle *nh;
ros::Publisher points_pub, points_ground_pub, pub_marker, raw_points_pub;
GroundPlanefitBase *GroundPlanefitor;
PolarGridBase *PolarGridBasor;
BoundingBoxCalculator *boxer;
Track *Trackor;
Fusion *Fusionor;
Eigen::Matrix3f car_pose_;

void Callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    pcl::console::TicToc tt;
    tt.tic();
    double frame_time = points_msg->header.stamp.toSec();
    std::cout.precision(20);
    std::cout<<"the points time is "<<frame_time<<std::endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*points_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_raw);
    GroundPlanefitor->genPlane(cloud_raw);
    GroundPlanefitor->computeNoground(noground_cloud, ground_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    PolarGridBasor->polarGridCluster(noground_cloud, clusters);
    std::vector<BoundingBoxCalculator::BoundingBox> boxes;
    std::vector<tracker> trackers;
    //simple classify the object
    for (int i = 0; i < clusters.size(); ++i)
    {
        BoundingBoxCalculator::BoundingBox tmp_box = boxer->calcBoundingBox(clusters[i]);
        if (tmp_box.size.x * tmp_box.size.y > 18)
            continue;
        Eigen::Vector2f v1(tmp_box.center.x, tmp_box.center.y);
        float distance = v1.norm();
        // if (tmp_box.size.z > 2.5 && distance > 8)
        //     continue;
        if (tmp_box.size.x / tmp_box.size.y > 4 && tmp_box.size.y < 0.4)
            continue;
        boxes.push_back(tmp_box);
        tracker tmp_track;
        tmp_track.kalman_init();
        tmp_track.num_points = clusters[i].points.size();
        tmp_track.center[0] = tmp_box.center.x;
        tmp_track.center[1] = tmp_box.center.y;
        tmp_track.center[2] = 0;
        tmp_track.size.x = tmp_box.size.x;
        tmp_track.size.y = tmp_box.size.y;
        tmp_track.size.z = tmp_box.size.z;
        tmp_track.corners[0] = tmp_box.corners[0];
        tmp_track.corners[1] = tmp_box.corners[1];
        tmp_track.corners[2] = tmp_box.corners[2];
        tmp_track.corners[3] = tmp_box.corners[3];
        tmp_track.corners[4] = tmp_box.corners[4];
        tmp_track.corners[5] = tmp_box.corners[5];
        tmp_track.corners[6] = tmp_box.corners[6];
        tmp_track.corners[7] = tmp_box.corners[7];
        tmp_track.latest_tracked_time = frame_time;
        trackers.push_back(tmp_track);
    }
    int box_count = 0;
    visualization_msgs::MarkerArray markers;
    markers.markers.clear();
    for (int i = 0; i < boxes.size(); ++i)
    {
        BoundingBoxCalculator::BoundingBox box = boxes[i];
        visualization_msgs::Marker marker1;
        boxer->draw_box(box, box_count++, marker1, 1.1);
        cv::Point3f pos;
        pos.x = box.center.x;
        pos.y = box.center.y;
        pos.z = box.center.z + 1.7;
        std::string info;
        float area = box.size.x * box.size.y;
        if (area > 1.5 && area < 6)
        {
            info = "Car";
            trackers[i].class_type = 1;
        }
        else if (area >= 6)
        {
            info = "Truck";
            trackers[i].class_type = 2;
        }
        else if (area > 0.5 && area <= 1.5)
        {
            info = "Pedestrian";
            trackers[i].class_type = 3;
        }
        else
        {
            info = "Unkown";
            trackers[i].class_type = 4;
        }
        visualization_msgs::Marker marker2;
        boxer->draw_text(pos, info, box_count++, marker2);
        markers.markers.push_back(marker1);
        markers.markers.push_back(marker2);
    }
    Trackor->setNewObjects(trackers,frame_time);
    std::vector<tracker> new_trackers;
    Trackor->getObjects(new_trackers);
    std::cout<<"the size of trackers is "<<new_trackers.size()<<std::endl;
    Fusionor->getLidarTrackers(new_trackers);
    PolarGridBasor->clusterstoColor(clusters, color_cloud);
    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(color_cloud, output_points);
    output_points.header.frame_id = "nuoen";
    points_pub.publish(output_points);

    sensor_msgs::PointCloud2 output_ground_points;
    pcl::toROSMsg(*ground_cloud, output_ground_points);
    output_ground_points.header.frame_id = "nuoen";
    points_ground_pub.publish(output_ground_points);

    sensor_msgs::PointCloud2 tmp_msg;
    tmp_msg = *points_msg;
    tmp_msg.header.frame_id = "nuoen";
    raw_points_pub.publish(tmp_msg);

    pub_marker.publish(markers);

    std::cout << "the cost time of one frame is " << tt.toc() << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(clusters);
    std::vector<BoundingBoxCalculator::BoundingBox>().swap(boxes);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_localization");
    nh = new ros::NodeHandle;
    ros::NodeHandle pri_nh("~");
    float planefit_thre;
    pri_nh.getParam("planefit_thre", planefit_thre);
    float abs_ground_height;
    pri_nh.getParam("abs_ground_height", abs_ground_height);
    float ground_thre;
    pri_nh.getParam("ground_thre", ground_thre);

    std::string sub_points_topic;
    pri_nh.getParam("sub_points_topic", sub_points_topic);
    std::string pub_points_topic;
    pri_nh.getParam("pub_points_topic", pub_points_topic);
    std::string pub_ground_points_topic;
    pri_nh.getParam("pub_ground_points_topic", pub_ground_points_topic);
    std::string pub_raw_points_topic;
    pri_nh.getParam("pub_raw_points_topic", pub_raw_points_topic);
    std::string pub_array_topic;
    pri_nh.getParam("pub_array_topic", pub_array_topic);
    std::string sub_radar_topic;
    pri_nh.getParam("sub_radar_topic", sub_radar_topic);


    GroundPlanefitor = new GroundPlanefitBase(planefit_thre, abs_ground_height, ground_thre);
    PolarGridBasor = new PolarGridBase();
    Trackor = new Track(*nh);
    Fusionor = new Fusion(*nh,sub_radar_topic);
    boxer = new BoundingBoxCalculator;
    boxer->init();

    points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_points_topic, 1, true);
    points_ground_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_ground_points_topic, 1, true);
    raw_points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_raw_points_topic, 1, true);
    pub_marker = nh->advertise<visualization_msgs::MarkerArray>(pub_array_topic, 1, true);

    ros::Subscriber points_sub;
    points_sub = nh->subscribe(sub_points_topic, 10, Callback);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(*nh, sub_points_topic, 1);
    // message_filters::Subscriber<nav_msgs::Odometry> odo_sub(*nh, "/rs_pose", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), points_sub, odo_sub);
    // sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();
    return 0;
}