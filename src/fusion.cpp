#include "fusion.h"
#include "tracker.h"
#include "hungarian_bigraph_matcher.h"

Fusion::Fusion(ros::NodeHandle nh, std::string sub_radar_topic)
{
    radar_sub = nh.subscribe(sub_radar_topic, 10, &Fusion::radarCallback, (Fusion *)this);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("fusion_info", 1);
}


 void Fusion::radarCallback(const sensor_msgs::PointCloud2ConstPtr &radar_msg)
 {
    radar_objects_.clear();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*radar_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_raw);
    for(int i=0;i<cloud_raw->points.size();++i)
    {
        radar_object tmp_object;
        tmp_object.center<<cloud_raw->points[i].x,cloud_raw->points[i].y,0;
        tmp_object.track_id = cloud_raw->points[i].z;
        radar_objects_.push_back(tmp_object);
    }
    que_radar_objects_.push_back(radar_objects_);
    if(que_radar_objects_.size()>3)
    {
        que_radar_objects_.pop_front();
    }
 }


void Fusion::getLidarTrackers(std::vector<tracker> trackers)
{
    fusion_objects_.clear();
    for(int i=0;i<trackers.size();++i)
    {
        fusion_object tmp_object;
        tmp_object.num_points = trackers[i].num_points;
        tmp_object.corners[0] = trackers[i].corners[0];
        tmp_object.corners[1] = trackers[i].corners[1];
        tmp_object.corners[2] = trackers[i].corners[2];
        tmp_object.corners[3] = trackers[i].corners[3];
        tmp_object.corners[4] = trackers[i].corners[4];
        tmp_object.corners[5] = trackers[i].corners[5];
        tmp_object.corners[6] = trackers[i].corners[6];
        tmp_object.corners[7] = trackers[i].corners[7];
        tmp_object.center = trackers[i].center;
        tmp_object.size = trackers[i].size;
        tmp_object.track_id = trackers[i].track_id;
        tmp_object.velocity = trackers[i].velocity;
        tmp_object.class_type = trackers[i].class_type;
        tmp_object.fusion_radar_object["radar"] = nullptr;
        fusion_objects_.push_back(tmp_object);
    }
    if(que_radar_objects_.size()==0)
        return;
    std::vector<radar_object> last_frame_radar_objects = que_radar_objects_.back();
    std::vector<std::vector<double> > cost;
    computeAssociateMatrix(fusion_objects_,last_frame_radar_objects,cost);
    std::vector<int> fusions_idx;
    std::vector<int> objects_idx;
    //匈牙利算法匹配
    HungarianOptimizer hungarian_optimizer(cost);
    hungarian_optimizer.minimize(&fusions_idx, &objects_idx);
    std::vector<std::pair<int, int> > local_assignments;
    std::vector<int> local_unassigned_fusions;
    std::vector<int> local_unassigned_objects;
    local_assignments.resize(fusion_objects_.size());
    local_unassigned_fusions.assign(fusion_objects_.size(), -1);
    local_unassigned_objects.assign(last_frame_radar_objects.size(), -1);
    AssignObjectsToTracks(fusions_idx, objects_idx, cost, &local_assignments, &local_unassigned_fusions, &local_unassigned_objects);
    updateFusions(fusion_objects_,last_frame_radar_objects,local_assignments);
    int box_count = 0;
    visualization_msgs::MarkerArray markers;
    markers.markers.clear();
     for(int i = 0;i<fusion_objects_.size();++i)
    {
        if(fusion_objects_[i].center[0]>10||fusion_objects_[i].center[0]<-20)
            continue;
        visualization_msgs::Marker marker1,marker2,marker3;
        draw_box(fusion_objects_[i],box_count++,marker1);
        markers.markers.push_back(marker1);
    }
    std::cout<<"the size of fusion objects is "<<box_count<<std::endl;
    marker_pub.publish(markers);
}

void Fusion::updateFusions(std::vector<fusion_object> &fusion_objects,
                           const std::vector<radar_object> &radar_objects, std::vector<std::pair<int, int> > local_assignments)
{
    for(int i=0;i<local_assignments.size();++i)
    {
        std::pair<int, int> tmp_pair = local_assignments[i];
        std::shared_ptr<radar_object> tmp_radar(new radar_object(radar_objects[tmp_pair.second]));
        fusion_objects[tmp_pair.first].fusion_radar_object["radar"] = tmp_radar;
    }
}

void Fusion::computeAssociateMatrix(
    const std::vector<fusion_object> &fusion_objects,
    const std::vector<radar_object> &radar_objects,
    std::vector<std::vector<double> > &cost)
{
    // Compute matrix of association distance
    Eigen::MatrixXf association_mat(fusion_objects.size(), radar_objects.size());
    int no_fusion = fusion_objects.size();
    int no_object = radar_objects.size();
    for (size_t i = 0; i < fusion_objects.size(); ++i)
    {
        for (size_t j = 0; j < radar_objects.size(); ++j)
        {
            float diff_position = (fusion_objects[i].center - radar_objects[j].center).block(0,0,2,1).norm();
            float position_cost = diff_position / 3.;
            float sum_cost = position_cost;
            // std::cout<<"the i and j is "<<num_cost<<","<<position_cost<<","<<sum_cost<<std::endl;
            association_mat(i, j) = sum_cost;
        }
    }
    cost.resize(fusion_objects.size() + radar_objects.size());
    for (int i = 0; i < no_fusion; ++i)
    {
        cost[i].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            cost[i][j] = association_mat(i, j);
        }
    }
    for (int i = 0; i < no_object; ++i)
    {
        cost[i + no_fusion].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            if (j == i)
            {
                cost[i + no_fusion][j] = 999 * 1.2f;
            }
            else
            {
                cost[i + no_fusion][j] = 999999.0f;
            }
        }
    }
}


void Fusion::AssignObjectsToTracks(
    std::vector<int> tracks_idx,
    std::vector<int> objects_idx,
    std::vector<std::vector<double> > cost_value,
    std::vector<std::pair<int, int> > * assignments,
    std::vector<int> *unassigned_tracks, std::vector<int> *unassigned_objects)
{
    int assignments_num = 0;
    int no_track = cost_value.size() - cost_value[0].size();
    int no_object = cost_value[0].size();
    std::vector<bool> tracks_used(no_track+no_object, false);
    std::vector<bool> objects_used(no_object, false);
    for (size_t i = 0; i < tracks_idx.size(); ++i)
    {
        if (tracks_idx[i] < 0 || tracks_idx[i] >= no_track || objects_idx[i] < 0 ||
            objects_idx[i] >= no_object)
        {
            continue;
        }
        if (cost_value[tracks_idx[i]][objects_idx[i]] <1.)
        {
            (*assignments)[assignments_num++] =
                std::make_pair(tracks_idx[i], objects_idx[i]);
            tracks_used[tracks_idx[i]] = true;
            objects_used[objects_idx[i]] = true;
        }
    }
    assignments->resize(assignments_num);
    unassigned_tracks->resize(no_track);
    int unassigned_tracks_num = 0;
    for (int i = 0; i < no_track; ++i)
    {
        if (tracks_used[i] == false)
        {
            (*unassigned_tracks)[unassigned_tracks_num++] = i;
        }
    }
    unassigned_tracks->resize(unassigned_tracks_num);
    unassigned_objects->resize(no_object);
    int unassigned_objects_num = 0;
    for (int i = 0; i < no_object; ++i)
    {
        if (objects_used[i] == false)
        {
            (*unassigned_objects)[unassigned_objects_num++] = i;
        }
    }
    unassigned_objects->resize(unassigned_objects_num);
}



void Fusion::draw_box(const fusion_object &tmp_tracker, const int &marker_id, visualization_msgs::Marker &marker)
{

    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "nuoen";
    std::vector<geometry_msgs::Point> cub_points;

    for (int i = 0; i < 8; ++i)
    {
        geometry_msgs::Point pts;
        pts.x = tmp_tracker.corners[i].x;
        pts.y = tmp_tracker.corners[i].y;
        pts.z = tmp_tracker.corners[i].z;
        cub_points.push_back(pts);
    }
    std::shared_ptr<radar_object> obj = nullptr;
    auto it = tmp_tracker.fusion_radar_object.find("radar");
    if (it != tmp_tracker.fusion_radar_object.end())
    {
        obj = it->second;
    }
    if (obj == nullptr)
        marker.color.b = 1;
    else
    {
        marker.color.g = 1;
    }
    marker.scale.x = 0.1;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.1);

    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[0]);
    // horizontal high points for lines
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[4]);
    // vertical points for lines
    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[7]);
}
