#include "ground_detect.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

GroundPlanefitBase::GroundPlanefitBase(const float &planefit_thre, const float &abs_ground_height, const float &ground_thre)
{
    planefit_thre_ = planefit_thre;
    abs_ground_height_ = abs_ground_height;
    ground_thre_ = ground_thre;

    std::cout << "the height is " << abs_ground_height << std::endl;
    ori_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    proc_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    has_input_ = false;
}

bool computeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return pt1.z < pt2.z;
}

bool GroundPlanefitBase::genPlane(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr)
{
    if (in_cloud_ptr->empty())
    {
        std::cerr << "the input cloud is empty!!" << std::endl;
        return false;
    }

    *ori_cloud_ptr_ = *in_cloud_ptr;

    int height = in_cloud_ptr->height;
    int width = in_cloud_ptr->width;

    label_mat_ = cv::Mat::zeros(height, width, CV_8U);

    //处理nan点和crop_area内的点,为了保持点云的有序性，自己写的相关算法
    proc_cloud_ptr_->clear();
    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            pcl::PointXYZI tmp_i = in_cloud_ptr->points[j * width + i];
            if (isInvalidPoint(tmp_i))
                continue;
            label_mat_.at<uchar>(j, i) = 1;
            proc_cloud_ptr_->push_back(tmp_i);
        }
    }

    //这里直接自己设定极坐标栅格的扇区大小，只是用于平面拟合，不用用户自己定义
    float delta_angle = 2.;
    float delta_rad = (delta_angle / 180.) * M_PI;
    int Msegs = (2 * M_PI) / delta_rad + 1;
    std::vector<std::vector<pcl::PointXYZI> > save_pts;

    //选择进行平面拟合的种子点的range
    save_pts.resize(Msegs);
    for (int i = 0; i < proc_cloud_ptr_->size(); ++i)
    {
        pcl::PointXYZI tmp_i = proc_cloud_ptr_->points[i];
        // float tmp_range = computeXYDis(tmp_i);
        float tmp_beta = computBeta(tmp_i);
        int tmp_index = (tmp_beta / delta_rad);
        save_pts[tmp_index].push_back(tmp_i);
    }

    for (int j = 0; j < save_pts.size(); ++j)
    {
        std::sort(save_pts[j].begin(), save_pts[j].end(), computeZ);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_plane_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j < save_pts.size(); ++j)
    {
        if (save_pts[j].empty())
            continue;
        float tmp_lowest = save_pts[j][0].z;

        for (int i = 0; i < save_pts[j].size(); ++i)
        {
            if (save_pts[j][i].z - tmp_lowest < planefit_thre_ && save_pts[j][i].z < (abs_ground_height_ + planefit_thre_))
            {
                tmp_plane_ptr->push_back(save_pts[j][i]);
            }
            else
            {
                break;
            }
        }
    }
    std::vector<std::vector<pcl::PointXYZI> >().swap(save_pts);
    if (tmp_plane_ptr->size() < 5)
    {
        std::cout << "too less points to fit a plane!" << std::endl;
        return false;
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_plane_ptr));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(0.05);
    ransac.computeModel();
    ransac.getModelCoefficients(plane_model_coefficients_);

    if (plane_model_coefficients_(2) < 0.)
    {
        plane_model_coefficients_(0) *= -1;
        plane_model_coefficients_(1) *= -1;
        plane_model_coefficients_(2) *= -1;
        plane_model_coefficients_(3) *= -1;
    }
    has_input_ = true;

    return true;
}

bool GroundPlanefitBase::isInvalidPoint(const pcl::PointXYZI &pt)
{
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) || std::isnan(pt.intensity) ||
        ((fabs(pt.x) < 1.e-6) && (fabs(pt.y) < 1.e-6) && (fabs(pt.z) < 1.e-6)))
    {
        return true;
    }
    return false;
}

float GroundPlanefitBase::computBeta(const pcl::PointXYZI &pt)
{
    float beta = atan2(pt.y, pt.x);
    beta = beta < 0. ? beta += 2 * M_PI : beta;
    beta = beta > (2 * M_PI) ? beta -= 2 * M_PI : beta;
    return beta;
}

float GroundPlanefitBase::computeXYDis(const pcl::PointXYZI &pt)
{
    return sqrtf(powf(pt.x, 2.) + powf(pt.y, 2.));
}

bool GroundPlanefitBase::getGroundPlaneModel(Eigen::VectorXf &plane_model_coefficients)
{
    if (!has_input_)
    {
        std::cout << "no input cloud!" << std::endl;
        return false;
    }
    plane_model_coefficients = plane_model_coefficients_;
    return true;
}

bool GroundPlanefitBase::computeNoground(pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr)
{
    if (!has_input_)
    {
        std::cout << "no input cloud!" << std::endl;
        return false;
    }
    float planeA = plane_model_coefficients_(0);
    float planeB = plane_model_coefficients_(1);
    float planeC = plane_model_coefficients_(2);
    float planeD = plane_model_coefficients_(3);

    for (int i = 0; i < proc_cloud_ptr_->size(); ++i)
    {
        pcl::PointXYZI locate = proc_cloud_ptr_->points[i];
        float locateZ = -(planeD + locate.x * planeA + locate.y * planeB) / planeC;
        if (locate.z > locateZ + 0.2)
        {
            out_cloud_ptr->push_back(locate);
        }
        else
        {
            in_cloud_ptr->push_back(locate);
        }
    }
    if (out_cloud_ptr->size() > 10)
        return true;
    else
        return false;
}

bool GroundPlanefitBase::getLabelMat(cv::Mat &label_mat)
{
    if (!has_input_)
    {
        std::cout << "no input cloud!" << std::endl;
        return false;
    }
    label_mat = label_mat_;
    return true;
}