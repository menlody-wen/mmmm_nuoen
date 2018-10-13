#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class GroundPlanefitBase
{
  public:
    /** @brief Init of GroundPlanefitBase.
        @param[in] is_crop,crop_area_min,crop_area_max param decide if erase points around lidar,
        because points near lidar always affect ground plane fit
        @param[in] abs_ground_height the abslute ground height(z value)
        @ingroup common
        */
    GroundPlanefitBase( const float &planefit_thre = 0.2,
                        const float &abs_ground_height = -1.7,
                       const float &ground_thre = 0.2);
    ~GroundPlanefitBase() {}

    /**@brief generate a gound plane for input cloud
       @param[in] in_cloud_ptr input cloud data
       */
    bool genPlane(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

    /**@brief for the ground plane ,wei can estimate the gound height(z value)
       @param[in] locate input location point(only x and y will be used)
       @param[out] locateZ the estimate gound plane height(z value)
       */
    bool computeLocateZ(const pcl::PointXYZI &locate, float &locateZ);

    /**@brief label the valid indice for the input cloud
       @param[out] label_mat the output label mat, same strut like input cloud data
       */
    bool getLabelMat(cv::Mat &label_mat);

    /**@brief get the ground plane model
       @param[out] plane_model_coefficients the output ground plane model
       */
    bool getGroundPlaneModel(Eigen::VectorXf &plane_model_coefficients);

    /**@brief get the no ground points
       @param[out] out_cloud_ptr the no ground points ptr
       */
    bool computeNoground(pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);

  protected:
    float planefit_thre_, abs_ground_height_,ground_thre_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ori_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr proc_cloud_ptr_;
    bool has_input_;
    cv::Mat label_mat_;
    Eigen::VectorXf plane_model_coefficients_;

  private:
    float computeXYDis(const pcl::PointXYZI &pt);
    float computBeta(const pcl::PointXYZI &pt);
    bool isInvalidPoint(const pcl::PointXYZI &pt);
    // bool computeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2);
};