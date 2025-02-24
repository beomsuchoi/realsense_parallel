#ifndef REALSENSE_PARALLEL_HPP_
#define REALSENSE_PARALLEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace realsense_parallel
{

class ParallelDetector : public rclcpp::Node
{
public:
  ParallelDetector();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void detectPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;
  
  // RANSAC 파라미터
  double distance_threshold_;
  int max_iterations_;
};

}  // namespace realsense_parallel

#endif  // REALSENSE_PARALLEL_HPP_