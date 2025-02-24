#include "realsense_parallel/parallel.hpp"

namespace realsense_parallel
{

    ParallelDetector::ParallelDetector()
        : Node("parallel_detector"),
          distance_threshold_(0.01), // 1cm
          max_iterations_(1000)
    {
        // RealSense 카메라로부터 포인트클라우드 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&ParallelDetector::pointCloudCallback, this, std::placeholders::_1));

        // 검출된 평면 발행
        plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "detected_plane", 10);

        RCLCPP_INFO(this->get_logger(), "Parallel Detector node has been initialized");
    }

    void ParallelDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ROS 메시지를 PCL 포인트클라우드로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        detectPlane(cloud);
    }

    void ParallelDetector::detectPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold_);
        seg.setMaxIterations(max_iterations_);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model.");
            return;
        }

        // 평면의 법선 벡터
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];

        // 법선 벡터 정규화
        double norm = sqrt(a * a + b * b + c * c);
        a /= norm;
        b /= norm;
        c /= norm;

        // x축(pitch)과 y축(roll)에 대한 회전 각도 계산
        double pitch = atan2(b, c) * 180.0 / M_PI;
        double roll = atan2(a, c) * 180.0 / M_PI;

        // 회전 방향 안내 (각도 임계값을 5도로 설정)
        std::string pitch_direction = "";
        std::string roll_direction = "";

        if (std::abs(pitch) > 5.0)
        {
            pitch_direction = pitch > 0 ? "카메라를 뒤로 기울이세요" : "카메라를 앞으로 기울이세요";
        }
        if (std::abs(roll) > 5.0)
        {
            roll_direction = roll > 0 ? "카메라를 왼쪽으로 기울이세요" : "카메라를 오른쪽으로 기울이세요";
        }

        RCLCPP_INFO(this->get_logger(),
                    "Pitch: %.2f degrees, Roll: %.2f degrees",
                    pitch, roll);

        // 수평 상태 체크 (3도 이내면 수평으로 간주)
        if (std::abs(pitch) <= 5.0 && std::abs(roll) <= 5.0)
        {
            RCLCPP_INFO(this->get_logger(), "=== 수평 상태 ===");
            RCLCPP_INFO(this->get_logger(), "평면이 카메라와 거의 평행합니다!");
            RCLCPP_INFO(this->get_logger(), "==================");
        }
        else if (!pitch_direction.empty() || !roll_direction.empty())
        {
            RCLCPP_INFO(this->get_logger(), "조정 방향:");
            if (!pitch_direction.empty())
            {
                RCLCPP_INFO(this->get_logger(), "  앞뒤: %s", pitch_direction.c_str());
            }
            if (!roll_direction.empty())
            {
                RCLCPP_INFO(this->get_logger(), "  좌우: %s", roll_direction.c_str());
            }
        }
    }

} // namespace realsense_parallel

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<realsense_parallel::ParallelDetector>());
    rclcpp::shutdown();
    return 0;
}