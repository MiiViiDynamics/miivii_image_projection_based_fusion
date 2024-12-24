#ifndef MIIVII_IMAGE_PROJECTION_BASED_FUSION_PROJECTION_NODE_HPP_
#define MIIVII_IMAGE_PROJECTION_BASED_FUSION_PROJECTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cuda_runtime.h>

#include "autoware_point_types/types.hpp"

namespace miivii_image_projection_based_fusion
{
    class ProjectPointsToImageNode : public rclcpp::Node
    {
    public:
        ProjectPointsToImageNode(const rclcpp::NodeOptions &options);

    private:
        // 回调函数
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t image_id);
        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, const std::size_t camera_id);

        // 订阅者和发布者
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;

        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

        // 相机信息和点云数据
        std::vector<sensor_msgs::msg::CameraInfo> camera_infos_;
        boost::circular_buffer<sensor_msgs::msg::PointCloud2> pointcloud_buffer_;

        // 成员函数
        void preprocess();
        sensor_msgs::msg::Image projectPoints(sensor_msgs::msg::Image::ConstSharedPtr image, sensor_msgs::msg::PointCloud2 pointcloud, const std::size_t camera_id);
        void publish(sensor_msgs::msg::Image msg, const std::size_t camera_id);
        void publish(sensor_msgs::msg::PointCloud2 msg);

        std::size_t camera_num_{1};
        std::size_t pointcloud_num_{10};

        std::mutex camera_mutex_;
        // std::size_t threshold_ms_;

        std::string input_pointcloud_topic_;
        std::string output_image_topic_prefix_;
        std::vector<std::string> input_image_topics_;
        std::vector<std::string> input_camera_info_topics_;
        std::string output_pointcloud_topic_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    };

} // namespace miivii_image_projection_based_fusion

#endif // MIIVII_IMAGE_PROJECTION_BASED_FUSION_PROJECTION_NODE_HPP_