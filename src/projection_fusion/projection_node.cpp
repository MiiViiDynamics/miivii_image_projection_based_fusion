#include "miivii_image_projection_based_fusion/projection_fusion/projection_node.hpp"
#include "miivii_image_projection_based_fusion/utils/utils.hpp"

#include <cuda_runtime.h>
extern "C" void projectPointsOnGPU(
    float *pointcloud_data_host,          // Flattened point cloud data (x, y, z, intensity)
    int pointcloud_size,                  // Size of point cloud
    float *camera_projection_matrix_host, // Camera projection matrix (3x3)
    float *lidar2cam_affine_host,         // LiDAR to camera affine matrix (4x4)
    unsigned char *host_image,
    int width, int height // Image width and height
);

namespace
{
    Eigen::Affine3f _transformToEigen(const geometry_msgs::msg::Transform &t)
    {
        Eigen::Affine3f a;
        a.matrix() = tf2::transformToEigen(t).matrix().cast<float>();
        return a;
    }

} // namespace

namespace miivii_image_projection_based_fusion
{
    ProjectPointsToImageNode::ProjectPointsToImageNode(const rclcpp::NodeOptions &options)
        : Node("project_points_to_image_node", options)
    {

        camera_num_ = static_cast<std::size_t>(declare_parameter<int32_t>("camera_num"));
        if (camera_num_ < 1)
        {
            RCLCPP_WARN(
                this->get_logger(), "minimum camera_num_ is 1. current camera_num_ is %zu", camera_num_);
            camera_num_ = 1;
        }
        if (camera_num_ > 8)
        {
            RCLCPP_WARN(
                this->get_logger(), "maximum camera_num_ is 8. current camera_num_ is %zu", camera_num_);
            camera_num_ = 8;
        }

        // threshold_ms_ = static_cast<int>(this->declare_parameter<int32_t>("threshold_ms"));
        // if (threshold_ms_ == 0)
        // {
        //     threshold_ms_ = 100;
        //     RCLCPP_WARN(
        //         this->get_logger(), "threshold_ms not set or set to 0ms. set threshold_ms to %zu", threshold_ms_);
        // }

        // 从参数服务器获取话题名称
        input_pointcloud_topic_ = this->declare_parameter<std::string>("input_pointcloud_topic", "input_point_cloud");
        output_image_topic_prefix_ = this->declare_parameter<std::string>("output_image_topic_prefix", "output_image_topic_");
        output_pointcloud_topic_ = this->declare_parameter<std::string>("output_pointcloud_topic", "output_pointcloud");

        pointcloud_num_ = static_cast<int>(this->declare_parameter<int32_t>("pointcloud_buffer_size"));
        if (pointcloud_num_ < 1)
        {
            pointcloud_num_ = 10;
            RCLCPP_WARN(
                this->get_logger(), "pointcloud_buffer_size not set or set to 0. set pointcloud_buffer_size to %zu", pointcloud_num_);
        }

        pointcloud_buffer_.set_capacity(pointcloud_num_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化订阅者和发布者
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_pointcloud_topic_, 10, std::bind(&ProjectPointsToImageNode::pointCloudCallback, this, std::placeholders::_1));

        input_image_topics_.resize(camera_num_);
        input_camera_info_topics_.resize(camera_num_);
        image_subs_.resize(camera_num_);
        camera_info_subs_.resize(camera_num_);
        camera_infos_.resize(camera_num_);

        for (std::size_t camera_i = 0; camera_i < camera_num_; ++camera_i)
        {
            input_image_topics_.at(camera_i) = declare_parameter<std::string>(
                "input/image" + std::to_string(camera_i),
                "/sensing/camera/camera" + std::to_string(camera_i));

            input_camera_info_topics_.at(camera_i) = declare_parameter<std::string>(
                "input/camera_info" + std::to_string(camera_i),
                "/sensing/camera/camera" + std::to_string(camera_i) + "/camera_info");
        }

        for (std::size_t camera_i = 0; camera_i < camera_num_; ++camera_i)
        {
            std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fnc1 =
                std::bind(&ProjectPointsToImageNode::cameraInfoCallback, this, std::placeholders::_1, camera_i);
            camera_info_subs_.at(camera_i) = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                input_camera_info_topics_.at(camera_i), rclcpp::QoS{1}.best_effort(), fnc1);

            std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr msg)> fnc2 =
                std::bind(&ProjectPointsToImageNode::imageCallback, this, std::placeholders::_1, camera_i);
            image_subs_.at(camera_i) = this->create_subscription<sensor_msgs::msg::Image>(
                input_image_topics_.at(camera_i), rclcpp::QoS{3}, fnc2);
        }

        // 创建多个相机的图像发布者
        for (size_t i = 0; i < camera_num_; ++i)
        {
            std::string output_topic = output_image_topic_prefix_ + std::to_string(i);
            image_pubs_.push_back(this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10));
        }

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_pointcloud_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Subscribers and publishers initialized");
    }

    void ProjectPointsToImageNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // 存储历史点云数据
        // RCLCPP_INFO(this->get_logger(), "pointCloudCallback, msg->width: %d, msg->height: %d", msg->width, msg->height);
        // std::lock_guard<std::mutex> lock(camera_mutex_);
        if (pointcloud_buffer_.size() > pointcloud_num_)
        {
            pointcloud_buffer_.pop_front();
        }
        pointcloud_buffer_.push_back(*msg);
        // preprocess();
        // publish(pointcloud_buffer_.back());
    }

    void ProjectPointsToImageNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t camera_id)
    {

        // RCLCPP_INFO(this->get_logger(), "imageCallback, msg->width: %d, msg->height: %d", msg->width, msg->height);

        builtin_interfaces::msg::Time image_timestamp = msg->header.stamp;
        rclcpp::Time image_rclcpp_time(image_timestamp.sec, image_timestamp.nanosec);

        // 打印时间
        // RCLCPP_INFO(this->get_logger(), "imageCallback, image_timestamp: %d.%09d", msg->header.stamp.sec, msg->header.stamp.nanosec);

        if (pointcloud_buffer_.size() == 0)
        {
            return;
        }

        double min_time_diff = std::numeric_limits<double>::max();
        std::size_t best_match_index = pointcloud_buffer_.size() - 1;

        if (pointcloud_buffer_.empty())
        {
            publish(*msg, camera_id);
        }

        for (int i = pointcloud_buffer_.size() - 1; i >= 0; i--)
        {
            // 获取当前点云的时间戳
            builtin_interfaces::msg::Time pointcloud_timestamp = pointcloud_buffer_[i].header.stamp;
            rclcpp::Time pointcloud_rclcpp_time(pointcloud_timestamp.sec, pointcloud_timestamp.nanosec);

            // 打印点云时间戳
            // RCLCPP_INFO(this->get_logger(), "imageCallback, pointcloud_timestamp: %d.%09d",
            //             pointcloud_buffer_[i].header.stamp.sec,
            //             pointcloud_buffer_[i].header.stamp.nanosec);

            rclcpp::Duration diff = pointcloud_rclcpp_time - image_rclcpp_time;
            double time_diff = std::abs(diff.nanoseconds() / 1000000.0); // 将纳秒转换为毫秒

            // 如果当前时间差小于已知的最小时间差，则更新最小时间差和最佳匹配索引
            if (time_diff < min_time_diff)
            {
                min_time_diff = time_diff;
                best_match_index = i;
            }
        }

        // 输出最终结果
        // RCLCPP_INFO(this->get_logger(), "imageCallback, min_time_diff = %f, best_match_index = %zu", min_time_diff, best_match_index);

        sensor_msgs::msg::Image projected_image = projectPoints(msg, pointcloud_buffer_[best_match_index], camera_id);

        publish(projected_image, camera_id);

        // if (min_time_diff < threshold_ms_)
        // {
        //     sensor_msgs::msg::Image projected_image = projectPoints(msg, pointcloud_buffer_[best_match_index], camera_id);
        //     publish(projected_image, camera_id);
        // }
        // else
        // {
        //     publish(*msg, camera_id);
        // }
    }

    void ProjectPointsToImageNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, const std::size_t camera_id)
    {
        // RCLCPP_INFO(this->get_logger(), "cameraInfoCallback, msg->width: %d, msg->height: %d", msg->width, msg->height);
        camera_infos_[camera_id] = *msg;
    }

    void ProjectPointsToImageNode::preprocess()
    {
    }

    sensor_msgs::msg::Image ProjectPointsToImageNode::projectPoints(
        sensor_msgs::msg::Image::ConstSharedPtr image,
        sensor_msgs::msg::PointCloud2 pointcloud,
        const std::size_t camera_id)
    {
        sensor_msgs::msg::Image projected_image = *image;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return *image;
        }

        // 获取激光雷达到相机的变换
        Eigen::Affine3f lidar2cam_affine;
        {
            const auto transform_stamped_optional = getTransformStamped(
                *tf_buffer_, /*target*/ image->header.frame_id,
                /*source*/ pointcloud.header.frame_id, image->header.stamp);
            if (!transform_stamped_optional)
            {
                return *image;
            }
            lidar2cam_affine = _transformToEigen(transform_stamped_optional.value().transform);
        }

        // 假设lidar2cam_affine已经是Eigen::Affine3f类型
        Eigen::Matrix4f lidar2cam_matrix = lidar2cam_affine.matrix();

        // 创建一个平坦化的数组来保存4x4的仿射变换矩阵
        float lidar2cam_affine_host[16];
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                lidar2cam_affine_host[i * 4 + j] = lidar2cam_matrix(i, j);
            }
        }

        Eigen::Matrix3f camera_projection;
        camera_projection << camera_infos_[camera_id].k[0], camera_infos_[camera_id].k[1], camera_infos_[camera_id].k[2],
            camera_infos_[camera_id].k[3], camera_infos_[camera_id].k[4], camera_infos_[camera_id].k[5],
            camera_infos_[camera_id].k[6], camera_infos_[camera_id].k[7], camera_infos_[camera_id].k[8];

        float camera_projection_host[9];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                camera_projection_host[i * 3 + j] = camera_projection(i, j);
            }
        }

        unsigned char *image_raw_data = cv_ptr->image.data;

        // 计算点云中点的数量
        size_t point_count = pointcloud.data.size() / pointcloud.point_step;

        std::vector<float> points_data;
        points_data.reserve(point_count * 4);

        // auto start_time1 = std::chrono::high_resolution_clock::now();

        const auto x_offset = pointcloud.fields.at(static_cast<size_t>(autoware_point_types::PointIndex::X)).offset;
        const auto y_offset = pointcloud.fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Y)).offset;
        const auto z_offset = pointcloud.fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Z)).offset;
        const auto intensity_offset = pointcloud.fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Intensity)).offset;
        const auto p_step = pointcloud.point_step;

        unsigned char *data = &pointcloud.data[0];
        for (int i = 0; i < static_cast<int>(pointcloud.data.size() / p_step); i++)
        {
            float p_x = *reinterpret_cast<const float *>(&data[i * p_step + x_offset]);
            float p_y = *reinterpret_cast<const float *>(&data[i * p_step + y_offset]);
            float p_z = *reinterpret_cast<const float *>(&data[i * p_step + z_offset]);
            float intensity = *reinterpret_cast<const float *>(&data[i * p_step + intensity_offset]);

            points_data.push_back(p_x);
            points_data.push_back(p_y);
            points_data.push_back(p_z);
            points_data.push_back(intensity);
        }

        float *pointcloud_data = points_data.data();

        // auto start_time2 = std::chrono::high_resolution_clock::now();
        // double elapsed_time2 = std::chrono::duration<double, std::milli>(start_time2 - start_time1).count();
        // RCLCPP_INFO(this->get_logger(), "convert points:  %f ms", elapsed_time2);

        // 调用CUDA内核进行投影
        projectPointsOnGPU(
            pointcloud_data,
            point_count,
            camera_projection_host,
            lidar2cam_affine_host,
            image_raw_data,
            image->width,
            image->height);

        // auto start_time3 = std::chrono::high_resolution_clock::now();
        // double elapsed_time3 = std::chrono::duration<double, std::milli>(start_time3 - start_time2).count();
        // RCLCPP_INFO(this->get_logger(), "gpu:  %f ms", elapsed_time3);

        // 更新ROS图像消息
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr->toImageMsg(projected_image);

        return projected_image;
    }

    void ProjectPointsToImageNode::publish(sensor_msgs::msg::Image msg, const std::size_t camera_id)
    {
        image_pubs_[camera_id]->publish(msg);
    }

    void ProjectPointsToImageNode::publish(sensor_msgs::msg::PointCloud2 msg)
    {
        pointcloud_pub_->publish(msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(miivii_image_projection_based_fusion::ProjectPointsToImageNode)
