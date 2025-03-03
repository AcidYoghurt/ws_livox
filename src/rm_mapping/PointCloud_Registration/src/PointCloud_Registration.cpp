#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <string>

class ThreeDMapper : public rclcpp::Node
{
public:
    ThreeDMapper() : Node("PointCloud_Registration")
    {
        // 声明参数
        this->declare_parameter<double>("voxel_size", 0.05);
        this->declare_parameter<int>("mean_k", 50);
        this->declare_parameter<double>("std_dev_multiplier", 1.0);

        // 创建配准后的点云发布器
        registered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_cloud", 10);

        // 初始化点云订阅器
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", 10, std::bind(&ThreeDMapper::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "三维点云配准节点已初始化");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_target_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_source_;
    std::string target_frame_id_;
    std_msgs::msg::Header target_header_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换ROS消息到PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 离群点滤波
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(this->get_parameter("mean_k").as_int());
        sor.setStddevMulThresh(this->get_parameter("std_dev_multiplier").as_double());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        sor.filter(*filtered_cloud);

        if (!raw_target_) {
            // 第一帧点云作为目标点云
            raw_target_ = filtered_cloud;
            target_frame_id_ = msg->header.frame_id;
            target_header_ = msg->header;
            RCLCPP_INFO(this->get_logger(), "收到第一帧点云");
        } else {
            // 后续点云作为源点云
            raw_source_ = raw_target_;
            raw_target_ = filtered_cloud;

            // 保存新的目标点云头信息
            std::swap(target_frame_id_, msg->header.frame_id);
            target_header_ = msg->header;

            // 执行点云配准
            alignPointClouds();
        }
    }

    void alignPointClouds()
    {
        // 从参数服务器获取体素大小
        double voxel_size = this->get_parameter("voxel_size").as_double();

        // 下采样处理
        pcl::PointCloud<pcl::PointXYZ>::Ptr target = small_gicp::voxelgrid_sampling_omp(*raw_target_, voxel_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source = small_gicp::voxelgrid_sampling_omp(*raw_source_, voxel_size);

        // 配置配准器
        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setNumThreads(4);
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(1.0);
        reg.setVoxelResolution(1.0);
        reg.setRegistrationType("VGICP");

        // 设置输入点云
        reg.setInputTarget(target);
        reg.setInputSource(source);

        // 执行配准
        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        reg.align(*aligned);

        if (reg.hasConverged()) {
            RCLCPP_INFO(this->get_logger(), "点云配准成功，分数: %.3f", reg.getFitnessScore());

            // 转换配准结果到ROS消息
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*aligned, output_msg);
            output_msg.header.stamp = this->now();
            output_msg.header.frame_id = target_frame_id_;

            // 发布配准后的点云
            registered_cloud_pub_->publish(output_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "点云配准失败");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThreeDMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}