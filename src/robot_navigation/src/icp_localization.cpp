#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

class ICPLocalizationWithLoopClosure : public rclcpp::Node {
public:
    ICPLocalizationWithLoopClosure() : Node("icp_localization_with_loop_closure") {
        // Load the PCD file as the target cloud
        target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("config/map.pcd", *target_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file map.pcd");
            return;
        }


        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&ICPLocalizationWithLoopClosure::topic_callback, this, std::placeholders::_1));

     
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        ndt_.setResolution(1.0);
        ndt_.setInputTarget(target_cloud_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(target_cloud_);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if (icp.hasConverged()) {
            RCLCPP_INFO(this->get_logger(), "ICP converged with score: %f", icp.getFitnessScore());
            transformation = icp.getFinalTransformation();

            
            pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
            ndt_.setInputSource(cloud);
            ndt_.align(aligned_cloud, transformation);

            if (ndt_.hasConverged()) {
                RCLCPP_INFO(this->get_logger(), "Loop closure detected with score: %f", ndt_.getFitnessScore());
                transformation = ndt_.getFinalTransformation();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }

        
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = transformation(0, 3);
        transform.transform.translation.y = transformation(1, 3);
        transform.transform.translation.z = transformation(2, 3);

        tf2::Quaternion q;
        q.setRPY(0, 0, atan2(transformation(1, 0), transformation(0, 0)));
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        
        tf_broadcaster_->sendTransform(transform);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPLocalizationWithLoopClosure>());
    rclcpp::shutdown();
    return 0;
}