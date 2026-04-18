#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <mutex>

class RelocalizationNode : public rclcpp::Node {
public:
  RelocalizationNode();

private:
  // === Callbacks ===
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void relocalizeService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // === Core Functions ===
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  preprocess(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  Eigen::Matrix4f
  global_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr target);

  Eigen::Matrix4f local_refinement(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                   const Eigen::Matrix4f &initial_guess);

  void publishPose(const Eigen::Matrix4f &tf);
  void broadcastMapToOdom(const Eigen::Matrix4f &map_to_base);

  // === ROS Interfaces ===
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;

  // === State ===
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_downsampled_;

  std::atomic<bool> map_ready_{false};
  std::atomic<bool> relocalize_requested_{false};

  std::mutex pose_mutex_;
  Eigen::Matrix4f last_pose_ = Eigen::Matrix4f::Identity();
};
