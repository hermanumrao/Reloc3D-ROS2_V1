#include "relocalization_3d/relocalization_node.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

RelocalizationNode::RelocalizationNode() : Node("relocalization_node") {
  // === Subscribers ===
  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      std::bind(&RelocalizationNode::scanCallback, this,
                std::placeholders::_1));

  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/saved_map", 1,
      std::bind(&RelocalizationNode::mapCallback, this, std::placeholders::_1));

  // === Publisher ===
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/relocalized_pose", 10);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/aligned_scan", 10);

  // === Service ===
  relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/relocalize_global",
      std::bind(&RelocalizationNode::relocalizeService, this,
                std::placeholders::_1, std::placeholders::_2));

  map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  map_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Relocalization node started");
}

void RelocalizationNode::mapCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::fromROSMsg(*msg, *map_);

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(map_);
  voxel.setLeafSize(0.3f, 0.3f, 0.3f);
  voxel.filter(*map_downsampled_);

  map_ready_ = true;

  RCLCPP_INFO(this->get_logger(), "Map received and processed");
}

void RelocalizationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!map_ready_)
    return;

  auto scan = preprocess(msg);

  Eigen::Matrix4f result_tf;

  if (relocalize_requested_) {
    RCLCPP_WARN(this->get_logger(), "[MODE] GLOBAL RELOCALIZATION");

    auto coarse_tf = global_registration(scan, map_downsampled_);
    result_tf = local_refinement(scan, map_downsampled_, coarse_tf);

    relocalize_requested_ = false;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "[MODE] TRACKING");
    std::lock_guard<std::mutex> lock(pose_mutex_);
    result_tf = local_refinement(scan, map_downsampled_, last_pose_);
  }

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    last_pose_ = result_tf;
  }

  publishPose(result_tf);
  broadcastMapToOdom(result_tf);
}

void RelocalizationNode::relocalizeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!map_ready_) {
    response->success = false;
    response->message = "Map not ready";
    return;
  }

  relocalize_requested_ = true;

  response->success = true;
  response->message = "Relocalization triggered";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RelocalizationNode::preprocess(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.3f, 0.3f, 0.3f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>());

  voxel.filter(*filtered);

  return filtered;
}

Eigen::Matrix4f
RelocalizationNode::local_refinement(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                     const Eigen::Matrix4f &initial_guess) {
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

  // === Input ===
  gicp.setInputSource(source);
  gicp.setInputTarget(target);

  // === Core Parameters (tuned for Livox) ===
  gicp.setMaxCorrespondenceDistance(2.0); // meters
  gicp.setMaximumIterations(50);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-3);

  // Optional but useful:
  gicp.setRANSACIterations(
      0); // disable internal RANSAC (we handle global separately)

  pcl::PointCloud<pcl::PointXYZ> aligned;
  gicp.align(aligned, initial_guess);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(aligned, out_msg);
  out_msg.header.frame_id = "map";
  debug_pub_->publish(out_msg);

  // === Check convergence ===
  if (!gicp.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "GICP did NOT converge");
    return initial_guess;
  }

  double fitness = gicp.getFitnessScore();

  RCLCPP_INFO(this->get_logger(), "GICP fitness: %.5f", fitness);

  // === Reject bad solutions ===
  if (fitness > 1.5) // tune this threshold
  {
    RCLCPP_ERROR(this->get_logger(), "GICP bad result, rejecting!");
    return initial_guess;
  }

  return gicp.getFinalTransformation();
}

void RelocalizationNode::publishPose(const Eigen::Matrix4f &tf) {
  geometry_msgs::msg::PoseStamped pose;

  pose.header.stamp = this->now();
  pose.header.frame_id = "map";

  pose.pose.position.x = tf(0, 3);
  pose.pose.position.y = tf(1, 3);
  pose.pose.position.z = tf(2, 3);

  Eigen::Matrix3f rot = tf.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  pose_pub_->publish(pose);
}

Eigen::Matrix4f
transformMsgToEigen(const geometry_msgs::msg::TransformStamped &tf_msg) {
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

  const auto &t = tf_msg.transform.translation;
  const auto &q = tf_msg.transform.rotation;

  Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
  Eigen::Matrix3f rot = quat.toRotationMatrix();

  mat.block<3, 3>(0, 0) = rot;
  mat(0, 3) = t.x;
  mat(1, 3) = t.y;
  mat(2, 3) = t.z;

  return mat;
}

void RelocalizationNode::broadcastMapToOdom(
    const Eigen::Matrix4f &map_to_base) {
  geometry_msgs::msg::TransformStamped odom_tf;

  try {
    odom_tf =
        tf_buffer_->lookupTransform("camera_init", "body", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  Eigen::Matrix4f odom_to_base = transformMsgToEigen(odom_tf);

  Eigen::Matrix4f map_to_odom = map_to_base * odom_to_base.inverse();

  // Convert to ROS msg
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "camera_init";

  tf_msg.transform.translation.x = map_to_odom(0, 3);
  tf_msg.transform.translation.y = map_to_odom(1, 3);
  tf_msg.transform.translation.z = map_to_odom(2, 3);

  Eigen::Matrix3f rot = map_to_odom.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);

  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}
