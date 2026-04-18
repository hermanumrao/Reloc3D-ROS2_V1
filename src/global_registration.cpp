#include "relocalization_3d/relocalization_node.hpp"

#include <teaser/registration.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <limits>

void computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(0.5);
  ne.compute(*normals);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  fpfh.setRadiusSearch(0.5);

  fpfh.compute(*features);
}

std::vector<std::pair<int, int>>
findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr src,
                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt) {
  std::vector<std::pair<int, int>> correspondences;

  for (size_t i = 0; i < src->size(); ++i) {
    int best_j = -1;
    float best_dist = std::numeric_limits<float>::max();

    for (size_t j = 0; j < tgt->size(); ++j) {
      float dist = 0.0f;
      for (int k = 0; k < 33; ++k) {
        float diff = src->points[i].histogram[k] - tgt->points[j].histogram[k];
        dist += diff * diff;
      }

      if (dist < best_dist) {
        best_dist = dist;
        best_j = j;
      }
    }

    if (best_j >= 0)
      correspondences.emplace_back(i, best_j);
  }

  return correspondences;
}

Eigen::Matrix4f RelocalizationNode::global_registration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
  // === Features ===
  auto src_feat = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  auto tgt_feat = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(
      new pcl::PointCloud<pcl::FPFHSignature33>());

  computeFPFH(source, src_feat);
  computeFPFH(target, tgt_feat);

  // === Correspondences ===
  auto correspondences = findCorrespondences(src_feat, tgt_feat);

  if (correspondences.size() < 50) {
    RCLCPP_WARN(this->get_logger(), "Not enough correspondences");
    return Eigen::Matrix4f::Identity();
  }

  // === Convert to Eigen matrices ===
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, correspondences.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, correspondences.size());

  for (size_t i = 0; i < correspondences.size(); ++i) {
    auto [si, ti] = correspondences[i];

    src.col(i) << source->points[si].x, source->points[si].y,
        source->points[si].z;

    tgt.col(i) << target->points[ti].x, target->points[ti].y,
        target->points[ti].z;
  }

  // === TEASER Solver ===
  teaser::RobustRegistrationSolver::Params params;
  params.cbar2 = 1;
  params.noise_bound = 0.5;
  params.estimate_scaling = false;

  teaser::RobustRegistrationSolver solver(params);
  solver.solve(src, tgt);

  auto solution = solver.getSolution();

  // === Convert to Eigen 4x4 ===
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();

  tf.block<3, 3>(0, 0) = solution.rotation.cast<float>();
  tf.block<3, 1>(0, 3) = solution.translation.cast<float>();

  RCLCPP_INFO(this->get_logger(), "TEASER global alignment done");

  return tf;
}
