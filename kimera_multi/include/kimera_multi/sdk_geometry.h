#pragma once

#include <geometry_msgs/Pose.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <cmath>
#include <stdexcept>

namespace odin_sdk {
inline Eigen::Isometry3d calibration(const Eigen::Matrix4d& matrix) {
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if (!matrix.allFinite() ||
      !matrix.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)) ||
      !(rotation.transpose() * rotation)
           .isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
      std::abs(rotation.determinant() - 1) > 1e-3)
    throw std::invalid_argument(
        "Camera T_BS must be a rigid camera-to-IMU transform");
  // Vendor YAML coefficients are rounded. Project the near-rotation onto SO(3)
  // once, preserving translation; Isometry inverses require orthonormality.
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = svd.matrixU() * svd.matrixV().transpose();
  result.translation() = matrix.topRightCorner<3, 1>();
  return result;
}

inline Eigen::Isometry3d transform(const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
  if (!q.coeffs().allFinite() || !t.allFinite() ||
      std::abs(q.norm() - 1.0) > .01) {
    throw std::invalid_argument(
        "Invalid SDK pose; require finite position and unit quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = t;
  return result;
}

inline geometry_msgs::Pose pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose result;
  const auto& t = transform.translation();
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  result.position.x = t.x();
  result.position.y = t.y();
  result.position.z = t.z();
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();
  return result;
}

inline geometry_msgs::Pose interpolate(const geometry_msgs::Pose& first,
                                       const geometry_msgs::Pose& second,
                                       double fraction) {
  if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
    throw std::invalid_argument("SDK pose interpolation cannot extrapolate");
  const auto a = transform(first), b = transform(second);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() =
      (1 - fraction) * a.translation() + fraction * b.translation();
  out.linear() = Eigen::Quaterniond(a.linear())
                     .slerp(fraction, Eigen::Quaterniond(b.linear()))
                     .toRotationMatrix();
  return pose(out);
}
}  // namespace odin_sdk
