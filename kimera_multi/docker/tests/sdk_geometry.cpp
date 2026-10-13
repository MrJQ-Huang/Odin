#include "kimera_multi/sdk_geometry.h"

#include <iostream>
#include <stdexcept>

void require(bool ok, const char* text) {
  if (!ok) throw std::runtime_error(text);
}
int main() {
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
  body.linear() =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  body.translation() = Eigen::Vector3d(1, 2, 3);
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.linear() =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  extrinsic.translation() = Eigen::Vector3d(.2, .1, .3);
  const auto camera = body * extrinsic;
  require(camera.translation().isApprox(Eigen::Vector3d(.9, 2.2, 3.3), 1e-12),
          "Camera lever arm must rotate with the SDK body orientation");
  require(odin_sdk::transform(odin_sdk::pose(camera))
              .matrix()
              .isApprox(camera.matrix(), 1e-12),
          "Quaternion serialization must preserve the full rigid transform");
  auto moved = body;
  moved.translation() += Eigen::Vector3d(0, 1, 0);
  const auto relative = camera.inverse() * (moved * extrinsic);
  require(relative.translation().isApprox(Eigen::Vector3d(1, 0, 0), 1e-12),
          "Odometry edge translation must be expressed in the previous camera "
          "frame");
  auto end = body;
  end.translation() = Eigen::Vector3d(3, 4, 5);
  end.linear() =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto midpoint = odin_sdk::transform(
      odin_sdk::interpolate(odin_sdk::pose(body), odin_sdk::pose(end), .5));
  require(midpoint.translation().isApprox(Eigen::Vector3d(2, 3, 4), 1e-12),
          "SDK translation interpolation");
  require(midpoint.linear().isApprox(
              Eigen::AngleAxisd(3 * M_PI / 4, Eigen::Vector3d::UnitZ())
                  .toRotationMatrix(),
              1e-12),
          "SDK shortest-arc rotation interpolation");
  bool failed = false;
  try {
    odin_sdk::interpolate(odin_sdk::pose(body), odin_sdk::pose(end), 1.1);
  } catch (const std::invalid_argument&) {
    failed = true;
  }
  require(failed, "Do not extrapolate SDK poses");
  failed = false;
  try {
    odin_sdk::transform(geometry_msgs::Pose{});
  } catch (const std::invalid_argument&) {
    failed = true;
  }
  require(failed, "Reject zero quaternions");
  auto rounded = extrinsic.matrix().eval();
  rounded(0, 0) += 5e-6;
  const auto rigid = odin_sdk::calibration(rounded);
  require((rigid.linear().transpose() * rigid.linear())
              .isApprox(Eigen::Matrix3d::Identity(), 1e-12),
          "Rounded vendor calibration must be projected to a rigid rotation");
  require(rigid.translation().isApprox(extrinsic.translation(), 1e-12),
          "Calibration correction must preserve lever arm");
  failed = false;
  auto reflection = Eigen::Matrix4d::Identity().eval();
  reflection(0, 0) = -1;
  try {
    odin_sdk::calibration(reflection);
  } catch (const std::invalid_argument&) {
    failed = true;
  }
  require(failed,
          "Reject reflective calibration rather than repairing a wrong frame");
  std::cout << "PASS: camera lever arm, SE3 serialization/relative edge, "
               "interpolation, and invalid SDK pose rejection\n";
}
