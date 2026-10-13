#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  require(argc == 2, "Pass a zero-distortion Odin camera calibration YAML");
  VIO::CameraParams params;
  params.parseYAML(argv[1]);
  VIO::KeypointsCV output{cv::Point2f(42, 42)};
  VIO::UndistorterRectifier::UndistortRectifyKeypoints({}, &output, params);
  require(output.empty(), "Empty input must clear stale output without throwing");

  const float cx = params.K_.at<double>(0, 2);
  const float cy = params.K_.at<double>(1, 2);
  const double fx = params.K_.at<double>(0, 0);
  const double fy = params.K_.at<double>(1, 1);
  const VIO::KeypointsCV points{{cx, cy}, {cx + 50, cy - 30}};
  VIO::UndistorterRectifier::UndistortRectifyKeypoints(points, &output, params);
  require(output.size() == 2, "Nonempty input must retain both points");
  require(std::abs(output[0].x) < 1e-6 && std::abs(output[0].y) < 1e-6,
          "Principal point must map to the optical axis");
  require(std::abs(output[1].x - 50 / fx) < 1e-6 &&
              std::abs(output[1].y + 30 / fy) < 1e-6,
          "Normal undistortion geometry must remain unchanged");

  auto camera = std::make_shared<VIO::Camera>(params);
  VIO::Tracker tracker(VIO::TrackerParams{}, camera);
  const cv::Mat image = cv::Mat::zeros(params.image_size_, CV_8UC1);
  VIO::Frame reference(0, 0, params, image);
  VIO::Frame current(1, 100000000, params, image);
  tracker.featureTracking(&reference, &current, gtsam::Rot3{},
                          VIO::FeatureDetectorParams{});
  require(current.keypoints_.empty(), "Empty reference must yield no tracks");
  reference.keypoints_.push_back({cx, cy});
  reference.landmarks_.push_back(-1);
  tracker.featureTracking(&reference, &current, gtsam::Rot3{},
                          VIO::FeatureDetectorParams{});
  require(current.keypoints_.empty(), "Invalid landmarks must yield no tracks");
  std::cout << "PASS: empty undistortion, normal geometry, empty/invalid tracking\n";
}
