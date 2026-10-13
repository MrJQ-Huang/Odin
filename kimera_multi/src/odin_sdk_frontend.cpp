// SDK-owned local trajectory; visual keyframes for the Kimera-Multi interfaces.
#include <DBoW2/DBoW2.h>
#include <DBoW2/FORB.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <pose_graph_tools_msgs/BowQueries.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphQuery.h>
#include <pose_graph_tools_msgs/VLCFrameQuery.h>
#include <pose_graph_tools_msgs/VLCFrames.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "kimera_multi/sdk_geometry.h"

class SdkFrontend {
  using Vocabulary =
      DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;
  struct Images {
    sensor_msgs::ImageConstPtr rgb, depth, mask;
  };
  ros::NodeHandle nh_, private_{"~"};
  ros::Subscriber odom_sub_, rgb_sub_, depth_sub_, mask_sub_, info_sub_;
  ros::Publisher odom_pub_, keyframe_pub_, graph_pub_, incremental_pub_,
      bow_pub_, vlc_pub_, stats_pub_;
  ros::ServiceServer frame_service_, graph_service_;
  ros::WallTimer timer_;
  Vocabulary vocabulary_;
  cv::Ptr<cv::ORB> orb_;
  sensor_msgs::CameraInfo camera_;
  Eigen::Isometry3d body_T_camera_ = Eigen::Isometry3d::Identity();
  std::map<uint64_t, geometry_msgs::Pose> poses_;
  std::map<uint64_t, Images> images_;
  std::map<uint32_t, pose_graph_tools_msgs::VLCFrameMsg> frames_;
  pose_graph_tools_msgs::PoseGraph graph_;
  std::map<std::string, uint64_t> counts_;
  ros::Time last_odom_, last_image_, last_keyframe_;
  std::string odom_frame_, body_frame_, camera_frame_, input_frame_,
      input_child_;
  int robot_id_, queue_capacity_, pose_capacity_, cache_capacity_,
      min_features_;
  bool camera_ready_ = false, use_mask_, graph_dirty_ = false;
  double max_gap_s_, keyframe_period_, min_interval_, translation_threshold_,
      rotation_threshold_;
  double min_depth_, max_depth_, translation_sigma_, rotation_sigma_,
      last_processing_ms_ = 0;
  Eigen::Isometry3d last_camera_ = Eigen::Isometry3d::Identity();

  std::string topic(const std::string& key) {
    std::string value;
    if (!private_.getParam(key, value) || value.empty())
      throw std::invalid_argument("Missing " + key);
    return value;
  }

 public:
  SdkFrontend() {
    cv::setNumThreads(1);
    private_.param("robot_id", robot_id_, 0);
    private_.param("queue_capacity", queue_capacity_, 30);
    private_.param("pose_capacity", pose_capacity_, 2000);
    private_.param("frame_cache_capacity", cache_capacity_, 2000);
    private_.param("min_depth_features", min_features_, 20);
    private_.param("use_measured_mask", use_mask_, true);
    private_.param("max_odom_gap_s", max_gap_s_, .12);
    private_.param("keyframe_period_s", keyframe_period_, 2.0);
    private_.param("min_keyframe_interval_s", min_interval_, .1);
    private_.param("keyframe_translation_m", translation_threshold_, .1);
    private_.param("keyframe_rotation_deg", rotation_threshold_, 10.0);
    private_.param("min_depth_m", min_depth_, .3);
    private_.param("max_depth_m", max_depth_, 8.0);
    private_.param("odom_translation_sigma_m", translation_sigma_, .1);
    private_.param("odom_rotation_sigma_rad", rotation_sigma_, .1);
    odom_frame_ = topic("odom_frame_id");
    body_frame_ = topic("body_frame_id");
    camera_frame_ = topic("camera_frame_id");
    if (robot_id_ < 0 || robot_id_ > 65535 || queue_capacity_ < 2 ||
        pose_capacity_ < 2 || cache_capacity_ < 1 || min_features_ < 3 ||
        max_gap_s_ <= 0 || min_interval_ <= 0 ||
        keyframe_period_ < min_interval_ || min_depth_ <= 0 ||
        max_depth_ <= min_depth_ || translation_sigma_ <= 0 ||
        rotation_sigma_ <= 0)
      throw std::invalid_argument("Invalid SDK frontend limits");
    cv::FileStorage file(topic("camera_params"), cv::FileStorage::READ);
    std::vector<double> data;
    file["T_BS"]["data"] >> data;
    if (data.size() != 16)
      throw std::invalid_argument("Camera T_BS must have 16 entries");
    Eigen::Matrix4d matrix;
    for (int i = 0; i < 16; ++i) matrix(i / 4, i % 4) = data[i];
    body_T_camera_ = odin_sdk::calibration(matrix);
    ROS_INFO("Camera calibration SO(3) numerical correction norm: %.9g",
             (body_T_camera_.matrix() - matrix).norm());
    int nfeatures;
    private_.param("orb_features", nfeatures, 1200);
    if (nfeatures < min_features_)
      throw std::invalid_argument("Too few requested ORB features");
    orb_ = cv::ORB::create(nfeatures);
    ROS_INFO("Loading the pinned ORB vocabulary for SDK frontend");
    vocabulary_.load(topic("vocabulary"));
    if (vocabulary_.empty()) throw std::runtime_error("Empty ORB vocabulary");
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1000);
    keyframe_pub_ = nh_.advertise<nav_msgs::Odometry>("keyframe_odometry", 100);
    graph_pub_ =
        nh_.advertise<pose_graph_tools_msgs::PoseGraph>("pose_graph", 1, true);
    incremental_pub_ = nh_.advertise<pose_graph_tools_msgs::PoseGraph>(
        "pose_graph_incremental", 100);
    bow_pub_ =
        nh_.advertise<pose_graph_tools_msgs::BowQueries>("bow_query", 100);
    vlc_pub_ =
        nh_.advertise<pose_graph_tools_msgs::VLCFrames>("vlc_frames", 100);
    stats_pub_ = nh_.advertise<std_msgs::String>("sdk_diagnostics", 1, true);
    frame_service_ = nh_.advertiseService("vlc_frame_query",
                                          &SdkFrontend::queryFrames, this);
    graph_service_ = nh_.advertiseService("pose_graph_query",
                                          &SdkFrontend::queryGraph, this);
    odom_sub_ = nh_.subscribe(topic("sdk_odom_topic"), 1000,
                              &SdkFrontend::odometry, this);
    info_sub_ = nh_.subscribe(topic("camera_info_topic"), 2,
                              &SdkFrontend::camera, this);
    rgb_sub_ = nh_.subscribe<sensor_msgs::Image>(
        topic("rgb_topic"), queue_capacity_,
        [this](const sensor_msgs::ImageConstPtr& msg) { image(msg, 0); });
    depth_sub_ = nh_.subscribe<sensor_msgs::Image>(
        topic("depth_topic"), queue_capacity_,
        [this](const sensor_msgs::ImageConstPtr& msg) { image(msg, 1); });
    if (use_mask_)
      mask_sub_ = nh_.subscribe<sensor_msgs::Image>(
          topic("measured_mask_topic"), queue_capacity_,
          [this](const sensor_msgs::ImageConstPtr& msg) { image(msg, 2); });
    timer_ = nh_.createWallTimer(ros::WallDuration(1),
                                 &SdkFrontend::publishStats, this);
    ROS_INFO(
        "SDK frontend ready: SDK body odometry; camera-pose graph; no VIO "
        "estimation");
  }

 private:
  void camera(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (msg->K[0] <= 0 || msg->K[4] <= 0 || !msg->width || !msg->height ||
        msg->header.frame_id != camera_frame_)
      throw std::invalid_argument("Invalid camera intrinsics/frame");
    for (double value : msg->K)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite camera intrinsics");
    for (double value : msg->D)
      if (value != 0)
        throw std::invalid_argument("SDK frontend requires rectified images");
    if (camera_ready_ && (camera_.K != msg->K || camera_.width != msg->width ||
                          camera_.height != msg->height))
      throw std::invalid_argument("Camera calibration changed during a run");
    camera_ = *msg;
    camera_ready_ = true;
    process();
  }

  void odometry(const nav_msgs::OdometryConstPtr& msg) {
    odin_sdk::transform(
        msg->pose.pose);  // validate; forwarded output is untouched
    if (!last_odom_.isZero() && msg->header.stamp <= last_odom_)
      throw std::invalid_argument(
          "SDK clock repeated/reset; start a new graph run");
    if (input_frame_.empty()) {
      input_frame_ = msg->header.frame_id;
      input_child_ = msg->child_frame_id;
    }
    if (input_frame_ != msg->header.frame_id ||
        input_child_ != msg->child_frame_id)
      throw std::invalid_argument("SDK pose frame changed during a run");
    last_odom_ = msg->header.stamp;
    poses_[last_odom_.toNSec()] = msg->pose.pose;
    while (poses_.size() > size_t(pose_capacity_)) poses_.erase(poses_.begin());
    nav_msgs::Odometry out = *msg;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = body_frame_;
    odom_pub_.publish(out);
    ++counts_["sdk_odometry_forwarded"];
    process();
  }

  void image(const sensor_msgs::ImageConstPtr& msg, int kind) {
    if (msg->header.frame_id != camera_frame_)
      throw std::invalid_argument("Image camera frame mismatch");
    const uint64_t stamp = msg->header.stamp.toNSec();
    if (!last_image_.isZero() && stamp <= last_image_.toNSec()) {
      ++counts_["late_image_messages"];
      return;
    }
    Images& pending = images_[stamp];
    if (kind == 0)
      pending.rgb = msg;
    else if (kind == 1)
      pending.depth = msg;
    else
      pending.mask = msg;
    while (images_.size() > size_t(queue_capacity_)) {
      images_.erase(images_.begin());
      ++counts_["image_queue_overflow"];
    }
    process();
  }

  bool lookup(uint64_t stamp, geometry_msgs::Pose* pose, bool* impossible) {
    *impossible = false;
    auto right = poses_.lower_bound(stamp);
    if (right != poses_.end() && right->first == stamp) {
      *pose = right->second;
      return true;
    }
    if (right == poses_.end())
      return false;  // wait for future SDK sample, never extrapolate
    if (right == poses_.begin()) {
      *impossible = true;
      return false;
    }
    auto left = std::prev(right);
    if ((right->first - left->first) * 1e-9 > max_gap_s_) {
      *impossible = true;
      return false;
    }
    *pose = odin_sdk::interpolate(
        left->second, right->second,
        double(stamp - left->first) / (right->first - left->first));
    ++counts_["interpolated_sdk_poses"];
    return true;
  }

  void process() {
    if (!camera_ready_) return;
    while (!images_.empty()) {
      auto first = images_.begin();
      const auto stamp = first->first;
      const Images input = first->second;
      if (!input.rgb || !input.depth || (use_mask_ && !input.mask)) return;
      geometry_msgs::Pose body;
      bool impossible;
      if (!lookup(stamp, &body, &impossible)) {
        if (!impossible) return;
        images_.erase(first);
        ++counts_["rgbd_without_sdk_pose"];
        continue;
      }
      last_image_.fromNSec(stamp);
      images_.erase(first);
      ++counts_["rgbd_pairs"];
      Eigen::Isometry3d camera_pose =
          odin_sdk::transform(body) * body_T_camera_;
      const double elapsed = (last_image_ - last_keyframe_).toSec();
      const auto motion = last_camera_.inverse() * camera_pose;
      const double rotation =
          Eigen::AngleAxisd(motion.linear()).angle() * 180 / M_PI;
      if (!last_keyframe_.isZero() &&
          (elapsed < min_interval_ ||
           (elapsed < keyframe_period_ &&
            motion.translation().norm() < translation_threshold_ &&
            rotation < rotation_threshold_))) {
        ++counts_["rgbd_not_selected"];
        continue;
      }
      auto start = std::chrono::steady_clock::now();
      keyframe(input, camera_pose);
      last_keyframe_ = last_image_;
      last_camera_ = camera_pose;
      last_processing_ms_ = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
    }
  }

  void keyframe(const Images& input, const Eigen::Isometry3d& camera_pose) {
    if (graph_.nodes.size() >= UINT32_MAX)
      throw std::runtime_error("Keyframe IDs exhausted");
    const uint32_t id = graph_.nodes.size();
    pose_graph_tools_msgs::PoseGraphNode node;
    node.header = input.rgb->header;
    node.header.frame_id = odom_frame_;
    node.robot_id = robot_id_;
    node.key = id;
    node.pose = odin_sdk::pose(camera_pose);
    pose_graph_tools_msgs::PoseGraph incremental;
    incremental.header = node.header;
    if (id > 0) {
      pose_graph_tools_msgs::PoseGraphEdge edge;
      edge.header = node.header;
      edge.key_from = id - 1;
      edge.key_to = id;
      edge.robot_from = robot_id_;
      edge.robot_to = robot_id_;
      edge.type = edge.ODOM;
      edge.pose = odin_sdk::pose(last_camera_.inverse() * camera_pose);
      for (int i = 0; i < 6; ++i)
        edge.covariance[i * 6 + i] =
            std::pow(i < 3 ? translation_sigma_ : rotation_sigma_, 2);
      incremental.nodes.push_back(graph_.nodes.back());
      incremental.edges.push_back(edge);
      graph_.edges.push_back(edge);
    }
    incremental.nodes.push_back(node);
    incremental_pub_.publish(incremental);
    graph_.header = node.header;
    graph_.nodes.push_back(node);
    graph_dirty_ = true;
    nav_msgs::Odometry camera_odom;
    camera_odom.header = node.header;
    camera_odom.child_frame_id = camera_frame_;
    camera_odom.pose.pose = node.pose;
    keyframe_pub_.publish(camera_odom);
    ++counts_["keyframes"];

    auto rgb =
        cv_bridge::toCvShare(input.rgb, sensor_msgs::image_encodings::BGR8);
    if (input.depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
      throw std::invalid_argument(
          "Depth must be float32 meters, not a converted arbitrary encoding");
    auto depth = cv_bridge::toCvShare(input.depth);
    cv::Mat mask;
    if (use_mask_)
      mask =
          cv_bridge::toCvShare(input.mask, sensor_msgs::image_encodings::MONO8)
              ->image;
    if (rgb->image.cols != int(camera_.width) ||
        rgb->image.rows != int(camera_.height) ||
        depth->image.size() != rgb->image.size() ||
        (use_mask_ && mask.size() != rgb->image.size()))
      throw std::invalid_argument(
          "RGB/depth/mask dimensions disagree with CameraInfo");
    cv::Mat gray, descriptors;
    cv::cvtColor(rgb->image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;
    orb_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
    pose_graph_tools_msgs::VLCFrameMsg frame;
    frame.robot_id = robot_id_;
    frame.pose_id = id;
    frame.submap_id = id;
    frame.T_submap_pose.orientation.w =
        1;  // camera graph node == visual keyframe frame
    cv::Mat valid_descriptors;
    std::vector<Eigen::Vector3f> bearings;
    for (size_t i = 0; i < keypoints.size(); ++i) {
      const auto& pixel = keypoints[i].pt;
      const int x = cvRound(pixel.x), y = cvRound(pixel.y);
      if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) continue;
      const float z = depth->image.at<float>(y, x);
      if (!std::isfinite(z) || z < min_depth_ || z > max_depth_ ||
          (use_mask_ && mask.at<uint8_t>(y, x) == 0))
        continue;
      Eigen::Vector3f bearing((pixel.x - camera_.K[2]) / camera_.K[0],
                              (pixel.y - camera_.K[5]) / camera_.K[4], 1);
      bearing.normalize();
      bearings.push_back(bearing);
      frame.depths.push_back(z);
      valid_descriptors.push_back(descriptors.row(i));
    }
    counts_["depth_supported_features"] += bearings.size();
    if (bearings.size() < size_t(min_features_)) {
      ++counts_["keyframes_without_visual_support"];
      return;
    }
    cv_bridge::CvImage image(input.rgb->header,
                             sensor_msgs::image_encodings::TYPE_8UC1,
                             valid_descriptors);
    image.toImageMsg(frame.descriptors_mat);
    frame.versors.header = input.rgb->header;
    sensor_msgs::PointCloud2Modifier modifier(frame.versors);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(bearings.size());
    sensor_msgs::PointCloud2Iterator<float> x(frame.versors, "x"),
        y(frame.versors, "y"), z(frame.versors, "z");
    for (const auto& v : bearings) {
      *x = v.x();
      *y = v.y();
      *z = v.z();
      ++x;
      ++y;
      ++z;
    }
    frames_[id] = frame;
    while (frames_.size() > size_t(cache_capacity_)) {
      frames_.erase(frames_.begin());
      ++counts_["vlc_cache_evicted"];
    }
    pose_graph_tools_msgs::VLCFrames visual;
    visual.header = input.rgb->header;
    visual.destination_robot_id = robot_id_;
    visual.frames.push_back(frame);
    vlc_pub_.publish(visual);
    ++counts_["visual_keyframes"];
    std::vector<cv::Mat> rows;
    for (int i = 0; i < valid_descriptors.rows; ++i)
      rows.push_back(valid_descriptors.row(i));
    DBoW2::BowVector bow;
    vocabulary_.transform(rows, bow);
    pose_graph_tools_msgs::BowQuery query;
    query.robot_id = robot_id_;
    query.pose_id = id;
    for (const auto& word : bow) {
      query.bow_vector.word_ids.push_back(word.first);
      query.bow_vector.word_values.push_back(word.second);
    }
    for (int target = 0; target <= robot_id_; ++target) {
      pose_graph_tools_msgs::BowQueries batch;
      batch.header = input.rgb->header;
      batch.destination_robot_id = target;
      batch.queries.push_back(query);
      bow_pub_.publish(batch);
      ++counts_["bow_queries"];
    }
  }

  bool queryFrames(pose_graph_tools_msgs::VLCFrameQuery::Request& req,
                   pose_graph_tools_msgs::VLCFrameQuery::Response& res) {
    if (req.robot_id != uint32_t(robot_id_)) return false;
    for (uint32_t id : req.pose_ids) {
      auto frame = frames_.find(id);
      if (frame == frames_.end()) {
        ++counts_["vlc_query_missing_frame"];
        return false;
      }
      res.frames.push_back(frame->second);
    }
    return true;
  }

  bool queryGraph(pose_graph_tools_msgs::PoseGraphQuery::Request& req,
                  pose_graph_tools_msgs::PoseGraphQuery::Response& res) {
    if (req.robot_id != robot_id_) return false;
    res.pose_graph = graph_;
    return true;
  }

  void publishStats(const ros::WallTimerEvent&) {
    if (graph_dirty_) {
      graph_pub_.publish(graph_);
      graph_dirty_ = false;
    }
    std_msgs::String out;
    std::ostringstream json;
    json << "{\"trajectory_source\":\"odin_sdk\",\"graph_pose_frame\":"
            "\"camera\",\"odom_covariance\":\"configured_initial_model\","
            "\"robot_id\":"
         << robot_id_;
    json << ",\"last_keyframe_processing_ms\":" << last_processing_ms_
         << ",\"pending_rgbd\":" << images_.size()
         << ",\"cached_visual_frames\":" << frames_.size() << ",\"counters\":{";
    bool first = true;
    for (const auto& item : counts_) {
      if (!first) json << ",";
      first = false;
      json << "\"" << item.first << "\":" << item.second;
    }
    json << "}}";
    out.data = json.str();
    stats_pub_.publish(out);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odin_sdk_frontend");
  try {
    SdkFrontend frontend;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("SDK frontend: %s", error.what());
    return 1;
  }
  return 0;
}
