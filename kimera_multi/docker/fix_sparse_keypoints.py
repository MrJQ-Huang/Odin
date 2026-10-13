"""Handle a complete loss of RGB-D feature tracks in the pinned Kimera source."""
from pathlib import Path
import sys

root=Path(sys.argv[1])
path=root/'src/frontend/UndistorterRectifier.cpp'
text=path.read_text()
marker='// Odin RGB-D: an empty feature set has an empty undistorted result.'
if marker not in text:
    old='''  CHECK_NOTNULL(undistorted_keypoints)->clear();
  switch (cam_param.distortion_model_) {'''
    new='''  CHECK_NOTNULL(undistorted_keypoints)->clear();
  // Odin RGB-D: an empty feature set has an empty undistorted result.
  // OpenCV 4.2 undistortPoints throws for an empty point vector.
  if (keypoints.empty()) {
    return;
  }
  switch (cam_param.distortion_model_) {'''
    if text.count(old)!=1:
        raise RuntimeError('Unexpected pinned UndistorterRectifier implementation')
    path.write_text(text.replace(old,new))

path=root/'src/frontend/Tracker.cpp'
text=path.read_text()
marker='// Odin RGB-D: no reference tracks; let the frontend detect new features.'
if marker not in text:
    old='''  // Setup termination criteria for optical flow.
  const cv::TermCriteria kTerminationCriteria('''
    new='''  // Odin RGB-D: no reference tracks; let the frontend detect new features.
  // Passing an empty point vector to PyrLK can also trigger an OpenCV assertion.
  if (px_ref.empty()) {
    CHECK(cur_frame->keypoints_.empty());
    return;
  }

  // Setup termination criteria for optical flow.
  const cv::TermCriteria kTerminationCriteria('''
    if text.count(old)!=1:
        raise RuntimeError('Unexpected pinned Tracker implementation')
    path.write_text(text.replace(old,new))
