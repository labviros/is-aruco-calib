#include <iostream>
#include <vector>
#include "is/msgs/camera.pb.h"
#include "is/msgs/common.pb.h"
#include "is/msgs/cv.hpp"
#include "is/msgs/io.hpp"
#include "is/msgs/validate.hpp"
#include "is/wire/core.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "options.pb.h"

int main(int argc, char* argv[]) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  auto leave_on_error = [](is::wire::Status const& status) {
    if (status.code() != is::wire::StatusCode::OK) is::critical("{}", status);
  };

  CalibrationOptions opts;
  leave_on_error(is::load(filename, &opts));
  auto eopts = opts.extrinsic();
  auto data_dir = fmt::format(opts.data_dir(), opts.camera_id());
  auto topic = fmt::format(opts.topic(), opts.camera_id());

  auto channel = is::Channel{opts.uri()};
  auto subscription = is::Subscription{channel};
  subscription.subscribe(topic);
  is::info("Using uri={} topic={}", opts.uri(), topic);

  auto detector_params = cv::aruco::DetectorParameters::create();
  auto dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(eopts.dictionary()));

  std::string calibration_file = data_dir;
  std::replace(calibration_file.begin(), calibration_file.end(), '/', '_');
  calibration_file = fmt::format("{}/{}.json", data_dir, calibration_file);

  is::vision::CameraCalibration calibration;
  leave_on_error(is::load(calibration_file, &calibration));
  calibration.PrintDebugString();
  auto intrinsic = is::to_mat(*calibration.mutable_intrinsic());
  auto distortion = is::to_mat(*calibration.mutable_distortion());

  cv::Mat image;
  cv::Vec3d rotation, translation;
  for (;;) {
    auto message = channel.consume();
    auto maybe_image = message.unpack<is::vision::Image>();

    std::vector<char> coded(maybe_image->data().begin(), maybe_image->data().end());
    image = cv::imdecode(coded, CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat blurred_image;
    cv::bilateralFilter(image, blurred_image, 7, 9, 7);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(blurred_image, dictionary, corners, ids, detector_params,
                             cv::noArray());

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Mat draw_image;
    cv::cvtColor(blurred_image, draw_image, CV_GRAY2BGR);
    if (ids.size() != 0) {
      cv::aruco::estimatePoseSingleMarkers(corners, eopts.marker_length(), intrinsic, distortion,
                                           rvecs, tvecs);
      cv::aruco::drawDetectedMarkers(draw_image, corners, ids);
    }

    auto id = std::find(ids.begin(), ids.end(), eopts.marker_id());
    auto detected_marker = id != ids.end();
    if (detected_marker) {
      int index = std::distance(ids.begin(), id);
      translation = tvecs[index];
      rotation = rvecs[index];
      auto axis_size = eopts.marker_length() * 0.5f;
      is::info("translation: {} ({}) rotation: {}", translation, cv::norm(translation), rotation);
      cv::aruco::drawAxis(draw_image, intrinsic, distortion, rotation, translation, axis_size);
    }

    cv::imshow("Extrinsic", draw_image);
    auto key = cv::waitKey(1);
    if (key == 27) { return 0; }
    if (key == 'k' && detected_marker) {
      auto key = cv::waitKey(0);
      if (key != 27) break;
    }
  }

  cv::Mat rotation_matrix, extrinsic;
  cv::Rodrigues(rotation, rotation_matrix);
  cv::hconcat(rotation_matrix, translation, extrinsic);       // [R3x3     T3x1]
  cv::Mat last_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);  // [Zeros1x3    1]
  cv::vconcat(extrinsic, last_row, extrinsic);                // [[R3x3     T3x1]
                                                              //  [Zeros1x3    1]]

  // ArUco Pose is measured in relation to the marker center. This allow the point to be
  // translated to a corner of the marker to be easier to place it in a desired point in the
  // world.
  cv::Mat offset_translation = cv::Mat::eye(4, 4, CV_64F);
  offset_translation.at<double>(0, 3) = eopts.offset();
  offset_translation.at<double>(1, 3) = eopts.offset();
  extrinsic = extrinsic * offset_translation;
  is::info("extrinsic: {}", extrinsic);

  auto extrinsics = calibration.mutable_extrinsic();
  auto tf = std::find_if(extrinsics->begin(), extrinsics->end(),
                         [=](is::vision::FrameTransformation const& tf) {
                           return tf.from() == opts.world_id() && tf.to() == opts.camera_id();
                         });
  auto tf_already_exists = tf != extrinsics->end();
  if (tf_already_exists) {
    *(tf->mutable_tf()) = is::to_tensor(extrinsic);
  } else {
    auto new_tf = calibration.add_extrinsic();
    *(new_tf->mutable_tf()) = is::to_tensor(extrinsic);
    new_tf->set_from(opts.world_id());
    new_tf->set_to(opts.camera_id());
  }

  is::save(calibration_file, calibration);
  if (opts.save_images()) {
    cv::imwrite(fmt::format("{}/extrinsic_{}.png", data_dir, opts.world_id()), image);
  }
  return 0;
}
