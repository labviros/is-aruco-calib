#include <chrono>
#include <regex>
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

struct CharucoDetection {
  std::vector<int> aruco_ids;
  std::vector<std::vector<cv::Point2f>> aruco_corners;
  cv::Mat charuco_ids;
  cv::Mat charuco_corners;
};

namespace aruco = cv::aruco;

struct Charuco {
  cv::Ptr<cv::aruco::DetectorParameters> detector_params;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board;
  cv::Ptr<cv::aruco::Board> board;

  Charuco(int dict, float marker_length, float square_length, int n_squares_x, int n_squares_y) {
    detector_params = aruco::DetectorParameters::create();
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict));
    charuco_board = aruco::CharucoBoard::create(n_squares_x, n_squares_y, square_length,
                                                marker_length, dictionary);
    board = charuco_board.staticCast<aruco::Board>();
  }

  CharucoDetection detect(cv::Mat const& image) {
    CharucoDetection d;
    aruco::detectMarkers(image, dictionary, d.aruco_corners, d.aruco_ids, detector_params,
                         cv::noArray());
    aruco::refineDetectedMarkers(image, board, d.aruco_corners, d.aruco_ids, cv::noArray());
    if (d.aruco_ids.size() > 0) {
      aruco::interpolateCornersCharuco(d.aruco_corners, d.aruco_ids, image, charuco_board,
                                       d.charuco_corners, d.charuco_ids);
    }
    return d;
  }

  is::vision::CameraCalibration calibrate_camera(
      std::vector<std::pair<cv::Mat, CharucoDetection>> const& detections) {
    // flatten data in preparation for calibration
    std::vector<std::vector<cv::Point2f>> all_corners_flatten;
    std::vector<int> all_ids_flatten;
    std::vector<int> markers_per_frame;
    markers_per_frame.reserve(detections.size());
    for (auto&& detection : detections) {
      markers_per_frame.push_back(detection.second.aruco_ids.size());
      for (auto&& corner : detection.second.aruco_corners) all_corners_flatten.push_back(corner);
      for (auto&& id : detection.second.aruco_ids) all_ids_flatten.push_back(id);
    }

    cv::Size image_size = detections[0].first.size();
    int calibration_flags = 0;

    // calibrate camera using aruco markers
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    double error = aruco::calibrateCameraAruco(
        all_corners_flatten, all_ids_flatten, markers_per_frame, board, image_size, camera_matrix,
        distortion_coeffs, cv::noArray(), cv::noArray(), calibration_flags);
    is::info("Initial Reprojection error: {}", error);

    // prepare data for charuco calibration: filter frames that cant be estimated with the
    std::vector<cv::Mat> good_charuco_corners;
    std::vector<cv::Mat> good_charuco_ids;
    good_charuco_corners.reserve(detections.size());
    good_charuco_ids.reserve(detections.size());
    for (auto&& detection : detections) {
      try {
        cv::Mat charuco_corners, charuco_ids;
        aruco::interpolateCornersCharuco(detection.second.aruco_corners, detection.second.aruco_ids,
                                         detection.first, charuco_board, charuco_corners,
                                         charuco_ids, camera_matrix, distortion_coeffs);
        good_charuco_corners.push_back(charuco_corners);
        good_charuco_ids.push_back(charuco_ids);
      } catch (...) {}
    }

    // recalibrate camera using filtered charuco corners
    std::vector<cv::Mat> rvecs, tvecs;
    error = aruco::calibrateCameraCharuco(good_charuco_corners, good_charuco_ids, charuco_board,
                                          image_size, camera_matrix, distortion_coeffs, rvecs,
                                          tvecs, 0);
    is::info("Final Reprojection error: {}\nUsed Images: {}/{}", error, good_charuco_corners.size(),
             detections.size());

    is::vision::CameraCalibration calibration;
    *calibration.mutable_calibrated_at() = is::to_timestamp(std::chrono::system_clock::now());
    calibration.set_error(error);
    calibration.mutable_resolution()->set_height(image_size.height);
    calibration.mutable_resolution()->set_width(image_size.width);
    *calibration.mutable_intrinsic() = is::to_tensor(camera_matrix);
    *calibration.mutable_distortion() = is::to_tensor(distortion_coeffs);
    return calibration;
  }
};

bool board_moved(CharucoDetection const& curr, CharucoDetection const& last,
                 int distance_threshold = 100) {
  for (int n = 0; n < curr.aruco_ids.size(); ++n) {
    auto it = std::find(last.aruco_ids.begin(), last.aruco_ids.end(), curr.aruco_ids[n]);
    if (it == last.aruco_ids.end()) continue;

    int found_at = std::distance(last.aruco_ids.begin(), it);
    int distance = cv::norm(curr.aruco_corners[n][0] - last.aruco_corners[found_at][0]);
    if (distance > distance_threshold) return true;
  }
  return false;
}

cv::Mat draw_detection(cv::Mat image, CharucoDetection const& d) {
  cv::Mat image_copy;
  cv::cvtColor(image, image_copy, CV_GRAY2BGR);
  if (d.aruco_ids.size() > 0) { cv::aruco::drawDetectedMarkers(image_copy, d.aruco_corners); }
  if (d.charuco_corners.total() > 0) {
    cv::aruco::drawDetectedCornersCharuco(image_copy, d.charuco_corners, d.charuco_ids);
  }
  return image_copy;
}

void save_images(std::string const& folder,
                 std::vector<std::pair<cv::Mat, CharucoDetection>> const& detections) {
  is::info("Saving images to folder={}", folder);
  for (int i = 0; i < detections.size(); ++i) {
    cv::imwrite(fmt::format("{}/{}.png", folder, i), detections[i].first);
  }
}

void warn_lapack_slow() {
  std::string info = cv::getBuildInformation();
  if (!std::regex_search(info, std::regex("Lapack:\\s+YES"))) {
    is::warn("OpenCV not built with lapack, calibration can be slow");
  }
}

int main(int argc, char* argv[]) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  auto leave_on_error = [](is::wire::Status const& status) {
    if (status.code() != is::wire::StatusCode::OK) is::critical("{}", status);
  };
  CalibrationOptions opts;
  leave_on_error(is::load(filename, &opts));
  leave_on_error(is::validate_message(opts));
  warn_lapack_slow();

  auto using_saved_images = opts.uri().empty();
  auto data_dir = fmt::format(opts.data_dir(), opts.camera_id());
  auto iopts = opts.intrinsic();
  auto charuco = Charuco{iopts.dictionary(), iopts.marker_length(), iopts.square_length(),
                         iopts.n_squares_x(), iopts.n_squares_y()};
  std::vector<std::pair<cv::Mat, CharucoDetection>> detections;
  if (!using_saved_images) {
    auto topic = fmt::format(opts.topic(), opts.camera_id());
    is::info("Using uri={} topic={}", opts.uri(), topic);
    auto channel = is::Channel{opts.uri()};
    auto subscription = is::Subscription{channel};
    subscription.subscribe(topic);

    while (detections.size() != iopts.samples()) {
      auto message = channel.consume();
      auto maybe_image = message.unpack<is::vision::Image>();
      std::vector<char> coded(maybe_image->data().begin(), maybe_image->data().end());
      auto image = cv::imdecode(coded, CV_LOAD_IMAGE_GRAYSCALE);

      auto detection = charuco.detect(image);

      bool detected_atleast_half =
          detection.aruco_ids.size() >= size_t(iopts.n_squares_x() * iopts.n_squares_y() / 2);
      bool is_first = detections.empty();
      if (detected_atleast_half && (is_first || board_moved(detection, detections.back().second))) {
        detections.emplace_back(image, detection);
      }

      auto image_copy = draw_detection(image, detection);
      cv::putText(image_copy, fmt::format("{}/{}", detections.size(), iopts.samples()),
                  cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.25, cv::Scalar(0, 255, 0), 2);
      cv::imshow("detection", image_copy);
      auto key = cv::waitKey(1);
      if (key == 27) { return 0; }
    }
    cv::destroyWindow("detection");
  } else {
    is::info("Using folder={}", data_dir);
    auto images = cv::VideoCapture(fmt::format("{}/%d.png", data_dir));
    while (images.isOpened()) {
      cv::Mat image;
      images >> image;
      if (image.empty()) break;
      auto detection = charuco.detect(image);
      detections.emplace_back(image, detection);
    }
  }

  is::info("Calibrating camera id={}", opts.camera_id());
  auto calibration = charuco.calibrate_camera(detections);
  calibration.set_id(opts.camera_id());

  std::string calibration_file = data_dir;
  std::replace(calibration_file.begin(), calibration_file.end(), '/', '_');
  is::save(fmt::format("{}/{}.json", data_dir, calibration_file), calibration);
  if (opts.save_images() && !using_saved_images) save_images(data_dir, detections);

  return 0;
}
