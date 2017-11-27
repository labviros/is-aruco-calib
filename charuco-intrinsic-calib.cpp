/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <is/msgs/camera.pb.h>
#include <is/msgs/common.pb.h>
#include <SFML/Audio.hpp>
#include <is/is.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "util.hpp"

bool board_moved(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> last_ids,
                 std::vector<std::vector<cv::Point2f>> last_corners) {
  for (std::size_t n = 0; n < ids.size(); ++n) {
    std::size_t found_at = std::distance(last_ids.begin(), std::find(last_ids.begin(), last_ids.end(), ids[n]));
    if (found_at == last_ids.size())
      continue;

    auto distance = cv::norm(corners[n][0] - last_corners[found_at][0]);
    if (distance > 50)
      return true;
  }
  return false;
}

void save_camera_calibration(std::string const& id, cv::Size size, double error, cv::Mat intrinsic,
                             cv::Mat distortion) {
  is::vision::CameraCalibration calibration;
  calibration.set_id(id);
  *calibration.mutable_calibrated_at() = is::current_time();
  calibration.set_error(error);
  calibration.mutable_resolution()->set_height(size.height);
  calibration.mutable_resolution()->set_width(size.width);
  *calibration.mutable_intrinsic() = make_double_tensor(intrinsic);
  *calibration.mutable_distortion() = make_double_tensor(distortion);
  auto filename = fmt::format("{}.json", id);
  is::save_to_json(filename, calibration);
}

void configure_camera(is::rmq::Channel::ptr_t channel, std::string const& tag, std::string const& id) {
  is::vision::CameraConfig config;
  config.mutable_sampling()->set_frequency(10.0f);
  config.mutable_image()->mutable_color_space()->set_value(is::vision::ColorSpaces::GRAY);

  auto endpoint = fmt::format("CameraGateway.{}.SetConfig", id);
  is::request(channel, tag, endpoint, config);
  auto envelope = is::consume_for(channel, tag, is::pb::TimeUtil::SecondsToDuration(2));
  if (envelope == nullptr)
    is::critical("No response from camera {}", endpoint);

  auto status = is::rpc_status(envelope);
  if (status.code() != is::common::StatusCode::OK)
    is::critical("Failed to configure camera {}: {}", id, is::common::StatusCode_Name(status.code()), status.why());
}

int main(int argc, char* argv[]) {
  std::string uri, cam_id;
  float marker_len, square_len;
  int dict_id, squares_x, squares_y, n_samples;

  is::po::options_description opts("options");
  auto&& opt_add = opts.add_options();
  opt_add("uri,u", is::po::value<std::string>(&uri)->required(), "broker uri");
  opt_add("camera,c", is::po::value<std::string>(&cam_id)->required(), "camera id");
  opt_add("dictionary,d", is::po::value<int>(&dict_id)->default_value(0), "dictionary id");
  opt_add("marker-length,m", is::po::value<float>(&marker_len)->required(), "marker length [m]");
  opt_add("square-length,s", is::po::value<float>(&square_len)->required(), "square length [m]");
  opt_add("width,w", is::po::value<int>(&squares_x)->required(), "number of squares in X direction");
  opt_add("height,h", is::po::value<int>(&squares_y)->required(), "number of squares in Y direction");
  opt_add("samples", is::po::value<int>(&n_samples)->default_value(100), "number of images to be used");
  is::parse_program_options(argc, argv, opts);

  sf::SoundBuffer buffer;
  buffer.loadFromFile("captured.wav");
  sf::Sound sound;
  sound.setBuffer(buffer);

  int calibrationFlags = 0;
  float aspectRatio = 1;
  auto detectorParams = cv::aruco::DetectorParameters::create();
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));
  auto charucoboard = cv::aruco::CharucoBoard::create(squares_x, squares_y, square_len, marker_len, dictionary);
  auto board = charucoboard.staticCast<cv::aruco::Board>();

  auto channel = is::rmq::Channel::CreateFromUri(uri);
  auto tag = is::declare_queue(channel);
  configure_camera(channel, tag, cam_id);
  is::subscribe(channel, tag, fmt::format("CameraGateway.{}.Frame", cam_id));

  std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
  std::vector<std::vector<int>> allIds;
  std::vector<cv::Mat> allImgs;

  save_camera_calibration(cam_id, cv::Size(), 0.99, cv::Mat(), cv::Mat());


  while (n_samples) {
    auto envelope = channel->BasicConsumeMessage(tag);
    auto maybe_image = is::unpack<is::vision::Image>(envelope);

    std::vector<char> coded(maybe_image->data().begin(), maybe_image->data().end());
    auto image = cv::imdecode(coded, CV_LOAD_IMAGE_GRAYSCALE);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

    cv::Mat currentCharucoCorners, currentCharucoIds;
    if (ids.size() > 0)
      cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);

    cv::Mat imageCopy;
    cv::cvtColor(image, imageCopy, CV_GRAY2BGR);
    if (ids.size() > 0)
      cv::aruco::drawDetectedMarkers(imageCopy, corners);

    if (currentCharucoCorners.total() > 0)
      cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

    cv::putText(imageCopy, fmt::format("'ESC' to finish and calibrate. Captured: {}", allIds.size()), cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1.25, cv::Scalar(0, 255, 0), 2);

    cv::imshow("Source", imageCopy);
    cv::waitKey(1);

    if (ids.size() >= size_t(squares_x * squares_y / 2) &&
        (allIds.empty() || board_moved(ids, corners, allIds.back(), allCorners.back()))) {
      sound.play();
      allCorners.push_back(corners);
      allIds.push_back(ids);
      allImgs.push_back(image);
      --n_samples;
    }
  }

  buffer.loadFromFile("done.wav");
  sound.play();

  cv::destroyWindow("Source");
  if (allIds.size() < 1) {
    is::critical("Not enough captures for calibration");
  }

  cv::Size imgSize = allImgs[0].size();

  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double repError;

  if (calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = aspectRatio;
  }

  // prepare data for calibration
  std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
  std::vector<int> allIdsConcatenated;
  std::vector<int> markerCounterPerFrame;
  markerCounterPerFrame.reserve(allCorners.size());
  for (unsigned int i = 0; i < allCorners.size(); i++) {
    markerCounterPerFrame.push_back((int)allCorners[i].size());
    for (unsigned int j = 0; j < allCorners[i].size(); j++) {
      allCornersConcatenated.push_back(allCorners[i][j]);
      allIdsConcatenated.push_back(allIds[i][j]);
    }
  }

  // calibrate camera using aruco markers
  double arucoRepErr;
  arucoRepErr =
      cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board, imgSize,
                                      cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);

  // prepare data for charuco calibration
  int nFrames = (int)allCorners.size();
  std::vector<cv::Mat> allCharucoCorners;
  std::vector<cv::Mat> allCharucoIds;
  std::vector<cv::Mat> filteredImages;
  allCharucoCorners.reserve(nFrames);
  allCharucoIds.reserve(nFrames);

  for (int i = 0; i < nFrames; i++) {
    // interpolate using camera parameters
    try {
      cv::Mat currentCharucoCorners, currentCharucoIds;
      cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard, currentCharucoCorners,
                                           currentCharucoIds, cameraMatrix, distCoeffs);

      allCharucoCorners.push_back(currentCharucoCorners);
      allCharucoIds.push_back(currentCharucoIds);
      filteredImages.push_back(allImgs[i]);
    } catch (...) {
      is::warn("Failed to interpolate {}", i);
    }
  }

  if (allCharucoCorners.size() < 4) {
    is::critical("Not enough corners for calibration");
  }

  // calibrate camera using charuco
  repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix,
                                               distCoeffs, rvecs, tvecs, calibrationFlags);

  is::info("Reprojection error - Final: {} | ArUco: {}", repError, arucoRepErr);
  save_camera_calibration(cam_id, imgSize, repError, cameraMatrix, distCoeffs);

  // show interpolated charuco corners for debugging
  for (int n = 0; n < filteredImages.size(); ++n) {
    cv::aruco::drawDetectedCornersCharuco(filteredImages[n], allCharucoCorners[n], allCharucoIds[n]);
  }

  int frame = 0;
  for (;;) {
    cv::imshow("Calibration Image", filteredImages[frame]);
    auto key = cv::waitKey();
    if (key == 27) {  // leave
      break;
    } else if (key == 'b' || key == 'p' || key == 'j') {  // backward
      frame = std::max(frame - 1, 0);
    } else {  // forward
      frame = std::min(frame + 1, static_cast<int>(filteredImages.size()) - 1);
    }
  }

  return 0;
}
