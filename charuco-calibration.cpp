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

#include <ctime>
#include <fstream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>

#include <SFML/Audio.hpp>

using namespace std;
using namespace cv;

namespace po = boost::program_options;

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters>& params) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

static bool saveCameraParams(const string& entity, Size imageSize, float aspectRatio, int flags,
                             const Mat& cameraMatrix, const Mat& distCoeffs, double totalAvgErr) {
  time_t tt;
  time(&tt);
  struct tm* t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  std::string filename =
      fmt::format("{}-{}-{}-{}-{}.yml", entity, t2->tm_year + 1900, t2->tm_mon, t2->tm_mday, t2->tm_min);

  FileStorage fs(filename, FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  fs << "calibration_time" << buf;
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  if (flags & CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if (flags != 0) {
    sprintf(buf, "flags: %s%s%s%s", flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
  }

  fs << "flags" << flags;
  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;
  fs << "avg_reprojection_error" << totalAvgErr;

  return true;
}

bool board_moved(vector<int> ids, vector<vector<Point2f>> corners, vector<int> last_ids,
                 vector<vector<Point2f>> last_corners) {
  for (size_t n = 0; n < ids.size(); ++n) {
    size_t found_at = std::distance(last_ids.begin(), std::find(last_ids.begin(), last_ids.end(), ids[n]));
    if (found_at == last_ids.size())
      continue;

    auto distance = cv::norm(corners[n][0] - last_corners[found_at][0]);
    if (distance > 50)
      return true;
  }
  return false;
}

int main(int argc, char* argv[]) {
  std::string uri, source_entity, detector_params_path;
  float marker_length, square_length;
  int dictionary_id, squares_x, squares_y;
  bool show_images, refind_strategy;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,h", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("source-entity,s", po::value<std::string>(&source_entity),
          "the name of the entity where this service will consume images");
  options("marker-dictionary,d", po::value<int>(&dictionary_id)->required(),
          "aruco dictionary id:\n"
          "DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
          "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
          "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
          "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16");
  options("marker-length,ml", po::value<float>(&marker_length)->required(), "marker side length in meters");
  options("square-length,sl", po::value<float>(&square_length)->required(), "square side length in meters");
  options("width,w", po::value<int>(&squares_x)->required(), "number of squares in X direction");
  options("height,h", po::value<int>(&squares_y)->required(), "number of squares in Y direction");
  options("detector-param,dp", po::value<std::string>(&detector_params_path),
          "path aruco algorithm yml configuration file");
  options("refind-strategy", po::bool_switch(&refind_strategy)->default_value(false), "");
  options("show-images", po::bool_switch(&show_images)->default_value(false),
          "Show detected chessboard corners after calibration");

  auto enviroment_map = [](std::string env) -> std::string {
    std::string prefix = "IS_";
    if (!boost::starts_with(env, prefix))
      return "";
    boost::replace_all(env, "_", "-");
    boost::to_lower(env);
    return env.substr(prefix.size());
  };

  std::ifstream config_file("options.conf");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  if (config_file.good()) {
    po::store(po::parse_config_file(config_file, description), vm);
  }
  po::store(po::parse_environment(description, enviroment_map), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("source-entity")) {
    std::cout << description << std::endl;
    return 1;
  }

  sf::SoundBuffer buffer;
  buffer.loadFromFile("captured.wav");
  sf::Sound sound;
  sound.setBuffer(buffer);

  int calibrationFlags = 0;
  float aspectRatio = 1;
  // if (parser.has("a")) {
  //   calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
  //   aspectRatio = parser.get<float>("a");
  // }
  // if (parser.get<bool>("zt"))
  //   calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
  // if (parser.get<bool>("pc"))
  //   calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
  if (vm.count("detector-params")) {
    bool ok = readDetectorParameters(detector_params_path, detectorParams);
    if (!ok) {
      is::log::critical("Invalid detector parameters file");
    }
  }

  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
  Ptr<aruco::CharucoBoard> charucoboard =
      aruco::CharucoBoard::create(squares_x, squares_y, square_length, marker_length, dictionary);
  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

  // collect data from each frame
  vector<vector<vector<Point2f>>> allCorners;
  vector<vector<int>> allIds;
  vector<Mat> allImgs;
  Size imgSize;

  auto is = is::connect("amqp://edge.is:30000");
  auto client = is::make_client(is);
  is::msg::common::SamplingRate rate;
  rate.rate = 20.0;
  is::msg::camera::Configuration config;
  config.sampling_rate = rate;
  config.image_type = is::msg::camera::image_type::gray;

  is::log::info("Configuring {} ...", source_entity);
  auto request = client.request(fmt::format("{}.set_configuration", source_entity), is::msgpack(config));
  auto reply = client.receive_for(std::chrono::seconds(1));
  if (reply == nullptr) {
    is::log::critical("Failed to configure {}", source_entity);
  }

  std::string topic = fmt::format("{}.frame", source_entity);
  is::log::info("Subscribing to {}", topic);
  auto ptgrey = is.subscribe(topic);

  for (;;) {
    Mat image, imageCopy;
    auto envelope = is.consume(ptgrey);
    auto compressed_image = is::msgpack<is::msg::camera::CompressedImage>(envelope);
    image = cv::imdecode(compressed_image.data, CV_LOAD_IMAGE_GRAYSCALE);

    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;

    // detect markers
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

    // refind strategy to detect more markers
    if (refind_strategy)
      aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

    // interpolate charuco corners
    Mat currentCharucoCorners, currentCharucoIds;
    if (ids.size() > 0)
      aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);

    // draw results
    image.copyTo(imageCopy);
    if (ids.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, corners);

    if (currentCharucoCorners.total() > 0)
      aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

    putText(imageCopy, "'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

    imshow("Source", imageCopy);
    char key = (char)waitKey(1);
    if (key == 27) {
      cv::destroyWindow("Source");
      break;
    }

    if (ids.size() >= size_t(squares_x * squares_y / 2) &&
        (allIds.empty() || board_moved(ids, corners, allIds.back(), allCorners.back()))) {
      sound.play();
      allCorners.push_back(corners);
      allIds.push_back(ids);
      cvtColor(image, image, CV_GRAY2BGR);
      allImgs.push_back(image);
      imgSize = image.size();
      cout << "\r" << allIds.size() << std::flush;
    }
  }

  if (allIds.size() < 1) {
    is::log::critical("Not enough captures for calibration");
  }

  Mat cameraMatrix, distCoeffs;
  vector<Mat> rvecs, tvecs;
  double repError;

  if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = aspectRatio;
  }

  // prepare data for calibration
  vector<vector<Point2f>> allCornersConcatenated;
  vector<int> allIdsConcatenated;
  vector<int> markerCounterPerFrame;
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
  arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board,
                                            imgSize, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

  // prepare data for charuco calibration
  int nFrames = (int)allCorners.size();
  vector<Mat> allCharucoCorners;
  vector<Mat> allCharucoIds;
  vector<Mat> filteredImages;
  allCharucoCorners.reserve(nFrames);
  allCharucoIds.reserve(nFrames);

  for (int i = 0; i < nFrames; i++) {
    // interpolate using camera parameters
    try {
      Mat currentCharucoCorners, currentCharucoIds;
      aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard, currentCharucoCorners,
                                       currentCharucoIds, cameraMatrix, distCoeffs);

      allCharucoCorners.push_back(currentCharucoCorners);
      allCharucoIds.push_back(currentCharucoIds);
      filteredImages.push_back(allImgs[i]);
    } catch (...) {
      is::log::warn("Failed to interpolate {}", i);
    }
  }

  if (allCharucoCorners.size() < 4) {
    is::log::critical("Not enough corners for calibration");
  }

  // calibrate camera using charuco
  repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix,
                                           distCoeffs, rvecs, tvecs, calibrationFlags);
  is::log::info("Reprojection error - Final: {} | ArUco: {}", repError, arucoRepErr);

  bool saveOk = saveCameraParams(source_entity, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);
  if (!saveOk) {
    is::log::error("Cannot save output file");
  }

  // show interpolated charuco corners for debugging
  if (show_images) {
    int frame = 0;
    for (;;) {
      Mat imageCopy = filteredImages[frame].clone();
      if (allIds[frame].size() > 0) {
        if (allCharucoCorners[frame].total() > 0) {
          aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);
        }
      }

      imshow("Calibration Image", imageCopy);
      auto key = waitKeyEx(0);
      if (key == 27) {  // leave
        break;
      } else if (key == 'b' || key == 'p' || key == 'j') {  // backward
        frame = std::max(frame - 1, 0);
      } else {  // forward
        frame = std::min(frame + 1, static_cast<int>(filteredImages.size()) - 1);
      }
    }
  }

  return 0;
}
