#include <fstream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <is/is.hpp>
#include <is/msgs/camera.hpp>

using namespace is::msg::camera;
using namespace std;
using namespace cv;
namespace po = boost::program_options;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string source_entity;
  std::string parameters_path;

  int dictionary_id;
  int marker_id;
  float marker_length;
  float axis_offset;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,h", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("source-entity,s", po::value<std::string>(&source_entity),
          "the name of the entity where this service will consume images");
  options("camera-parameters,p", po::value<std::string>(&parameters_path)->default_value("./camera-parameters.yml"),
          "path to camera parameters directory");

  options("marker-dictionary,d", po::value<int>(&dictionary_id)->required(),
          "aruco dictionary id:\n"
          "DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
          "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
          "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
          "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16");
  options("marker-id,m", po::value<int>(&marker_id)->required(), "id of the marker to be tracked");
  options("marker-length,l", po::value<float>(&marker_length)->required(), "marker side length in meters");
  options("axis-offset,o", po::value<float>(&axis_offset)->default_value(0), "");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);

  std::string config_path = "aruco-extrinsic-calib.conf";
  std::ifstream config_file(config_path);
  if (config_file.good()) {
    is::log::info("Reading config file {} ...", config_path);
    po::store(po::parse_config_file(config_file, description), vm);
  }

  auto enviroment_map = [](std::string env) -> std::string {
    std::string prefix = "IS_";
    if (!boost::starts_with(env, prefix))
      return "";
    boost::replace_all(env, "_", "-");
    boost::to_lower(env);
    return env.substr(prefix.size());
  };
  po::store(po::parse_environment(description, enviroment_map), vm);

  po::notify(vm);
  if (vm.count("help") || !vm.count("source-entity")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto source_id = source_entity.substr(source_entity.find_last_of('.') + 1);
  auto source_topic = fmt::format("{}.frame", source_entity);
  is::log::info("{}", source_topic);

  auto is = is::connect(uri);
  auto images = is.subscribe(source_topic);

  auto detector_parameters = cv::aruco::DetectorParameters::create();
  detector_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;

  {
    cv::FileStorage fs(parameters_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      is::log::critical("Unable to read camera parameters from: \"{}\"", parameters_path);
    }

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coefficients;
  }

  for (;;) {
    auto envelope = is.consume(images);
    auto compressed_image = is::msgpack<CompressedImage>(envelope);
    auto image = cv::imdecode(compressed_image.data, CV_LOAD_IMAGE_GRAYSCALE);

    std::vector<int> all_ids;
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::detectMarkers(image, dictionary, all_corners, all_ids, detector_parameters);

    auto found = std::find(all_ids.begin(), all_ids.end(), marker_id);
    if (found != all_ids.end()) {
      int index = std::distance(all_ids.begin(), found);

      cv::aruco::estimatePoseSingleMarkers(all_corners, marker_length, camera_matrix, distortion_coefficients, rvecs,
                                           tvecs);

      cvtColor(image, image, CV_GRAY2BGR);
      cv::aruco::drawDetectedMarkers(image, all_corners, all_ids);
      cv::aruco::drawAxis(image, camera_matrix, distortion_coefficients, rvecs[index], tvecs[index],
                          marker_length * 0.5f);

      cv::Mat offset_translation = cv::Mat::eye(4, 4, CV_64F);
      offset_translation.at<double>(0,3) = axis_offset;
      offset_translation.at<double>(1,3) = axis_offset;

      cv::Mat rotation, extrinsic;
      cv::Rodrigues(rvecs[index], rotation);
      cv::hconcat(rotation, tvecs[index], extrinsic);
      cv::Mat lastRow = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
      cv::vconcat(extrinsic, lastRow, extrinsic);
      std::cout << "Extrinsic: " << extrinsic << std::endl;
       
      if (std::fabs(axis_offset) > 1e-6) {
        std::cout << "Offset: " << offset_translation << std::endl;
        extrinsic = extrinsic * offset_translation;
        std::cout << "Extrinsic w/ offset: " << extrinsic << std::endl;
      }
      cv::imshow("Extrinsic", image);
      
      auto key = waitKeyEx(0);
      if (key == 27) {  // leave
        return 1;
      } else if (key == 'k') {  // ok
        cv::FileStorage fs(parameters_path, cv::FileStorage::APPEND);
        if (!fs.isOpened()) {
          is::log::critical("Unable to append camera parameters from: \"{}\"", parameters_path);
        }
        fs << "world_matrix" << extrinsic;
        return 0;
      }
    }
  }
}
