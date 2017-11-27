#include <is/msgs/camera.pb.h>
#include <is/is.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "util.hpp"

int main(int argc, char* argv[]) {
  std::string uri, cam_id;
  float marker_len, axis_offset;
  int dict_id, marker_id;

  is::po::options_description opts("options");
  auto&& opt_add = opts.add_options();
  opt_add("uri,u", is::po::value<std::string>(&uri)->required(), "broker uri");
  opt_add("camera,c", is::po::value<std::string>(&cam_id)->required(), "camera id");
  opt_add("dictionary,d", is::po::value<int>(&dict_id)->default_value(0), "dictionary id");
  opt_add("marker,i", is::po::value<int>(&marker_id)->default_value(0), "marker id");
  opt_add("marker-length,m", is::po::value<float>(&marker_len)->required(), "marker length [m]");
  opt_add("axis-offset,o", is::po::value<float>(&axis_offset)->default_value(0),
          "offset from marker topleft point [m]");
  is::parse_program_options(argc, argv, opts);

  auto channel = is::rmq::Channel::CreateFromUri(uri);
  auto tag = is::declare_queue(channel);
  is::subscribe(channel, tag, fmt::format("CameraGateway.{}.Frame", cam_id));

  auto detector_parameters = cv::aruco::DetectorParameters::create();
  detector_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));

  auto calibration = is::load_from_json<is::vision::CameraCalibration>(fmt::format("{}.json", cam_id));
  auto intrinsic = cv_matrix_view(calibration->mutable_intrinsic());
  auto distortion = cv_matrix_view(calibration->mutable_distortion());

  for (;;) {
    auto envelope = channel->BasicConsumeMessage(tag);
    auto maybe_image = is::unpack<is::vision::Image>(envelope);

    std::vector<char> coded(maybe_image->data().begin(), maybe_image->data().end());
    auto image = cv::imdecode(coded, CV_LOAD_IMAGE_GRAYSCALE);

    std::vector<int> all_ids;
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::detectMarkers(image, dictionary, all_corners, all_ids, detector_parameters);

    if (all_ids.size() != 0) {
      cv::aruco::estimatePoseSingleMarkers(all_corners, marker_len, intrinsic, distortion, rvecs, tvecs);
      cv::cvtColor(image, image, CV_GRAY2BGR);
      cv::aruco::drawDetectedMarkers(image, all_corners, all_ids);
    }

    auto found = std::find(all_ids.begin(), all_ids.end(), marker_id);
    int index = std::distance(all_ids.begin(), found);
    if (found != all_ids.end())
      cv::aruco::drawAxis(image, intrinsic, distortion, rvecs[index], tvecs[index], marker_len * 0.5f);

    cv::imshow("Extrinsic", image);
    auto key = cv::waitKey(1);
    if (key == 27) {  // leave
      return 0;
    }

    if (key == 'k' && found != all_ids.end()) {
      cv::Mat offset_translation = cv::Mat::eye(4, 4, CV_64F);
      offset_translation.at<double>(0, 3) = axis_offset;
      offset_translation.at<double>(1, 3) = axis_offset;

      cv::Mat rotation, extrinsic;
      cv::Rodrigues(rvecs[index], rotation);
      cv::hconcat(rotation, tvecs[index], extrinsic);
      cv::Mat lastRow = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
      cv::vconcat(extrinsic, lastRow, extrinsic);
      std::cout << "Extrinsic: " << extrinsic << std::endl;

      if (std::fabs(axis_offset) > 1e-3) {
        std::cout << "Offset: " << offset_translation << std::endl;
        extrinsic = extrinsic * offset_translation;
        std::cout << "Extrinsic w/ offset: " << extrinsic << std::endl;
      }

      *(calibration->mutable_extrinsic()->mutable_tf()) = make_double_tensor(extrinsic.inv());
      calibration->PrintDebugString();
      is::save_to_json(fmt::format("{}.json", cam_id), *calibration);
      return 0;
    }
  }
}
