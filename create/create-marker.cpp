#include <fstream>
#include <iostream>
#include <sstream>
#include "google/protobuf/util/json_util.h"
#include "is/msgs/io.hpp"
#include "is/msgs/validate.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/highgui.hpp"
#include "options.pb.h"
#include "spdlog/fmt/fmt.h"

void leave_on_error(is::wire::Status const& status) {
  if (status.code() != is::wire::StatusCode::OK) {
    std::cerr << status.why() << std::endl;
    std::exit(-1);
  }
}

int main(int argc, char* argv[]) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";

  CreateMarker options;
  leave_on_error(is::load(filename, &options));
  leave_on_error(is::validate_message(options));

  cv::Mat image;
  if (options.has_charuco()) {
    auto opts = options.charuco();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(opts.dictionary()));

    int margins = opts.square_length() - opts.marker_length();

    cv::Size image_size;
    image_size.width = opts.n_squares_x() * opts.square_length() + 2 * margins;
    image_size.height = opts.n_squares_y() * opts.square_length() + 2 * margins;

    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::aruco::CharucoBoard::create(opts.n_squares_x(), opts.n_squares_y(),
                                        opts.square_length(), opts.marker_length(), dictionary);

    board->draw(image_size, image, margins, opts.border_bits());

    std::stringstream ss;
    ss << "charuco_" << opts.n_squares_x() << "x" << opts.n_squares_y() << "_"
       << ArucoDictionary_Name(opts.dictionary()) << ".png";

    cv::imwrite(ss.str(), image);
  } else if (options.has_aruco()) {
    auto opts = options.aruco();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(opts.dictionary()));

    cv::aruco::drawMarker(dictionary, opts.marker_id(), opts.marker_length(), image,
                          opts.border_bits());

    std::stringstream ss;
    ss << "aruco_" << opts.marker_id() << "_" << ArucoDictionary_Name(opts.dictionary()) << ".png";
    cv::imwrite(ss.str(), image);
  }

  cv::imshow("board", image);
  cv::waitKey(0);

  return 0;
}
