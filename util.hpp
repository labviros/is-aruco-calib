#ifndef __TENSOR_UTILS_HPP__
#define __TENSOR_UTILS_HPP__

#include <is/msgs/common.pb.h>
#include <opencv2/core.hpp>

is::common::Tensor make_double_tensor(cv::Mat mat) {
  is::common::Tensor tensor;
  tensor.mutable_shape()->add_dims()->set_size(mat.rows);
  tensor.mutable_shape()->add_dims()->set_size(mat.cols);
  tensor.set_type(is::common::DataType::DOUBLE_TYPE);

  if (mat.rows != 0 && mat.cols != 0) {
    auto doubles = tensor.mutable_doubles();
    doubles->Reserve(mat.rows * mat.cols);
    std::copy(mat.begin<double>(), mat.end<double>(), google::protobuf::RepeatedFieldBackInserter(doubles));
  }
  return tensor;
}

cv::Mat cv_matrix_view(is::common::Tensor* tensor) {
  auto shape = tensor->shape();
  int rows = 0;
  int cols = 0;

  if (shape.dims_size() == 1) {
    rows = 1;
    cols = shape.dims(0).size();
  } else if (shape.dims_size() == 2) {
    rows = shape.dims(0).size();
    cols = shape.dims(1).size();
  }

  if (tensor->type() == is::common::DataType::FLOAT_TYPE)
    return cv::Mat(rows, cols, CV_32F, tensor->mutable_floats()->mutable_data());
  if (tensor->type() == is::common::DataType::DOUBLE_TYPE)
    return cv::Mat(rows, cols, CV_64F, tensor->mutable_doubles()->mutable_data());
  if (tensor->type() == is::common::DataType::INT32_TYPE)
    return cv::Mat(rows, cols, CV_32S, tensor->mutable_ints32()->mutable_data());
  return cv::Mat();
}

#endif