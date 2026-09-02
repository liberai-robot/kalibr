// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <vector>
#include <boost/cstdint.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <aslam/cameras/GridCalibrationTargetCharuco.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

namespace {

// Grayscale image type for numpy_eigen auto-conversion (与 aslam_cv_python 一致).
typedef Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_t;

/// \brief Python-facing computeObservation: numpy 灰度图 -> (success, imagePoints, cornerObserved)。
///   GridCalibrationTargetBase 未在 Python 暴露虚函数 computeObservation，
///   因此这里显式包装 Charuco 的检测逻辑，供在线检测节点直接调用。
boost::python::tuple computeObservationPy(
    aslam::cameras::GridCalibrationTargetCharuco *target,
    const image_t &image) {
  cv::Mat cvImage;
  eigen2cv(image, cvImage);

  Eigen::MatrixXd outImagePoints;
  std::vector<bool> outCornerObserved;
  bool success = target->computeObservation(cvImage, outImagePoints,
                                            outCornerObserved);

  // std::vector<bool> 没有 numpy_eigen 转换器 -> 打包成 0/1 的 int 向量返回
  Eigen::VectorXi observed(outCornerObserved.size());
  for (size_t i = 0; i < outCornerObserved.size(); ++i)
    observed(i) = outCornerObserved[i] ? 1 : 0;

  return boost::python::make_tuple(success, outImagePoints, observed);
}

}  // namespace

BOOST_PYTHON_MODULE(libaslam_cameras_charuco_python)
{
  using namespace boost::python;
  using namespace aslam::cameras;

  class_<GridCalibrationTargetCharuco::CharucoOptions>("CharucoOptions", init<>())
    .def_readwrite("doSubpixRefinement", &GridCalibrationTargetCharuco::CharucoOptions::doSubpixRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetCharuco::CharucoOptions::showExtractionVideo)
    .def_readwrite("minCornersForValidObs", &GridCalibrationTargetCharuco::CharucoOptions::minCornersForValidObs)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCharuco::CharucoOptions>());

  class_<GridCalibrationTargetCharuco, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetCharuco>, boost::noncopyable>(
      "GridCalibrationTargetCharuco",
      init<size_t, size_t, double, double, int, GridCalibrationTargetCharuco::CharucoOptions>(
          "GridCalibrationTargetCharuco(size_t squaresX, size_t squaresY, double squareLength, double markerLength, int dictionaryId, CharucoOptions options)"))
      .def(init<size_t, size_t, double, double, int>(
          "GridCalibrationTargetCharuco(size_t squaresX, size_t squaresY, double squareLength, double markerLength, int dictionaryId)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def("computeObservation", &computeObservationPy,
           "computeObservation(gray_image) -> (success, imagePoints, cornerObserved)")
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCharuco>());
}
