#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <aslam/cameras/GridCalibrationTargetCharuco.hpp>
#include <sm/eigen/serialization.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct a ChArUco calibration target.
///   squaresX/squaresY: number of chessboard squares in x/y direction
///   (the interior corner grid is (squaresY-1) x (squaresX-1)).
///   squareLength/markerLength: side lengths [m] (markerLength < squareLength).
///   dictionaryId: cv::aruco::PREDEFINED_DICTIONARY_NAME enum value.
GridCalibrationTargetCharuco::GridCalibrationTargetCharuco(
    size_t squaresX, size_t squaresY, double squareLength, double markerLength,
    int dictionaryId, const CharucoOptions &options)
    : GridCalibrationTargetBase(squaresY - 1, squaresX - 1),
      _squaresX(squaresX),
      _squaresY(squaresY),
      _squareLength(squareLength),
      _markerLength(markerLength),
      _dictionaryId(dictionaryId),
      _options(options) {
  SM_ASSERT_GE(Exception, squaresX, static_cast<size_t>(2),
               "squaresX has to be >= 2 (need at least one interior corner)");
  SM_ASSERT_GE(Exception, squaresY, static_cast<size_t>(2),
               "squaresY has to be >= 2 (need at least one interior corner)");
  SM_ASSERT_GT(Exception, squareLength, 0.0, "squareLength has to be positive");
  SM_ASSERT_GT(Exception, markerLength, 0.0, "markerLength has to be positive");
  SM_ASSERT_LT(Exception, markerLength, squareLength,
               "markerLength has to be smaller than squareLength");

  initialize();
}

/// \brief default constructor (only for serialization / pickle; members are restored in load())
GridCalibrationTargetCharuco::GridCalibrationTargetCharuco()
    : GridCalibrationTargetBase() {}

/// \brief build the ArUco dictionary / board / detector parameters and the grid points
void GridCalibrationTargetCharuco::initialize() {
  _dictionary = cv::aruco::getPredefinedDictionary(
      cv::aruco::PREDEFINED_DICTIONARY_NAME(_dictionaryId));
  _board = cv::aruco::CharucoBoard::create(
      _squaresX, _squaresY, static_cast<float>(_squareLength),
      static_cast<float>(_markerLength), _dictionary);
  _detectorParams = cv::aruco::DetectorParameters::create();

  createGridPoints();

  if (_options.showExtractionVideo) {
    cv::namedWindow("Charuco corners", cv::WINDOW_NORMAL);
    cv::resizeWindow("Charuco corners", 640, 480);
    cv::startWindowThread();
  }
}

/// \brief initialize the grid points from OpenCV's own precalculated chessboard corners.
///   OpenCV stores them row-major and the ids returned by interpolateCornersCharuco
///   index directly into this vector, so copying them guarantees index consistency.
void GridCalibrationTargetCharuco::createGridPoints() {
  _points.resize(size(), 3);
  const std::vector<cv::Point3f> &obj = _board->chessboardCorners;
  SM_ASSERT_EQ(Exception, obj.size(), size(),
               "unexpected number of chessboard corners");
  for (size_t i = 0; i < obj.size(); ++i)
    _points.row(i) = Eigen::Matrix<double, 1, 3>(obj[i].x, obj[i].y, obj[i].z);
}

/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetCharuco::computeObservation(
    const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
    std::vector<bool> &outCornerObserved) const {
  outImagePoints.resize(size(), 2);
  outCornerObserved.assign(size(), false);

  // detectMarkers / interpolateCornersCharuco expect a single-channel gray image
  cv::Mat grayImage;
  if (image.channels() == 1)
    grayImage = image;
  else
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(grayImage, _dictionary, markerCorners, markerIds,
                           _detectorParams);
  if (markerIds.empty())
    return false;

  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, grayImage,
                                       _board, charucoCorners, charucoIds);
  if (charucoIds.size() < _options.minCornersForValidObs)
    return false;

  // optional subpixel refinement of the interpolated corners
  if (_options.doSubpixRefinement) {
    cv::cornerSubPix(grayImage, charucoCorners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
  }

  if (_options.showExtractionVideo) {
    cv::Mat imageCopy = grayImage.clone();
    cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);
    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds);
    cv::imshow("Charuco corners", imageCopy);
    cv::waitKey(1);
  }

  // map by charuco id: ids index directly into _board->chessboardCorners (and
  // therefore into _points, filled from that same vector). Unobserved corners
  // keep outCornerObserved == false (partial observation, like the aprilgrid).
  for (size_t k = 0; k < charucoIds.size(); ++k) {
    const int id = charucoIds[k];
    SM_ASSERT_GE_LT(Exception, id, 0, static_cast<int>(size()),
                    "charuco id out of range");
    outImagePoints.row(id) =
        Eigen::Matrix<double, 1, 2>(charucoCorners[k].x, charucoCorners[k].y);
    outCornerObserved[id] = true;
  }

  return true;
}

}  // namespace cameras
}  // namespace aslam

// export explicit instantiations for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCharuco);
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCharuco::CharucoOptions);
