#ifndef ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/serialization/export.hpp>

namespace aslam {
namespace cameras {

/**
 * \class GridCalibrationTargetCharuco
 * \brief a ChArUco calibration target (chessboard + ArUco markers) detected via OpenCV aruco
 *
 * The grid is the chessboard interior corners: (squaresY-1) x (squaresX-1) points.
 * Each corner is identified by its ArUco id, which allows partial observation
 * (occluded corners are simply marked as not observed, like the aprilgrid).
 */
class GridCalibrationTargetCharuco : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetCharuco> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCharuco> ConstPtr;

  // target extraction options
  struct CharucoOptions {
    CharucoOptions() :
      doSubpixRefinement(true),
      showExtractionVideo(false),
      minCornersForValidObs(4) {};

    /// \brief subpixel refinement of extracted corners
    bool doSubpixRefinement;

    /// \brief show video during extraction
    bool showExtractionVideo;

    /// \brief min. number of charuco corners for a valid observation
    unsigned int minCornersForValidObs;

    /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER();
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar << BOOST_SERIALIZATION_NVP(minCornersForValidObs);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar >> BOOST_SERIALIZATION_NVP(minCornersForValidObs);
    }
  };

  /// \brief initialize based on a ChArUco board geometry.
  ///        squaresX/squaresY: number of chessboard squares in each direction
  ///        (interior corner count is squaresX-1 x squaresY-1).
  ///        dictionaryId: cv::aruco::PREDEFINED_DICTIONARY_NAME enum value.
  GridCalibrationTargetCharuco(size_t squaresX, size_t squaresY, double squareLength,
                               double markerLength, int dictionaryId,
                               const CharucoOptions &options = CharucoOptions());

  virtual ~GridCalibrationTargetCharuco() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief number of chessboard squares in x/y direction
  size_t _squaresX, _squaresY;

  /// \brief chessboard square side length [m]
  double _squareLength;

  /// \brief ArUco marker side length [m]
  double _markerLength;

  /// \brief ArUco predefined dictionary id (cv::aruco::PREDEFINED_DICTIONARY_NAME)
  int _dictionaryId;

  /// \brief target extraction options
  CharucoOptions _options;

  // ArUco board / dictionary / detector parameters. These are NOT serialized;
  // they are rebuilt in initialize() (called from the constructor and from load()).
  cv::Ptr<cv::aruco::Dictionary> _dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> _board;
  cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetCharuco();

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetCharuco, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCharuco *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_squaresX);
    ar << BOOST_SERIALIZATION_NVP(_squaresY);
    ar << BOOST_SERIALIZATION_NVP(_squareLength);
    ar << BOOST_SERIALIZATION_NVP(_markerLength);
    ar << BOOST_SERIALIZATION_NVP(_dictionaryId);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetCharuco, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCharuco *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_squaresX);
    ar >> BOOST_SERIALIZATION_NVP(_squaresY);
    ar >> BOOST_SERIALIZATION_NVP(_squareLength);
    ar >> BOOST_SERIALIZATION_NVP(_markerLength);
    ar >> BOOST_SERIALIZATION_NVP(_dictionaryId);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCharuco);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCharuco::CharucoOptions);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetCharuco);

#endif /* ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP */
