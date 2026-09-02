# Import the numpy to Eigen type conversion.
import numpy_eigen
import os
import aslam_cv

isCompiled = False
pathToSo = os.path.dirname(os.path.realpath(__file__))
if os.path.isfile(os.path.join(pathToSo,"libaslam_cameras_charuco_python.so")):
    # Import the the C++ exports from your package library.
    from .libaslam_cameras_charuco_python import *
    # Import other files in the directory
    # from mypyfile import *
    isCompiled = True
else:
    print("Warning: the package aslam_cameras_charuco_python is not compiled.")
    PACKAGE_IS_NOT_COMPILED = True;

# ArUco predefined dictionary name -> OpenCV enum value (stable across OpenCV 4.x).
# The YAML config may use either a name (string) or the integer id directly.
ARUCO_DICTIONARIES = {
    'DICT_4X4_50': 0,
    'DICT_4X4_100': 1,
    'DICT_4X4_250': 2,
    'DICT_4X4_1000': 3,
    'DICT_5X5_50': 4,
    'DICT_5X5_100': 5,
    'DICT_5X5_250': 6,
    'DICT_5X5_1000': 7,
    'DICT_6X6_50': 8,
    'DICT_6X6_100': 9,
    'DICT_6X6_250': 10,
    'DICT_6X6_1000': 11,
    'DICT_7X7_50': 12,
    'DICT_7X7_100': 13,
    'DICT_7X7_250': 14,
    'DICT_7X7_1000': 15,
    'DICT_ARUCO_ORIGINAL': 16,
}
