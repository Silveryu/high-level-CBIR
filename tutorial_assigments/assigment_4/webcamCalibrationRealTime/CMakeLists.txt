CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( webcamCalibration )
FIND_PACKAGE( OpenCV REQUIRED )
#find_package(PCL 1.2 REQUIRED)

ADD_EXECUTABLE( webcamCalibration chessboard.cpp )
TARGET_LINK_LIBRARIES( webcamCalibration ${OpenCV_LIBS} )
