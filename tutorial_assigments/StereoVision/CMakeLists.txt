CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( calib )
FIND_PACKAGE( OpenCV REQUIRED )
#find_package(PCL 1.2 REQUIRED)

ADD_EXECUTABLE( test test.cpp )
TARGET_LINK_LIBRARIES( test ${OpenCV_LIBS} )

ADD_EXECUTABLE( calib chessboard1.cpp )
TARGET_LINK_LIBRARIES( calib ${OpenCV_LIBS} )

ADD_EXECUTABLE( stereoCalib stereoCalibration.cpp )
TARGET_LINK_LIBRARIES( stereoCalib ${OpenCV_LIBS} )

ADD_EXECUTABLE( lensDist LensDistortion.cpp )
TARGET_LINK_LIBRARIES( lensDist ${OpenCV_LIBS} )

ADD_EXECUTABLE( imageRect ImageRectification.cpp )
TARGET_LINK_LIBRARIES( imageRect ${OpenCV_LIBS} )