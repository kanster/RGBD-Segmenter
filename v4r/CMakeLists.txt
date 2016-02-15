add_subdirectory(TomGine)

cmake_minimum_required (VERSION 2.6)
SET(V4R_SEGMENTER_LIGHT 1 CACHE BOOL "Surface segmenter framework" )
IF(V4R_SEGMENTER_LIGHT)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

PROJECT(v4rSegmenterLight)
SET(SOURCE_CPP
  disjoint-set.cpp
  GraphCut.cpp
  Graph.cpp
  ClusterNormalsToPlanes.cc
  ZAdaptiveNormals.cc
  SurfaceModeling.cc
  BoundaryRelations.cpp
  ColorHistogram3D.cpp
  StructuralRelationsLight.cpp
  Texture.cpp
  SegmenterLight.cpp
  ContourDetector.cpp
  svm.cpp
  SVMPredictorSingle.cpp
  Capture.cpp
  Device.cpp
  Kinect.cpp
  KinectData.cpp
  FileSystem.cc
  Utils.cc
)

SET(SOURCE_H
  disjoint-set.h
  Edge.h
  GraphCut.h
  Graph.h
  ClusterNormalsToPlanes.hh
  PPlane.h
  ZAdaptiveNormals.hh
  SurfaceModeling.hh
  BoundaryRelations.h
  ColorHistogram3D.h
  StructuralRelationsLight.h
  Texture.h
  SegmenterLight.h
  ContourDetector.h
  SurfaceModel.hpp
  Relation.h
  svm.h
  SVMPredictorSingle.h
  Capture.h
  Device.h
  Kinect.h
  KinectData.h
  FileSystem.hh
  Utils.hh
)

SET(SOURCE_MODEL
  model/param.txt
  model/PP-Trainingsset.txt.scaled.model
  model/param.txt.fast
  model/PP-Trainingsset.txt.scaled.model.fast
)

SET(SOURCE_CONFIG
  KinectConfig.xml
)

# copy svm model to system
string(REGEX REPLACE "v4r" "" SVM_INCLUDE_NAME ${PROJECT_NAME})
set(SVM_MODEL ${CMAKE_INSTALL_PREFIX}/include/v4r/${SVM_INCLUDE_NAME}/model/)
add_definitions(-DSVM_MODEL="${SVM_MODEL}")
message(STATUS "SVM_MODEL is ${SVM_MODEL}")
install(DIRECTORY DESTINATION include/v4r/${SVM_INCLUDE_NAME}/model)
install(FILES ${SOURCE_MODEL} DESTINATION include/v4r/${SVM_INCLUDE_NAME}/model)

# copy Kinect config file to system
string(REGEX REPLACE "v4r" "" INTERFACE_INCLUDE_NAME ${PROJECT_NAME})
set(KINECT_CONFIG ${CMAKE_INSTALL_PREFIX}/include/v4r/${INTERFACE_INCLUDE_NAME}/config/)
add_definitions(-DKINECT_CONFIG="${KINECT_CONFIG}${SOURCE_CONFIG}")
message(STATUS "KINECT_CONFIG is ${KINECT_CONFIG}${SOURCE_CONFIG}")
install(DIRECTORY DESTINATION include/v4r/${INTERFACE_INCLUDE_NAME}/config)
install(FILES ${SOURCE_CONFIG} DESTINATION include/v4r/${INTERFACE_INCLUDE_NAME}/config)

# create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

# create executeable app
SET(BINARY_NAME SegmenterLight)
add_executable(${BINARY_NAME} SegmenterLightTest.cpp)
target_link_libraries (${BINARY_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
target_link_libraries (${BINARY_NAME} v4rTomGine v4rSegmenterLight)
v4r_add_binary(${BINARY_NAME})

ENDIF(V4R_SEGMENTER_LIGHT)
