/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file Kinect.h
 * @author Richtsfeld Andreas
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the OpenNI Driver for the Kinect sensor
 */

#ifndef KINECT_KINECT_H
#define KINECT_KINECT_H

#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Capture.h"

namespace Kinect
{

/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char b;  // Blue channel
    unsigned char g;  // Green channel
    unsigned char r;  // Red channel
    unsigned char a;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


class Kinect
{
private:
  bool ni_pause;                        ///< Set pause
  
  int rgbWidth;                         ///< Width of color image
  int rgbHeight;                        ///< Heigth of color image
  int depWidth;                         ///< Width of depth image
  int depHeight;                        ///< Heigth of depth image
  
  cv::Mat grayImage;                    ///< captured gray image (bayer pattered)
  cv::Mat rgbImage;                     ///< captured rgb (or ir) image
  cv::Mat depImage;                     ///< captured depth image
  
  float centerX, centerY;               ///< center of the image -0.5f
  double pixel_size;                    ///< pixel size of the kinect camera
  XnUInt64 depth_focal_length_SXGA;     ///< 
  float depth_focal_length_SXGA_;       ///< 
  float depthScale;                     ///< 
  float depthFocalLength;               ///< 
  float constant;                       ///< constant factor for world point calculation (from focal length)

  XnUInt64 shadow_value;                ///< Return value for shadow point
  XnUInt64 no_sample_value;             ///< Return value for no sample

  bool Init(const char *kinect_xml_file);
  void MapMetaData2IplImage(const MapMetaData* pImageMD, IplImage **iplImg);
  void DepthMetaData2IplImage(const DepthMetaData* pDepthMD, IplImage **iplImg);
  void rgbUndistort(cv::Mat src);
  void depUndistort(cv::Mat src);
  cv::Vec4f DepthToWorld_Vec4f(const int &x, const int &y, const int &depthValue);
  cv::Point3f DepthToWorld(int x, int y, int depthValue);
  
public:
  Kinect(const char *kinect_xml_file);
  Kinect();
  ~Kinect();

  void StartCapture(int delay);
  void StopCapture();
  bool GetColorImage(IplImage **iplImg);
  bool NextFrame();
  bool GetImages(cv::Mat &rgbImg, cv::Mat &depImg);

  cv::Point3f Get3dWorldPoint(unsigned x, unsigned y);
  cv::Vec4f Get3dWorldPointWithColor(unsigned x, unsigned y);
  cv::Point3f WorldToColor(unsigned x, unsigned y);
  float PointToColorFloat(unsigned x, unsigned y);
  void Get3dWorldPointCloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud, int scale = 1);
  void Get3dWorldPointCloud(cv::Mat_<cv::Vec4f> &cloud, int scale = 1);
  void Get3dWorldPointCloudSmooth(cv::Mat_<cv::Vec4f> &cloud, int scale = 1);
  void TogglePause() {ni_pause = !ni_pause;}
  void GetColorVideoSize(CvSize &size) {size = cvSize(rgbWidth, rgbHeight);}
  void GetDepthVideoSize(CvSize &size) {size = cvSize(depWidth, depHeight);}
};

}

#endif

