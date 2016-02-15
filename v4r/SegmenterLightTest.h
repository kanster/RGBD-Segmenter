/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
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
 * @file SegmenterLightTest.h
 * @author Andreas Richtsfeld
 * @date January 2013
 * @version 0.1
 * @brief Test program for the SegmenterLight.
 */


#ifndef SEGMENTERLIGHTTEST_H
#define SEGMENTERLIGHTTEST_H

#include <cstdio>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "KinectData.h"
#include "SegmenterLight.h"

#ifdef V4R_TOMGINE
#include "v4r/TomGine/tgTomGineThread.h"
#endif

namespace segment
{
  
typedef union
{
  struct
  {
    unsigned char b; // Blue channel
    unsigned char g; // Green channel
    unsigned char r; // Red channel
    unsigned char a; // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;

inline float GetRandomColor()
{
  RGBValue x;
  x.b = std::rand()%255;
  x.g = std::rand()%255;
  x.r = std::rand()%255;
  x.a = 0.; 
  return x.float_value;
}
  
/**
 * @class SegmenterLightTest
 */
class SegmenterLightTest
{
private:
  double z_min, z_max;                                          ///< Minimum and maximum z-values
  double overall_runtime;

  std::string database_path;                                    ///< database path
  std::string rgbd_filename;                                    ///< depth image filename
  unsigned startIdx, endIdx;                                    ///< Start and end index of images
  bool data_live;                                               ///< load data live from Kinect
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;             ///< original pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled;    ///< original pcl point cloud

  KinectData *kinect;                                           ///< Load kinect data from file or live from Kinect

  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
  void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pcl_cloud, 
                             std::vector<cv::Vec4f> &cvCloud/*, bool random_colors = false*/);

public:

private:
  void init();
  void process();
  
public:
  SegmenterLightTest();
  ~SegmenterLightTest();
  
  /** Set minimum and maximum depth values **/
  void setMinMaxDepth(double _z_min, double _z_max) {z_min = _z_min; z_max = _z_max;}

  /** Run the pre-segmenter **/
  void run(std::string _rgbd_filename, int _startIdx, int _endIdx, bool _live); 

};

}

#endif
