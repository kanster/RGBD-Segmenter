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
 * @file Texture.h
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate texture feature to compare surface texture.
 */

#ifndef SURFACE_TEXTURE_H
#define SURFACE_TEXTURE_H

#include <vector>
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "SurfaceModel.hpp"


namespace surface
{

class Texture
{
public:
  
protected:

private:

  bool have_cloud;
  bool have_surfaces;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;                     ///< Input cloud
  surface::View view;                                                   ///< Surface models
  std::vector<double> textureRate;                                      ///< Texture rate for each surface
  cv::Mat_<cv::Vec3b> matImage;                                         ///< Image as cv-matrix

  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);

public:
  Texture();
  ~Texture();
  
  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud);

  /** Set input image **/
  void setInputImage(cv::Mat_<cv::Vec3b> &_matImge);

  /** Set input surface patches **/
  void setSurfaceModels(surface::View & _view);

  /** Compute the texture **/
  void compute();
  
  /** Compare surface texture **/
  double compare(int id_0, int id_1);
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

