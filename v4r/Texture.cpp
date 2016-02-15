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
 * @file Texture.cpp
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate texture feature to compare surface texture.
 */

#include "Texture.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Texture::Texture()
{
  have_cloud = false;
  have_surfaces = false;
}

Texture::~Texture()
{
}

// ================================= Private functions ================================= //

void Texture::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                                                cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  
  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[2] = pt.r;
      cvp[1] = pt.g;
      cvp[0] = pt.b;
    }
  }
}

// ================================= Public functions ================================= //


void Texture::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _pcl_cloud)
{
  pcl_cloud = _pcl_cloud;
  ConvertPCLCloud2Image(pcl_cloud, matImage);
  have_cloud = true;
}

void Texture::setInputImage(cv::Mat_<cv::Vec3b> & _matImge)
{
  matImage = _matImge;
  have_cloud = true;
}

void Texture::setSurfaceModels(surface::View & _view)
{
  view = _view;
  have_surfaces = true;
}

void Texture::compute()
{
  if(!have_cloud) {
    printf("[Texture::compute] Error: No input cloud set.\n");
    exit(0);
  }

  if(!have_surfaces) {
    printf("[Texture::compute] Error: No surface models set.\n");
    exit(0);
  }

  double lowThreshold = 5.;
  double highThreshold = 140.;
  int kernel_size = 3;
  cv::Mat gray_image;
  cv::Mat edges;
  
  cv::cvtColor(matImage, gray_image, CV_BGR2GRAY );
  cv::blur(gray_image, edges, cv::Size(3,3));
  cv::Canny(edges, edges, lowThreshold, highThreshold, kernel_size);

  textureRate.resize(view.surfaces.size());
  for(unsigned i=0; i<view.surfaces.size(); i++) {
    int tex_area = 0;
    for(unsigned j=0; j<view.surfaces[i]->indices.size(); j++) {
      if(edges.data[view.surfaces[i]->indices[j]] == 255)
        tex_area++;
    }
    if(view.surfaces[i]->indices.size() == 0) 
      textureRate[i] = 0.0f;
    else
      textureRate[i] = (double) ((double)tex_area / view.surfaces[i]->indices.size());
  }
}


double Texture::compare(int id_0, int id_1)
{
  return 1. - fabs(textureRate[id_0] - textureRate[id_1]);
}


} // end surface












