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
 * @file StructuralRelationsLight.h
 * @author Richtsfeld
 * @date December 2012
 * @version 0.1
 * @brief Calculate patch relations for structural level: Efficient version without fourier and gabor filter.
 */

#ifndef SURFACE_STRUCTURAL_RELATIONS_LIGHT_H
#define SURFACE_STRUCTURAL_RELATIONS_LIGHT_H

#include <omp.h>
#include <vector>
#include <cstdio>
#include <opencv2/highgui/highgui.hpp>

#include "ColorHistogram3D.h"
#include "Texture.h"
#include "BoundaryRelations.h"

#include "SurfaceModel.hpp"


namespace surface
{
  
class StructuralRelationsLight
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  
  bool have_patches;
  bool have_input_cloud;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;             ///< Input point cloud
  surface::View *view;                                          ///< View with surface models

  surface::BoundaryRelations boundary;
  std::vector< std::vector< surface::Relation > > relations;

  void computeNeighbors();
  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
  
public:
  StructuralRelationsLight();
  ~StructuralRelationsLight();

  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _pcl_cloud);
  
  /** Set input surface models **/
  void setView(surface::View *_view);

  /** Compute relations for the segmenter **/
  void computeRelations();
  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

