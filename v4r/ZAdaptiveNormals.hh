/**
 * $Id$
 *
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

#ifndef SURFACE_ZADAPTIVE_NORMALS_HH
#define SURFACE_ZADAPTIVE_NORMALS_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <boost/shared_ptr.hpp>


namespace surface 
{
/**
 * Surface normals estimation
 */
class ZAdaptiveNormals
{
public:
  class Parameter
  {
    public:
      double radius;            // euclidean inlier radius
      int kernel;               // kernel radius [px]
      bool adaptive;            // Activate z-adaptive normals calcualation
      float kappa;              // gradient
      float d;                  // constant
      float kernel_radius[8];   // Kernel radius for each 0.5 meter intervall (0-4m)
      Parameter(double _radius=0.02, int _kernel=5, bool _adaptive=false, float _kappa=0.005125, float _d = 0.0)
       : radius(_radius), kernel(_kernel), adaptive(_adaptive), kappa(_kappa), d(_d) {}
  };

private:
  Parameter param;

  static float NaN;
  int width, height;

  float sqr_radius;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  void ComputeCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
        const std::vector<int> &indices, const Eigen::Vector4f &mean, Eigen::Matrix3f &cov);
  void EstimateNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &normals);
  void GetIndices(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int u, int v, int kernel, std::vector<int> &indices);
  float ComputeNormal(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors);


  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ZAdaptiveNormals(Parameter p=Parameter());
  ~ZAdaptiveNormals();

  void setParameter(Parameter p);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  void compute();
  void compute(const std::vector<int> &mask);

  void getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
};




/*********************** INLINE METHODES **************************/

inline int ZAdaptiveNormals::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short ZAdaptiveNormals::X(int idx)
{
  return idx%width;
}

inline short ZAdaptiveNormals::Y(int idx)
{
  return idx/width;
}


}

#endif

