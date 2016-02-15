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
 * @file ColorHistogram3D.h
 * @author Andreas Richtsfeld
 * @date March 2012
 * @version 0.1
 * @brief Color histogram class for 3D comparison of YUV-Histograms
 */

#ifndef SURFACE_COLOR_HISTOGRAM3D_HH
#define SURFACE_COLOR_HISTOGRAM3D_HH

#include <vector>
#include <string.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

namespace surface
{

/**
 * @brief Class ColorHistogram3D
 */
class ColorHistogram3D
{
private:
  int nr_bins;
  double UVthreshold;
  
  bool computed;
  bool have_input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
  bool have_indices;
  std::vector<int> indices;
  
  std::vector< std::vector< std::vector<double> > > yuvHist;  /// 3D y,u,v histogram
  
public:
  ColorHistogram3D(int _nr_bins, double _UVTheshold);
  
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud);
  void setIndices(pcl::PointIndices::Ptr &_indices);
  void setIndices(std::vector<int> _indices);
  void compute();
  double compare(ColorHistogram3D &ch);
  
  void printHistogram();
};

}

#endif

