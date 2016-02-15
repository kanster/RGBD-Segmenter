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
 * @file ColorHistogram3D.cpp
 * @author Andreas Richtsfeld
 * @date March 2012
 * @version 0.1
 * @brief Color histogram class for 3D comparison of YUV-Histograms.
 */

#include "ColorHistogram3D.h"

namespace surface
{

ColorHistogram3D::ColorHistogram3D(int _nr_bins, double _UVTheshold)
{
  nr_bins = _nr_bins;
  computed = false;
  have_input_cloud = false;
  have_indices = false;
  UVthreshold = _UVTheshold;
}

void ColorHistogram3D::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud)
{
  input_cloud = _input_cloud;
  have_input_cloud = true;
  computed = false;
}

void ColorHistogram3D::setIndices(pcl::PointIndices::Ptr &_indices)
{
  printf("[ColorHistogram3D::setIndices] Warning: Antiquated funtion.\n");
  if(!have_input_cloud) {
    printf("[ColorHistogram3D::setIndices] Error: No input cloud available.\n");
    return;
  }
  indices = _indices->indices;
  have_indices = true;
}

void ColorHistogram3D::setIndices(std::vector<int> _indices)
{
  if(!have_input_cloud) {
    printf("[ColorHistogram3D::setIndices] Error: No input cloud available.\n");
    return;
  }
  indices = _indices;
  have_indices = true;
}

void ColorHistogram3D::compute()
{
  yuvHist.clear();
  for(int i=0; i< nr_bins; i++) {
    std::vector< std::vector<double> > hist;
    for(int j=0; j< nr_bins; j++) {
      std::vector<double> yHist;
      for(int k=0; k< nr_bins; k++)
        yHist.push_back(0.0f);
      hist.push_back(yHist);
    }
    yuvHist.push_back(hist);
  }

  int noCol = 0;
  for(unsigned i=0; i<indices.size(); i++) {
    //pcl::RGBValue color = ....   // out of date! instead use PointXYZRGB::r directly
    pcl::PointXYZRGB &pt = input_cloud->points[indices[i]];
    double yBin = 0.;
    double uBin = 0.;
    double vBin = 0.;
    int Y =  (0.257 * pt.r) + (0.504 * pt.g) + (0.098 * pt.b) + 16;
    int U = -(0.148 * pt.r) - (0.291 * pt.g) + (0.439 * pt.b) + 128;
    int V =  (0.439 * pt.r) - (0.368 * pt.g) - (0.071 * pt.b) + 128;

    int U2 = U-128;        // shifted to the middle
    int V2 = V-128;        // shifted to the middle
    if((U2*U2 + V2*V2) < UVthreshold) {
      noCol++;
    }
    else {
        yBin = Y*(double)nr_bins/255.;
        uBin = U*(double)nr_bins/255.;
        vBin = V*(double)nr_bins/255.;
        yuvHist[(int)yBin][(int)uBin][(int)vBin] += 1.;
    }
  } 
  
  double normalization = indices.size() - noCol;
  if(normalization != 0)
    for(int i=0; i<nr_bins; i++)
      for(int j=0; j<nr_bins; j++)
        for(int k=0; k<nr_bins; k++)
          yuvHist[i][j][k] /= normalization;

  computed = true;
}


double ColorHistogram3D::compare(ColorHistogram3D &ch)
{
  if(nr_bins != ch.nr_bins) {
    printf("[ColorHistogram3D::Compare] Error: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  if(!computed || !ch.computed) {
    printf("[ColorHistogram3D::Compare] Error: Color histogram not computed.\n");
    return 0.;
  }
  
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for(int i=0; i<nr_bins; i++)
    for(int j=0; j<nr_bins; j++)
      for(int k=0; k<nr_bins; k++)
        overall_sum += sqrt(yuvHist[i][j][k]*ch.yuvHist[i][j][k]);
      
  double fidelity = overall_sum;
//   double bhattacharyya = -log(overall_sum);
  
  if(fidelity != fidelity) {
    printf("[ColorHistogram3D::Compare] Error: Color distance is nan!\n");
    printHistogram();
    ch.printHistogram();
  }
  
  return fidelity;
}

void ColorHistogram3D::printHistogram()
{
  printf("Print histogram:\n");
  for(int i=0; i<nr_bins; i++) {
    printf("y = %u\n", i);
    for(int j=0; j<nr_bins; j++) {
      for(int k=0; k<nr_bins; k++) {
        printf("  %4.3f", yuvHist[i][j][k]);
      }
      printf("\n");
    }
    printf("\n");
  }
}
  
}

