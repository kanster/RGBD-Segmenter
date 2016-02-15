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
 * @file StructuralRelationsLight.cpp
 * @author Richtsfeld
 * @date December 2012
 * @version 0.1
 * @brief Calculate patch relations for structural level: Efficient version without fourier and gabor filter.
 */

#include "StructuralRelationsLight.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

StructuralRelationsLight::StructuralRelationsLight()
{
  have_patches = false;
  have_input_cloud = false;
}

StructuralRelationsLight::~StructuralRelationsLight()
{
}

// ================================= Private functions ================================= //

/**
 * computeNeighbors 
 */
void StructuralRelationsLight::computeNeighbors()
{
  double z_max = 0.01;
  
  cv::Mat_<int> patches;
  if(view->havePatchImage) {
    patches = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
    view->patchImage.copyTo(patches);
  }
  else {
    patches = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
    patches.setTo(-1);
    for(unsigned i=0; i<view->surfaces.size(); i++)
      for(unsigned j=0; j<view->surfaces[i]->indices.size(); j++)
        patches.at<int>(view->surfaces[i]->indices[j] / pcl_cloud->width, view->surfaces[i]->indices[j] % pcl_cloud->width) = i;
    patches.copyTo(view->patchImage);
  }
  unsigned nr_patches = view->surfaces.size();

  bool nbgh_matrix3D[nr_patches][nr_patches];
  bool nbgh_matrix2D[nr_patches][nr_patches];
  for(unsigned i=0; i<nr_patches; i++)
    for(unsigned j=0; j<nr_patches; j++) {
      nbgh_matrix3D[i][j] = false;
      nbgh_matrix2D[i][j] = false;
    }
  
  for(int row=1; row<patches.rows; row++) { //1ms
    for(int col=1; col<patches.cols; col++) {
      if(patches.at<int>(row, col) != -1) {
        if(patches.at<int>(row-1, col) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col)) {
            int pos_0 = row*pcl_cloud->width+col;
            int pos_1 = (row-1)*pcl_cloud->width+col;
            double dis = fabs(pcl_cloud->points[pos_0].z - pcl_cloud->points[pos_1].z);
            if( dis < z_max) {
              nbgh_matrix3D[patches.at<int>(row-1, col)][patches.at<int>(row, col)] = true;
              nbgh_matrix3D[patches.at<int>(row, col)][patches.at<int>(row-1, col)] = true;
            }
            nbgh_matrix2D[patches.at<int>(row-1, col)][patches.at<int>(row, col)] = true;
            nbgh_matrix2D[patches.at<int>(row, col)][patches.at<int>(row-1, col)] = true;
          }
        }
        if(patches.at<int>(row, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row, col-1)) {
            int pos_0 = row*pcl_cloud->width+col;
            int pos_1 = row*pcl_cloud->width+col-1;
            double dis = fabs(pcl_cloud->points[pos_0].z - pcl_cloud->points[pos_1].z);
            if( dis < z_max) {
              nbgh_matrix3D[patches.at<int>(row, col-1)][patches.at<int>(row, col)] = true;
              nbgh_matrix3D[patches.at<int>(row, col)][patches.at<int>(row, col-1)] = true;
            }
            nbgh_matrix2D[patches.at<int>(row, col-1)][patches.at<int>(row, col)] = true;
            nbgh_matrix2D[patches.at<int>(row, col)][patches.at<int>(row, col-1)] = true;
          }
        }
        if(patches.at<int>(row-1, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col-1)) {
            int pos_0 = row*pcl_cloud->width+col;
            int pos_1 = (row-1)*pcl_cloud->width+col-1;
            double dis = fabs(pcl_cloud->points[pos_0].z - pcl_cloud->points[pos_1].z);
            if( dis < z_max) {
              nbgh_matrix3D[patches.at<int>(row-1, col-1)][patches.at<int>(row, col)] = true;
              nbgh_matrix3D[patches.at<int>(row, col)][patches.at<int>(row-1, col-1)] = true;
            }
            nbgh_matrix2D[patches.at<int>(row-1, col-1)][patches.at<int>(row, col)] = true;
            nbgh_matrix2D[patches.at<int>(row, col)][patches.at<int>(row-1, col-1)] = true;
          }
        }
      }
    }
  }

  for(unsigned i=0; i<nr_patches; i++){
    view->surfaces[i]->neighbors2D.clear();
    view->surfaces[i]->neighbors2DNrPixel.clear();
    for(unsigned j=i+1; j<nr_patches; j++)
      if(nbgh_matrix2D[i][j]) {
        view->surfaces[i]->neighbors2D.push_back(j);
        view->surfaces[i]->neighbors2DNrPixel.push_back(0);
      }
  }
  
  for(unsigned i=0; i<nr_patches; i++){
    view->surfaces[i]->neighbors3D.clear();
    for(unsigned j=i+1; j<nr_patches; j++)
      if(nbgh_matrix3D[i][j])
        view->surfaces[i]->neighbors3D.push_back(j);
  }
}

void StructuralRelationsLight::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
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

void StructuralRelationsLight::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _pcl_cloud)
{
  pcl_cloud = _pcl_cloud;
  have_input_cloud = true;
}


void StructuralRelationsLight::setView(surface::View *_view)
{
  view = _view;
  have_patches = true;
}

void StructuralRelationsLight::computeRelations()
{
printf("StructuralRelationsLight::computeRelations start!\n");

  if(!have_input_cloud || !have_patches) {
    printf("[StructuralRelationsLight::computeRelations] Error: No input cloud and patches available.\n");
    return;
  }
  view->relations.clear();
  
  cv::Mat_<cv::Vec3b> matImage;
  ConvertPCLCloud2Image(pcl_cloud, matImage);

  std::vector<ColorHistogram3D> hist3D;
  surface::Texture texture;

  relations.resize(view->surfaces.size());

#pragma omp parallel sections
  {

    #pragma omp section
    {
      computeNeighbors();
    }

    #pragma omp section 
    {
      for(unsigned i=0; i<view->surfaces.size(); i++) { 
        int nr_hist_bins = 4;
        double uvThreshold = 0.0f;
        hist3D.push_back(ColorHistogram3D(nr_hist_bins, uvThreshold));                               
        hist3D[i].setInputCloud(pcl_cloud);
        hist3D[i].setIndices(view->surfaces[i]->indices);
        hist3D[i].compute();
      }
    }

    #pragma omp section 
    {
      texture.setInputImage(matImage);
      texture.setSurfaceModels(*view);
      texture.compute();
    }      
      
    #pragma omp section 
    {
      boundary.setInputCloud(pcl_cloud);
      boundary.setView(view);
    }

//     #pragma omp section 
//     {
//       surface::Relation rel;
//       rel.valid = false;
//       relations.clear();
//       for(unsigned i=0; i<view->surfaces.size(); i++)
//         for(unsigned j=0; j<view->surfaces.size(); j++)
//           relations[i].push_back(rel);
//     }
  } // end parallel sections


  std::vector< std::vector<surface::Relation> > relations(view->surfaces.size());
  for ( int i = 0; i < relations.size(); ++ i )
    relations[i].resize( view->surfaces.size() );
//  surface::Relation relations[view->surfaces.size()][view->surfaces.size()];
  surface::Relation rel;
  rel.valid = false;
  for(unsigned i=0; i<view->surfaces.size(); i++)
    for(unsigned j=i+1; j<view->surfaces.size(); j++)
      relations[i][j] = rel;
  
#pragma omp parallel for
  for(int i=0; i<(int)view->surfaces.size(); i++) {
    for(int j=0; j<(int)view->surfaces[i]->neighbors3D.size(); j++) {
      bool valid_relation = true;
      int p0 = i;
      int p1 = view->surfaces[i]->neighbors3D[j];
            
      if(p0 > p1)
        continue;
      
      double colorSimilarity = hist3D[p0].compare(hist3D[p1]);
      double textureRate = texture.compare(p0, p1);
      double relSize = std::min((double)view->surfaces[p0]->indices.size()/(double)view->surfaces[p1]->indices.size(), 
                                (double)view->surfaces[p1]->indices.size()/(double)view->surfaces[p0]->indices.size());
      
      std::vector<double> boundary_relations;
      if(!boundary.compare(p0, p1, boundary_relations)) {
        valid_relation = false;
        printf("[StructuralRelationsLight::computeRelations] Warning: Boundary relation invalid.\n");
      }
    
      if(valid_relation) {
          Relation r;
          r.groundTruth = -1;
          r.prediction = -1;
          r.type = 1;                                     // structural level = 1
          r.id_0 = p0;
          r.id_1 = p1;

          r.rel_value.push_back(colorSimilarity);         // r_co ... color similarity (histogram) of the patch
          r.rel_value.push_back(textureRate);             // r_tr ... difference of texture rate
          r.rel_value.push_back(relSize);                 // r_rs ... relative patch size difference

          r.rel_value.push_back(boundary_relations[0]);     // r_co3 ... color similarity on 3D border
          r.rel_value.push_back(boundary_relations[4]);     // r_cu3 ... mean curvature of 3D neighboring points
          r.rel_value.push_back(boundary_relations[1]);     // r_di2 ... depth mean value between border points (2D)
          r.rel_value.push_back(boundary_relations[2]);     // r_vd2 ... depth variance value
          r.rel_value.push_back(boundary_relations[5]);     // r_cu3 ... curvature variance of 3D neighboring points
          r.rel_value.push_back(boundary_relations[6]);     // r_3d2 ... relation 3D neighbors / 2D neighbors

          r.valid = true;
          relations[p0][p1] = r;
      }
    }
  }
  
  // copy relations to view
  for(unsigned i=0; i<view->surfaces.size(); i++)
    for(unsigned j=i+1; j<view->surfaces.size(); j++) {
      if(relations[i][j].valid) {
        view->relations.push_back(relations[i][j]);
#ifdef DEBUG
        printf("r_st_l: [%u][%u]: ", relations[i][j].id_0, relations[i][j].id_1);
        for(unsigned ridx=0; ridx<relations[i][j].rel_value.size(); ridx++)
          printf("%4.3f ", relations[i][j].rel_value[ridx]);
        printf("\n");
#endif
      }
    }
}

} // end surface models





