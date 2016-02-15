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
 * @file BoundaryRelations.cpp
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate boundary relations between patches.
 */

#include "BoundaryRelations.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelations::BoundaryRelations()
{
  have_cloud = false;
  have_surfaces = false;
  have_normals = false;
  projectPts = true;

  max3DDistancePerMeter = 0.007;        // 7mm
}

BoundaryRelations::~BoundaryRelations()
{
}

// ================================= Private functions ================================= //


/** Project the datapoints of the plane surfaces to the model surface **/
void BoundaryRelations::projectPts2Model()
{
  pcl_cloud_model.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::copyPointCloud(*pcl_cloud, *pcl_cloud_model);

  #pragma omp parallel for
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    if(view->surfaces[i]->type == pcl::SACMODEL_PLANE) {
      pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices);
      pcl_indices->indices = view->surfaces[i]->indices;
      pcl::ModelCoefficients::Ptr mc (new pcl::ModelCoefficients);
      mc->values = view->surfaces[i]->coeffs;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(pcl_cloud);
      proj.setIndices(pcl_indices);
      proj.setModelCoefficients(mc);
      proj.filter(*new_cloud);
      for (unsigned j = 0; j < new_cloud->points.size(); j++)
        (*pcl_cloud_model).points[pcl_indices->indices[j]] = new_cloud->points[j];
    }
  }
}


// ================================= Public functions ================================= //

void BoundaryRelations::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _pcl_cloud)
{
  pcl_cloud = _pcl_cloud;
  have_cloud = true;
  have_normals = false;
}

void BoundaryRelations::setView(surface::View *_view)
{
  view = _view;
  
  projectPts = true; // project plane points to plane model
  if (projectPts)
    projectPts2Model();

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
  
  pcl_model_normals = view->normals;
  have_surfaces = true;
}


bool BoundaryRelations::compare(int p0, int p1, std::vector<double> &rel_value)
{
  if(!have_cloud) {
    printf("[BoundaryRelations::compute] Error: No input cloud set.\n");
    exit(0);
  }

  if(!have_surfaces) {
    printf("[BoundaryRelations::compute] Error: No surface models set.\n");
    exit(0);
  }

  rel_value.resize(0);
  
  int first = 0;
  int second = 0;
  std::vector<int> first_ngbr;
  std::vector<int> second_ngbr;
  std::vector<int> first_3D_ngbr;
  std::vector<int> second_3D_ngbr;
  std::vector<int> dir_3D_ngbr;
  
  // get neighbouring pixel-pairs in 2D and in 3D
  for(int row=1; row<patches.rows; row++) {
    for(int col=1; col<patches.cols; col++) {
      bool found = false;
      double distance = 10.;
      double z_dist = pcl_cloud->points[row*pcl_cloud->width + col].z;
      int b_first = 0, b_second = 0;
      double b_distance = 10.;
      int dir_3D = 0;
      if((patches.at<int>(row, col) == p0 && patches.at<int>(row-1, col-1) == p1) ||  // left-upper pixel
         (patches.at<int>(row, col) == p1 && patches.at<int>(row-1, col-1) == p0)) {
        first = row*patches.cols + col;
        second = (row-1)*patches.cols + col-1;
        distance = fabs(pcl_cloud->points[first].z - pcl_cloud->points[second].z);
        if(distance < b_distance) {
          b_first = first;
          b_second = second;
          b_distance = distance;
        }
        dir_3D = 1;
        found = true;
      }
      if((patches.at<int>(row, col) == p0 && patches.at<int>(row-1, col+1) == p1) ||  // right-upper pixel
         (patches.at<int>(row, col) == p1 && patches.at<int>(row-1, col+1) == p0)) {
        first = row*patches.cols + col;
        second = (row-1)*patches.cols + col+1;
        distance = fabs(pcl_cloud->points[first].z - pcl_cloud->points[second].z);
        if(distance < b_distance) {
          b_first = first;
          b_second = second;
          b_distance = distance;
        }
        dir_3D = 3;
        found = true;
      }
      if((patches.at<int>(row, col) == p0 && patches.at<int>(row, col-1) == p1) ||    // left pixel
         (patches.at<int>(row, col) == p1 && patches.at<int>(row, col-1) == p0)) {
        first = row*patches.cols + col;
        second = row*patches.cols + col-1;
        distance = fabs(pcl_cloud->points[first].z - pcl_cloud->points[second].z);
        if(distance < b_distance) {
          b_first = first;
          b_second = second;
          b_distance = distance;
        }
        dir_3D = 0;
        found = true;
      }
      if((patches.at<int>(row, col) == p0 && patches.at<int>(row-1, col) == p1) ||    // upper pixel
         (patches.at<int>(row, col) == p1 && patches.at<int>(row-1, col) == p0)) {
        first = row*patches.cols + col;
        second = (row-1)*patches.cols + col;
        distance = fabs(pcl_cloud->points[first].z - pcl_cloud->points[second].z);
        if(distance < b_distance) {
          b_first = first;
          b_second = second;
          b_distance = distance;
        }
        dir_3D = 2;
        found = true;
      }
      if(found) {
        first_ngbr.push_back(b_first);
        second_ngbr.push_back(b_second);
        double adaptive_distance = max3DDistancePerMeter*z_dist;
        if(b_distance < adaptive_distance) {
          first_3D_ngbr.push_back(b_first);
          second_3D_ngbr.push_back(b_second);
          dir_3D_ngbr.push_back(dir_3D);
        }
      }
    }
  }
  
  // Write number of neighbours into Surface model
  for(unsigned i=0; i<(unsigned)view->surfaces[p0]->neighbors2D.size(); i++)
    if((int)view->surfaces[p0]->neighbors2D[i] == p1)
      view->surfaces[p0]->neighbors2DNrPixel[i] = first_ngbr.size();
  for(unsigned i=0; i<(unsigned)view->surfaces[p1]->neighbors2D.size(); i++)
    if((int)view->surfaces[p1]->neighbors2D[i] == p0)
      view->surfaces[p1]->neighbors2DNrPixel[i] = first_ngbr.size();
    
  int nr_valid_points_color = 0;
  int nr_valid_points_depth = 0;
  double sum_uv_color_distance = 0.0f;
  double sum_2D_curvature = 0.0f;
  double sum_depth = 0.0f;
  double sum_depth_var = 0.0f;
  std::vector<double> depth_vals;
  
  // calculate mean depth
  for(unsigned i=0; i<first_ngbr.size(); i++) {
    double p0_z, p1_z;
    if (projectPts) {
      p0_z = pcl_cloud_model->points[first_ngbr[i]].z;
      p1_z = pcl_cloud_model->points[second_ngbr[i]].z;
    } else {
      p0_z = pcl_cloud->points[first_ngbr[i]].z;
      p1_z = pcl_cloud->points[second_ngbr[i]].z;
    }
    double depth = fabs(p0_z - p1_z);
    depth_vals.push_back(depth);
    if(depth == depth) { // no nan
      nr_valid_points_depth++;
      sum_depth += depth;
    }
#ifdef DEBUG      
    else 
      printf("[BoundaryRelations::compare] Warning: Invalid depht points (nan): Should not happen! Why?\n");
#endif
  } 
  
  // normalize depth sum and calculate depth variance
  if(nr_valid_points_depth != 0) {
    sum_depth /= nr_valid_points_depth;
    for(unsigned i=0; i<depth_vals.size(); i++)
      sum_depth_var += fabs(depth_vals[i] - sum_depth);
    sum_depth_var /= nr_valid_points_depth;
  }
#ifdef DEBUG      
  else 
    std::printf("[BoundaryRelations::compare] Warning: Number of valid depth points is zero: sum_depth: %4.3f\n", sum_depth);
#endif

    
  /// calcuate curvature / depth
  int nr_valid_points_curvature3D = 0;
  double sum_3D_curvature = 0.0f;
  double sum_3D_curvature_var = 0.0f;
  std::vector<double> curvature_vals;  // single curvature values
  for(unsigned i=0; i<first_3D_ngbr.size(); i++)
  {
    /// calculate color similarity on 3D border
    nr_valid_points_color++;

    //double p0_Y =  (0.257 * p0_color.b) + (0.504 * p0_color.g) + (0.098 * p0_color.r) + 16;
    double p0_U = -(0.148 * pcl_cloud->points[first_3D_ngbr[i]].b) - 
                   (0.291 * pcl_cloud->points[first_3D_ngbr[i]].g) + 
                   (0.439 * pcl_cloud->points[first_3D_ngbr[i]].r) + 128;    // use bgr
    double p0_V =  (0.439 * pcl_cloud->points[first_3D_ngbr[i]].b) - 
                   (0.368 * pcl_cloud->points[first_3D_ngbr[i]].g) - 
                   (0.071 * pcl_cloud->points[first_3D_ngbr[i]].r) + 128;
    //double p1_Y =  (0.257 * p1_color.b) + (0.504 * p1_color.g) + (0.098 * p1_color.r) + 16;
    double p1_U = -(0.148 * pcl_cloud->points[second_3D_ngbr[i]].b) - 
                   (0.291 * pcl_cloud->points[second_3D_ngbr[i]].g) + 
                   (0.439 * pcl_cloud->points[second_3D_ngbr[i]].r) + 128;
    double p1_V =  (0.439 * pcl_cloud->points[second_3D_ngbr[i]].b) - 
                   (0.368 * pcl_cloud->points[second_3D_ngbr[i]].g) - 
                   (0.071 * pcl_cloud->points[second_3D_ngbr[i]].r) + 128;
    
    double u_1 = p0_U/255 - p1_U/255;
    double u_2 = u_1 * u_1;
    double v_1 = p0_V/255 - p1_V/255;
    double v_2 = v_1 * v_1;
    double cDist = sqrt(u_2 + v_2);
    sum_uv_color_distance += cDist;
    
    /// calculate mean 3D curvature
    cv::Vec3f pt0, pt1;
    if (projectPts) {
      pt0[0]= pcl_cloud_model->points[first_3D_ngbr[i]].x;
      pt0[1]= pcl_cloud_model->points[first_3D_ngbr[i]].y;
      pt0[2]= pcl_cloud_model->points[first_3D_ngbr[i]].z;
      pt1[0]= pcl_cloud_model->points[second_3D_ngbr[i]].x;
      pt1[1]= pcl_cloud_model->points[second_3D_ngbr[i]].y;
      pt1[2]= pcl_cloud_model->points[second_3D_ngbr[i]].z;
    } else {
      pt0[0]= pcl_cloud->points[first_3D_ngbr[i]].x;
      pt0[1]= pcl_cloud->points[first_3D_ngbr[i]].y;
      pt0[2]= pcl_cloud->points[first_3D_ngbr[i]].z;
      pt1[0]= pcl_cloud->points[second_3D_ngbr[i]].x;
      pt1[1]= pcl_cloud->points[second_3D_ngbr[i]].y;
      pt1[2]= pcl_cloud->points[second_3D_ngbr[i]].z;
    }
    
    if(pt0 == pt0 || pt1 == pt1)
    {
      cv::Vec3f p0_normal;
      p0_normal[0] = pcl_model_normals->points[first_3D_ngbr[i]].normal_x;
      p0_normal[1] = pcl_model_normals->points[first_3D_ngbr[i]].normal_y;
      p0_normal[2] = pcl_model_normals->points[first_3D_ngbr[i]].normal_z;
      cv::Vec3f p1_normal;
      p1_normal[0] = pcl_model_normals->points[second_3D_ngbr[i]].normal_x;
      p1_normal[1] = pcl_model_normals->points[second_3D_ngbr[i]].normal_y;
      p1_normal[2] = pcl_model_normals->points[second_3D_ngbr[i]].normal_z;

      cv::Vec3f pp;
      if(dir_3D_ngbr[i] == 0) {
        pp[0] = -1.0; pp[1] = 0.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 1) {
        pp[0] = -1.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 2) {
        pp[0] = 0.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 3) {
        pp[0] = 1.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      cv::Vec3f pp_dir = cv::normalize(pp);
      
      double a_p0_pp = acos(p0_normal.ddot(pp_dir));
      pp_dir = -pp_dir; // invert direction between points
      double a_p1_pp = acos(p1_normal.ddot(pp_dir));

      double curvature = 0.0;
      if(a_p0_pp == a_p0_pp && a_p1_pp == a_p1_pp) {
        nr_valid_points_curvature3D++;
        curvature = a_p0_pp + a_p1_pp - M_PI;
        curvature_vals.push_back(curvature);
        sum_3D_curvature += curvature;
      }
#ifdef DEBUG      
      else
        printf("[BoundaryRelations::compare] Warning: Invalid curvature points (nan): Should not happen! DO SOMETHING!\n");
#endif
    }
  }

  if(nr_valid_points_color != 0)
    sum_uv_color_distance /= nr_valid_points_color;
#ifdef DEBUG      
  else 
    printf("[BoundaryRelations::compare] Warning: Number of valid color points is zero: sum_color: %4.3f\n", sum_uv_color_distance);
#endif
    
  if(nr_valid_points_curvature3D != 0) {
    sum_3D_curvature /= nr_valid_points_curvature3D;
    for(unsigned i=0; i<curvature_vals.size(); i++)
      sum_3D_curvature_var += fabs(curvature_vals[i] - sum_3D_curvature);
    sum_3D_curvature_var /= nr_valid_points_depth;
  }
#ifdef DEBUG      
  else 
    printf("[BoundaryRelations::compare] Warning: Number of valid 3D curvature points is zero: sum_3D_curvature: %4.3f\n", sum_3D_curvature);
#endif
    
  rel_value.push_back(1.-sum_uv_color_distance);
  rel_value.push_back(sum_depth);
  rel_value.push_back(sum_depth_var);
  rel_value.push_back(sum_2D_curvature);            /// TODO We do not use that: Remove that at one point
  rel_value.push_back(sum_3D_curvature);
  rel_value.push_back(sum_3D_curvature_var);
  rel_value.push_back((double)first_3D_ngbr.size() / (double) first_ngbr.size());

  return true;
}



} // end surface












