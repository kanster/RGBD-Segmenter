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
 * @file ContourDetector.cpp
 * @author Richtsfeld
 * @date January 2013
 * @version 0.1
 * @brief Base class for contour detection.
 */

#include "ContourDetector.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

ContourDetector::ContourDetector()
{
  have_cloud = false;
  have_view = false;
  initialized = false;
  have_contours = false;
}

ContourDetector::~ContourDetector()
{
}

// ================================= Public functions ================================= //

void ContourDetector::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _pcl_cloud)
{
  pcl_cloud = _pcl_cloud;
  have_cloud = true;
  have_view = false;
  initialized = false;
  have_contours = false;
  
  pre_corners.resize(pcl_cloud->width * pcl_cloud->height);
  pre_edgels.resize(pcl_cloud->width * pcl_cloud->height);
}


void ContourDetector::setView(surface::View *_view)
{
  view = _view;
  view->corners.clear();
  view->edges.clear();
  view->edgels.clear();
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    view->surfaces[i]->contours.clear();
    view->surfaces[i]->edges.clear();
  }
  have_view = true;
}

void ContourDetector::RecursiveContourClustering(int id, int start_x, int start_y,
                                                  int x, int y, int dir, 
                                                  std::vector<int> &contour,
                                                  bool &end)
{
  if(end) {
    printf("[ContourDetector::RecursiveContourClustering] Warning: Unexpected end found.\n");
    return;
  }
  
  bool found = false;
  dir += 5;     // change direction
  for(unsigned i=0; i<8; i++) {
    int di = i + dir;
    di = di%8;

    if(di == 0 && !found) {
      if(y-1 >= 0 && contours.at<int>(y-1, x) == id) {  // top pixel
        found = true;
        if(x == start_x && y-1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x, y-1));
          RecursiveContourClustering(id, start_x, start_y, x, y-1, 0, contour, end);
        }
      }  
    }
    else if (di == 1 && !found) {
      if(y-1 >= 0 && x+1 < (int) pcl_cloud->width && contours.at<int>(y-1, x+1) == id) {  // top right pixel
        found = true;
        if(x+1 == start_x && y-1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x+1, y-1));
          RecursiveContourClustering(id, start_x, start_y, x+1, y-1, 1, contour, end);
        }
      }
    }
    else if (di == 2 && !found) {
      if(x+1 < (int) pcl_cloud->width && contours.at<int>(y, x+1) == id) {  // right pixel
        found = true;
        if(x+1 == start_x && y == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x+1, y));
          RecursiveContourClustering(id, start_x, start_y, x+1, y, 2, contour, end);
        }
      }
    }
    else if (di == 3 && !found) {
      if(x+1 < (int) pcl_cloud->width && y+1 < (int) pcl_cloud->height && contours.at<int>(y+1, x+1) == id) { // right bottom pixel
        found = true;
        if(x+1 == start_x && y+1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x+1, y+1));
          RecursiveContourClustering(id, start_x, start_y, x+1, y+1, 3, contour, end);
        }
      }  
    }
    else if (di == 4 && !found) {
      if(y+1 < (int) pcl_cloud->height && contours.at<int>(y+1, x) == id) { // bottom pixel
        found = true;
        if(x == start_x && y+1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x, y+1));
          RecursiveContourClustering(id, start_x, start_y, x, y+1, 4, contour, end);
        }
      }  
    }
    else if (di == 5 && !found) {
      if(x-1 >= 0 && y+1 < (int) pcl_cloud->height && contours.at<int>(y+1, x-1) == id) { // bottom left pixel
        found = true;
        if(x-1 == start_x && y+1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x-1, y+1));
          RecursiveContourClustering(id, start_x, start_y, x-1, y+1, 5, contour, end);
        }
      }    
    }
    else if (di == 6 && !found) {
      if(x-1 >= 0 && contours.at<int>(y, x-1) == id) { // left pixel
        found = true;
        if(x-1 == start_x && y == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x-1, y));
          RecursiveContourClustering(id, start_x, start_y, x-1, y, 6, contour, end);
        }
      }
    }
    else if (di == 7 && !found) {
      if(x-1 >= 0 && y-1 >= 0 && contours.at<int>(y-1, x-1) == id) { // top left pixel
        found = true;
        if(x-1 == start_x && y-1 == start_y && contour.size() > 3) {
          end = true;
          return;
        }
        else {
          contour.push_back(GetIdx(x-1, y-1));
          RecursiveContourClustering(id, start_x, start_y, x-1, y-1, 7, contour, end);
        }
      }
    }
  }
}


/**
 * @brief Construct patch and contour image. Initialize pre_edgels and pre_corners
 */
void ContourDetector::initialize()
{
  if(!have_cloud) {
    printf("[ContourDetector::initialize] Error: No input cloud set.\n");
    exit(0);
  }

  if(!have_view) {
    printf("[ContourDetector::initialize] Error: No view set.\n");
    exit(0);
  }
    
  if(view->havePatchImage) {
    patches = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
    view->patchImage.copyTo(patches);
  }
  else {
    patches = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
    patches.setTo(-1);
    for(unsigned i=0; i<view->surfaces.size(); i++)
      for(unsigned j=0; j<view->surfaces[i]->indices.size(); j++)
        patches.at<int>(Y(view->surfaces[i]->indices[j]), X(view->surfaces[i]->indices[j])) = i;
    patches.copyTo(view->patchImage);
    view->havePatchImage = true;
  }
  
  contours = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
  contours.setTo(-1);
  contours2 = cv::Mat_<int>(pcl_cloud->height, pcl_cloud->width);
  contours2.setTo(-1);
  
  #pragma omp parallel for      // => 4ms
  for(int row=0; row<patches.rows; row++) {
    for(int col=0; col<patches.cols; col++) {
      int idx = GetIdx(col, row);

      // initialize corner
      pre_corners[idx].index = -1;
      
      // initialize edge
      pre_edgels[idx].index = -1;
      pre_edgels[idx].corner_idx = -1;
      pre_edgels[idx].horizontal = false;
      pre_edgels[idx].h_valid = false;
      pre_edgels[idx].vertical = false;
      pre_edgels[idx].v_valid = false;

      // contour at image border
      if((row == 0 || row == (int) pcl_cloud->height) && patches.at<int>(row, col) != -1)   
        contours.at<int>(row, col) = patches.at<int>(row, col);
      if((col == 0  || col == (int) pcl_cloud->width) && patches.at<int>(row, col) != -1)
        contours.at<int>(row,col) = patches.at<int>(row,col);
      
      if(col < patches.cols-1) {
        if(patches.at<int>(row, col) != patches.at<int>(row, col+1)) {    // horizontal edge
          contours.at<int>(row, col) = patches.at<int>(row, col);
          contours.at<int>(row, col+1) = patches.at<int>(row, col+1);
        }
      }
        
      if(row < patches.rows-1) {
        if(patches.at<int>(row, col) != patches.at<int>(row+1, col)) {    // vertical edge
          contours.at<int>(row, col) = patches.at<int>(row,col);
          contours.at<int>(row+1, col) = patches.at<int>(row+1, col);
        }
      }
    }
  }
  contours.copyTo(contours2);
  

  #pragma omp parallel for     // => 7ms
  for(int row=0; row<patches.rows; row++) {
    for(int col=0; col<patches.cols; col++) {
      int idx = GetIdx(col, row);

      unsigned found = 0;
      if(col < patches.cols-1)
        if(patches.at<int>(row, col) != patches.at<int>(row, col+1))
          found++;
      if(col < patches.cols-1 && row < patches.rows-1)
        if(patches.at<int>(row, col+1) != patches.at<int>(row+1, col+1))
          found++;
      if(col < patches.cols-1 && row < patches.rows-1)
        if(patches.at<int>(row+1, col+1) != patches.at<int>(row+1, col))
          found++;
      if(row < patches.rows-1)
        if(patches.at<int>(row+1, col) != patches.at<int>(row, col))
          found++;
      
      if(col < patches.cols-1 && row < patches.rows-1) {
        if(found >= 3) {
          pre_corners[idx].index = idx;
          pre_corners[idx].ids[0] = patches.at<int>(row, col);
          pre_corners[idx].ids[1] = patches.at<int>(row, col+1);
          pre_corners[idx].ids[2] = patches.at<int>(row+1, col);
          pre_corners[idx].ids[3] = patches.at<int>(row+1, col+1);

          if(patches.at<int>(row, col) != patches.at<int>(row, col+1)) {
            pre_edgels[idx].corner_idx = idx;
            pre_edgels[idx].index = idx;
            pre_edgels[idx].horizontal = true;
            pre_edgels[idx].h_valid = true;
            pre_edgels[idx].h_ids[0] = patches.at<int>(row, col);
            pre_edgels[idx].h_ids[1] = patches.at<int>(row, col+1);
          }
          if(patches.at<int>(row+1, col) != patches.at<int>(row+1, col+1)) {
            int idx2 = GetIdx(col, row+1);
            pre_edgels[idx2].corner_idx = idx;
            pre_edgels[idx2].index = idx;
            pre_edgels[idx2].horizontal = true;
            pre_edgels[idx2].h_valid = true;
            pre_edgels[idx2].h_ids[0] = patches.at<int>(row+1, col);
            pre_edgels[idx2].h_ids[1] = patches.at<int>(row+1, col+1);
          }
          
          if(patches.at<int>(row, col) != patches.at<int>(row+1, col)) {
            pre_edgels[idx].corner_idx = idx;
            pre_edgels[idx].index = idx;
            pre_edgels[idx].vertical = true;
            pre_edgels[idx].v_valid = true;
            pre_edgels[idx].v_ids[0] = patches.at<int>(row, col);
            pre_edgels[idx].v_ids[1] = patches.at<int>(row+1, col);
          }
          if(patches.at<int>(row, col+1) != patches.at<int>(row+1, col+1)) {
            int idx2 = GetIdx(col+1, row);
            pre_edgels[idx2].corner_idx = idx;
            pre_edgels[idx2].index = idx;
            pre_edgels[idx2].vertical = true;
            pre_edgels[idx2].v_valid = true;
            pre_edgels[idx2].v_ids[0] = patches.at<int>(row, col+1);
            pre_edgels[idx2].v_ids[1] = patches.at<int>(row+1, col+1);
          }
        }
        else {  // if no corner, create also pre_edgel
          if(patches.at<int>(row, col) != patches.at<int>(row, col+1)) {
            pre_edgels[idx].index = idx;
            pre_edgels[idx].horizontal = true;
            pre_edgels[idx].h_valid = true;
            pre_edgels[idx].h_ids[0] = patches.at<int>(row, col);
            pre_edgels[idx].h_ids[1] = patches.at<int>(row, col+1);
          }
          if(patches.at<int>(row, col) != patches.at<int>(row+1, col)) {
            pre_edgels[idx].index = idx;
            pre_edgels[idx].vertical = true;
            pre_edgels[idx].v_valid = true;
            pre_edgels[idx].v_ids[0] = patches.at<int>(row, col);
            pre_edgels[idx].v_ids[1] = patches.at<int>(row+1, col);
          }        
        }
      }
    }
  }
  initialized = true;
}


void ContourDetector::computeContours()
{
  if(!initialized)
    initialize();
  
  size_t min_contour_size = 5;
//   std::vector<int> empty_contour;

  // start at top left and go through contours
  for(int row=0; row<contours.rows; row++) {
    for(int col=0; col<contours.cols; col++) {
      if(contours.at<int>(row,col) != -1) {
        int id = contours.at<int>(row,col);
        std::vector<int> new_contour;
        
        new_contour.push_back(GetIdx(col, row));
        bool end = false;
        RecursiveContourClustering(id, col, row, col, row, 4, new_contour, end);
        
        if(!end && new_contour.size() > 6)
          printf("[ContourDetector::computeContours] This has found NO end: %u => size: %lu\n", id, new_contour.size());
        
        // save contour
        if(new_contour.size() > min_contour_size) {
          view->surfaces[id]->contours.push_back(new_contour);
        }
        else {
          #ifdef DEBUG      
          printf("[ContourDetector::computeContours] Warning: Contour %u with too less points: %lu\n", id, new_contour.size());
          #endif
//           view->surfaces[id]->contours.push_back(empty_contour);  // push back a empty contour!
        }
        
        // delete contour points from contours map!
        for(unsigned i=0; i<new_contour.size(); i++)
          contours.at<int>(Y(new_contour[i]), X(new_contour[i])) = -1;
      }
    }
  }
  
  have_contours = true;
}


} // end surface












