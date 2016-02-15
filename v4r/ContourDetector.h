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
 * @file ContourDetector.h
 * @author Richtsfeld
 * @date January 2013
 * @version 0.1
 * @brief Base class for contour detection.
 */

#ifndef SURFACE_CONTOUR_DETECTOR_H
#define SURFACE_CONTOUR_DETECTOR_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "SurfaceModel.hpp"


namespace surface
{
  
/** Edgel **/
struct aEdgel {
  int index;                            ///< index in point cloud
  int corner_idx;                       ///< corner index in image space
  bool horizontal;                      ///< horizontal edge
  bool h_valid;                         ///< horizontal edge validity
  int h_ids[2];                         ///< surface ids (left/right)
  bool vertical;                        ///< vertical edge
  bool v_valid;                         ///< vertical edge validity
  int v_ids[2];                         ///< surface ids (top/bottom)

  void print() {
    printf("Edge: %u\n", index);
    if(corner_idx != -1)
      printf("  is corner: true\n");
    else
      printf("  is corner: false\n");

    if(horizontal) {
      printf("  horizontal: true\n");
      printf("  h_ids[0]: %u\n", h_ids[0]);
      printf("  h_ids[1]: %u\n", h_ids[1]);
    }
    else
      printf("  horizontal: false\n");

    if(vertical) {
      printf("  vertical: true\n");
      printf("  v_ids[0]: %u\n", v_ids[0]);
      printf("  v_ids[1]: %u\n", v_ids[1]);
    }
    else
      printf("  vertical: false\n");
  }
};

/** @brief Corner: Corner, consisting of left/top point index and 4 surface id's **/
struct aCorner {
  int index;                            ///< Index of corner point in image
  int ids[4];                           ///< surface ids (top/left to bottom/right)

  void print() {
    printf("Corner: %u\n", index);
    printf("  surface ids: %u %u %u %u\n", ids[0], ids[1], ids[2], ids[3]);
  }
};

/** @brief Edge: Edge, consisting of start and end corner and edgels in between **/
struct aEdge {
  bool has_corners;                     ///< Flag for edge chain with cornerns
  int corners[2];                       ///< Indices of the two corners
  std::vector<unsigned> edgels;         ///< Indices list to view.edges
  unsigned surfaces[2];                 ///< SurfaceModel indices for left right surface
};
  

class ContourDetector
{
public:
  
protected:

  bool have_cloud;
  bool have_view;
  bool initialized;
  bool have_contours;
  
  cv::Mat_<int> patches;                                                ///< Patch image
  cv::Mat_<int> contours;                                               ///< Contour image
  cv::Mat_<int> contours2;                                              ///< (same) Contour image, later used for splitting
  
  std::vector<surface::aEdgel> pre_edgels;                              ///< detected edgels
  std::vector<surface::aCorner> pre_corners;                            ///< detected corners
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;                     ///< Input cloud
  surface::View *view;                                                  ///< Surface models
  
  /** compute edgels and corners **/
  void initialize();
  
  /** Recursive search of neighbouring contour pixels **/
  void RecursiveContourClustering(int id, int start_x, int start_y,
                                  int x, int y, int dir,
                                  std::vector<int> &contour,
                                  bool &end);

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);

private:

public:
  ContourDetector();
  ~ContourDetector();
  
  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud);
  
  /** Set input view with surfaces **/
  void setView(surface::View *_view);

  /** Trace contour image and create surface contours **/
  void computeContours();

};


/*************************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int ContourDetector::GetIdx(short x, short y)
{
  return y*pcl_cloud->width + x;
}

/** Return x coordinate for index **/
inline short ContourDetector::X(int idx)
{
  return idx%pcl_cloud->width;
}

/** Return y coordinate for index **/
inline short ContourDetector::Y(int idx)
{
  return idx/pcl_cloud->width;
}


} //--END--

#endif

