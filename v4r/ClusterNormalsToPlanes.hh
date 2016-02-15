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

#ifndef SURFACE_CLUSTER_NORMALS_TO_PLANES_HH
#define SURFACE_CLUSTER_NORMALS_TO_PLANES_HH

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#ifdef DEBUG
  #include "v4r/TomGine/tgTomGineThread.h"
#endif

#include "SurfaceModel.hpp"

#include "PPlane.h"

namespace surface
{

template<typename T1,typename T2>
extern T1 Dot3(const T1 v1[3], const T2 v2[3]);
  
template<typename T1,typename T2, typename T3>
extern void Mul3(const T1 v[3], T2 s, T3 r[3]);
  

template<typename T1,typename T2, typename T3>
extern void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);


/**
 * ClusterNormalsToPlanes
 */
class ClusterNormalsToPlanes
{
public:
  class Parameter
  {
  public:
    float thrAngle;             // Threshold of angle for normal clustering
    float inlDist;              // Maximum inlier distance
    int minPoints;              // Minimum number of points for a plane
    
    bool adaptive;              // use adaptive functions for threshold of angle
    float d_c;                  // salient point (from constant to gradient)
    float epsilon_c;            // constant value for angle threshold (radiant)
    float epsilon_g;            // gradient value for angle threshold (radiant gradient)
    
    float omega_c;              // constant value for inlier (normal) distance threshold
    float omega_g;              // gradient value for inlier (normal) distance threshold
    
    float ra_dist;              // maximum distance for reasigning points to other patches
    
    Parameter(float thrAngleNC=0.6, float _inlDist=0.02, int _minPoints=9, bool _adaptive=false, float _d_c = 0.0, 
              float _epsilon_c=0.54, float _epsilon_g=0.1, float _omega_c=-0.004, float _omega_g=0.015, float _ra_dist = 0.01) : 
              thrAngle(thrAngleNC), inlDist(_inlDist), minPoints(_minPoints), adaptive(_adaptive), d_c(_d_c),
              epsilon_c(_epsilon_c), epsilon_g(_epsilon_g), omega_c(_omega_c), omega_g(_omega_g), ra_dist(_ra_dist) {} 
  };

private:
  Parameter param;
  int width, height;
  float cosThrAngleNC;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;                         ///< Point cloud
  surface::View *view;                                                  ///< Surface models

  std::vector< std::vector<float> > srt_curvature;
  
  std::vector<unsigned char> mask;
  std::vector<float> p_adaptive_cosThrAngleNC;                          ///< adaptive angle
  std::vector<float> p_adaptive_inlDist;                                ///< adaptive inlier distance
  
  bool pixel_check;
  int max_neighbours;              //< Maximum pixel neighbors for pixel check
  int max_nneighbours;             //< Maximum neighbouring neighbors for pixel check
  cv::Mat_<int> patches;           //< Patch indices(+1) on 2D image grid

  void CalcAdaptive();
  void CreatePatchImage();
  void CountNeighbours(std::vector< std::vector<int> > &reassign_idxs, 
                       int nb, int nnb, int inc);
  bool ReasignPoints(std::vector< std::vector<int> > &reassign_idxs);
  void DeleteEmptyPlanes();
  void SinglePixelCheck();
  void PixelCheck();
  void ClusterRest(unsigned idx, 
                   pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                   pcl::PointCloud<pcl::Normal> &normals, 
                   std::vector<int> &pts,
                   pcl::Normal &normal);
  void ClusterNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
        pcl::PointCloud<pcl::Normal> &normals,
        const std::vector<int> &indices, 
        std::vector<SurfaceModel::Ptr> &planes); 
  void ClusterNormals(unsigned idx, pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
        pcl::PointCloud<pcl::Normal> &normals, 
        std::vector<int> &pts, pcl::Normal &normal);
  void ComputeLSPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
        std::vector<SurfaceModel::Ptr> &planes);
  void AddNormals();
  
  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);
  inline bool IsNaN(const pcl::PointXYZRGB &pt);
  inline bool Contains(std::vector<int> &indices, int idx);
  inline float SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef DEBUG
  cv::Mat dbg;
  cv::Ptr<TomGine::tgTomGineThread> dbgWin;
#endif
  
  ClusterNormalsToPlanes(Parameter p=Parameter());
  ~ClusterNormalsToPlanes();

  /** Set parameters for plane estimation **/
  void setParameter(Parameter p);

  /** Set input cloud **/
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  /** Set input surface patches **/
  void setView(surface::View *_view);
  
  /** Set surface check: try to reasign single pixels to planes **/
  void setPixelCheck(bool check, int neighbors);

  /** Compute planes by surface normal grouping **/
  void compute();
  
   /** Compute planes by surface normal grouping **/
  void compute(const std::vector<int> &indices);

};



/*********************** INLINE METHODES **************************/


inline int ClusterNormalsToPlanes::GetIdx(short x, short y)
{
  return y*width+x; 
}

inline short ClusterNormalsToPlanes::X(int idx)
{
  return idx%width;
}

inline short ClusterNormalsToPlanes::Y(int idx)
{
  return idx/width;
}

}

#endif

