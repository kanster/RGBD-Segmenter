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

#ifndef SURFACE_SURFACEMODELING_HH
#define SURFACE_SURFACEMODELING_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/on_nurbs/sequential_fitter.h>

#ifdef V4R_TOMGINE
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

struct merge
{
  int id_1;
  int id_2;
  double savings;
};

// --------------- Surface Modeling --------------- //

class SurfaceModeling
{
public:
  class Parameter
  {
  public:
    pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;

    double sigmaError;        // error for gausian error model
    double kappa1;            // base cost (0.002, 0.005 1 cyl nurb)
    double kappa2;            // weights the error (0.8)
    double plane_savings;
    double bspline_savings;   
    int planePointsFixation;  // classified planes will not be merged with b-splines anymore (5000 for 640x480)
    double z_max;             // Maximum z-value for 3D neighborhood

    Parameter(pcl::on_nurbs::SequentialFitter::Parameter nurbs=pcl::on_nurbs::SequentialFitter::Parameter(),
       double _sigmaError=0.003, double _kappa1=0.003, double _kappa2=0.9, int _pPF=5000, double _z_max=0.01)
     : nurbsParams(nurbs), sigmaError(_sigmaError),
       kappa1(_kappa1), kappa2(_kappa2), planePointsFixation(_pPF), z_max(_z_max) {}
  };

private:
  bool dbgPts3D, dbgTxt3D;
  int width, height;

  bool haveIntr, haveExtr;
  Eigen::Matrix4d camIntr;
  Eigen::Matrix4d camExtr;

  double invSqrSigmaError;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;		//< pcl point cloud
  surface::View *view;                                  //< View with surface patches

  std::vector< std::vector<unsigned> > neighbors2D;     //< Neighboring surface patches in image space
  std::vector< std::vector<unsigned> > neighbors3D;     //< Neighboring surface patches (with z_max value)

  void ComputeLSPlane(SurfaceModel &plane);
  void FitPlane(SurfaceModel &plane);
  void FitNurbs(SurfaceModel &model);
  void ModelSelectionParallel();
  void ModelSelection();
  void ComputePointProbs(std::vector<double> &errs, std::vector<double> &probs);
  static void ComputePointProbs(std::vector<double> &errs, std::vector<double> &probs, double sigmaError);
  void ComputePointError(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                         const std::vector<SurfaceModel::Ptr> &planes);
  double ComputeSavings(int numParams, std::vector<double> &probs);
  double ComputePlaneSavingsNormalized(double numParams, std::vector<double> &probs, double norm, double kappa1, double kappa2);
  double ComputeSavingsNormalized(double numParams, std::vector<double> &probs, double norm);
  void InitDataStructure();
  void AddToQueue(const std::vector<unsigned> &nbs, std::vector<unsigned> &queue);

  cv::Point ComputeMean(std::vector<int> &indices);
  void computeNeighbors();
  void copySurfaces(bool addUnknownData = true);

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Parameter param;

#ifdef V4R_TOMGINE
  cv::Mat dbg;
  cv::Ptr<TomGine::tgTomGineThread> dbgWin;
#endif

  SurfaceModeling(Parameter p=Parameter(), bool dbgPts3D=false, bool dbgTxt3D=false);
  ~SurfaceModeling();

  /** Set input pcl cloud **/
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  /** set pre-clustered patches **/
  void setView(surface::View *_view);

  /** set intrinsic camera parameter **/
  void setIntrinsic(double fx, double fy, double cx, double cy);

  /** set extrinsic camera parameter **/
  void setExtrinsic(Eigen::Matrix4d &pose);

  /** compute surface modeling **/
  void compute();
  
  static double computeSavingsNormalized(int numParams, std::vector<double> &errs, double norm,
                                         double kappa1, double kappa2, double sigmaError);

#ifdef V4R_TOMGINE
  /** set debug windows **/
  void SetDebugWin(cv::Ptr<TomGine::tgTomGineThread> &win);
#endif
};




/*********************** INLINE METHODES **************************/
inline bool CmpSavings(const merge &i, const merge &j)
{
  return (i.savings > j.savings);
}

inline int SurfaceModeling::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short SurfaceModeling::X(int idx)
{
  return idx%width;
}

inline short SurfaceModeling::Y(int idx)
{
  return idx/width;
}



}

#endif

