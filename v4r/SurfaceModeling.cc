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

#include "SurfaceModeling.hh"

#define USE_UNKNOWN_PATCHES     // Use also unexplained patches for merging (non-planar patches)
// #define TRY_MERGING_PLANES      // Merge planes before merging planes to b-splines
#define TRY_MERGING             // Try merging of patches
#define MS_PARALLEL             // parallel plane/NURBS calculation and merging
// #define NO_MS_CHECK             // do not check model merging (not suggested!)

namespace surface {

#define COSTS_NURBS_PARAMS      3.0     // Parameter costs: for each control point of a b_spline (3*9 = 27)
#define COSTS_PLANE_PARAMS     12.0     // Parameter costs for a plane (3*4pts = 12)

  
using namespace std;

template<typename T1,typename T2>
inline T1 Dot3(const T1 v1[3], const T2 v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

template<typename T1,typename T2, typename T3>
inline void Mul3(const T1 v[3], T2 s, T3 r[3])
{
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

/********************** SurfaceModeling ************************
 * Constructor/Destructor
 */
SurfaceModeling::SurfaceModeling(Parameter p, bool _dbgPts3D, bool _dbgTxt3D) :
  dbgPts3D(_dbgPts3D), dbgTxt3D(_dbgTxt3D), haveIntr(false), haveExtr(false), param(p)
{}

SurfaceModeling::~SurfaceModeling()
{
}

/************************** PRIVATE ************************/

/**
 * ComputeSavings
 */
double SurfaceModeling::ComputeSavings(int numParams, std::vector<double> &probs)
{
  double savings = probs.size() - param.kappa1 * (double) numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  savings -= param.kappa2 * err;

  return (savings > 0 ? savings : 0);
}


/**
 * ComputePlaneSavingsNormalized
 */
double SurfaceModeling::ComputePlaneSavingsNormalized(double numParams, std::vector<double> &probs, double norm, double kappa1, double kappa2)
{
  norm = 1. / norm;
  double savings = /*norm * (double) probs.size() 1.0 */ - kappa1 * numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  double probs_size = 1./(double) probs.size();
  savings -= probs_size * kappa2 * err;

//   return (savings > 0 ? savings : 0);
  return savings;
}

/**
 * ComputeSavingsNormalized
 */
double SurfaceModeling::ComputeSavingsNormalized(double numParams, std::vector<double> &probs, double norm)
{
  norm = 1. / norm;
  double savings = norm * (double) probs.size() - param.kappa1 * numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  savings -= norm * param.kappa2 * err;

  return savings;
//   return (savings > 0 ? savings : 0);
}


/**
 * AddToQueue
 */
void SurfaceModeling::AddToQueue(const std::vector<unsigned> &nbs, std::vector<unsigned> &queue)
{
  for (unsigned i = 0; i < nbs.size(); i++) {
    queue.push_back(nbs[i]);
  }
}

void SurfaceModeling::ComputeLSPlane(SurfaceModel &plane)
{
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> lsPlane(cloud);
  Eigen::VectorXf coeffs(4);
  Eigen::Vector3d n0(0., 0., 1.);

  if(plane.type == pcl::SACMODEL_PLANE && plane.indices.size() > 4) {

    lsPlane.optimizeModelCoefficients(plane.indices, coeffs, coeffs);
    if (Dot3(&coeffs[0], &n0[0]) > 0)
      coeffs*=-1.;
    if (Dot3(&coeffs[0], &plane.coeffs[0]) > 0.6) {
      plane.coeffs.resize(4);
      plane.coeffs[0] = coeffs[0];
      plane.coeffs[1] = coeffs[1];
      plane.coeffs[2] = coeffs[2];
      plane.coeffs[3] = coeffs[3];
    }
    else
      printf("[SurfaceModeling::ComputeLSPlanes] Warning: Problematic plane found.\n");
    
    // compute error
    float a, b, c, d;
    plane.error.resize(plane.indices.size());
    a=plane.coeffs[0], b=plane.coeffs[1], c=plane.coeffs[2], d=plane.coeffs[3];
    for (unsigned i=0; i<plane.indices.size(); i++)
      plane.error[i] = Plane::ImpPointDist(a,b,c,d, &cloud->points[plane.indices[i]].x);
  }
}
  
/**
 * FitPlane
 */
void SurfaceModeling::FitPlane(SurfaceModel &plane)
{
  ComputeLSPlane(plane);
  ComputePointProbs(plane.error, plane.probs);
}  
  
/**
 * FitNurbs
 */
void SurfaceModeling::FitNurbs(SurfaceModel &surface)
{
  pcl::PointIndices::Ptr points(new pcl::PointIndices());
  points->indices = surface.indices;

  cv::Ptr<pcl::on_nurbs::SequentialFitter> nurbsFitter;
  nurbsFitter = new pcl::on_nurbs::SequentialFitter(param.nurbsParams);
  if (haveIntr && haveExtr) {
    nurbsFitter->setProjectionMatrix(camIntr, camExtr);
  } else {
    printf("[SurfaceModeling::FitNurbs] Warning, projection matrix not set!\n");
  }

  nurbsFitter->setInputCloud(cloud);
  nurbsFitter->setInterior(points);
  nurbsFitter->compute();
  nurbsFitter->getInteriorError(surface.error);
  surface.nurbs = nurbsFitter->getNurbs();
  nurbsFitter->getInteriorNormals(surface.normals);
  nurbsFitter->getInteriorParams(surface.nurbs_params);

  // check orientation of normals and calculate probabilities
  for (unsigned i = 0; i < surface.normals.size(); i++) {
    Eigen::Vector3d &n = surface.normals[i];
    if (Dot3(&n[0], &cloud->points[surface.indices[surface.indices.size()/2]].x) > 0)
      Mul3(&n[0], -1, &n[0]);
  }

  ComputePointProbs(surface.error, surface.probs);
}

/**
 * ModelSelection with omp parallel
 */
void SurfaceModeling::ModelSelectionParallel()
{
  /// HACK: Call FitNurbs once, otherwise parallelization wont work. Why?
  int i = 0;
  if (view->surfaces[i]->selected && !view->surfaces[i]->used  && view->surfaces[i]->type == pcl::SACMODEL_PLANE) {
    if((int)view->surfaces[i]->indices.size() < param.planePointsFixation) {
      SurfaceModel::Ptr model;
      model.reset(new SurfaceModel());
      model->indices = view->surfaces[i]->indices;
      model->type = view->surfaces[i]->type;
      model->idx = view->surfaces.size();
      model->selected = true;
      model->used = false;
      model->neighbors3D = view->surfaces[i]->neighbors3D;

      if(model->indices.size() > 3)
        FitNurbs(*model);

      view->surfaces[i]->savings = ComputeSavingsNormalized(COSTS_PLANE_PARAMS, view->surfaces[i]->probs, view->surfaces[i]->indices.size());
      model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, model->probs, view->surfaces[i]->indices.size());

#ifdef DEBUG
      cout << "[SurfaceModeling::ModelSelectionParallel] Savings plane/NURBS id=" << i << ": " << view->surfaces[i]->savings << "/" << model->savings;
#endif
      
      if (model->savings > view->surfaces[i]->savings) { // => create NURBS
        model->type = MODEL_NURBS;
        model->savings = view->surfaces[i]->savings;
        view->surfaces[i] = model;
      }
#ifdef DEBUG
      cout << " -> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
#endif
    }
  }

  /// HACK END: Than we can do that parall
  #pragma omp parallel for 
  for (unsigned i = 1; i < view->surfaces.size(); i++) {
    if (view->surfaces[i]->selected && !view->surfaces[i]->used  && view->surfaces[i]->type == pcl::SACMODEL_PLANE) {
      if((int)view->surfaces[i]->indices.size() < param.planePointsFixation) {
        SurfaceModel::Ptr model;
        model.reset(new SurfaceModel());
        model->indices = view->surfaces[i]->indices;
        model->type = view->surfaces[i]->type;
        model->idx = view->surfaces.size();
        model->selected = true;
        model->used = false;
        model->neighbors3D = view->surfaces[i]->neighbors3D;

        if(model->indices.size() > 3)
          FitNurbs(*model);

        view->surfaces[i]->savings = ComputeSavingsNormalized(COSTS_PLANE_PARAMS, view->surfaces[i]->probs, view->surfaces[i]->indices.size());
        model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, model->probs, view->surfaces[i]->indices.size());

#ifdef DEBUG
        cout << "[SurfaceModeling::ModelSelectionParallel] Savings plane/NURBS id=" << i << ": " << view->surfaces[i]->savings << "/" << model->savings;
#endif
        
        if (model->savings > view->surfaces[i]->savings) { // => create NURBS
          model->type = MODEL_NURBS;
          model->savings = view->surfaces[i]->savings;
          view->surfaces[i] = model;
        }
#ifdef DEBUG
        cout << " -> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
#endif
      }
    }
  }

#ifdef TRY_MERGING_PLANES
#ifdef DEBUG
        cout << "[SurfaceModeling::ModelSelectionParallel] -> Try merging of surface models:" << endl;
#endif

  // Merge first planes
  for (unsigned i = 0; i < view->surfaces.size(); i++) {
    if (view->surfaces[i]->selected && !view->surfaces[i]->used) {
      for(unsigned j =0; j<view->surfaces[i]->neighbors3D.size(); j++) {
        unsigned idx = view->surfaces[i]->neighbors3D[j];
        if(idx > i && view->surfaces[i]->type == pcl::SACMODEL_PLANE && view->surfaces[idx]->type == pcl::SACMODEL_PLANE) {
          SurfaceModel::Ptr mergedModel;
          mergedModel.reset(new SurfaceModel());
          (*mergedModel) = *view->surfaces[i];         
          view->surfaces[idx]->AddTo(*mergedModel);
          
          if(mergedModel->indices.size() > 3)
            FitPlane(*mergedModel);
          mergedModel->type = pcl::SACMODEL_PLANE;

          mergedModel->savings = ComputePlaneSavingsNormalized(0., mergedModel->probs, mergedModel->indices.size(), 0.003, 0.9);
          view->surfaces[i]->savings = ComputePlaneSavingsNormalized(0., view->surfaces[i]->probs, mergedModel->indices.size(), 0.003, 0.9)/2.;
          view->surfaces[idx]->savings = ComputePlaneSavingsNormalized(0., view->surfaces[idx]->probs, mergedModel->indices.size(), 0.003, 0.9)/2.;
          
          if (mergedModel->savings > (view->surfaces[i]->savings + view->surfaces[idx]->savings)) {
#ifdef DEBUG
  cout << "[ModelSelectionParallel] -> Merge planes: " << i << "-" << idx << " s: " << mergedModel->savings << " > " << (view->surfaces[i]->savings+view->surfaces[idx]->savings) << " ( " << view->surfaces[i]->savings << "+" << view->surfaces[idx]->savings << ")" << endl;
#endif
          }
          else
             cout << "[ModelSelectionParallel] -> No merge planes: " << i << "-" << idx << " s: " << mergedModel->savings << " < " << (view->surfaces[i]->savings+view->surfaces[idx]->savings) << " ( " << view->surfaces[i]->savings << "+" << view->surfaces[idx]->savings << ")" << endl;

        }
      }
    }
  }
#endif

#ifdef TRY_MERGING
  // Merge nurbs
  std::vector<merge> merge_pairs;
  
#pragma omp parallel for shared(merge_pairs)
  for (unsigned i = 0; i < view->surfaces.size(); i++) {
    if (view->surfaces[i]->selected && !view->surfaces[i]->used) {
      if(view->surfaces[i]->indices.size() < (size_t) param.planePointsFixation) {
        for(unsigned j =0; j<view->surfaces[i]->neighbors3D.size(); j++) {
          unsigned idx = view->surfaces[i]->neighbors3D[j];
          if(idx > i) {
            SurfaceModel::Ptr mergedModel;
            mergedModel.reset(new SurfaceModel());
            (*mergedModel) = *view->surfaces[i];
            view->surfaces[idx]->AddTo(*mergedModel);

            if(mergedModel->indices.size() > 3)
              FitNurbs(*mergedModel);
            
            mergedModel->savings = ComputeSavingsNormalized(mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, 
                                                            mergedModel->probs, mergedModel->indices.size());
            view->surfaces[i]->savings = ComputeSavingsNormalized(
                    (view->surfaces[i]->type == MODEL_NURBS ? view->surfaces[i]->nurbs.m_cv_count[0] * view->surfaces[i]->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS), 
                    view->surfaces[i]->probs, mergedModel->indices.size());
            view->surfaces[idx]->savings = ComputeSavingsNormalized(
                    (view->surfaces[idx]->type == MODEL_NURBS ? view->surfaces[idx]->nurbs.m_cv_count[0] * view->surfaces[idx]->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
                     view->surfaces[idx]->probs, mergedModel->indices.size());

            if (mergedModel->savings > (view->surfaces[i]->savings + view->surfaces[idx]->savings)) {
              merge m;
              m.id_1 = i;
              m.id_2 = idx;
              m.savings = mergedModel->savings;

#pragma omp critical
              {
                merge_pairs.push_back(m);
#ifdef DEBUG
  cout << "[SurfaceModeling::ModelSelectionParallel] -> Merge candidates: " << m.id_1 << "-" << m.id_2 << endl;
#endif
              }
            }
          }
        }
      }
    }
  }

  // merge the surfaces from the best to the weakest connection
  std::sort(merge_pairs.begin(), merge_pairs.end(), CmpSavings);

  for(unsigned i=0; i<merge_pairs.size(); i++) {
    if(merge_pairs[i].id_1 != merge_pairs[i].id_2) {

#ifdef NO_MS_CHECK
      view->surfaces[merge_pairs[i].id_2]->AddTo(*view->surfaces[merge_pairs[i].id_1]);
      FitNurbs(*view->surfaces[merge_pairs[i].id_1]);
      view->surfaces[merge_pairs[i].id_2]->selected = false;

  #ifdef DEBUG        
      printf("[SurfaceModeling::ModelSelectionParallel]  => MERGED: %u-%u\n", merge_pairs[i].id_1, merge_pairs[i].id_2);
  #endif

      for(unsigned j=i+1; j < merge_pairs.size(); j++) {
        if(merge_pairs[j].id_1 == merge_pairs[i].id_2)
          merge_pairs[j].id_1 = merge_pairs[i].id_1;
        if(merge_pairs[j].id_2 == merge_pairs[i].id_2)
          merge_pairs[j].id_2 = merge_pairs[i].id_1;
      }
#else

      SurfaceModel::Ptr mergedModel;
      mergedModel.reset(new SurfaceModel());
      (*mergedModel) = *view->surfaces[merge_pairs[i].id_1];
      view->surfaces[merge_pairs[i].id_2]->AddTo(*mergedModel);
      if(mergedModel->indices.size() > 3)
        FitNurbs(*mergedModel);
      mergedModel->type = MODEL_NURBS;
      mergedModel->savings = ComputeSavingsNormalized(
                mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, mergedModel->probs, mergedModel->indices.size());
      view->surfaces[merge_pairs[i].id_1]->savings = ComputeSavingsNormalized(
                (view->surfaces[merge_pairs[i].id_1]->type == MODEL_NURBS ? view->surfaces[merge_pairs[i].id_1]->nurbs.m_cv_count[0] * 
                 view->surfaces[merge_pairs[i].id_1]->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
                view->surfaces[merge_pairs[i].id_1]->probs, mergedModel->indices.size());
      view->surfaces[merge_pairs[i].id_2]->savings = ComputeSavingsNormalized(
                (view->surfaces[merge_pairs[i].id_2]->type == MODEL_NURBS ? view->surfaces[merge_pairs[i].id_2]->nurbs.m_cv_count[0] * 
                view->surfaces[merge_pairs[i].id_2]->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
                view->surfaces[merge_pairs[i].id_2]->probs, mergedModel->indices.size());

#ifdef DEBUG        
      printf("[SurfaceModeling::ModelSelectionParallel] Try merging %u and %u (%1.5f > %1.5f)\n", merge_pairs[i].id_1, merge_pairs[i].id_2, mergedModel->savings, 
             view->surfaces[merge_pairs[i].id_1]->savings + view->surfaces[merge_pairs[i].id_2]->savings);
#endif

      if (mergedModel->savings > (view->surfaces[merge_pairs[i].id_1]->savings + view->surfaces[merge_pairs[i].id_2]->savings)) {

#ifdef DEBUG        
        printf("[SurfaceModeling::ModelSelectionParallel]  => MERGED: %u-%u\n", merge_pairs[i].id_1, merge_pairs[i].id_2);
#endif
        view->surfaces[merge_pairs[i].id_1] = mergedModel;
        view->surfaces[merge_pairs[i].id_2]->selected = false;

        for(unsigned j=i+1; j < merge_pairs.size(); j++) {
          if(merge_pairs[j].id_1 == merge_pairs[i].id_2)
            merge_pairs[j].id_1 = merge_pairs[i].id_1;
          if(merge_pairs[j].id_2 == merge_pairs[i].id_2)
            merge_pairs[j].id_2 = merge_pairs[i].id_1;
        }
      }
#endif // NO_MS_CHECK
    }
  }
#endif // TRY_MERGING
}


/**
 * Old ModelSelection (non-parallel version)
 */
void SurfaceModeling::ModelSelection()
{
  SurfaceModel::Ptr model, mergedModel;
  vector<unsigned> queue;

  for (unsigned i = 0; i < view->surfaces.size(); i++) {
    // compare single model
    if (view->surfaces[i]->selected && !view->surfaces[i]->used) {
      model.reset(new SurfaceModel());
      model->indices = view->surfaces[i]->indices;

      FitNurbs(*model);

      view->surfaces[i]->savings = ComputeSavingsNormalized(COSTS_PLANE_PARAMS, view->surfaces[i]->probs, view->surfaces[i]->indices.size());
      model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, model->probs,
          view->surfaces[i]->indices.size());
#ifdef DEBUG
      cout << "[SurfaceModeling::ModelSelection] Savings plane/NURBS id=" << i << ": " << view->surfaces[i]->savings << "/" << model->savings;
#endif

      if (model->savings > view->surfaces[i]->savings) {
        view->surfaces[i]->selected = false;

        model->type = MODEL_NURBS;
        model->idx = view->surfaces.size();
        model->selected = true;
        model->neighbors3D = view->surfaces[i]->neighbors3D;

        view->surfaces.push_back(model);
      }
      else
        model = view->surfaces[i];

#ifdef DEBUG
      cout << " -> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
#endif

      // try merge;
#ifdef TRY_MERGING
      unsigned idx;
      if((model->type != MODEL_NURBS &&  (int)model->indices.size() < param.planePointsFixation) || model->type == MODEL_NURBS)
      {
        queue = model->neighbors3D;
        while (queue.size() > 0) {
          idx = queue.back();
          if (view->surfaces[idx]->selected && !view->surfaces[idx]->used) {
            mergedModel.reset(new SurfaceModel());
            (*mergedModel) = *model;
            view->surfaces[idx]->AddTo(*mergedModel);

            FitNurbs(*mergedModel);

            mergedModel->savings = ComputeSavingsNormalized(mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, 
                                                            mergedModel->probs, mergedModel->indices.size());
            model->savings = ComputeSavingsNormalized((model->type == MODEL_NURBS ? model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS), 
                                                      model->probs, mergedModel->indices.size());
            view->surfaces[idx]->savings = ComputeSavingsNormalized(
                    (view->surfaces[idx]->type == MODEL_NURBS ? view->surfaces[idx]->nurbs.m_cv_count[0] * view->surfaces[idx]->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
                    view->surfaces[idx]->probs, mergedModel->indices.size());
#ifdef DEBUG
            cout << "[SurfaceModeling::ModelSelection] Try merging ids " << i << "-" << idx << ": " << model->savings << "+" << view->surfaces[idx]->savings << "="
                << model->savings + view->surfaces[idx]->savings << " / " << mergedModel->savings;
#endif

            if (mergedModel->savings > (model->savings + view->surfaces[idx]->savings)) {
#ifdef DEBUG
              cout << " MERGED" << endl;
              if(model->type != MODEL_NURBS)
                printf("[SurfaceModeling::ModelSelection]               ===> MERGE PLANE TO NURBS (size: %lu)!!!\n", model->indices.size());
#endif
              view->surfaces[idx]->selected = false;
              model->selected = false;

              mergedModel->type = MODEL_NURBS;
              mergedModel->idx = view->surfaces.size();
              mergedModel->selected = true;

              view->surfaces.push_back(mergedModel);

              queue.pop_back();
              AddToQueue(view->surfaces[idx]->neighbors3D, queue);
              model = mergedModel;
              model->used = true;
              view->surfaces[idx]->used = true;
            }
            else {
#ifdef DEBUG
              cout << endl;
#endif
              queue.pop_back();
            }
          }
          else
            queue.pop_back();
        }
        view->surfaces[i]->used = true;
      }
#endif
    }
  }
}


/**
 * ComputePointProbs
 */
void SurfaceModeling::ComputePointProbs(std::vector<double> &errs, std::vector<double> &probs)
{
  probs.resize(errs.size());
  for (unsigned i = 0; i < errs.size(); i++)
    probs[i] = exp(-((errs[i]*errs[i]) * invSqrSigmaError));
}

void SurfaceModeling::ComputePointProbs(std::vector<double> &errs, std::vector<double> &probs, double sigmaError)
{
  double invSqrSigmaError = 1.0 / (sigmaError*sigmaError);
  probs.resize(errs.size());
  for (unsigned i = 0; i < errs.size(); i++)
    probs[i] = exp(-((errs[i]*errs[i]) * invSqrSigmaError));
}

/**
 * ComputePointError
 */
void SurfaceModeling::ComputePointError(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                        const std::vector<SurfaceModel::Ptr> &planes/*,
                                        std::vector< std::vector<double> > &error*/)
{
  for (unsigned j=0; j<planes.size(); j++) {
    planes[j]->error.clear();
    SurfaceModel &plane = *planes[j];
    planes[j]->error.resize(plane.indices.size());

    if(plane.type == pcl::SACMODEL_PLANE) {
      float a, b, c, d;
      a=plane.coeffs[0], b=plane.coeffs[1], c=plane.coeffs[2], d=plane.coeffs[3];

      for (unsigned i=0; i<plane.indices.size(); i++)
        planes[j]->error[i] = Plane::ImpPointDist(a,b,c,d, &cloud.points[plane.indices[i]].x);
    }
    else {
      for(unsigned i=0; i<plane.indices.size(); i++)
        planes[j]->error[i] = 0.0f;  // no error, if we do not have the model
    }
  }
}

/**
 * InitDataStructure
 */
void SurfaceModeling::InitDataStructure()
{
  ComputePointError(*cloud, view->surfaces/*, error*/);

  for (unsigned i = 0; i < view->surfaces.size(); i++)
    ComputePointProbs(view->surfaces[i]->error, view->surfaces[i]->probs);
}

/**
 * ComputeMean
 */
cv::Point SurfaceModeling::ComputeMean(std::vector<int> &_indices)
{
  cv::Point mean(0., 0.);

  if (_indices.size() == 0)
    return mean;

  for (unsigned i = 0; i < _indices.size(); i++) {
    mean += cv::Point(X(_indices[i]), Y(_indices[i]));
  }

  mean.x /= _indices.size();
  mean.y /= _indices.size();

  return mean;
}

/**
 * computeNeighbors
 */
void SurfaceModeling::computeNeighbors()
{
  cv::Mat_<int> patches;
  patches = cv::Mat_<int>(cloud->height, cloud->width);
  patches.setTo(-1);
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    for(unsigned j=0; j<view->surfaces[i]->indices.size(); j++) {
      int row = view->surfaces[i]->indices[j] / cloud->width;
      int col = view->surfaces[i]->indices[j] % cloud->width;
      patches.at<int>(row, col) = i;
    }
  }
  unsigned nr_patches = view->surfaces.size();

  bool nbgh_matrix3D[nr_patches][nr_patches];
  bool nbgh_matrix2D[nr_patches][nr_patches];
  for(unsigned i=0; i<nr_patches; i++)
    for(unsigned j=0; j<nr_patches; j++) {
      nbgh_matrix3D[i][j] = false;
      nbgh_matrix2D[i][j] = false;
    }

  for(int row=1; row<patches.rows; row++) {
    for(int col=1; col<patches.cols; col++) {
      if(patches.at<int>(row, col) != -1) {
        if(patches.at<int>(row-1, col) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col)) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = (row-1)*cloud->width+col;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
              nbgh_matrix3D[patches.at<int>(row-1, col)][patches.at<int>(row, col)] = true;
              nbgh_matrix3D[patches.at<int>(row, col)][patches.at<int>(row-1, col)] = true;
            }
            nbgh_matrix2D[patches.at<int>(row-1, col)][patches.at<int>(row, col)] = true;
            nbgh_matrix2D[patches.at<int>(row, col)][patches.at<int>(row-1, col)] = true;
          }
        }
        if(patches.at<int>(row, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row, col-1)) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = row*cloud->width+col-1;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
              nbgh_matrix3D[patches.at<int>(row, col-1)][patches.at<int>(row, col)] = true;
              nbgh_matrix3D[patches.at<int>(row, col)][patches.at<int>(row, col-1)] = true;
            }
            nbgh_matrix2D[patches.at<int>(row, col-1)][patches.at<int>(row, col)] = true;
            nbgh_matrix2D[patches.at<int>(row, col)][patches.at<int>(row, col-1)] = true;
          }
        }
        if(patches.at<int>(row-1, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col-1)) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = (row-1)*cloud->width+col-1;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
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

  for(unsigned i=0; i<nr_patches; i++) {
    view->surfaces[i]->neighbors2D.clear();
    for(unsigned j=0; j<nr_patches; j++)
      if(nbgh_matrix2D[i][j])
        view->surfaces[i]->neighbors2D.push_back(j);
  }

  for(unsigned i=0; i<nr_patches; i++) {
    view->surfaces[i]->neighbors3D.clear();
    for(unsigned j=0; j<nr_patches; j++)
      if(nbgh_matrix3D[i][j])
        view->surfaces[i]->neighbors3D.push_back(j);
  }
}

/**
 * getSurfaceModels
 */
void SurfaceModeling::copySurfaces(bool addUnknownData)
{
  for (unsigned i = 0; i < view->surfaces.size(); i++) {
    view->surfaces[i]->neighbors2D.clear();
    view->surfaces[i]->neighbors3D.clear();
  }
  std::vector<SurfaceModel::Ptr> surfaces;
  for (unsigned i = 0; i < view->surfaces.size(); i++) {
    if (view->surfaces[i]->selected) {
      if(addUnknownData) {
        view->surfaces[i]->idx = surfaces.size();
        surfaces.push_back(view->surfaces[i]);
      }
      else {
        if(view->surfaces[i]->type == pcl::SACMODEL_PLANE || view->surfaces[i]->type == MODEL_NURBS) {
          view->surfaces[i]->idx = surfaces.size();
          surfaces.push_back(view->surfaces[i]);
        }
      }
    }
  }
  view->surfaces = surfaces;

  // copy surface normals to view
  for(unsigned s=0; s<view->surfaces.size(); s++)
    for(unsigned i=0; i<view->surfaces[s]->indices.size(); i++) {
      pcl::Normal &pt = view->normals->points[view->surfaces[s]->indices[i]];
      pt.normal_x = view->surfaces[s]->normals[i][0];
      pt.normal_y = view->surfaces[s]->normals[i][1];
      pt.normal_z = view->surfaces[s]->normals[i][2];
    }  
}

/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void SurfaceModeling::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
}


/**
 * setView
 */
void SurfaceModeling::setView(surface::View *_view)
{
  view = _view;
//   error.clear();
}

/**
 * setIntrinsic
 */
void SurfaceModeling::setIntrinsic(double fx, double fy, double cx, double cy)
{
  camIntr = Eigen::Matrix4d::Zero();
  camIntr(0, 0) = fx;
  camIntr(1, 1) = fy;
  camIntr(0, 2) = cx;
  camIntr(1, 2) = cy;
  camIntr(2, 2) = 1.0;
  haveIntr = true;
}

/**
 * setExtrinsic
 */
void SurfaceModeling::setExtrinsic(Eigen::Matrix4d &pose)
{
  camExtr = pose;
  haveExtr = true;
}

/**
 * Operate
 */
void SurfaceModeling::compute()
{
#ifdef DEBUG
  cout << "[SurfaceModeling::compute] MODEL A NEW FRAME *********************************" << endl;
#endif
  
  invSqrSigmaError = 1. / (param.sigmaError * param.sigmaError);
  InitDataStructure();
  computeNeighbors();

#ifdef MS_PARALLEL
  ModelSelectionParallel();
#else
  ModelSelection();
#endif

  bool copyUnknownData = false;
  copySurfaces(copyUnknownData);
#ifdef DEBUG
//   if (!dbgWin.empty() && (dbgPts3D || dbgTxt3D)) {
//     dbgWin->Clear();
//     cv::Mat_<cv::Vec4f> cvCloud;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPatch(new pcl::PointCloud<pcl::PointXYZRGB>());
//     unsigned cntNurbs = 0, cntPlanes = 0;
//     for (unsigned i = 0; i < models.size(); i++) {
//       if (models[i]->selected) {
//         if (dbgPts3D) {
//           pcl::copyPointCloud(*cloud, models[i]->indices, *cloudPatch);
//           float col = surface::GetRandomColor();
//           for (unsigned j = 0; j < cloudPatch->points.size(); j++)
//             cloudPatch->points[j].rgb = col;
//           pclA::ConvertPCLCloud2CvMat(cloudPatch, cvCloud);
//           dbgWin->AddPointCloud(cvCloud);
//         }
//         //draw labels
//         if (dbgTxt3D) {
//           cv::Point pt = ComputeMean(models[i]->indices);
//           pcl::PointXYZRGB &pt3 = (*cloud)(pt.x, pt.y);
//           string label = P::toString(models[i]->idx, 0);
//           if (models[i]->type == MODEL_NURBS) {
//             label = label + "N";
//             cntNurbs++;
//           } else {
//             cntPlanes++;
//             label = label + "P";
//           }
//           dbgWin->AddLabel3D(label, 12, pt3.x, pt3.y, pt3.z);
//         }
//       }
//     }
//     cout << "cntPlanes=" << cntPlanes << ", cntNurbs=" << cntNurbs << " -> selected=" << cntPlanes + cntNurbs << endl;
//   }
#endif
}

// added by tm
double SurfaceModeling::computeSavingsNormalized(int numParams, std::vector<double> &errs, double norm,
                                                 double kappa1, double kappa2, double sigmaError)
{
  std::vector<double> probs;
  ComputePointProbs(errs, probs, sigmaError);

  norm = 1. / norm;
  double savings = norm * (double) probs.size() - kappa1 * (double) numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  savings -= norm * kappa2 * err;

  return (savings > 0 ? savings : 0);
}


#ifdef DEBUG
/**
 * SetDebugWin
 */
void SurfaceModeling::SetDebugWin(cv::Ptr<TomGine::tgTomGineThread> &win)
{
  dbgWin = win;
  dbgWin->SetRotationCenter(0.0, 0.0, 1.0);
}
#endif

} //-- THE END --

