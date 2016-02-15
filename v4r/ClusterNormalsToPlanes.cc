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


#include "ClusterNormalsToPlanes.hh"

namespace surface 
{
  
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

template<typename T1,typename T2, typename T3>
inline void Add3(const T1 v1[3], const T2 v2[3], T3 r[3])
{
  r[0] = v1[0]+v2[0];
  r[1] = v1[1]+v2[1];
  r[2] = v1[2]+v2[2];
}

/********************** ClusterNormalsToPlanes ************************
 * Constructor/Destructor
 */
ClusterNormalsToPlanes::ClusterNormalsToPlanes(Parameter p)
{
  setParameter(p);
  pixel_check = false;
  max_neighbours = 4;
  max_nneighbours = 2*max_neighbours;
  
  srt_curvature.resize(20);
}

ClusterNormalsToPlanes::~ClusterNormalsToPlanes()
{
}

/************************** PRIVATE ************************/

void ClusterNormalsToPlanes::CreatePatchImage()
{
  patches = cv::Mat_<int>(cloud->height, cloud->width);
  patches.setTo(0);
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    for(unsigned j=0; j<view->surfaces[i]->indices.size(); j++) {
      int row = view->surfaces[i]->indices[j] / cloud->width;
      int col = view->surfaces[i]->indices[j] % cloud->width;
      patches(row, col) = i+1;   // plane 1,2,...,n
    }
  }
}

/**
 * Count number of neighbouring pixels (8-neighbourhood)
 * @param nb Maximum neighbouring pixels
 * @param nnb Maximum neighbouring pixels with neighbouring neighbors
 * @param nnb_inc Increment value for neighbouring neighbors
 */
void ClusterNormalsToPlanes::CountNeighbours(std::vector< std::vector<int> > &reassign_idxs,
                                             int nb, int nnb, int nnb_inc)
{
  reassign_idxs.clear();
  reassign_idxs.resize(view->surfaces.size());
  
  #pragma omp parallel for
  for(int row=1; row<patches.rows-1; row++) {
    for(int col=1; col<patches.cols-1; col++) {
      
      int nb_counter = 0;
      int nnb_counter = 0;
      bool neighbors[8] = {false, false, false, false, false, false, false, false};
      if(patches(row, col) == 0) continue;
      if(patches(row, col) == patches(row, col-1)) {nb_counter++; neighbors[0] = true;}
      if(patches(row, col) == patches(row-1, col-1)) {nb_counter++; neighbors[1] = true;}
      if(patches(row, col) == patches(row-1, col)) {nb_counter++; neighbors[2] = true;}
      if(patches(row, col) == patches(row-1, col+1)) {nb_counter++; neighbors[3] = true;}
      if(patches(row, col) == patches(row, col+1)) {nb_counter++; neighbors[4] = true;}
      if(patches(row, col) == patches(row+1, col+1)) {nb_counter++; neighbors[5] = true;}
      if(patches(row, col) == patches(row+1, col)) {nb_counter++; neighbors[6] = true;}
      if(patches(row, col) == patches(row+1, col-1)) {nb_counter++; neighbors[7] = true;}
        
      for(unsigned i=0; i<8; i++) {
        int j = i+1;
        if(j > 7) j=0;
        if(neighbors[i] == true && neighbors[j] == true)
          nnb_counter += nnb_inc;
      }

      if(nb_counter <= nb && (nb_counter+nnb_counter) <= nnb)
        if(mask[row*patches.cols + col] == 0) {
          #pragma omp critical 
          {
            mask[row*patches.cols + col] = 1;
            reassign_idxs[patches(row, col) - 1].push_back(row*patches.cols + col);
          }
        }
    }
  }
}

/**
 * Reasign points to neighbouring patches
 * We allow the inlier distance to assign points to other patches
 */
bool ClusterNormalsToPlanes::ReasignPoints(std::vector< std::vector<int> > &reassign_idxs)
{
  bool ready = false;
  bool assigned = false;
  bool have_assigned = false;
  while(!ready) {
    assigned = false;
    
    for(int i=(int)view->surfaces.size()-1; i>=0 ; i--) {
      std::vector<int> not_assigned_indexes;
      for(int j=0; j< (int) reassign_idxs[i].size(); j++) {
        int idx = reassign_idxs[i][j];
        unsigned row = reassign_idxs[i][j] / cloud->width;
        unsigned col = reassign_idxs[i][j] % cloud->width;
        if(row >= 0 && row < cloud->height && col >= 0 && col < cloud->width) {
          int surounding[view->surfaces.size()];
          for(unsigned s=0; s<view->surfaces.size(); s++)
            surounding[s] = 0;
        
          for(unsigned v=row-1; v<=row+1; v++) {
            for(unsigned u=col-1; u<=col+1; u++) {
              if(v!=row || u!=col) {
                int a_idx = v*cloud->width + u;
            
                float dist = 1.0;
                if(!isnan(cloud->points[a_idx].z))
                  dist = (cloud->points[idx].getVector3fMap() - cloud->points[a_idx].getVector3fMap()).norm();
                
                if(patches(v, u) != 0 && 
                  patches(v, u) != patches(row, col) &&
                  dist < param.ra_dist)
                  surounding[patches(v, u) -1]++;
              }
            }
          }
          
          int max_neighbors = 0;
          int most_id = 0;
          for(unsigned nr=0; nr<view->surfaces.size(); nr++) {
            if(max_neighbors < surounding[nr]) {
              max_neighbors = surounding[nr];
              most_id = nr;
            }
          }

          if(max_neighbors > 0) {
            have_assigned = true;
            assigned = true;
            view->surfaces[most_id]->indices.push_back(idx);

            std::vector<int> surfaces_indices_copy;
            for(int su=0; su < (int)view->surfaces[i]->indices.size(); su++)
              if(view->surfaces[i]->indices[su] != idx)
                surfaces_indices_copy.push_back(view->surfaces[i]->indices[su]);
            view->surfaces[i]->indices = surfaces_indices_copy;
              
            patches(row, col) = most_id+1;
          }
          else {
            not_assigned_indexes.push_back(idx);
          }
        }
      }
      reassign_idxs[i] = not_assigned_indexes;
    }
    if(!assigned)
      ready = true;
  }
  return have_assigned;
}


void ClusterNormalsToPlanes::DeleteEmptyPlanes()
{ 
  std::vector<surface::SurfaceModel::Ptr> planes_copy;
  for(int su=0; su<(int)view->surfaces.size(); su++) {
    if((int)view->surfaces[su]->indices.size() > 0) {
      if((int)view->surfaces[su]->indices.size() < param.minPoints) {
        view->surfaces[su]->type = -1;
        planes_copy.push_back(view->surfaces[su]);
      }
      else
        planes_copy.push_back(view->surfaces[su]);
    }
  }
  view->surfaces = planes_copy;
}

/* Reasign single pixels (line-ends) */
void ClusterNormalsToPlanes::SinglePixelCheck()
{
  int max_nb = 2;
  int max_nnb = 1;
  int nnb_inc = -1;
  bool assigned = true;
  std::vector< std::vector<int> > reassign_idxs;
  while(assigned) {
    CountNeighbours(reassign_idxs, max_nb, max_nnb, nnb_inc);
    assigned = ReasignPoints(reassign_idxs);
  }
}


void ClusterNormalsToPlanes::PixelCheck()
{ 
  #ifdef DEBUG
    printf("[ClusterNormalsToPlanes::PixelCheck] Surfaces before deletion: %lu\n", view->surfaces.size());
  #endif

  std::vector< std::vector<int> > reassign_idxs;
  mask.clear(); // move pixel only once
  mask.resize(cloud->width*cloud->height, 0);

  // Reassign patches which have less neighbouring points than minPoints
  CreatePatchImage();
  CountNeighbours(reassign_idxs, max_neighbours, max_nneighbours, 1);

  for(unsigned i=0; i<view->surfaces.size(); i++) {
    if((view->surfaces[i]->indices.size() - reassign_idxs[i].size()) < param.minPoints)
      reassign_idxs[i] = view->surfaces[i]->indices;
    else
      reassign_idxs[i].clear();
  }
  ReasignPoints(reassign_idxs);

  // Reassign single-neighboured points of patches
  mask.clear();
  mask.resize(cloud->width*cloud->height,0);
  SinglePixelCheck();
  DeleteEmptyPlanes();
  
  #ifdef DEBUG
    printf("[ClusterNormalsToPlanes::PixelCheck] Surfaces after deletion: %lu\n", view->surfaces.size());
  #endif   
}


/**
 * Cluster rest of the points
 */
void ClusterNormalsToPlanes::ClusterRest(unsigned idx, 
                                         pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                         pcl::PointCloud<pcl::Normal> &normals, 
                                         std::vector<int> &pts,
                                         pcl::Normal &normal)
{
  short x,y;
  normal = normals.points[idx];
  pts.clear();

  if (isnan(cloud.points[idx].x))
    printf("[ClusterNormalsToPlanes::ClusterRest] Error: NAN found.\n");

  mask[idx] = 1;
  pts.push_back(idx);
  
  int queue_idx = 0;
  std::vector<int> queue;
  queue.reserve(cloud.width*cloud.height);
  queue.push_back(idx);       
  
  while ((int)queue.size() > queue_idx) {
    idx = queue[queue_idx];
    queue_idx++;
    x = X(idx);
    y = Y(idx);

    for (int v=y-1; v<=y+1; v++) {
      for (int u=x-1; u<=x+1; u++) {
        if (v>=0 && u>=0 && v<height && u<width) {
          idx = GetIdx(u,v);
          if (mask[idx] == 0) {
              mask[idx] = 1;

              Mul3(&normal.normal[0], pts.size(), &normal.normal[0]);
              Add3(&normal.normal[0], &normals.points[idx].normal[0], &normal.normal[0]);

              pts.push_back(idx);
              queue.push_back(idx);
              
              Mul3(&normal.normal[0], 1./(float)pts.size(), &normal.normal[0]);
              normal.getNormalVector3fMap().normalize();
          }
        }
      }
    }
  }
}

void ClusterNormalsToPlanes::CalcAdaptive()
{
  p_adaptive_cosThrAngleNC.resize(cloud->width*cloud->height);
  p_adaptive_inlDist.resize(cloud->width*cloud->height);

  #pragma omp parallel for
  for(unsigned i=0; i<cloud->points.size(); i++) {
    Eigen::Vector3f curPt = cloud->points[i].getVector3fMap();
    if(curPt[2] <= param.d_c)
      p_adaptive_cosThrAngleNC[i] = cos(param.epsilon_c);
    else
      p_adaptive_cosThrAngleNC[i] = cos(param.epsilon_c + param.epsilon_g*(curPt[2]-param.d_c));
    p_adaptive_inlDist[i] = param.omega_c + param.omega_g*curPt[2];
  }
}

/**
 * ClusterNormals
 */
void ClusterNormalsToPlanes::ClusterNormals(unsigned idx, 
                                            pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                            pcl::PointCloud<pcl::Normal> &normals, 
                                            std::vector<int> &pts,
                                            pcl::Normal &normal)
{
  normal = normals.points[idx];
  pts.clear();

  if (normal.normal[0]!=normal.normal[0])
    return;

  mask[idx] = 1;
  pts.push_back(idx);
  
  int queue_idx = 0;
  std::vector<int> queue;
  queue.reserve(cloud.width*cloud.height);
  queue.push_back(idx);
  
  EIGEN_ALIGN16 Eigen::Vector3f pt = cloud.points[idx].getVector3fMap();

  while ((int)queue.size() > queue_idx) {
    idx = queue[queue_idx];
    queue_idx++;

    std::vector<int> n4ind;
    n4ind.push_back(idx-1);
    n4ind.push_back(idx+1);
    n4ind.push_back(idx+width);
    n4ind.push_back(idx-width);
    for(unsigned i=0; i<n4ind.size(); i++) {
      int u = n4ind[i] % width;
      int v = n4ind[i] / width;

      if (v>=0 && u>=0 && v<height && u<width) {
        idx = GetIdx(u,v);
        if (mask[idx]==0) {
          const float *n = &normals.points[idx].normal[0];

          if (n[0]!= n[0])
            continue;

          float newCosThrAngleNC = cosThrAngleNC;
          float newInlDist = param.inlDist;
          if(param.adaptive) {
            newCosThrAngleNC = p_adaptive_cosThrAngleNC[idx];
            newInlDist = p_adaptive_inlDist[idx];
          }
          
          if (Dot3(&normal.normal[0], n) > newCosThrAngleNC && 
              fabs(Plane::NormalPointDist(&pt[0], &normal.normal[0], &cloud.points[idx].x)) < newInlDist)
          {
            mask[idx] = 1;
            
            Mul3(&normal.normal[0], pts.size(), &normal.normal[0]);
            Add3(&normal.normal[0], &normals.points[idx].normal[0], &normal.normal[0]);
            pt = pt*pts.size();
            pt += cloud.points[idx].getVector3fMap();

            pts.push_back(idx);
            queue.push_back(idx);

            Mul3(&normal.normal[0], 1./(float)pts.size(), &normal.normal[0]);
            pt /= (float)pts.size();
            normal.getNormalVector3fMap().normalize();
          }
        }
      }
    }
  }

  // check if all points are on the plane 
  unsigned ptsSize = pts.size();
  if( (int) pts.size() >= param.minPoints) {
    for(unsigned i=0; i<ptsSize; i++) {
      const float *n = &normals.points[pts[i]].normal[0];

      float newCosThrAngleNC = cosThrAngleNC;
      float newInlDist = param.inlDist;
      if(param.adaptive) {
        newCosThrAngleNC = p_adaptive_cosThrAngleNC[idx];
        newInlDist = p_adaptive_inlDist[idx];
      }          
            
      if (Dot3(&normal.normal[0], n) < newCosThrAngleNC && 
          fabs(Plane::NormalPointDist(&pt[0], &normal.normal[0], &cloud.points[pts[i]].x)) > newInlDist)
      {
        mask[pts[i]]=0;
        pts.erase(pts.begin()+i);
        ptsSize--;
        i--;
      }
    }

    // recalculate plane normal
    if(pts.size() > 0) {
      EIGEN_ALIGN16 Eigen::Vector3f new_n = normals.points[pts[0]].getNormalVector3fMap();
      for(unsigned i=1; i<pts.size(); i++)
        new_n += normals.points[pts[i]].getNormalVector3fMap();
      new_n.normalize();
      normal.normal[0] = new_n[0];
      normal.normal[1] = new_n[1];
      normal.normal[2] = new_n[2];
    }
  }
}


/**
 * ClusterNormals
 */
void ClusterNormalsToPlanes::ClusterNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                            pcl::PointCloud<pcl::Normal> &normals, 
                                            const std::vector<int> &indices, 
                                            std::vector<SurfaceModel::Ptr> &planes)
{
  pcl::Normal normal;
  SurfaceModel::Ptr plane;
  
  // init mask (without nans)
  mask.clear();
  if (indices.size()==0) {
    mask.resize(cloud.width*cloud.height,0);
    for (unsigned i=0; i<cloud.width*cloud.height; i++)
      if(isnan(cloud.points[i].x))
        mask[i] = 1;
  } else {
    mask.resize(cloud.width*cloud.height,1);
    for (unsigned i=0; i<indices.size(); i++)
      if(!isnan(cloud.points[i].x))
        mask[indices[i]] = 0;
  }
  
  for(unsigned i=0; i< srt_curvature.size(); i++)
    srt_curvature[i].clear();
  for(int idx=0; idx< (int) view->normals->points.size(); idx++) {
    if(view->normals->points[idx].curvature == 0.0) {
      if (mask[idx]==0)
        srt_curvature[0].push_back(idx);
    }
    else {
      double curvature = view->normals->points[idx].curvature*1000;
      unsigned curv = (unsigned) curvature;
      if(curv > 19)
        curv = 19;
      if (mask[idx]==0)
        srt_curvature[curv].push_back(idx);
    }
  }
  
  // cluster points
  bool morePlanes = true;
  while(morePlanes) {
    morePlanes = false;
    for(unsigned i=0; i<srt_curvature.size(); i++) {
      for(unsigned j=0; j<srt_curvature[i].size(); j++) {
        unsigned idx = srt_curvature[i][j]; 
        if (mask[idx]==0) {
          plane.reset(new SurfaceModel());
          plane->type = pcl::SACMODEL_PLANE;

          ClusterNormals(idx, cloud, normals, plane->indices, normal);

          if (((int)plane->indices.size()) >= param.minPoints) {
            morePlanes = true;
            planes.push_back(plane);
            plane->coeffs.resize(3);
            float *n = &plane->coeffs[0];
            n[0]=normal.normal[0];
            n[1]=normal.normal[1];
            n[2]=normal.normal[2];
          }
          else {
            std::vector<int> &pts = plane->indices;
            for (unsigned i=0; i<pts.size(); i++)
              mask[pts[i]]=0;
          }
        }
      }
    }
  }
  
  // cluster rest of unclustered point cloud in 2D
  for (unsigned v=0; v<cloud.height; v++) {
    for (unsigned u=0; u<cloud.width; u++) {
      unsigned idx = GetIdx(u,v);
      if (mask[idx] == 0) {
        plane.reset(new SurfaceModel());
        plane->type = -1; // No model
        plane->coeffs.resize(3);
        float *n = &plane->coeffs[0];
        n[0]=normal.normal[0];
        n[1]=normal.normal[1];
        n[2]=normal.normal[2];
        ClusterRest(idx, cloud, normals, plane->indices, normal);
        if (((int)plane->indices.size()) > 0)
          planes.push_back(plane);
      }
    }
  }
}

/**
 * ComputeLSPlanes to check plane models
 */
void ClusterNormalsToPlanes::ComputeLSPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                                             std::vector<SurfaceModel::Ptr> &planes)
{
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> lsPlane(cloud);
  Eigen::VectorXf coeffs(4);

  for (unsigned i=0; i<planes.size(); i++) {
    if(planes[i]->type == pcl::SACMODEL_PLANE && planes[i]->indices.size() > 4) {
      SurfaceModel &plane = *planes[i];

      lsPlane.optimizeModelCoefficients(plane.indices, coeffs, coeffs);
      if (Dot3(&coeffs[0], &cloud->points[plane.indices[0]].x) > 0)
        coeffs*=-1.;
      if (Dot3(&coeffs[0], &plane.coeffs[0]) > cosThrAngleNC) {
        plane.coeffs.resize(4);
        plane.coeffs[0] = coeffs[0];
        plane.coeffs[1] = coeffs[1];
        plane.coeffs[2] = coeffs[2];
        plane.coeffs[3] = coeffs[3];
      }
      else {
        planes[i]->type = -1;
#ifdef DEBUG
        printf("[ClusterNormalsToPlanes::ComputeLSPlanes] Warning: Problematic plane found: %u\n", i);
#endif
      }
    }
  }
}

/**
 * Add normals to surface patches. Add directed plane normals (to camera view).
 */
void ClusterNormalsToPlanes::AddNormals()
{
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    view->surfaces[i]->normals.resize(view->surfaces[i]->indices.size());
    if(view->surfaces[i]->type != pcl::SACMODEL_PLANE ){     // no PLANE model
      for(unsigned ni=0; ni<view->surfaces[i]->indices.size(); ni++) {
        Eigen::Vector3f nor = view->normals->points[view->surfaces[i]->indices[ni]].getNormalVector3fMap();
        view->surfaces[i]->normals[ni][0] = nor[0];
        view->surfaces[i]->normals[ni][1] = nor[1];
        view->surfaces[i]->normals[ni][2] = nor[2];
      }
    }
    else {
      float coeffs[3];  // kann man das mit float * machen?
      coeffs[0] = view->surfaces[i]->coeffs[0];
      coeffs[1] = view->surfaces[i]->coeffs[1];
      coeffs[2] = view->surfaces[i]->coeffs[2];
      
      for (unsigned j = 0; j < view->surfaces[i]->indices.size(); j++) {
        view->surfaces[i]->normals[j][0] = coeffs[0];
        view->surfaces[i]->normals[j][1] = coeffs[1];
        view->surfaces[i]->normals[j][2] = coeffs[2];
        // copy model normals to view
        view->normals->points[view->surfaces[i]->indices[j]].normal_x = coeffs[0];
        view->normals->points[view->surfaces[i]->indices[j]].normal_y = coeffs[1];
        view->normals->points[view->surfaces[i]->indices[j]].normal_z = coeffs[2];
      }
    }
  }
}


/************************** PUBLIC *************************/

/**
 * set the input cloud for detecting planes
 */
void ClusterNormalsToPlanes::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (_cloud->height<=1 || _cloud->width<=1 || !_cloud->isOrganized())
    throw std::runtime_error("[ClusterNormalsToPlanes::setInputCloud] Invalid point cloud (height must be > 1)");

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
}

void ClusterNormalsToPlanes::setView(surface::View *_view)
{
  view = _view; 
}

/**
 * setParameter
 */
void ClusterNormalsToPlanes::setParameter(Parameter p)
{
  param = p;
  cosThrAngleNC = cos(param.thrAngle);
}

/**
 * @brief Check if there are patch models with "line"-style (not more than n neighbors)
 * @param check True to check
 * @param neighbors Threshold for line_check neighbors
 */
void ClusterNormalsToPlanes::setPixelCheck(bool check, int neighbors)
{
  pixel_check = check;
  max_neighbours = neighbors;
  max_nneighbours = neighbors*2;
}


/**
 * Compute
 */
void ClusterNormalsToPlanes::compute(const std::vector<int> &indices)
{
  #ifdef DEBUG
  std::cout << "********************** start detecting planes **********************" << std::endl;
  #endif

  if (cloud.get()==0 || view->normals.get()==0)
    throw std::runtime_error("[ClusterNormalsToPlanes::compute] Point cloud or view not set!"); 

  if(param.adaptive)
    CalcAdaptive();

  ClusterNormals(*cloud, *(view->normals), indices, view->surfaces);
  
  if(pixel_check)
    PixelCheck();

  ComputeLSPlanes(cloud, view->surfaces);

  AddNormals();
}

/**
 * Compute
 */
void ClusterNormalsToPlanes::compute()
{
  std::vector<int> indices;
  compute(indices);
}


} //-- THE END --

