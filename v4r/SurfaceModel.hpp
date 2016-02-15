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

#ifndef SURFACE_SURFACEMODEL_HPP
#define SURFACE_SURFACEMODEL_HPP

#ifndef MODEL_NURBS
#define MODEL_NURBS 20  // Should be higher than pcl::SACMODEL_STICK
#endif

#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/surface/on_nurbs/sequential_fitter.h>

#include "Relation.h"

#ifdef V4R_TOMGINE
  #include "v4r/TomGine/tgRenderModel.h"
#endif


namespace surface
{

/** Edgel **/
struct Edgel {
  float x;
  float y;
};

/** Corner **/
struct Corner {
  float x;
  float y;
};

/** Edge **/
struct Edge {
  int surface[2];               // indices of surfaces
  int corner[2];                // indices of corners in view
  std::vector<int> edgels;      // indices of edgels in view
  void Print() {
    printf("  surfaces: %u-%u\n", surface[0], surface[1]);
    printf("  corner: %u-%u\n", corner[0], corner[1]);
    printf("  edge-size: %lu\n", edgels.size());
  }
};


/** @brief Surface model: describing a parametrized surface **/
class SurfaceModel
{
  public:
    int idx;                                            ///< for merging in surface modeling
    int type;                                           ///< type of surface model (plane, NURBS, (same than pcl::SACMODEL))
    int label;                                          ///< object assignment label
    int label_ass;                                      ///< object assignment label for assembly level
    int level;                                          ///< pyramid level for MoSPlanes TODO remove
    bool used;                                          ///< for surface modeling TODO unused?
    bool selected;                                      ///< for surface modeling TODO unused?
    double savings;                                     ///< for surface modeling
    Eigen::Matrix4d pose;                               ///< transformation from View to SurfaceModel coordinate frame

    std::vector<int> indices;                           ///< index list for 2D data
    std::vector<double> error;                          ///< error of each point
    std::vector<double> probs;                          ///< probability of each points
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > normals;

    std::vector<float> coeffs;                          ///< model coefficients
    std::vector<unsigned> neighbors2D;                  ///< 2D neighbors of patch (not related to idx => related with view->surfaces[i])
    std::vector<unsigned> neighbors2DNrPixel;           ///< Number of neighboring pixels for 2D neighbors
    std::vector<unsigned> neighbors3D;                  ///< 3D neighbors of patch (not related to idx=> related with view->surfaces[i])
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > nurbs_params;

    std::vector< std::vector<int> > contours;           ///< ordered boundary contours (for all contours)
    std::vector<Edge*> edges;                           ///< Boundary graph: Reference to edges

    std::vector<int> concave;                           ///< image indices of concave pts on contour
    std::vector<int> convex;                            ///< image indices of convex pts on contour
    std::vector< std::vector<int> > split_pathes;       ///< index list of splitting pathes
    std::vector<int> costs;                             ///< costs of the pathes

    ON_NurbsSurface nurbs;                              ///< B-spline surface
    std::vector<ON_NurbsCurve> curves_image;            ///< B-spline curves in image space
    std::vector<ON_NurbsCurve> curves_param;            ///< B-spline curves in image space

#ifdef V4R_TOMGINE
    TomGine::tgRenderModel mesh;                        ///< mesh model for displaying @ tgTomGineThread
#endif

    SurfaceModel(): idx(-1), label(-1), label_ass(-1), used(false), selected(true) {}
    SurfaceModel(int _idx) : idx(_idx), label(-1), label_ass(-1), used(false), selected(true) {}

    void AddTo(SurfaceModel &model)
    {
      for (unsigned i=0; i<indices.size(); i++)
        model.indices.push_back(indices[i]);
      for (unsigned i=0; i<neighbors3D.size(); i++)
        model.neighbors3D.push_back(neighbors3D[i]);
      for (unsigned i=0; i<neighbors2D.size(); i++)
        model.neighbors2D.push_back(neighbors2D[i]);
    }

    typedef boost::shared_ptr< ::surface::SurfaceModel> Ptr;
    typedef boost::shared_ptr< ::surface::SurfaceModel const> ConstPtr;

    void Print() {
      printf("[SurfaceModel]\n  ");
      printf("  idx: %d\n  ", idx);
      printf("  type: %d\n  ", type);
      printf("  level: %d\n  ", level);
      printf("  label: %d\n  ", label);
      printf("  label_ass: %d\n  ", label_ass);
      printf("  used: %d\n  ", used);
      printf("  selected: %d\n  ", selected);
      printf("  savings: %f\n  ", savings);
      printf("  indices.size: %lu\n  ", indices.size());
      printf("  errors.size: %lu\n  ", error.size());
      printf("  probs.size: %lu\n  ", probs.size());
      printf("  coeffs.size %lu: ", coeffs.size());
      for(size_t i=0; i<coeffs.size(); i++)
        printf("%f ", coeffs[i]);
      printf("\n  ");
      printf("  Number of contours: %lu\n", contours.size());
      for(size_t i=0; i<contours.size(); i++)
        printf("  contours[%lu].size %lu\n  ", i, contours[i].size());
      printf("  neighbors2D [%lu]: ", neighbors2D.size());
      for(size_t i=0; i<neighbors2D.size(); i++)
        printf("%d ", neighbors2D[i]);
      printf("\n  ");
      printf("  neighbors3D [%lu]: ", neighbors3D.size());
      for(size_t i=0; i<neighbors3D.size(); i++)
        printf("%d ", neighbors3D[i]);
      printf("\n  ");
      printf("  nurbs: order: %dx%d, control-points: %dx%d\n  ",
             nurbs.Order(0), nurbs.Order(1), nurbs.CVCount(0), nurbs.CVCount(1));
      printf("  curves_image: %lu\n", curves_image.size());
      printf("  curves_param: %lu\n", curves_param.size());
#ifdef V4R_TOMGINE
      printf("  mesh: faces: %lu, vertices: %lu\n", mesh.m_faces.size(), mesh.m_vertices.size());
#endif
      printf("\n");
    }
};


/** View **/
class View{
public:
  unsigned width;
  unsigned height;
  Eigen::Matrix3d intrinsic;
  Eigen::Matrix4d extrinsic;
  std::vector<SurfaceModel::Ptr> surfaces;              ///< Surface models
  std::vector<Relation> relations;                      ///< PG relation vectors between surface patches
  std::vector< std::vector<unsigned> > graphCutGroups;  ///< Object model groups /// TODO remove later => surface->label

  bool havePatchImage;
  cv::Mat_<int> patchImage;                             ///< Patch image
  
  std::vector<Edgel> edgels;                            ///< Boundary graph: Edgels
  std::vector<Corner> corners;                          ///< Boundary graph: Corners
  std::vector<Edge> edges;                              ///< Boundary graph: Edges

  bool haveNormals;
  pcl::PointCloud<pcl::Normal>::Ptr normals;            ///< Normals of the point cloud (similar to surface normals)

  View(): havePatchImage(false), haveNormals(false) {}

  void Reset() {
    havePatchImage = false;
    surfaces.clear();
    relations.clear();
    graphCutGroups.clear();
    edgels.clear();
    corners.clear();
    edges.clear();
    normals.reset(new pcl::PointCloud<pcl::Normal>);
  }
  
  void PrintEdges() {
    for(unsigned i=0; i<edges.size(); i++) {
      printf("Edge %u: \n", i);
      edges[i].Print();
    }
  }

  void Print() {
    for(size_t i=0; i<relations.size(); i++)
      printf("Relation [%u, %u]: %d (GT: %d)\n", relations[i].id_0, relations[i].id_1, relations[i].prediction, relations[i].groundTruth);
    for(size_t i=0; i<surfaces.size(); i++) {
      printf("%lu ", i);
      surfaces[i]->Print();
    }
  }
};

class Scene{
  // todo timestamp
  std::vector<View> views;
};


/* --------------------------- inline --------------------------- */

/** create a deep copy of surface pointers **/
inline std::vector<surface::SurfaceModel::Ptr>
deepCopy(const std::vector<surface::SurfaceModel::Ptr> &_surfaces) {
  std::vector<surface::SurfaceModel::Ptr> surfaces;
  for(unsigned i=0; i<_surfaces.size(); i++) {
    surface::SurfaceModel::Ptr c_surface;
    c_surface.reset(new surface::SurfaceModel());
    (*c_surface) = *_surfaces[i];
    surfaces.push_back(c_surface);
  }
  return surfaces;
}

/** create a deep copy of surface pointers of specific type**/
inline std::vector<surface::SurfaceModel::Ptr>
deepCopyType(const std::vector<surface::SurfaceModel::Ptr> &_surfaces, int type) {
  std::vector<surface::SurfaceModel::Ptr> surfaces;
  for(unsigned i=0; i<_surfaces.size(); i++) {
    if(_surfaces[i]->type == type) {
      surface::SurfaceModel::Ptr c_surface;
      c_surface.reset(new surface::SurfaceModel());
      (*c_surface) = *_surfaces[i];
      surfaces.push_back(c_surface);
    }
  }
  return surfaces;
}


}

#endif

