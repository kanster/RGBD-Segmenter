/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Thomas Mörwald nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @file tgShapeCreator.h
 * @author Thomas Mörwald
 * @date March 2010
 * @version 0.1
 * @brief Generating a basic shapes.
 * @namespace TomGine
 */

#ifndef TGSHAPECREATOR_H
#define TGSHAPECREATOR_H

#include <vector>

#include "tgModel.h"

namespace TomGine {

/** @brief Creates basic shapes as triangle/quadrangle mesh (tgModel) */
class tgShapeCreator
{
private:

  static void init_tetrahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices, unsigned &n_faces,
      unsigned &n_edges);
  static void init_octahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices, unsigned &n_faces,
      unsigned &n_edges);
  static void init_icosahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices, unsigned &n_faces,
      unsigned &n_edges);

  static int search_midpoint(int &index_start, int &index_end, unsigned &n_vertices, int &edge_walk, std::vector<int> &midpoint,
      std::vector<int> &start, std::vector<int> &end, std::vector<float> &vertices);
  static void subdivide(unsigned &n_vertices, unsigned &n_edges, unsigned &n_faces, std::vector<float> &vertices,
      std::vector<int> &faces);

  static void CreatePlaneIndices(tgModel & model, unsigned vidx, unsigned segX, unsigned segY);

  // for polygon triangulation
  static bool Snip(const std::vector<vec2> &contour, int u, int v, int w, int n, int *V);
  static bool TriangulatePolygon(const std::vector<vec2> &contour, std::vector<vec2> &result);
  static float Area(const std::vector<vec2> &contour);
  static bool InsideTriangle(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float Px, float Py);

public:

  /** @brief Polyhedral primitives (used for sphere creation) */
  enum Polyhedron
  {
    TETRAHEDRON = 0, OCTAHEDRON = 1, ICOSAHEDRON = 2
  };

  /** @brief	Create a triangulated sphere with center (0,0,0).
   *  @param	model	The resulting triangle mesh.
   *  @param	radius	The radius of the sphere.
   *  @param	subdevisions	Iterations of refinement. (subdevisions==0 leads to the polyhedron given by 'method')
   *  @param	methos	Primitive used as basis for sphere approximation (Tetrahedron, Octahedron, Icosahedron)	 */
  static void CreateSphere(tgModel& model, float radius = 0.5f, unsigned subdevisions = 1, int method = 0);

  /** @brief	Create a box with dimension x,y,z */
  static void CreateBox(tgModel& model, float x = 1.0f, float y = 1.0f, float z = 1.0f);

  /** @brief	Create a cylinder.
   *  @param	model	The resulting model consisting of triangles and quadrangles.
   *  @param	radius	The radius of the cylinder.
   *  @param 	height	The height of the cylinder.
   *  @param	slices	Number of segments used for approximating the perimeter.
   *  @param	stacks	Number of segments subdeviding the height.
   *  @param	closed	Close the caps of the cylinder if true.	 */
  static void CreateCylinder(tgModel &model, float radius = 0.5f, float height = 1.0f, unsigned slices = 16, unsigned stacks = 1,
      bool closed = true);

  /** @brief	Create a cone.
   *  @param	model	The resulting model consisting of triangles and quadrangles.
   *  @param	radius	The radius of the cone.
   *  @param 	height	The height of the cone.
   *  @param	slices	Number of segments used for approximating the perimeter.
   *  @param	stacks	Number of segments subdeviding the height.
   *  @param	closed	Close the caps of the cone if true.	 */
  static void CreateCone(tgModel &model, float radius = 0.5f, float height = 1.0f, unsigned slices = 16, unsigned stacks = 1,
      bool closed = true);

  /** @brief	Create a plane in the XY plane.
   *  @param	x0,y0,z0	Initial point of the plane (z0 beeing the offset to the z axis).
   *  @param	w,h		Size of the plane (width, height in X,Y direction respectively).
   *  @param	segX,segY	Number of segments to subdevide the plane in X,Y direction respectively. */
  static void CreatePlaneXY(tgModel &model, float x0 = 0.0f, float y0 = 0.0f, float z0 = 0.0f, float w = 1.0f, float h = 1.0f,
      unsigned segX = 1, unsigned segY = 1);

  /** @brief	Create a plane in the YZ plane.
   *  @param	x0,y0,z0	Initial point of the plane (x0 beeing the offset to the x axis).
   *  @param	w,h		Size of the plane (width, height in Y,Z direction respectively).
   *  @param	segY,segZ	Number of segments to subdevide the plane in Y,Z direction respectively. */
  static void CreatePlaneYZ(tgModel &model, float x0 = 0.0f, float y0 = 0.0f, float z0 = 0.0f, float w = 1.0f, float h = 1.0f,
      unsigned segY = 1, unsigned segZ = 1);

  /** @brief  Create a plane in the ZX plane.
   *  @param  x0,y0,z0  Initial point of the plane (y0 beeing the offset to the y axis).
   *  @param  w,h   Size of the plane (width, height in Z,X direction respectively).
   *  @param  segX,segY Number of segments to subdevide the plane in Z,X direction respectively. */
  static void CreatePlaneZX(tgModel &model, float x0 = 0.0f, float y0 = 0.0f, float z0 = 0.0f, float w = 1.0f, float h = 1.0f,
      unsigned segX = 1, unsigned segZ = 1);

  /** @brief  Create a segmented line.
   *  @param  line  Definition of the line
   *  @param  seg Number of segments to subdevide the line. */
  static void CreateLine(tgModel &model, tgLine &line, unsigned seg = 1);

  /** @brief	Triangulate a polygon.
   *  @param	model	The resulting triangle mesh.
   *  @param	points	Sequential corner points of the polygon. */
  static void PolygonMesh(tgModel& model, const std::vector<vec3> &points);

};

}

#endif
