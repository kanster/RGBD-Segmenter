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
 * @author thomas.moerwald
 *
 * @file tgCollission.h
 * @author Thomas Mörwald
 * @date June 2010
 * @version 0.1
 * @brief Intersection tests between objects (Point, Line, Ray, Sphere, Boxes, ...)
 * @namespace TomGine
 */

#ifndef TG_COLLISSION_H
#define TG_COLLISSION_H

#include <vector>
#include "tgMathlib.h"
#include "tgModel.h"
#include "tgRenderModel.h"

namespace TomGine {

/** @brief Intersection and collission tests of triangles, rays, models, ... */
class tgCollission
{
private:
  static bool valueInRange(int value, int min, int max)
  {
    return (value >= min) && (value <= max);
  }

public:
  /** @brief Get normal vector of a triangle.
   *  @param v1,v2,v3 vertices of the triangle. */
  static vec3 GetNormal(const vec3& v1, const vec3& v2, const vec3& v3);

  /** @brief Intersetion tests of two rectangles in 2D
   *  @param ll1,ll2		lower left corner of rectangle 1 and 2
   *  @param ur2,ur2		upper right corner of rectangle 1 and 2	 */
  static bool IntersectRectangles2D(const tgRect2D& A, const tgRect2D& B);

  /** @brief Intersection test between ray and triangle.
   *  @return bool	true if ray and triangle intersect, false if no intersection exists.
   *  @return &p		point of intersection.
   *  @return &n		normal at point of intersetion.
   *  @return &z		distance from start point of ray to intersection point.
   *  @param ray		ray for intersection.
   *  @param t1,t2,t3 vertices of triangle for intersection	. */
  static bool IntersectRayTriangle(vec3& p, vec3& n, double& z, const tgRay& ray, const vec3& t1, const vec3& t2, const vec3& t3);

  /** @brief Intersection test between ray and sphere.
   *  @return bool  true if ray and sphere intersect, false if no intersection exists.
   *  @return &t    intersection point, distance from origin along ray direction
   *  @param ray    ray for intersection.
   *  @param sphere sphere for intersection	 */
  static bool IntersectRaySphere(const tgRay &ray, const BoundingSphere &sphere, float &t);

  /** @brief Intersection test between ray and model.
   *  @return bool	true if ray and model intersect, false if no intersection exists.
   *  @return &pl		point-list of intersections.
   *  @return &nl		normal-list of points of intersections.
   *  @return &zl		distance-list from start point of ray to intersection points.
   *  @param ray		ray for intersection.
   *  @param model	model for intersection.	 */
  static bool IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray,
      const tgModel& model);

  /** @brief Intersection test between ray and model.
   *  @return bool  true if ray and model intersect, false if no intersection exists.
   *  @return &pl   point-list of intersections.
   *  @return &nl   normal-list of points of intersections.
   *  @return &zl   distance-list from start point of ray to intersection points.
   *  @param ray    ray for intersection.
   *  @param model  model for intersection.  */
  static bool IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray,
      const tgRenderModel& model);

  /** @brief not implemented */
  static bool IntersectModels(const tgModel &m1, const tgModel &m2, const tgPose &p1, const tgPose& p2);
  /** @brief not implemented */
  static bool IntersectModels(const tgRenderModel &m1, const tgRenderModel &m2);

  /** @brief Tests if point 'p1' lies on the same side as 'p2'.
   *  @param p1,p2	Points to test if they lie on the same side.
   *  @param a,b		Line to test points against.
   *  @return bool	true if points 'p1','p2' are on the same side with respect to the line 'a','b'	 */
  static bool PointOnSameSide(const vec3& p1, const vec3& p2, const vec3& a, const vec3& b);

  /** @brief Tests if a point lies in or outside a triangle.
   *  @param p		Point to test
   *  @param t1,t2,t3 Triangle given by vertices
   *  @return bool	true if point lies in triangle, false if outside. 	 */
  static bool PointInTriangle(const vec3& p, const vec3& t1, const vec3& t2, const vec3& t3);

};

}

#endif /* TG_COLLISSION_H */
