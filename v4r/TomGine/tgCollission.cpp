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
 */

#include "tgCollission.h"
#include <stdexcept>

using namespace TomGine;

vec3 tgCollission::GetNormal(const vec3& v1, const vec3& v2, const vec3& v3)
{
  vec3 n, e1, e2;

  e1 = v2 - v1;
  e2 = v3 - v1;

  n.cross(e1, e2);
  n.normalize();

  return n;
}

bool tgCollission::IntersectRectangles2D(const tgRect2D& A, const tgRect2D& B)
{
  bool xOverlap = valueInRange(A.x, B.x, B.x + B.w) || valueInRange(B.x, A.x, A.x + A.w);

  bool yOverlap = valueInRange(A.y, B.y, B.y + B.h) || valueInRange(B.y, A.y, A.y + B.h);

  return xOverlap && yOverlap;
}

bool tgCollission::IntersectRayTriangle(vec3& p, vec3& n, double& z, const tgRay& ray, const vec3& t1, const vec3& t2,
    const vec3& t3)
{
  // p = ray.start + ray.dir * z (1)
  // p*n + d = 0   ; point p must be on plane (with normal vector n and distance d) (2)

  n = GetNormal(t1, t2, t3); // plane normal

  double d = -t1 * n; // distance of plane form origin

  // from (1) and (2)
  z = -(ray.start * n + d) / (ray.dir * n);

  p = ray.start + ray.dir * (float) z;

  // test if point lies in triangle or not
  return PointInTriangle(p, t1, t2, t3);
}

bool tgCollission::IntersectRaySphere(const tgRay &ray, const BoundingSphere &sphere, float &t)
{
  //Compute A, B and C coefficients
  float a = (ray.dir * ray.dir);
  float b = 2 * (ray.start * ray.start);
  float c = (ray.start * ray.start) - (sphere.radius * sphere.radius);

  //Find discriminant
  float disc = b * b - 4 * a * c;

  // if discriminant is negative there are no real roots, so return
  // false as ray misses sphere
  if (disc < 0)
    return false;

  // compute q as described above
  float distSqrt = sqrtf(disc);
  float q;
  if (b < 0)
    q = (-b - distSqrt) / 2.0;
  else
    q = (-b + distSqrt) / 2.0;

  // compute t0 and t1
  float t0 = q / a;
  float t1 = c / q;

  // make sure t0 is smaller than t1
  if (t0 > t1) {
    // if t0 is bigger than t1 swap them around
    float temp = t0;
    t0 = t1;
    t1 = temp;
  }

  // if t1 is less than zero, the object is in the ray's negative direction
  // and consequently the ray misses the sphere
  if (t1 < 0)
    return false;

  // if t0 is less than zero, the intersection point is at t1
  if (t0 < 0) {
    t = t1;
    return true;
  }
  // else the intersection point is at t0
  else {
    t = t0;
    return true;
  }
}

bool tgCollission::IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl,
    const tgRay& ray, const tgModel& model)
{
  bool hit = false;
  pl.clear();
  zl.clear();

  for (unsigned int i = 0; i < model.m_faces.size(); i++) {
    vec3 p;
    vec3 n;
    double z;

    if (model.m_faces[i].v.size() < 3)
      throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");

    vec3 t1 = model.m_vertices[model.m_faces[i].v[0]].pos;
    vec3 t2 = model.m_vertices[model.m_faces[i].v[1]].pos;
    vec3 t3 = model.m_vertices[model.m_faces[i].v[2]].pos;

    if (IntersectRayTriangle(p, n, z, ray, t1, t2, t3)) {
      pl.push_back(p);
      nl.push_back(n);
      zl.push_back(z);
      hit = true;
    }

    if (model.m_faces[i].v.size() == 4) {
      t1 = model.m_vertices[model.m_faces[i].v[2]].pos;
      t2 = model.m_vertices[model.m_faces[i].v[3]].pos;
      t3 = model.m_vertices[model.m_faces[i].v[0]].pos;

      if (IntersectRayTriangle(p, n, z, ray, t1, t2, t3)) {
        pl.push_back(p);
        nl.push_back(n);
        zl.push_back(z);
        hit = true;
      }
    }

    if (model.m_faces[i].v.size() > 4)
      throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");

  }
  return hit;
}

bool tgCollission::IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl,
    const tgRay& ray, const tgRenderModel& model)
{
  bool hit = false;
  pl.clear();
  zl.clear();

  for (unsigned int i = 0; i < model.m_faces.size(); i++) {
    vec3 p;
    vec3 n;
    double z;

    if (model.m_faces[i].v.size() < 3)
      throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");

    vec3 t1 = model.m_pose * model.m_vertices[model.m_faces[i].v[0]].pos;
    vec3 t2 = model.m_pose * model.m_vertices[model.m_faces[i].v[1]].pos;
    vec3 t3 = model.m_pose * model.m_vertices[model.m_faces[i].v[2]].pos;

    if (IntersectRayTriangle(p, n, z, ray, t1, t2, t3)) {
      pl.push_back(p);
      nl.push_back(n);
      zl.push_back(z);
      hit = true;
    }

    if (model.m_faces[i].v.size() == 4) {
      t1 = model.m_pose * model.m_vertices[model.m_faces[i].v[2]].pos;
      t2 = model.m_pose * model.m_vertices[model.m_faces[i].v[3]].pos;
      t3 = model.m_pose * model.m_vertices[model.m_faces[i].v[0]].pos;

      if (IntersectRayTriangle(p, n, z, ray, t1, t2, t3)) {
        pl.push_back(p);
        nl.push_back(n);
        zl.push_back(z);
        hit = true;
      }
    }

    if (model.m_faces[i].v.size() > 4)
      throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");

  }
  return hit;
}

bool tgCollission::IntersectModels(const tgModel &m1, const tgModel &m2, const tgPose &p1, const tgPose& p2)
{
  vec3 t1, t2;
  mat3 R1, R2;

  p1.GetPose(R1, t1);
  p2.GetPose(R2, t2);

  vec3 center1 = t1 + R1 * m1.m_bs.center;
  vec3 center2 = t2 + R2 * m2.m_bs.center;

  float d = (center1 - center2).length();

  // 	printf("tgCollission::IntersectModels: %f %f\n", d, (m1.m_bs.radius+m2.m_bs.radius));

  if (d >= (m1.m_bs.radius + m2.m_bs.radius))
    return false;

  // further tests
  //	printf("[tgCollission::IntersectModels] Warning, function not implemented.\n");

  return true;
}

bool tgCollission::IntersectModels(const tgRenderModel &m1, const tgRenderModel &m2)
{
  vec3 t1, t2;
  mat3 R1, R2;

  m1.m_pose.GetPose(R1, t1);
  m2.m_pose.GetPose(R2, t2);

  vec3 center1 = t1 + R1 * m1.m_bs.center;
  vec3 center2 = t2 + R2 * m2.m_bs.center;

  float d = (center1 - center2).length();

  // 	printf("tgCollission::IntersectModels: %f %f\n", d, (m1.m_bs.radius+m2.m_bs.radius));

  if (d >= (m1.m_bs.radius + m2.m_bs.radius))
    return false;

  // further tests
  //	printf("[tgCollission::IntersectModels] Warning, function not implemented.\n");

  return true;
}

bool tgCollission::PointOnSameSide(const vec3& p1, const vec3& p2, const vec3& a, const vec3& b)
{
  vec3 cp1, cp2;
  cp1.cross(b - a, p1 - a);
  cp2.cross(b - a, p2 - a);
  if ((cp1 * cp2) > 0.0)
    return true;

  return false;
}

bool tgCollission::PointInTriangle(const vec3& p, const vec3& t1, const vec3& t2, const vec3& t3)
{
  return (PointOnSameSide(p, t1, t2, t3) && PointOnSameSide(p, t2, t3, t1) && PointOnSameSide(p, t3, t1, t2));
}

