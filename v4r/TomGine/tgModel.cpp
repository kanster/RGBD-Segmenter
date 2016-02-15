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

#include "tgModel.h"
#include "headers.h"
#include "tgShapeCreator.h"

#include <map>

using namespace TomGine;

//tgModel& tgModel::operator+=(const tgModel& m)
void
tgModel::Merge (const tgModel &m)
{
  unsigned vi = m_vertices.size ();
  for (unsigned i = 0; i < m.m_vertices.size (); i++)
    m_vertices.push_back (m.m_vertices[i]);

  for (unsigned i = 0; i < m.m_faces.size (); i++)
  {
    m_faces.push_back (m.m_faces[i]);
    TomGine::tgFace &f = m_faces[m_faces.size () - 1];
    for (unsigned j = 0; j < f.v.size (); j++)
      f.v[j] += vi;
  }

  for (unsigned i = 0; i < m.m_lines.size (); i++)
    m_lines.push_back (m.m_lines[i]);

  for (unsigned i = 0; i < m.m_points.size (); i++)
    m_points.push_back (m.m_points[i]);

  for (unsigned i = 0; i < m.m_colorpoints.size (); i++)
    m_colorpoints.push_back (m.m_colorpoints[i]);

}

void
tgModel::Draw ()
{
  DrawFaces ();
  DrawLines ();
  DrawPoints ();
  DrawColorPoints ();
}

void
tgModel::DrawVertices () const
{
  glBegin (GL_POINTS);
  for (unsigned v = 0; v < m_vertices.size (); v++)
  {
    glTexCoord2f (m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
    glNormal3f (m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
    glVertex3f (m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
  }
  glEnd ();
}

void
tgModel::DrawFaces () const
{
  int i, j;
  int v;

  glLineWidth(m_line_width);    // for wire-frame mode

  for (i = 0; i < (int)m_faces.size (); i++)
  {

    if (m_faces[i].v.size () == 3)
      glBegin (GL_TRIANGLES);
    else if (m_faces[i].v.size () == 4)
      glBegin (GL_QUADS);
    else
    {
      printf ("[tgModel::DrawFaces()] Warning, no suitable face format\n");
      printf ("[tgModel::DrawFaces()] Face has %d vertices (supported: 3 or 4)\n", (int)m_faces[i].v.size ());
      glEnd ();
      continue;
    }

    for (j = 0; j < (int)m_faces[i].v.size (); j++)
    {
      v = m_faces[i].v[j];
      glTexCoord2f (m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
      glNormal3f (m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
      glVertex3f (m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
    }
    glEnd ();
  }
}

void
tgModel::DrawLines () const
{
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_LIGHTING);
  glColor3f (m_line_color.x, m_line_color.y, m_line_color.z);
  glLineWidth (m_line_width);

  glBegin (GL_LINES);
  for (unsigned i = 0; i < m_lines.size (); i++)
  {
    glVertex3f (m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z);
    glVertex3f (m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
  }
  glEnd ();
  glColor3f (1.0f, 1.0f, 1.0f);
}

void
tgModel::DrawPoints () const
{
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_LIGHTING);
  glColor3f (m_point_color.x, m_point_color.y, m_point_color.z);
  glPointSize (m_point_size);

  glBegin (GL_POINTS);
  for (unsigned i = 0; i < m_points.size (); i++)
  {
    glVertex3f (m_points[i].x, m_points[i].y, m_points[i].z);
  }
  glEnd ();
  glColor3f (1.0f, 1.0f, 1.0f);
}

void
tgModel::DrawColorPoints () const
{
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_LIGHTING);
  glPointSize (m_point_size);

  glBegin (GL_POINTS);
  for (unsigned i = 0; i < m_colorpoints.size (); i++)
  {
    glColor3ub (m_colorpoints[i].color[0], m_colorpoints[i].color[1], m_colorpoints[i].color[2]);
    glVertex3f (m_colorpoints[i].pos.x, m_colorpoints[i].pos.y, m_colorpoints[i].pos.z);
  }
  glEnd ();
  glColor3f (1.0f, 1.0f, 1.0f);
}

void
tgModel::DrawNormals (float length) const
{ // draw normals
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_LIGHTING);
  glLineWidth (m_line_width);

  glBegin (GL_LINES);

  for (unsigned j = 0; j < m_vertices.size (); j++)
  {
    glVertex3f (m_vertices[j].pos.x, m_vertices[j].pos.y, m_vertices[j].pos.z);
    glVertex3f (m_vertices[j].pos.x + m_vertices[j].normal.x * length,
                m_vertices[j].pos.y + m_vertices[j].normal.y * length,
                m_vertices[j].pos.z + m_vertices[j].normal.z * length);
  }
  glEnd ();

  glColor3f (1.0f, 1.0f, 1.0f);
}

bool
tgModel::CheckFaces () const
{
  for (size_t i = 0; i < m_faces.size (); i++)
  {
    const TomGine::tgFace &f = m_faces[i];

    if (f.v.size () != 3 && f.v.size () != 4)
    {
      printf ("[tgModel::CheckFaces()] No suitable face format\n");
      printf ("[tgModel::CheckFaces()] Face has %d vertices (supported: 3 or 4)\n", (int)m_faces[i].v.size ());
      return false;
    }

    for (size_t j = 0; j < f.v.size (); j++)
    {
      if (f.v[j] < 0 || f.v[j] >= m_vertices.size ())
      {
        printf ("[tgModel::CheckFaces()] Vertex index out of bounds (0 <= %d < %lu)\n", f.v[j], m_vertices.size ());
        return false;
      }
    }
  }
  return true;
}

void
tgModel::RemoveUnusedVertices ()
{
  // get vertices in use
  std::vector<bool> v_used (m_vertices.size (), false);
  for (size_t i = 0; i < m_faces.size (); i++)
  {
    TomGine::tgFace &f = m_faces[i];
    for (size_t j = 0; j < f.v.size (); j++)
      v_used[f.v[j]] = true;
  }

  // calculate relation between old and new indices
  std::map<size_t, size_t> idxmap;
  size_t idx (0);
  for (size_t i = 0; i < v_used.size (); i++)
  {
    if (v_used[i] == true)
    {
      idxmap[i] = idx;
      idx++;
    }
  }

  // update indices of faces
  for (size_t i = 0; i < m_faces.size (); i++)
  {
    TomGine::tgFace &f = m_faces[i];
    for (size_t j = 0; j < f.v.size (); j++)
      f.v[j] = idxmap[f.v[j]];
  }

  // reallocate vertices
  std::vector<TomGine::tgVertex> vertices (idxmap.size ());
  std::map<size_t, size_t>::iterator it;
  for (it = idxmap.begin (); it != idxmap.end (); it++)
    vertices[it->second] = m_vertices[it->first];

  m_vertices = vertices;
}

void
tgModel::DublicateCommonVertices ()
{
  std::vector<bool> vertex_used (m_vertices.size (), false);

  for (size_t i = 0; i < m_faces.size (); i++)
  {
    TomGine::tgFace &f = m_faces[i];

    for (size_t j = 0; j < f.v.size (); j++)
    {
      unsigned vidx = f.v[j];

      if (vertex_used[vidx])
      {
        f.v[j] = m_vertices.size ();
        m_vertices.push_back (m_vertices[vidx]);
        vertex_used.push_back (true);
      }
      else
      {
        vertex_used[vidx] = true;
      }

    }
  }
}

// Compute normal vectors of vertices
void
tgModel::ComputeNormals ()
{
  ComputeFaceNormals ();

  std::map<size_t, std::vector<size_t> > vertex_faces;

  for (size_t i = 0; i < m_faces.size (); i++)
  {
    tgFace &f = m_faces[i];
    for (size_t j = 0; j < m_faces[i].v.size (); j++)
      vertex_faces[f.v[j]].push_back (i);
  }

  for (size_t i = 0; i < vertex_faces.size (); i++)
  {
    tgVertex &v = m_vertices[i];
    v.normal = vec3 (0.0, 0.0, 0.0);
    for (size_t j = 0; j < vertex_faces[i].size (); j++)
      v.normal += m_faces[vertex_faces[i][j]].normal;

    v.normal.normalize ();
  }

  //  unsigned i, j;
  //  tgFace* f;
  //  vec3 v0, v1, v2, e1, e2, n;
  //
  //  // calculate vertex normals using the face normal
  //  for (i = 0; i < m_faces.size(); i++) {
  //    f = &m_faces[i];
  //
  //    v0 = vec3(m_vertices[f->v[0]].pos.x, m_vertices[f->v[0]].pos.y, m_vertices[f->v[0]].pos.z);
  //    v1 = vec3(m_vertices[f->v[1]].pos.x, m_vertices[f->v[1]].pos.y, m_vertices[f->v[1]].pos.z);
  //    v2 = vec3(m_vertices[f->v[2]].pos.x, m_vertices[f->v[2]].pos.y, m_vertices[f->v[2]].pos.z);
  //    e1 = v1 - v0;
  //    e2 = v2 - v0;
  //
  //    n.cross(e1, e2);
  //    n.normalize();
  //    f->normal = vec3(n);
  //    for (j = 0; j < m_faces[i].v.size(); j++) {
  //      m_vertices[f->v[j]].normal.x = n.x;
  //      m_vertices[f->v[j]].normal.y = n.y;
  //      m_vertices[f->v[j]].normal.z = n.z;
  //    }
  //  }
}

void
tgModel::FlipNormals ()
{
  for (size_t i = 0; i < m_vertices.size (); i++)
    m_vertices[i].normal *= -1.0f;
}

void
tgModel::ComputeFaceNormals ()
{
  vec3 v0, v1, v2, e1, e2, n;

  // calculate vertex normals using the face normal
  for (size_t i = 0; i < m_faces.size (); i++)
  {
    tgFace &f = m_faces[i];

    v0 = vec3 (m_vertices[f.v[0]].pos.x, m_vertices[f.v[0]].pos.y, m_vertices[f.v[0]].pos.z);
    v1 = vec3 (m_vertices[f.v[1]].pos.x, m_vertices[f.v[1]].pos.y, m_vertices[f.v[1]].pos.z);
    v2 = vec3 (m_vertices[f.v[2]].pos.x, m_vertices[f.v[2]].pos.y, m_vertices[f.v[2]].pos.z);
    e1 = v1 - v0;
    e2 = v2 - v0;

    n.cross (e1, e2);
    n.normalize ();
    f.normal = vec3 (n);
  }
}

void
tgModel::FlipFaceNormals ()
{
  for (size_t i = 0; i < m_faces.size (); i++)
    m_faces[i].normal *= -1.0f;
}

void
tgModel::FlipFaces ()
{
  for (size_t i = 0; i < m_faces.size (); i++)
  {
    tgFace &f = m_faces[i];

    std::vector<unsigned> v;
    for (size_t j = 0; j < f.v.size (); j++)
      v.push_back (f.v[f.v.size () - 1 - j]);

    f.v = v;
  }
}

void
tgModel::ComputeBoundingSphere ()
{
  vec3 min;
  vec3 max;
  vec3 v;

  // 	const tgModel &m1, vec3 &center, float &radius

  if (m_vertices.empty ())
    return;

  min = m_vertices[0].pos;
  max = min;

  for (unsigned i = 1; i < m_vertices.size (); i++)
  {
    v = m_vertices[i].pos;
    if (v.x < min.x)
      min.x = v.x;
    if (v.y < min.y)
      min.y = v.y;
    if (v.z < min.z)
      min.z = v.z;

    if (v.x > max.x)
      max.x = v.x;
    if (v.y > max.y)
      max.y = v.y;
    if (v.z > max.z)
      max.z = v.z;
  }
  m_bs.center.x = (max.x + min.x) * 0.5f;
  m_bs.center.y = (max.y + min.y) * 0.5f;
  m_bs.center.z = (max.z + min.z) * 0.5f;

  float rad = 0.0f;
  for (unsigned i = 0; i < m_vertices.size (); i++)
  {
    v = m_vertices[i].pos - m_bs.center;
    float rv = v.x * v.x + v.y * v.y + v.z * v.z;
    if (rad < rv)
      rad = rv;
  }

  m_bs.radius = sqrt (rad);

}

void
tgModel::Clear ()
{
  m_vertices.clear ();
  m_faces.clear ();
  m_lines.clear ();
  m_points.clear ();
  m_colorpoints.clear ();
}

void
tgModel::Print () const
{

  for (unsigned i = 0; i < m_vertices.size (); i++)
  {
    printf ("Vertex %d: %f %f %f, %f %f %f\n", i, m_vertices[i].pos.x, m_vertices[i].pos.y, m_vertices[i].pos.z,
            m_vertices[i].normal.x, m_vertices[i].normal.y, m_vertices[i].normal.z);
  }

  for (unsigned i = 0; i < m_faces.size (); i++)
  {

    printf ("Face %d:", i);
    for (unsigned j = 0; j < m_faces[i].v.size (); j++)
    {
      printf (" %d", m_faces[i].v[j]);
    }
    printf ("\n");
  }
}

// void tgModel::ComputeQuadstripNormals(){
// 	int i,j,s;
// 	Face* f;
// 	vec3 v0, v1, v2, e1, e2, n, n1, n2;
// 	
// 	for(i=0; i<(int)m_quadstrips.size(); i++){
// 		f = &m_quadstrips[i];
// 		s = (int)f->v.size();
// 		for(j=0; j<(int)s; j++){
// 				
// 			v0 = vec3(m_vertices[f->v[(j+0)%s]].pos.x, m_vertices[f->v[(j+0)%s]].pos.y, m_vertices[f->v[(j+0)%s]].pos.z);
// 			v1 = vec3(m_vertices[f->v[(j+1)%s]].pos.x, m_vertices[f->v[(j+1)%s]].pos.y, m_vertices[f->v[(j+1)%s]].pos.z);
// 			v2 = vec3(m_vertices[f->v[(j+2)%s]].pos.x, m_vertices[f->v[(j+2)%s]].pos.y, m_vertices[f->v[(j+2)%s]].pos.z);
// 			e1 = v1 - v0;
// 			e2 = v2 - v0;
// 			n1.cross(e1,e2);
// 			n1.normalize();
// 			
// 			v0 = vec3(m_vertices[f->v[(j+0)%s]].pos.x, m_vertices[f->v[(j+0)%s]].pos.y, m_vertices[f->v[(j+0)%s]].pos.z);
// 			v1 = vec3(m_vertices[f->v[(j+s-1)%s]].pos.x, m_vertices[f->v[(j+s-1)%s]].pos.y, m_vertices[f->v[(j+s-1)%s]].pos.z);
// 			v2 = vec3(m_vertices[f->v[(j+s-2)%s]].pos.x, m_vertices[f->v[(j+s-2)%s]].pos.y, m_vertices[f->v[(j+s-2)%s]].pos.z);
// 			e1 = v1 - v0;
// 			e2 = v2 - v0;
// 			n2.cross(e2,e1);
// 			n2.normalize();
// 			
// 			n = (n1 + n2) * 0.5;
// 			
// 			if(j%2) n = n * -1.0;
// 			
// 			m_vertices[f->v[j]].normal.x = n.x;
// 			m_vertices[f->v[j]].normal.y = n.y;
// 			m_vertices[f->v[j]].normal.z = n.z;
// 		}
// 	}
// }


// void tgModel::DrawTriangleFan() const{
// 	int i,j,v;
// 	for(i=0; i<(int)m_trianglefans.size(); i++){
// 		glBegin(GL_TRIANGLE_FAN);		
// 			for(j=0; j<(int)m_trianglefans[i].v.size(); j++){
// 				v = m_trianglefans[i].v[j];
// 				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
// 				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
// 				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
// 			}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawQuadstrips() const{
// 	int i,j,v;
// 	for(i=0; i<(int)m_quadstrips.size(); i++){
// 		glBegin(GL_QUAD_STRIP);		
// 			for(j=0; j<(int)m_quadstrips[i].v.size(); j++){
// 				v = m_quadstrips[i].v[j];
// 				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
// 				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
// 				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
// 			}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawLines() const{
// 	for(int i=0; i<(int)m_lines.size(); i++){
// 		glBegin(GL_LINES);
// 			glVertex3f(m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z);
// 			glVertex3f(m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawLineLoops() const{
// 	int i,j;
// 	vec3 p;
// 	for(i=0; i<(int)m_lineloops.size(); i++){
// 		glBegin(GL_LINE_LOOP);
// 		for(j=0; j<(int)m_lineloops[i].points.size(); j++){
// 				p = m_lineloops[i].points[j];
// 				glVertex3f(p.x, p.y, p.z);
// 		}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawPoints() const{
// 	glDisable(GL_LIGHTING);
// 	for(int i=0; i<(int)m_points.size(); i++){
// 		glBegin(GL_POINTS);
// 			glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
// 		glEnd();
// 	}
// 	glEnable(GL_LIGHTING);
// }


