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

#include "tgRenderModel.h"
#include "tgShapeCreator.h"
#include "tgError.h"
#include <GL/gl.h>

using namespace TomGine;

tgRenderModel::tgRenderModel()
{
  m_material.Random();
  m_bsmodel = 0;
  m_displaylist_initialized = false;
  m_bufferobject_initialized = false;
}

tgRenderModel::tgRenderModel(const tgModel& model) :
    tgModel(model)
{
  m_material.Random();
  m_bsmodel = 0;
  m_displaylist_initialized = false;
  m_bufferobject_initialized = false;
}

tgRenderModel::~tgRenderModel()
{
  if (m_bsmodel)
    delete (m_bsmodel);

  if (m_displaylist_initialized)
    glDeleteLists(m_displaylist, 1);

  if (m_bufferobject_initialized) {
    glDeleteBuffers(1, &m_vertexVBO);
    glDeleteBuffers(1, &m_triangleIBO);
    glDeleteBuffers(1, &m_quadIBO);
    m_bufferobject_initialized = false;
  }
}

void tgRenderModel::GenDisplayList()
{
  if (m_displaylist_initialized) {
    glDeleteLists(m_displaylist, 1);
    m_displaylist_initialized = false;
  }

  m_displaylist = glGenLists(1);

  glNewList(m_displaylist, GL_COMPILE_AND_EXECUTE);
  tgModel::DrawFaces();
  glEndList();

  m_displaylist_initialized = true;
}

void tgRenderModel::GenBufferObject()
{
  if (m_bufferobject_initialized) {
    glDeleteBuffers(1, &m_vertexVBO);
    glDeleteBuffers(1, &m_triangleIBO);
    glDeleteBuffers(1, &m_quadIBO);
    m_bufferobject_initialized = false;
  }

  // collect data
  m_triangleIDX.clear();
  m_quadIDX.clear();
  for (unsigned i = 0; i < m_faces.size(); i++) {
    if (m_faces[i].v.size() == 3) {
      m_triangleIDX.push_back(m_faces[i].v[0]);
      m_triangleIDX.push_back(m_faces[i].v[1]);
      m_triangleIDX.push_back(m_faces[i].v[2]);
    } else if (m_faces[i].v.size() == 4) {
      m_quadIDX.push_back(m_faces[i].v[0]);
      m_quadIDX.push_back(m_faces[i].v[1]);
      m_quadIDX.push_back(m_faces[i].v[2]);
      m_quadIDX.push_back(m_faces[i].v[3]);
    } else {
      printf("[tgRenderModel::GenBufferObject] Warning, no suitable face format\n");
      printf("[tgRenderModel::GenBufferObject] Face has %d vertices (supported: 3 or 4)\n", (int) m_faces[i].v.size());
      return;
    }
  }

  // generate VBOs
  glGenBuffers(1, &m_vertexVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(tgVertex) * m_vertices.size(), &m_vertices[0].pos.x, GL_STATIC_DRAW);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate vertex buffer");
#endif
  glGenBuffers(1, &m_triangleIBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * m_triangleIDX.size(), &m_triangleIDX[0], GL_STATIC_DRAW);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate triangle buffer");
#endif
  glGenBuffers(1, &m_quadIBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadIBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * m_quadIDX.size(), &m_quadIDX[0], GL_STATIC_DRAW);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate quad buffer");
#endif
  m_bufferobject_initialized = true;

}

#define BUFFER_OFFSET(i) ((char *)NULL + (i))
void tgRenderModel::DrawBufferObject()
{

  GLsizei vsize = sizeof(tgVertex);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glEnableClientState(GL_INDEX_ARRAY);

  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  glVertexPointer(3, GL_FLOAT, vsize, BUFFER_OFFSET(0));
  glNormalPointer(GL_FLOAT, vsize, BUFFER_OFFSET(12));
  glTexCoordPointer(2, GL_FLOAT, vsize, BUFFER_OFFSET(24));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIBO);
  glDrawElements(GL_TRIANGLES, m_triangleIDX.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadIBO);
  glDrawElements(GL_QUADS, m_quadIDX.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#ifdef DEBUG
  tgCheckError("[tgRenderModel::DrawBufferObject] drawing quads");
#endif
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisableClientState(GL_INDEX_ARRAY);
}

void tgRenderModel::Draw()
{
  this->DrawFaces();
  this->DrawLines();
  this->DrawPoints();
  this->DrawColorPoints();
}

void tgRenderModel::DrawFaces(bool lighting, RenderMode rmode)
{

  if (lighting) {
    m_material.Activate();
  } else {
    glDisable(GL_LIGHTING);
    glColor3f(m_material.color.x, m_material.color.y, m_material.color.z);
  }

  m_pose.Activate();
  switch (rmode) {
  case DISPLAYLIST:
    if (m_displaylist_initialized)
      glCallList(m_displaylist);
    else
      GenDisplayList();
    break;
  case BUFFEROBJECT:
    if (m_bufferobject_initialized)
      DrawBufferObject();
    else
      GenBufferObject();
    break;
  case RENDERNORMAL:
  default:
    tgModel::DrawFaces();
    break;

  }
  m_pose.Deactivate();

  if (lighting) {
    m_material.Deactivate();
  }
}

void tgRenderModel::DrawNormals(float normal_length)
{
  m_pose.Activate();
  tgModel::DrawNormals(normal_length);
  m_pose.Deactivate();
}

void tgRenderModel::DrawBoundingSphere()
{
  if (!m_bsmodel) {
    m_bsmodel = new tgModel();
    tgShapeCreator creator;
    creator.CreateSphere(*m_bsmodel, m_bs.radius, 2, tgShapeCreator::ICOSAHEDRON);
    if (!m_bsmodel)
      return;
  } else {

    m_pose.Activate();
    glPushMatrix();
    glTranslatef(m_bs.center.x, m_bs.center.y, m_bs.center.z);
    m_bsmodel->DrawFaces();
    glPopMatrix();
    m_pose.Deactivate();
  }
}
