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

#include "tgFont.h"
#include "headers.h"
#include <stdexcept>
#include <stdio.h>

using namespace TomGine;

tgFont::tgFont()
{
#ifdef USE_FTGL_FONT
  m_font = new FTPixmapFont(TTF_FONT); // FTPixmapFont(TTF_FONT) -> better, but buggy
  if(m_font->Error())
  {
    char err[64];
    sprintf(err, "[tgFont::tgFont()] Error cannot create font '%s'", TTF_FONT);
    throw std::runtime_error(err);
  }
#endif
}

tgFont::~tgFont()
{
#ifdef USE_FTGL_FONT
  delete(m_font);
#endif
}

void tgFont::Set(tgFont::Type type)
{
#ifdef USE_FTGL_FONT
  delete(m_font);
  switch(type)
  {
    case BITMAP_FONT:
    m_font = new FTBitmapFont(TTF_FONT);
    break;

    case PIXMAP_FONT:
    m_font = new FTPixmapFont(TTF_FONT);
    break;

    default:
    m_font = new FTBitmapFont(TTF_FONT);
    break;
  }
  if(m_font->Error())
  {
    char err[64];
    sprintf(err, "[tgFont::Set()] Error cannot create font '%s'", TTF_FONT);
    throw std::runtime_error(err);
  }
#endif
}

void tgFont::Print(const char* text, int size, int x, int y, float r, float g, float b, float a) const
{
#ifdef USE_FTGL_FONT
  glColor4f(r,g,b,a);
  GLfloat raster_pos[4];
  glGetFloatv(GL_CURRENT_RASTER_POSITION, raster_pos);
  //  printf("[tgFont::Print] %f %f %f %f\n", raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]);
  //  glRasterPos4f(raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]);  // must be called for color to take effect
  glRasterPos4f(0.0f, 0.0f, 0.0f, 1.0f); // must be called for color to take effect
  m_font->FaceSize(size);
  m_font->Render(text, -1, FTPoint(x,y));
#endif
#ifdef GLX_FONT
  if (!glIsList(m_font_base))
  throw std::runtime_error("[tgFont::Print]: Bad display list. - Exiting.\n");

  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  glColor4f(r, g, b, a);
  glRasterPos2i(x, y);

  glPushAttrib(GL_LIST_BIT);
  glListBase(m_font_base);
  glCallLists(strlen(text), GL_UNSIGNED_BYTE, (GLubyte*)text);
  glPopAttrib();
#endif
}

void tgFont::Print(const tgLabel2D &label) const
{
#ifdef USE_FTGL_FONT
  glColor4f(label.rgba.x,label.rgba.y,label.rgba.z,label.rgba.w);
  GLfloat raster_pos[4];
  glGetFloatv(GL_CURRENT_RASTER_POSITION, raster_pos);
  glRasterPos4f(raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]); // must be called for color to take effect
  m_font->FaceSize(label.size);
  m_font->Render(label.text.c_str(), -1, FTPoint(label.x,label.y));
#endif
#ifdef GLX_FONT
  Print(label.text.c_str(), 20, label.x, label.y, label.rgba.x, label.rgba.y, label.rgba.z, label.rgba.w);
#endif
}
