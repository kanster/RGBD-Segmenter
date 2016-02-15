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
#include "tgMaterial.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif

using namespace TomGine;

tgMaterial::tgMaterial()
{
  ambient = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  diffuse = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  specular = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  color = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  shininess = 50.0f;
}

void tgMaterial::Activate() const
{
  if(color.w < 1.0-epsilon)
    glEnable(GL_BLEND);

  glEnable(GL_LIGHTING);

  Apply();
}

void tgMaterial::Apply() const
{
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
}

void tgMaterial::Deactivate() const
{
  glDisable(GL_BLEND);
}

void tgMaterial::Color(float r, float g, float b, float a, float amb, float diff, float spec, float emis, float shiny)
{
  color = vec4(r, g, b, a);
  ambient = vec4(r, g, b, a) * amb;
  diffuse = vec4(r, g, b, a) * diff;
  specular = vec4(1.0f, 1.0f, 1.0f, a) * spec;
  emission = vec4(r, g, b, a) * emis;
  shininess = shiny;// * float(rand())/RAND_MAX;
}

void tgMaterial::Color(float Ra, float Ga, float Ba, float Aa,
                       float Rd, float Gd, float Bd, float Ad,
                       float Rs, float Gs, float Bs, float As,
                       float shiny)
{
  color = vec4(Rd, Gd, Bd, Ad);
  ambient = vec4(Ra, Ga, Ba, Aa);
  diffuse = vec4(Rd, Gd, Bd, Ad);
  specular = vec4(Rs, Gs, Bs, As);
  emission = vec4(0.0, 0.0, 0.0, 0.0);
  shininess = shiny;
}

void tgMaterial::Random(float brightness)
{
  vec4 c;
  c.random();
  c *= brightness;
  Color(c.x, c.y, c.z);
}

void tgMaterial::Exposure(float val)
{
  color *= val;
  ambient *= val;
  diffuse *= val;
  specular *= val;
  emission *= val;
  shininess *= val;
}


