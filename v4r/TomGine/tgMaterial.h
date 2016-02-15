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
 * @file tgMaterial.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Specify material parameters for the OpenGL lighting model.
 */

#ifndef TG_MATERIAL_MODEL
#define TG_MATERIAL_MODEL

#include "tgMathlib.h"

namespace TomGine {

/** @brief Specify material parameters for the OpenGL lighting model. */
class tgMaterial
{
public:
  vec4 ambient; ///< ambient RGBA reflectance of the material
  vec4 diffuse; ///< diffuse RGBA reflectance of the material
  vec4 specular; ///< specular RGBA reflectance of the material
  vec4 color; ///< RGBA color (used directly if lighting disabled)
  vec4 emission;
  float shininess; ///< RGBA specular exponent of the material

  /** @brief Create grey material. */
  tgMaterial();

  /** @brief Enables lighting and applies material to OpenGL. */
  void Activate() const;
  /** @brief Disables blending. */
  void Deactivate() const;
  /** @brief Applies material to OpenGL. */
  void Apply() const;

  /** @brief Create material with a specific color reflectance RGBA */
  void Color(float r, float g, float b, float a = 1.0f, float amb = 0.7f, float diff = 1.0f, float spec = 0.2f,
      float emis = 0.0f, float shiny = 10.0f);

  void Color(float Ra, float Ga, float Ba, float Aa,
             float Rd, float Gd, float Bd, float Ad,
             float Rs, float Gs, float Bs, float As,
             float shiny);

  /** @brief Create material with random color reflectance RGBA. */
  void Random(float brightness = 1.0f);

  void Exposure(float val);
};

} // namespace TomGine

#endif
