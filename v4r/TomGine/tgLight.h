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
 * @file tgLight.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Light settings, representing a light. See glLight in the OpenGL spec. for more information.
 */

#ifndef TG_LIGHTING
#define TG_LIGHTING

#include "headers.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Light settings, representing a light. See glLight in the OpenGL spec. for more information. */
class tgLight
{
public:
	vec4 ambient;	///< ambient RGBA intensity of light.
	vec4 diffuse;	///< diffuse RGBA intensity of light.
	vec4 specular;	///< specular RGBA intensity of light.
	vec4 position;	///< position of the light in homogeneous coordinates.

	/** @brief Create white light. */
	tgLight();

	/** @brief Enables lighting and applies colors and position of the light.
	 *  @param id	OpenGL light id ranging from 0 to GL_MAX_LIGHTS-1 */
	void Activate(int id=0) const;

	/** @brief Creates light with a specific color intensity RGBA */
	void Color(float r, float g, float b, float a=1.0f);

	/** @brief Create light with random color intensity RGBA. */
	void Random();
};

} // namespace TomGine

#endif
