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
 * @file tgFrustum.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief View frustum of a camera.
 */

#ifndef TG_FRUSTUM
#define TG_FRUSTUM

#include "headers.h"

#include "tgMathlib.h"

namespace TomGine{

/** @brief View frustum, defined by 6 planes. (near, far, left, right, bottom, top) */
class tgFrustum{
    
public:
	float frustum[6][4];	///< The plane parameters
	mat4 m_intrinsic;		///< intrinsic camera matrix
	mat4 m_extrinsic;		///< extrinsic camera matrix
	vec3 m_color;

	void ExtractFrustum (const mat4 &i, const mat4 &e);

	/** @brief Extracts the view frustum from the currently set projection- and modelview-matrix. */
	void ExtractFrustum();

	friend class tgCamera;

public:

	/** @brief Check whether a point lies in the frustum.
	 *  @return true	Point lies in the frustum.
	 *  @return false	Point lies outside the frustum.
	 *  @param x,y,z	The point to check. */
	bool PointInFrustum( float x, float y, float z );

	/** @brief Check whether a shpere lies completely in the frustum.
	 *  @return true	Sphere lies completely in the frustum.
	 *  @return false	Point lies completely outside the frustum.
	 *  @param x,y,z	The center of the sphere.
	 *  @param radius	The readius of the sphere. */
	bool SphereInFrustum( float x, float y, float z, float radius );

	/** @brief Draws the view frustum using GL_LINES. */
	void DrawFrustum();

};

} // namespace TomGine

#endif
