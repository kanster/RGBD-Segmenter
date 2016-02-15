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
 * @file tgQuaternion.h
 * @author Thomas Mörwald
 * @date September 2009
 * @version 0.1
 * @brief Quaternion representing rotations (avoiding singularity locks).
 */
 
#ifndef TG_QUATERNION
#define TG_QUATERNION

#include <math.h>

#include "tgMathlib.h"

namespace TomGine{



/** @brief Quaternion representing rotations (avoiding singularity locks). */
class tgQuaternion
{
public:
	float x,y,z,w;		///< x,y,z,w	Coefficients of the quaternion.

	/** @brief Creates quaternion with coefficients (0,0,0,1). */
	tgQuaternion();
	/** @brief Creates quaternion with coefficients specified. */
	tgQuaternion(float x, float y, float z, float w);
	
	/** @brief Normalises the coefficients of the quaternion. */
	void normalise();
	/** @brief Calculates the conjungate of the quaternion. */
	tgQuaternion getConjugate() const;

	/** @brief Compares two quaternions for equality wrt. rotation. */
	bool operator==(const tgQuaternion &q) const;
	/** @brief Add coefficients. */
	tgQuaternion operator+ (const tgQuaternion &q2) const;
	/** @brief Subtract coefficients. */
	tgQuaternion operator- (const tgQuaternion &q2) const;

	/** @brief Multiplying rq with q applies the rotation q to rq. */
	tgQuaternion operator* (const tgQuaternion &rq);
	/** @brief Multiply coefficients with scalar f. */
	tgQuaternion operator* (const float f);
	
	/** @brief Multiplying quaternion q with a vector v applies the rotation to v. */
	vec3 operator* (vec3 v);
	
	/** @brief Get coefficients of quaternion from axis-angle representation. */
	void fromAxis(const vec3 &v, float angle);
	/** @brief Get coefficients of quaternion from Euler angles. */
	void fromEuler(float pitch, float yaw, float roll);
	/** @brief Get coefficients of quaternion from 3x3 matrix. */
	void fromMatrix(mat3 m);
	/** @brief Get coefficients of quaternion from 4x4 matrix. */
	void fromMatrix(mat4 m);
	/** @brief Get 4x4 matrix from quaternion representation. */
	mat4 getMatrix4() const;
	/** @brief Get 3x3 matrix from quaternion representation. */
	mat3 getMatrix3() const;
	/** @brief Get axis-angle representation from quaternion. */
	void getAxisAngle(vec3& axis, float& angle) const;
	
	/** @brief Print the coefficients of the quaternion to console. */
	void print() const;

	void printAxisAngle() const;

};

} // namespace TomGine

#endif
