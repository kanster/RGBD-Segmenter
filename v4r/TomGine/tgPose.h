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
 *
 * @file tgPose.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining the position and orientation of an object in 3D.
 */
 
#ifndef TG_POSE
#define TG_POSE

#include "tgMathlib.h"
#include "tgQuaternion.h"

namespace TomGine{

/** @brief Represents a pose in 3D space with position and orientation.
 *  See Special Euclidean Space SE(3).
 *  Orientation is implemented using quaternions. */
class tgPose{
public:
	vec3 t;				///< Position of the pose. (or translation)
	tgQuaternion q;		///< Orientation of the pose. (or rotation)
	
	/** @brief	Compares two poses for equality. */
	bool operator==(const tgPose &p) const;
	/** @brief	Transform pose p into this coordinate frame. */
	tgPose operator*(const tgPose& p) const;
	/** @brief	Transform vector t into this coordinate frame. */
	vec3 operator*(const vec3& t) const;
	/** @brief	Add poses. */
	tgPose operator+(const tgPose &p) const;
	/** @brief	Subtract poses. */
	tgPose operator-(const tgPose &p) const;
	/** @brief	Calculate the transpose (inverse for similarity poses). */
	tgPose Transpose() const;
	
	/** @brief	Prints the components of the position and orientation to the console. */
	void Print() const;

	/** @brief	Pushes the pose as transformation matrix into the OpenGL matrix stack. */
	void Activate() const;	
	/** @brief	Pops (removes) the pose from the OpenGL matrix stack. */
	void Deactivate() const;
	/**	@brief Draws a simple coordinate frame at this pose */
	void DrawCoordinates(float linelength = 1.0f, float linewidth = 1.0f) const;
	
	/** @brief	Set the pose as rotation r and translation p. */
	void SetPose(mat3 r, vec3 p);	
	/** @brief	Gets the pose as rotation r and translation p. */
	void GetPose(mat3 &r, vec3 &p) const;
	/** @brief      Gets the rotation matrix of the pose. */
	mat3 GetRotation() const;
	/** @brief      Gets the pose as homogenious transformation. */
	mat4 GetPose() const;

	/** @brief	Rotate the pose using Euler angles.
	 *  @param	x,y,z	Rotation about x,y,z axis respectively. */
	void Rotate(float x, float y, float z);	
	/** @brief	Rotate pose through length(r) in radians about axis given by r. */
	void RotateAxis(vec3 r);
	/** @brief	Rotate the pose using Euler angles.
	 *  @param	r	Rotation about x,y,z axis respectively. */
	void RotateEuler(vec3 r);
	/** @brief	Translation in x,y,z direction. */
	void Translate(float x, float y, float z);	
	/** @brief	Translation along the vector t by the length of t. */
	void Translate(vec3 t);	
};

} // namespace TomGine

#endif
