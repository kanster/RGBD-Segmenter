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
 * @file tgCamera.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief OpenGL Camera for moving around in 3D space (including internal and external camera parameters).
 */
 
#ifndef TG_CAMERA
#define TG_CAMERA

#include "headers.h"
#include "tgFrustum.h"
#include "tgMathlib.h"
#include "tgPose.h"

namespace TomGine{

/** @brief Class tgCamera. Storage for intrinsic and extrinsic camera matrix.
 *  Providing functions for movement like translation and rotation. */
class tgCamera
{
private:
	// tgCamera definition
	vec3 pos;			///< Position of camera (absolute)
	vec3 view;			///< Viewpoint of camera (absolute)
	vec3 up;			///< The camera upside (relative)
	
	vec3 f;		///< Vector of camera pointing forward
	vec3 s;		///< Vector of camera pointing sidewards (right)
	vec3 u;		///< Vector of camera pointing up
	
	unsigned m_width, m_height;
	float m_fovy;
	float m_zNear, m_zFar;
	unsigned short m_projection;	
	mat4 m_extrinsic;	///< extrinsic camera matrix; corresponds to f,s,u.
	mat4 m_intrinsic;	///< intrinsix camera matrix (projective transformation from camera to image space)
	
	tgFrustum m_frustum;
	
	void SetPos(float x, float y, float z){ pos.x=x; pos.y=y; pos.z=z; }

	/** @brief convert pos, view, up to front, side, up description. */
	void pvu2fsu();
	void fsu2pvu();
	void fsu2extrinsic();
	void extrinsic2fsu();
	void fwh2intrinsic();

public:
	tgCamera();
	
	/** @brief Type of projection: orthographic or perspective. */
	enum Projection{
		GL_ORTHO = 0,
		GL_PERSPECTIVE = 1,
	};

	struct Parameter{
		Parameter();
		// image dimension
		unsigned width;
		unsigned height;
		// Instrinsic parameters:
		// entries of the camera matrix
		float fx;
		float fy;
		float cx;
		float cy;
		// radial distortion parameters
		float k1;
		float k2;
		float k3;
		// tangential distortion parameters
		float p1;
		float p2;
		// extrinsic parameters: 3D pose of camera w.r.t. world
		mat3 rot;
		vec3 pos;
		// Clipping planes of virtual camera
		float zNear;
		float zFar;
		
		/** @brief Print parameter to console. */
		void print(){
			printf("%d %d\n", width, height);
			printf("%f %f\n", fx, fy);
			printf("%f %f\n", cx, cy);
			printf("%f %f %f\n", k1, k2, k3);
			printf("%f %f\n", p1, p2);
			printf("\n %f %f %f\n", rot.mat[0], rot.mat[1], rot.mat[2]);
			printf(" %f %f %f\n", rot.mat[3], rot.mat[4], rot.mat[5]);
			printf(" %f %f %f\n", rot.mat[6], rot.mat[7], rot.mat[8]);
			printf("\n %f %f %f\n", pos.x, pos.y, pos.z);
			printf("%f %f\n", zNear, zFar);
		}
	};
	
	/** @brief Set camera matrices using intrinsic and extrinsic matrices (converts to OpenGL representation). */
	static tgCamera
	Set (mat3 intrinsic, mat3 R, vec3 T, unsigned width, unsigned height, float near, float far);

	/** @brief Set camera matrices using parameter description. */
	void Set(tgCamera::Parameter camPar);
	
	/** @brief Define camera matrices by coordinate system and projection parameter.
	 *  @param pos		position of camera
	 *  @param view		view vector of camera
	 *  @param up		vector defining the up direction of camera.
	 *  @param fovy		field-of-view in y axis,
	 *  @param width	width of camera image in pixel
	 *  @param height	height of camera image in pixel
	 *  @param proj		type of projection (orthographic or perspective)
	 */
	void Set(	vec3 pos, vec3 view, vec3 up,
				float fovy=45.0f, unsigned width=800, unsigned height=600,
				float zNear=0.1f, float zFar=100.0f,
				tgCamera::Projection proj=GL_PERSPECTIVE );
	/** @brief Set camera extrinsic matrix directly */
	void SetExtrinsic(float* M);
	/** @brief Set camera intrinsic matrix directly */
	void SetIntrinsic(float* M);
	/** @brief Set camera intrinsic matrix using projection parameter */
	void SetIntrinsic(float fovy, unsigned width, unsigned height, float zNear, float zFar, unsigned short projection);
	/** @brief Set viewport of camera (look up glViewport() of OpenGL spec.) */
	void SetViewport(unsigned w, unsigned h);
	/** @brief Set range of near and far clipping plane */
	void SetZRange(float near, float far);
	
	/** @brief Convert point in world space to image space */
	vec2 ToImageSpace(const vec3 &world_space) const;
	
	/** @brief Activate camera before drawing objects from its point of view. */
	void Activate();
	/** @brief Print camera matrices to console. */
	void Print() const;
	
	// Gets
	TomGine::tgPose GetPose() const;

	vec3 GetF() const {return f;}
	vec3 GetS() const {return s;}
	vec3 GetU() const {return u;}
	
	vec3 GetPos() const {return pos;}
	vec3 GetView() const {return view;}
	vec3 GetUp() const {return up;}
	
	float GetZNear() const { return m_zNear; }
	float GetZFar() const { return m_zFar; }
	unsigned GetWidth() const { return m_width; }
	unsigned GetHeight() const {return m_height; }
	
	float GetFOVY() const { return m_fovy; }
	unsigned short GetProjection() const { return m_projection; }
	mat4 GetIntrinsic() const { return m_intrinsic; }
	mat4 GetExtrinsic() const { return m_extrinsic; }
	
	tgFrustum* GetFrustum(){ return &m_frustum; }

	vec2 GetTexCoords (vec3 point) const;

	vec2 ProjectInto(vec3 point) const;

	void GetViewRay(int u, int v, vec3 &start, vec3 &dir) const;

	// Translations
	/** @brief Translate camera along a vector.  */
	void Translate(vec3 v);
	/** @brief Translate camera along a vector. */
	void Translate(float x, float y, float z, float fDist);
	/** @brief Translate camera in forward direction. */
	void TranslateF(float fDist);
	/** @brief Translate camera in side direction. */
	void TranslateS(float fDist);
	/** @brief Translate camera in up direction. */
	void TranslateU(float fDist);
	
	// Rotations
	/** @brief Rotate camera a given angel about an axis */
	void Rotate(float x, float y, float z, float fAngle);
	/** @brief Rotate camera about forward vector. */
	void RotateF(float fAngle);
	/** @brief Rotate camera about side vector. */
	void RotateS(float fAngle);
	/** @brief Rotate camera about up vector. */
	void RotateU(float fAngle);
	
	/** @brief Rotate camera about z vector (absolute). */
	void RotateY(float fAngle);
	
	/** @brief Rotate camera about an axis using a point as center of rotation. */
	void Orbit(vec3 vPoint, vec3 vAxis, float fAngle);
	
	/** @brief Make camera look at a specific point (without changing position)
	 *  @param	pov		Point to look at	 */
	void LookAt(const vec3 &pov);

	/** @brief Convert transformations to extrinsic camera matrix. */
	void ApplyTransform();
	
	/** @brief Draw view frustum of this camera. */
	void DrawFrustum(){ m_frustum.DrawFrustum(); }

};

} // namespace TomGine

#endif
