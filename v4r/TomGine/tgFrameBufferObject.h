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
 * @file tgFrameBufferObject
 * @author Thomas Mörwald
 * @date Jun 2011
 * @version 0.1
 * @brief Class for managing a frame buffer object
 */

#ifndef TG_FRAMEBUFFEROBJECT
#define TG_FRAMEBUFFEROBJECT

#include "tgTexture.h"

namespace TomGine{

/** @brief Class for managing an OpenGL frame buffer object.
 *
 *  For the usage of frame buffer objects please refer to http://www.opengl.org/wiki/Framebuffer_Object */
class tgFrameBufferObject
{
private:
	unsigned m_width, m_height;	///< width and height of the frame buffer object

	GLuint m_fbo_id;		///< ID of the OpenGL frame buffer object.

public:
	tgTexture2D texColor;	///< GPU texture storage for the color buffer.
	tgTexture2D texDepth;	///< GPU texture storage for the depth buffer.

	/** @brief Create the frame buffer object on the GPU using OpenGL.
	 * 	@param w,h		The size (width, height) of the frame buffer object in pixel.
	 * 	@param colorInternal	The internal format for the color texture used as color buffer (texColor).
	 * 	@param depthInternal	The internal format for the depth texture used as depth buffer (texDepth). */
	tgFrameBufferObject(unsigned w, unsigned h, GLint colorInternal=GL_RGBA, GLint depthInternal=GL_DEPTH_COMPONENT);

	/** @brief Destroy the frame buffer object on the GPU. */
	~tgFrameBufferObject();

	/** @brief Clears the color and depth buffer with the value defined by glClearColor/glClearDepth. */
	void Clear();
	/** @brief Activates the frame buffer object. OpenGL renders to the texture storage and not to the screen. */
	void Bind();
	/** @brief Deactivates the frame buffer object. OpenGL renders to the screen and not to the texture storage. */
	void Unbind();

	/** @brief Save color buffer (texColor) to a file. */
	void SaveColor(const char* filename);
	/** @brief Save depth buffer (texDepth) to a file. */
	void SaveDepth(const char* filename);

};

}

#endif
