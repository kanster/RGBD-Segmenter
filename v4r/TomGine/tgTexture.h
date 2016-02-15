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
 * @file tgTexture.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Class for managing textures (load from file, bind, copy, ...).
 */
 
#ifndef TG_TEXTURE
#define TG_TEXTURE

#include "headers.h"

namespace TomGine{

/** @brief 1D Texture (GPU memory). */
class tgTexture1D
{
private:
	GLuint m_texture_id;
	int m_width;
	GLenum m_intFormat;

	// copying of a texture should be done with tgImageProcessor (deep copy)
	tgTexture1D(const tgTexture1D &t);
	tgTexture1D& operator=(const tgTexture1D &t);

public:
	/** @brief	Create OpenGL texture and set default parameter. */
	tgTexture1D();
	/** @brief Destroy OpenGL texture. */
	~tgTexture1D();

	/** @brief Load data into texture. (See glTexImage1D for more information)
	 *  @param data		Pointer to data with size according to width and format.
	 *  @param width	Number of elements in data.
	 *  @param internal	Defines how the data is stored on the GPU.
	 *  @param format	Specifies the format of an element (i.e. number of channels).
	 *  @param type		Specifies the data type of a channel of an element.	 */
	bool Load(const void* data, int width, GLenum internal, GLenum format, GLenum type);

	/** @brief Bind/Activate this texture to a certain texture stage. */
	void Bind(int stage=0) const;

	/** @brief	Get the OpenGL texture id. */
	inline GLuint GetTextureID(){ return m_texture_id; }
	/** @brief	Get the texture width / number of elements. */
	inline int GetWidth() const{ return m_width; }
	/** @brief	Get the texture data format. */
	inline GLenum GetFormat() const { return m_intFormat; }
};

/** @brief 2D Texture (GPU memory). */
class tgTexture2D
{
private:
	GLuint m_texture_id;
	int m_width;
	int m_height;
	GLenum m_intFormat;

	// copying of a texture should be done with tgImageProcessor (deep copy)
	tgTexture2D(const tgTexture2D &t);
	tgTexture2D& operator=(const tgTexture2D &t);

public:
	/** @brief	Create OpenGL texture and set default parameter. */
	tgTexture2D();
	/** @brief Destroy OpenGL texture. */
	~tgTexture2D();

	/** @brief Load data into texture. (See glTexImage2D for more information)
	 *  @param data		Pointer to data with size according to width and format.
	 *  @param w,h		Size of data in 2D.
	 *  @param internal	Defines how the data is stored on the GPU.
	 *  @param format	Specifies the format of an element in data (i.e. number of channels).
	 *  @param type		Specifies the data type of a channel of an element.	 */
	bool Load(const void* data, int w, int h, GLenum internal=GL_RGB, GLenum format=GL_RGB, GLenum type=GL_UNSIGNED_BYTE);

	/** @brief Load texture from an image file (See cv::imread in OpenCV spec.).
	 *  @param	filename	Path to image file. */
	bool Load(const char* filename);

	/** @brief Save texture to an image file (See cv::imwrite in OpenCV spec.).
	 *  @param	filename	Path to image file. */
	bool Save(const char* filename);

	/** @brief	Get data in texture as unsigned char */
	bool GetImageData(unsigned char* image_data);
	
	/** @brief Bind/Activate this texture to a certain texture stage. */
	void Bind(int stage=0) const;

	/** @brief	Copy frame buffer from window coordinates (0,0) to (width,height) into texture. */
	void CopyTexImage2D(int width, int height, GLenum internal=GL_RGB);

	/** @brief	Copy frame buffer from window coordinates (x,y) to (width,height) into texture. */
	void CopyTexImage2D(int x, int y, int width, int height, GLenum internal=GL_RGB);
	
	void CopyFromTexture(const tgTexture2D& tex, int x, int y, unsigned w, unsigned h);

	/** @brief	Get the OpenGL texture id. */
	inline GLuint GetTextureID(){ return m_texture_id; }
	/** @brief	Get the texture width. */
	inline int GetWidth() const{ return m_width; }
	/** @brief	Get the texture height. */
	inline int GetHeight() const{ return m_height; }
	/** @brief	Get the texture data format. */
	inline GLenum GetFormat() const { return m_intFormat; }

};

/** @brief 3D Texture (GPU memory). */
class tgTexture3D
{
private:
	GLuint m_texture_id;
	int m_width, m_height, m_depth;
	GLenum m_intFormat;

	// copying of a texture should be done with tgImageProcessor (deep copy)
	tgTexture3D(const tgTexture3D &t);
	tgTexture3D& operator=(const tgTexture3D &t);

public:
	/** @brief	Create OpenGL texture and set default parameter. */
	tgTexture3D();
	/** @brief Destroy OpenGL texture. */
	~tgTexture3D();

	/** @brief Load data into texture. (See glTexImage3D for more information)
	 *  @param data		Pointer to data with size according to width and format.
	 *  @param w,h,d	Size of data in 3D.
	 *  @param internal	Defines how the data is stored on the GPU.
	 *  @param format	Specifies the format of an element in data (i.e. number of channels).
	 *  @param type		Specifies the data type of a channel of an element.	 */
	bool Load(const void* data, int w, int h, int d, GLenum internal, GLenum format, GLenum type);

	/** @brief Bind/Activate this texture to a certain texture stage. */
	void Bind(int stage=0) const;

	/** @brief	Get the OpenGL texture id. */
	inline GLuint GetTextureID(){ return m_texture_id; }
	/** @brief	Get the texture width. */
	inline int GetWidth() const{ return m_width; }
	/** @brief	Get the texture height. */
	inline int GetHeight() const{ return m_height; }
	/** @brief	Get the texture depth. */
	inline int GetDepth() const{ return m_depth; }
	/** @brief	Get the texture data format. */
	inline GLenum GetFormat() const { return m_intFormat; }
};

} // namespace TomGine

#endif
