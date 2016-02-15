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
#include "tgTexture.h"
#include "tgError.h"
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>

using namespace TomGine;

// ############################################################################################
// tgTexture1D

tgTexture1D::tgTexture1D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::tgTexture1D()]");
#endif
}

tgTexture1D::~tgTexture1D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture1D::Load(const void* data, int width, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glTexImage1D(GL_TEXTURE_1D, 0, internal, m_width, 0, format, type, data);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::Load()]");
#endif
	return true;
}

void tgTexture1D::Bind(int stage) const{
	glEnable(GL_TEXTURE_1D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::Bind()]");
#endif
}


// ############################################################################################
// tgTexture2D

tgTexture2D::tgTexture2D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture2D::tgTexture2D()]");
#endif
}

tgTexture2D::~tgTexture2D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture2D::Load(const void* image_data, int width, int height, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_height = height;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexImage2D(GL_TEXTURE_2D, 0, internal, (int)m_width, (int)m_height, 0, format, type, image_data);
#ifdef DEBUG
	tgCheckError("[tgTexture2D::Load(void*, int, int, GLenum, GLenum, GLenum)]");
#endif
	return true;
}

bool tgTexture2D::Load(const char* filename){
//	cv::Mat img = cv::imread(filename);
//	if(img.data==NULL){
//		std::string errmsg = std::string("[tgTexture2D::Load(const char*)] Error loading file '") + filename + "'.";
//		throw std::runtime_error(errmsg.c_str());
//	}
//	return Load((unsigned char*)img.data, img.cols, img.rows, 3, GL_BGR, GL_UNSIGNED_BYTE);

	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
	return Load((unsigned char*)img->imageData, img->width, img->height, GL_RGBA, GL_BGR, GL_UNSIGNED_BYTE);
}

bool tgTexture2D::Save(const char* filename){
//	Bind();
//	cv::Mat img( m_height, m_width, CV_8UC3 );
//	glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
//
//	if(cv::imwrite(filename, img))
//		return true;
//	else
//		return false;
	Bind();
	IplImage* img = cvCreateImage ( cvSize ( m_width, m_height ), IPL_DEPTH_8U, 3 );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, img->imageData);
	cvConvertImage(img, img, CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);
	cvSaveImage(filename, img);
	cvReleaseImage(&img);
	return true;
}

bool tgTexture2D::GetImageData(unsigned char* image_data){
	Bind();
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
	return true;
}

void tgTexture2D::Bind(int stage) const{
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
}

void tgTexture2D::CopyTexImage2D(int width, int height, GLenum internal){
	m_width = width;
	m_height = height;
	m_intFormat = internal;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, m_intFormat, 0, 0, m_width, m_height, 0);
}

void tgTexture2D::CopyTexImage2D(int x, int y, int w, int h, GLenum internal){
	m_width = w;
	m_height = h;
	m_intFormat = internal;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, m_intFormat, x, y, m_width, m_height, 0);
}

// ############################################################################################
// tgTexture3D

tgTexture3D::tgTexture3D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::tgTexture3D()]");
#endif
}

tgTexture3D::~tgTexture3D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture3D::Load(const void* data, int width, int height, int depth, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_height = height;
	m_depth = depth;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glTexImage3D(GL_TEXTURE_3D, 0, internal, m_width, m_height, m_depth, 0, format, type, data);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::Load()]");
#endif
	return true;
}

void tgTexture3D::Bind(int stage) const{
	glEnable(GL_TEXTURE_3D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::Bind()]");
#endif
}

