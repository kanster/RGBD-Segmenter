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

#include "tgFrameBufferObject.h"
#include "tgError.h"
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>

using namespace TomGine;

tgFrameBufferObject::tgFrameBufferObject(unsigned w, unsigned h, GLint colorInternal, GLint depthInternal){

	m_width = w;
	m_height = h;

	texColor.Bind();
	texColor.Load(NULL, m_width, m_height, colorInternal, GL_RGBA, GL_UNSIGNED_BYTE);
	tgCheckError("[main] fbo_tex");

	texDepth.Bind();
	texDepth.Load(NULL, m_width, m_height, depthInternal, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);
	tgCheckError("[main] fbo_depth_tex");

	glGenFramebuffers(1,&m_fbo_id);
	Bind();

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColor.GetTextureID(), 0);
	tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach color texture");

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth.GetTextureID(), 0);
	tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach depth texture");

	Unbind();

	if(	tgCheckFBError(GL_FRAMEBUFFER, "[tgFrameBufferObject::tgFrameBufferObject]")!=GL_FRAMEBUFFER_COMPLETE ||
		tgCheckError("[tgFrameBufferObject::tgFrameBufferObject]")!=GL_NO_ERROR)
	{
		std::string errmsg = std::string("[tgFrameBufferObject::tgFrameBufferObject] Error generating frame buffer objects");
		throw std::runtime_error(errmsg.c_str());
	}
	glDisable(GL_TEXTURE_2D);
}
tgFrameBufferObject::~tgFrameBufferObject(){
	if(glIsFramebuffer(m_fbo_id))
		glDeleteFramebuffers(1, &m_fbo_id);
}

void tgFrameBufferObject::Clear(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void tgFrameBufferObject::Bind(){
	glBindFramebuffer(GL_FRAMEBUFFER, m_fbo_id);
	tgCheckError("[tgFrameBufferObject::Bind]");
}

void tgFrameBufferObject::Unbind(){
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void tgFrameBufferObject::SaveColor(const char* filename){
	texColor.Bind();
	cv::Mat img( m_height, m_width, CV_8UC3 );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
	glDisable(GL_TEXTURE_2D);
	tgCheckError("[tgFrameBufferObject::SaveColor]");
	cv::imwrite(filename, img);
}

void tgFrameBufferObject::SaveDepth(const char* filename){
	texDepth.Bind();
	cv::Mat img( m_height, m_width, CV_8U );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, img.data);
	glDisable(GL_TEXTURE_2D);
	tgCheckError("[tgFrameBufferObject::SaveDepth]");
	cv::imwrite(filename, img);
}
