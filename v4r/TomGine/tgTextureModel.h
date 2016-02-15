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
 * @file tgTextureModel.h
 * @author Thomas Mörwald
 * @date February 2012
 * @version 0.1
 * @brief Defining a textured model for rendering.
 */

#ifndef TG_TEXTURE_MODEL
#define TG_TEXTURE_MODEL

#include "tgRenderModel.h"
#include "tgTexture.h"
#include <opencv2/core/core.hpp>

namespace TomGine {


class tgTextureModel: public tgRenderModel
{
public: //todo PRIVATE !!!
  bool sync;
  std::vector<tgTexture2D*> m_tex_gl;
  void syncTextures();
  void clearGLTextures();

public:
  std::vector<cv::Mat3b> m_tex_cv;          ///< Textures of the model as cv::Mat3b
  std::vector<unsigned> m_face_tex_id;      ///< index of tgTexture2D in m_textures for each face in tgModel::m_face

  /** @brief constructors and copy-constructors */
  tgTextureModel();
  tgTextureModel(const tgTextureModel &m);
  tgTextureModel(const tgRenderModel &m);
  tgTextureModel(const tgModel &m);
  ~tgTextureModel();

  tgTextureModel& operator=(const tgTextureModel &m);

  /** @brief Calls DrawTexFaces if a texture is available, otherwise use tgRenderModel::DrawFaces */
  virtual void Draw();

  /** @brief Synchronizes textures with OpenGL if necessary and draws textured faces. */
  virtual void DrawFaces() const;

  /** @brief Allows to force synchronization of textures with OpenGL */
  inline void Sync(){ sync = false; }

  /** @brief Crop textures to minimum area necessary
   *  @brief Adjust tex coordinates according to new tex size */
  void OptimizeTextures();

};

}

#endif
