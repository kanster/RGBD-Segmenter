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
 * @file tgModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace TomGine
 */
 
#ifndef TG_MODELLOADER
#define TG_MODELLOADER

#include <stdlib.h>
#include <stddef.h>
#include <string>

#include "ply.h"
#include "PlyStructure.h"
#include "tgModel.h"
#include "tgRenderModel.h"


namespace TomGine{

/** @brief Loading geometrical models from file. */
class tgModelLoader
{
private:

	static bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	
	/** @brief Loads PLY files (Polygon File Format, Stanford Triangle Format)
	*		@param model	Model to load file to.
	*		@param filename	Path and filename of file to load.
	*		@return True on success, false on failure. */
	static bool LoadPly(tgModel &model, const char* filename);
	
	/** @brief Saves PLY files (Polygon File Format, Stanford Triangle Format)
	*	@param model Model to save.
	*	@param filename Path and filename of file to save to.
	*	@return True on success, false on failure. */
	static bool SavePly(const tgModel &model, const std::string &filename);
	   
};

} // namespace TomGine

#endif
