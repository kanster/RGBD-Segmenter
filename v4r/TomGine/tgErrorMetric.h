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

#ifndef ERROR_METRIC_H
#define ERROR_METRIC_H

#include <vector>

#include "tgModel.h"
#include "tgPose.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Measures the distance between two poses of an object.
 *
 *	Randomly distribute points on the surface of the model and measure the distance between corresponding points.
 *	*/
class tgErrorMetric
{
private:
	
	/** @brief Generate a random point within a triangle */
	vec3 GetRandPointInTriangle(const vec3& t1, const vec3& t2, const vec3& t3, unsigned trials=100) const;
	
public:
	std::vector<vec3> pointlist;	///< points randomly distributed on the surface of the model.
	
	/** @brief Initialise the error metric.
	 *  @param model	The geometry of the surface on which the points are distributed.
	 *  @param num_points	Number of points generated.	 */
	tgErrorMetric(TomGine::tgModel model, unsigned num_points=10000);
	
	/** @brief Get mean error between poses.
	 * 	@param p1,p2	Poses to compare.
	 * 	@return vec3	Error in each direction in object space. (absolute error = vec3::length()) */
	vec3 GetMean(const TomGine::tgPose &p1, const TomGine::tgPose &p2);
	
	/** @brief Copy pointlist. */
	void GetPoints(std::vector<vec3> &pl){ pl = pointlist; }
	
	/** @brief Get the number of points distributed on the object surface. */
	inline unsigned GetNumPoints(){ return pointlist.size(); }
	
};

} // namespace TomGine

#endif
