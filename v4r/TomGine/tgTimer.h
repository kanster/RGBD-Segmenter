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
 * @file tgTimer.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Real-time clock for MS Windows and Linux.
 */

#ifndef TG_TIMER
#define TG_TIMER

#ifdef _WIN32
	#include <windows.h>
#else
	#include <time.h>
	#include <sys/time.h>
#endif

// ***********************************************************************************

namespace TomGine{

/** @brief Timer handling frame- and application times. */
class tgTimer
{
private:
#ifdef WIN32
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	double fNow;
#else	
	struct timespec AppStart, act, old;
#endif
	double m_fAppTime;			// Time since application started
	double m_fTime;				// Time between two Update calls

public:
	/** @brief Create and start timer. */
	tgTimer(void);
	
	/** @brief Reset application time and start timer. */
	void	Reset();
	/** @brief Calculate time since last call to Update and update application time.
	 *  @return Time since last call of Update(). */
	double	Update();
	
	/** @brief Get Time since last call to Update. Does not update application time.
	 *  @return Time since last call of Update(). */
	double	GetFrameTime() const { return m_fTime;}

	/** @brief Get Time since last call of Reset() or constructor tgTimer().
	 *  @return Time since last call of Reset() or constructor tgTimer(). */
	double	GetApplicationTime() const { return m_fAppTime;}
};

} // namespace TomGine

#endif

