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
#include "tgTimer.h"

using namespace TomGine;

// ***********************************************************************************

tgTimer::tgTimer(void) {
	Reset();
}

void tgTimer::Reset() {
#ifdef WIN32
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
#else
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
#endif
	m_fAppTime = 0.0;
}

double tgTimer::Update() {
#ifdef WIN32
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_EndTicks));
	fNow = (double)(m_EndTicks - m_StartTicks) / m_Frequency;
	m_fTime = fNow - m_fAppTime;
	m_fAppTime = fNow;
#else
	clock_gettime(CLOCK_REALTIME, &act);
	m_fTime = (act.tv_sec - old.tv_sec) + (act.tv_nsec - old.tv_nsec) / 1e9;
	old = act;		
	m_fAppTime += m_fTime;
#endif
	return m_fTime;
}
