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
 * @file GLWindow.h
 * @author Thomas Moerwald (Vienna University of Technology)
 * @date June 2010
 * @version 0.1
 * @brief Device Context for handling OpenGL windows in MS GLWindows.
 */

#ifndef _GL_WINDOW_
#define _GL_WINDOW_

#include "GLEvent.h"
#include "GLInput.h"

#ifdef WIN32
#include <windows.h>
#include <gl/glew.h>
#include <gl/gl.h>

#else

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <semaphore.h>

// Extend list of Buttons in X.h
#define Button6     6
#define Button7     7
#define Button8     8
#define Button9     9

#endif

/** @brief BLORT namespace for GLWindow */
namespace TomGine {

#ifdef WIN32
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif
class Event;

/** @brief Class GLWindow */
class GLWindow
{

public:
  /** @brief Construction of an OpenGL Window (Rendering Context)
   *   @param width Window and OpenGL viewport width in pixel
   *   @param height Window and OpenGL viewport height in pixel
   *   @param name Caption of the window in the titel bar */
  GLWindow(unsigned width = 320, unsigned height = 240, const char* name = "GLWindow",
      bool threaded = false, bool stereo = false);
  ~GLWindow();

  /** @brief Activate window for usage (set focus) */
  void Activate();

  /** @brief Update OpenGL GLWindow (Swap Buffers) */
  void Update();

  /** @brief Query input event 
   *   @return true if there are events in the event que
   *		@param Event defined in TMGLEvent.h */
  bool GetEvent(Event &event);
  void GetEventBlocking(Event &event);
  void UnBlockGetEvent();

#ifdef WIN32
  HWND gethWnd() const {return hWnd;}
#else
  inline GLuint GetFontBase()
  {
    return font_base;
  }
#endif

private:
#ifdef WIN32
  WNDCLASS wc;
  HWND hWnd;
  HDC hDC;
  HGLRC hRC;
  MSG msg;
#else
  Display *dpy;
  Window root;
  XVisualInfo *vi;
  XFontStruct *font_info;
  GLuint font_base;
  Colormap cmap;
  XSetWindowAttributes swa;
  Window glWin;
  Window btWin;
  Atom wmDelete;
  GLXContext glc;
  XWindowAttributes gwa;
  bool threaded;
#endif

  void init(unsigned int width, unsigned int height, const char* name, bool threaded = false,
      bool stereo = false);
  void quit();

};

} /* namespace */

#endif /* _GL_WINDOW_ */
