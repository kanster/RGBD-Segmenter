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

#include "GLWindow.h"
#include <stdio.h>
#include <stdexcept>
#include <vector>

namespace TomGine {

void GLWindow::init(unsigned int width, unsigned int height, const char* name, bool threaded,
    bool stereo)
{

  XInitThreads();

  dpy = XOpenDisplay(NULL);

  if (dpy == NULL) {
    throw std::runtime_error("[GLWindow::init] Error cannot connect to X server");
  }

  root = DefaultRootWindow(dpy);

  if (stereo) {
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, GLX_STEREO, None };
    vi = glXChooseVisual(dpy, 0, att);
  } else {
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
    vi = glXChooseVisual(dpy, 0, att);
  }

  if (vi == NULL)
    throw std::runtime_error("[GLWindow::init] Error no appropriate visual found");

  cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

  swa.colormap = cmap;
  swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask
      | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask;

  glWin = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
      CWColormap | CWEventMask, &swa);
  wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", true);
  XSetWMProtocols(dpy, glWin, &wmDelete, 1);

  XMapWindow(dpy, glWin);
  XStoreName(dpy, glWin, name);

  glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
  glXMakeCurrent(dpy, glWin, glc);

  printf("GLWindow '%s' %dx%d\nOpenGL Version: %s\n", name, width, height, glGetString(GL_VERSION));

  /* Load the font. */
#ifdef GLX_FONT
  font_info = XLoadQueryFont(dpy, GLX_FONT);
  font_base = glGenLists(256);
  if (!font_info) {
    fprintf(stderr, "XLoadQueryFont() failed - Exiting.\n");
    exit(-1);
  }
  else {
    /* Tell GLX which font & glyphs to use. */
    int first = font_info->min_char_or_byte2;
    int last = font_info->max_char_or_byte2;
    glXUseXFont(font_info->fid, first, last-first+1, font_base+first);
  }
#endif

  this->threaded = threaded;

  glXSwapBuffers(dpy, glWin);
}

void GLWindow::quit()
{
  glXMakeCurrent(dpy, None, NULL);
#ifdef GLX_FONT
  glDeleteLists(font_base, 256);
  XFreeFont(dpy, font_info);
#endif
  glXDestroyContext(dpy, glc);
  XFlush(dpy);
  XDestroyWindow(dpy, glWin);
  XFreeColormap(dpy, cmap);
  XCloseDisplay(dpy);
}

GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name, bool threaded,
    bool stereo)
{
  init(width, height, name, threaded, stereo);
}
GLWindow::~GLWindow()
{
  quit();
}

void GLWindow::Activate()
{
  glXMakeCurrent(dpy, glWin, glc);
}

void GLWindow::Update()
{
  glXSwapBuffers(dpy, glWin);
}

} /* namespace */
