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

#ifdef WIN32

#include "GLWindow.h"
#include <stdexcept>

namespace V4R{

void GLWindow::init(unsigned int width, unsigned int height, const char* name){

	// register window class
	LPCSTR lpName = reinterpret_cast<LPCSTR>(name);
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = GetModuleHandle(0);
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH)GetStockObject( BLACK_BRUSH );
	wc.lpszMenuName = NULL;
	wc.lpszClassName = lpName;
  
	if (!RegisterClass(&wc)) 
		throw std::exception("[GLWindow::init] RegisterClass failed");
		
	const int w = width + 2 * GetSystemMetrics(SM_CXFIXEDFRAME); //SM_CXSIZEFRAME
	const int h = height + 2 * GetSystemMetrics(SM_CYFIXEDFRAME) + GetSystemMetrics(SM_CYCAPTION);

  // create main window
  hWnd = CreateWindow(lpName, lpName, 
					  WS_CAPTION | WS_POPUPWINDOW | WS_VISIBLE,
					  0, 0, (w), (h),
					  NULL, NULL, wc.hInstance, NULL );

  // enable OpenGL for the window
  PIXELFORMATDESCRIPTOR pfd;
  int format;

  // get the device context (DC)
  hDC = GetDC( hWnd );

  // set the pixel format for the DC
  ZeroMemory( &pfd, sizeof( pfd ) );
  pfd.nSize = sizeof( pfd );
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
  pfd.iPixelType = PFD_TYPE_RGBA;
  pfd.cColorBits = 24;
  pfd.cDepthBits = 16;
  pfd.iLayerType = PFD_MAIN_PLANE;
  format = ChoosePixelFormat( hDC, &pfd );
  SetPixelFormat( hDC, format, &pfd );

  // create and enable the render context (RC)
  hRC = wglCreateContext( hDC );
  wglMakeCurrent( hDC, hRC );

  GLenum err = glewInit();
  if (GLEW_OK != err){
	  // Problem: glewInit failed, something is seriously wrong.
	  fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(err));
	    throw std::exception("[GLWindow::init] GLEW Error");
	  return;
  }

  SwapBuffers( hDC );
}


void GLWindow::quit(){
	wglMakeCurrent( NULL, NULL );
	
	wglDeleteContext( hRC );
	ReleaseDC( hWnd, hDC );

	// destroy the window explicitly
	DestroyWindow( hWnd );
	
	if(!UnregisterClass(wc.lpszClassName, wc.hInstance))
		throw std::exception("[GLWindow::quit] UnregisterClass failed");
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  switch (message)
  {
    case WM_CREATE:
      return 0;
    case WM_CLOSE:
      PostQuitMessage( 0 );
      return 0;
    case WM_DESTROY:
      return 0;
    case WM_KEYDOWN:
      switch ( wParam )
      {    
	case VK_ESCAPE:
	  PostQuitMessage(0);
	  return 0;
      }
      return 0;
    default:
      return DefWindowProc( hWnd, message, wParam, lParam );
  }
}

GLWindow::GLWindow(){
  init(320,240,"OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height){
  init(width, height, "OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name){
  init(width, height, name);
}
GLWindow::~GLWindow(){
  quit();
}

void GLWindow::Activate(){
  wglMakeCurrent( hDC, hRC );
}

void GLWindow::Update(){
  SwapBuffers( hDC );
}

} /* namespace */

#endif /* WIN32 */
