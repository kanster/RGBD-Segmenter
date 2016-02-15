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

#include "tgEngine.h"
#include "tgError.h"
#include <stdexcept>

using namespace TomGine;

tgEngine::tgEngine (unsigned width, unsigned height, float far, float near, const char* name, bool bfc, bool threaded,
                    bool stereo)
{
  m_window = new GLWindow (width, height, name, threaded, stereo);
  g_font->SetGLXFontBase (m_window->GetFontBase ());

  m_width = width;
  m_height = height;
  m_far = far;
  m_near = near;
  m_bfc = bfc;
  m_stereo = stereo;

  m_input_rotation_speed = 1.0f;
  m_input_translation_speed = 1.0f;
  m_input_zoom_speed = 1.0f;

  m_cor = vec3 (0.0, 0.0, 0.0);

  m_button_left = false;
  m_button_middle = false;
  m_button_right = false;

  m_wireframe = false;
  m_smoothshading = false;
  m_background_image_show = false;
  m_activate2D = false;

  m_mouse_pos[0] = 0;
  m_mouse_pos[1] = 0;

  float da = 0.25f * (m_far - m_near);
  // Setup 3D camera
  m_camera.Set (vec3 (da, da, da), // Position of camera
                vec3 (0, 0, 0), // Point where camera looks at
                vec3 (0, 1, 0), // UP-Vector of Camera
                45, m_width, m_height, // field of view in degree in y, image width, image height
                m_near, m_far, // near clipping plane, far clipping plane
                tgCamera::GL_PERSPECTIVE); // Perspective camera
  UpdateCameraViews (m_camera);

  // Setup 2D camera
  m_cam_ortho.Set (vec3 (0, 0, 1), vec3 (0, 0, 0), vec3 (0, 1, 0), 45, m_width, m_height, 0.1f, 2.0f,
                   tgCamera::GL_ORTHO);

  vec3 cam_f = m_camera.GetF ();
  m_light0.ambient = vec4 (1.0f, 1.0f, 1.0f, 1.0f);
  m_light0.diffuse = vec4 (1.0f, 1.0f, 1.0f, 1.0f);
  m_light0.specular = vec4 (1.0f, 1.0f, 1.0f, 1.0f);
  m_light0.position = vec4 (-cam_f.x, -cam_f.y, -cam_f.z, 0.0f);
  m_light0.Activate ();

  m_material.Random ();

  m_background_image = 0;
  // 	Welcome();

  // OpenGL settings
  glDepthFunc(GL_LEQUAL);
  glEnable (GL_DEPTH_TEST);
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel (GL_SMOOTH);
  glDisable (GL_TEXTURE_2D);
  glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, 1);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

tgEngine::~tgEngine ()
{
  if (m_window)
    delete (m_window);
  //  if (m_NurbsSurfaceShader)
  //    delete (m_NurbsSurfaceShader);
  if (m_background_image)
    delete (m_background_image);
}

void
tgEngine::Welcome ()
{
  printf ("\n   ***   TomGine Render Engine   ***\n\n");
  printf ("rotate: left mouse button\n");
  printf ("slide:  right mouse button\n");
  printf ("zoom:   mouse wheel\n");
  printf ("[f]:    toggle shading mode\n");
  printf ("[w]:    draw wireframed\n");
  printf ("[esc]:	quit\n");
  printf ("\n");
}

void
tgEngine::Update ()
{
  float fTime;
  Update (fTime);
}

void
tgEngine::Update (float &fTime)
{
  // swap to screen and clear render buffer
  Swap (fTime);

  Activate3D ();

  // Light pointing along camera viewing axis
  tgLight light;
  vec3 cam_f = m_camera.GetF ();
  light.ambient = vec4 (0.4f, 0.4f, 0.4f, 1.0f);
  light.diffuse = vec4 (1.0f, 1.0f, 1.0f, 1.0f);
  light.specular = vec4 (1.0f, 1.0f, 1.0f, 1.0f);
  light.position = vec4 (-cam_f.x, -cam_f.y, -cam_f.z, 0.0f);
  light.Activate ();
}

bool
tgEngine::ProcessEvents ()
{
  Event event;
  bool run = true;
  std::vector<Event> eventlist;
  while (m_window->GetEvent (event))
  {
    eventlist.push_back (event);
    run = run && InputControl (event);
  }
  return run;
}

bool
tgEngine::GetEventList (std::vector<Event> &eventlist)
{
  Event event;
  bool run = true;
  eventlist.clear ();
  while (m_window->GetEvent (event))
  {
    eventlist.push_back (event);
    run = run && InputControl (event);
  }
  return run;
}

void
tgEngine::WaitForEvent (Event &event)
{
  m_window->GetEventBlocking (event);
}

void
tgEngine::UnWaitForEvent ()
{
  m_window->UnBlockGetEvent ();
}

bool
tgEngine::InputControl (Event &event)
{
  int x_rel = 0;
  int y_rel = 0;

  switch (event.type)
  {

    // *********************************************************
    //			KeyCode:		/usr/include/X11/keysymdef.h
    case TMGL_Quit:
      return false;
      break;
    case TMGL_Press:
      // 			printf("event.key.keysym: %x\n", event.key.keysym);
      //    printf("[tgEngine::InputControl] Pressed: %d\n", event.input);
      switch (event.input)
      {
        case TMGL_Escape:
          return false;
          break;
        case TMGL_w:
          if (m_wireframe)
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
          else
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
          m_wireframe = !m_wireframe;
          break;
        case TMGL_KP_0:
        case TMGL_z:
          m_camera = m_cam[0];
          break;
        case TMGL_KP_7:
          m_camera = m_cam[1];
          break;
        case TMGL_KP_6:
          m_camera = m_cam[2];
          break;
        case TMGL_KP_4:
          m_camera = m_cam[3];
          break;
        case TMGL_KP_2:
          m_camera = m_cam[4];
          break;
        case TMGL_KP_8:
          m_camera = m_cam[5];
          break;

        case TMGL_Button1:
          m_button_left = true;
          break;
        case TMGL_Button2:
          m_button_middle = true;
          break;
        case TMGL_Button3:
          m_button_right = true;
          break;
        case TMGL_Button4:
          m_camera.TranslateF (0.01f * (m_far - m_near) * m_input_zoom_speed);
          break;
        case TMGL_Button5:
          m_camera.TranslateF (-0.01f * (m_far - m_near) * m_input_zoom_speed);
          break;
        default:
          break;
      }
      break;

      // *********************************************************
    case TMGL_Release:
      switch (event.input)
      {
        case TMGL_Button1:
          m_button_left = false;
          break;
        case TMGL_Button2:
          m_button_middle = false;
          break;
        case TMGL_Button3:
          m_button_right = false;
          break;
        default:
          break;
      }
      break;

      // *********************************************************
    case TMGL_Motion:
      x_rel = event.motion.x - m_mouse_pos[0];
      y_rel = event.motion.y - m_mouse_pos[1];

      if (m_button_left)
      {
        m_camera.Orbit (m_cor, m_camera.GetU (), -0.05f * x_rel * m_input_rotation_speed);
        m_camera.Orbit (m_cor, m_camera.GetS (), -0.05f * y_rel * m_input_rotation_speed);
      }
      else if (m_button_right)
      {
        m_camera.TranslateS (-0.0005f * (m_far - m_near) * x_rel * m_input_translation_speed);
        m_camera.TranslateU (0.0005f * (m_far - m_near) * y_rel * m_input_translation_speed);
      }
      else if (m_button_middle)
      {
        m_camera.TranslateF (0.001f * (m_far - m_near) * x_rel * m_input_zoom_speed);
        m_camera.TranslateF (0.001f * (m_far - m_near) * y_rel * m_input_zoom_speed);
      }

      m_mouse_pos[0] = event.motion.x;
      m_mouse_pos[1] = event.motion.y;

      break;

      // *********************************************************
    case TMGL_Expose:
      m_width = event.expose.width;
      m_height = event.expose.height;
      m_camera.SetViewport (m_width, m_height);
      m_cam[0].SetViewport (m_width, m_height);
      m_cam[1].SetViewport (m_width, m_height);
      m_cam[2].SetViewport (m_width, m_height);
      m_cam[3].SetViewport (m_width, m_height);
      m_cam[4].SetViewport (m_width, m_height);
      m_cam[5].SetViewport (m_width, m_height);
      m_cam_ortho.Set (vec3 (0, 0, 1), vec3 (0, 0, 0), vec3 (0, 1, 0), 45, m_width, m_height, 0.1f, 2.0f,
                       tgCamera::GL_ORTHO);
      break;

      // *********************************************************
      // 		case ClientMessage:
      // 			if(event.clientmessage.stop)
      // 				return false;
      // 			break;

      // *********************************************************
    default:
      break;

  } // switch(event.type)
  return true;
}

void
tgEngine::DrawCoordinates (float linelength, float linewidth)
{
  tgPose p;
  p.DrawCoordinates (linelength, linewidth);
}

void
tgEngine::DrawFPS ()
{
  bool activate2D = m_activate2D;
  Activate2D ();
  char charbuffer[16];
  sprintf (charbuffer, "%d", (int)(1.0 / this->m_frametime));
  g_font->Print (charbuffer, 20, 10, 10);
  if (!activate2D)
    Activate3D ();
}

void
tgEngine::SetCamera (tgCamera cam)
{
  m_width = cam.GetWidth ();
  m_height = cam.GetHeight ();
  m_far = cam.GetZFar ();
  m_near = cam.GetZNear ();
  m_camera = cam;
  UpdateCameraViews (cam);
}

void
tgEngine::UpdateCameraViews (tgCamera cam)
{
  m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = cam;
  m_cam[1].Orbit (m_cor, m_cam[1].GetU (), M_PI);
  m_cam[2].Orbit (m_cor, m_cam[2].GetU (), M_PI * 0.5);
  m_cam[3].Orbit (m_cor, m_cam[3].GetU (), -M_PI * 0.5);
  m_cam[4].Orbit (m_cor, m_cam[4].GetS (), M_PI * 0.5);
  m_cam[5].Orbit (m_cor, m_cam[5].GetS (), -M_PI * 0.5);
}

void
tgEngine::SetCamera (cv::Mat &intrinsic, unsigned &width, unsigned &height, cv::Mat &R, cv::Mat &T)
{
  TomGine::tgCamera::Parameter param;

  if (intrinsic.empty ())
  {
    printf ("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (empty).\n");
    return;
  }

  if (intrinsic.rows < 3 || intrinsic.cols < 3)
  {
    printf ("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (size).\n");
    return;
  }

  if (R.empty () || T.empty ())
  {
    printf ("[tgEngine::SetCameraPose] Warning, arguments 'R, T' not valid (empty).\n");
    return;
  }

  if (R.rows < 3 || R.cols < 3)
  {
    printf ("[tgEngine::SetCameraPose] Warning, argument 'R' not valid (size).\n");
    return;
  }

  if (T.rows < 3 && T.cols < 3)
  {
    printf ("[tgEngine::SetCameraPose] Warning, argument 'T' not valid (size).\n");
    return;
  }

  bool activate2D = m_activate2D;
  param.width = width;
  param.height = height;
  param.zFar = m_camera.GetZFar ();
  param.zNear = m_camera.GetZNear ();

  // Instrinsic parameters:
  // entries of the camera matrix
  cv::Mat intrinsic32 = intrinsic;
  if (intrinsic.type () != CV_32F)
    intrinsic.convertTo (intrinsic32, CV_32F);
  float *d = intrinsic32.ptr<float> (0);
  param.fx = d[0];
  param.fy = d[4];
  param.cx = d[2];
  param.cy = d[5];

  // radial distortion parameters
  param.k1 = 0.0;
  param.k2 = 0.0;
  param.k3 = 0.0;
  // tangential distortion parameters
  param.p1 = 0.0;
  param.p2 = 0.0;

  cv::Mat R32 = R;
  if (R.type () != CV_32F)
    R.convertTo (R32, CV_32F);
  param.rot = mat3 (R32.ptr<float> (0));

  cv::Mat T32 = T;
  if (T.type () != CV_32F)
    T.convertTo (T32, CV_32F);
  param.pos = vec3 (T32.ptr<float> (0));

  m_camera.Set (param);
  UpdateCameraViews (m_camera);
  if (activate2D)
    Activate2D ();
  else
    Activate3D ();
}

void
tgEngine::SetCenterOfRotation (float x, float y, float z)
{
  m_cor = vec3 (x, y, z);
}

void
tgEngine::ActivateLeft ()
{
  if (m_stereo)
    glDrawBuffer(GL_BACK_LEFT);
}

void
tgEngine::ActivateRight ()
{
  if (m_stereo)
    glDrawBuffer(GL_BACK_RIGHT);
}

void
tgEngine::Activate3D ()
{
  if (m_bfc)
    glEnable (GL_CULL_FACE);
  else
    glDisable (GL_CULL_FACE);
  m_camera.ApplyTransform ();
  m_camera.Activate ();
  glEnable (GL_LIGHTING);
  m_activate2D = false;
}

void
tgEngine::Activate2D ()
{
  glDisable (GL_CULL_FACE);
  m_cam_ortho.Activate ();
  glDisable (GL_LIGHTING);
  m_activate2D = true;
}

void
tgEngine::Swap ()
{
  float fTime;
  Swap (fTime);
}

void
tgEngine::Swap (float &fTime)
{
  // update frametime
  fTime = m_frametime = (float)m_timer.Update ();
  m_window->Update ();
  glClear (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void
tgEngine::LoadBackgroundImage (unsigned char* image_data, int width, int height, GLenum format, bool flip)
{

  if (!m_background_image)
    m_background_image = new tgTexture2D ();

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  m_background_image->Load (image_data, width, height, GL_RGB, format, GL_UNSIGNED_BYTE);
  m_background_image_show = true;
  m_background_image_flip = flip;
}

void
tgEngine::DrawBackgroundImage (float alpha)
{
  if (m_background_image)
  {
    float w = (float)m_width;
    float h = (float)m_height;
    bool activate2D = m_activate2D;
    Activate2D ();
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

    if (alpha < 1.0f)
      glEnable (GL_BLEND);

    glDepthMask (0);
    glColor4f (1.0f, 1.0f, 1.0f, alpha);
    m_background_image->Bind ();
    if (m_background_image_flip)
    {
      glBegin (GL_QUADS);
      glTexCoord2f (0.0f, 1.0f);
      glVertex3f (0., 0., 0.0f);
      glTexCoord2f (1.0f, 1.0f);
      glVertex3f (w, 0., 0.0f);
      glTexCoord2f (1.0f, 0.0f);
      glVertex3f (w, h, 0.0f);
      glTexCoord2f (0.0f, 0.0f);
      glVertex3f (0., h, 0.0f);
      glEnd ();
    }
    else
    {
      glBegin (GL_QUADS);
      glTexCoord2f (0.0f, 0.0f);
      glVertex3f (0., 0., 0.0f);
      glTexCoord2f (1.0f, 0.0f);
      glVertex3f (w, 0., 0.0f);
      glTexCoord2f (1.0f, 1.0f);
      glVertex3f (w, h, 0.0f);
      glTexCoord2f (0.0f, 1.0f);
      glVertex3f (0., h, 0.0f);
      glEnd ();
    }
    glDisable (GL_TEXTURE_2D);
    glDepthMask (1);
    glDisable (GL_BLEND);

    if (m_wireframe)
      glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

    if (!activate2D)
      Activate3D ();
  }
}

void
tgEngine::DrawForegroundImage (float alpha)
{
  if (m_background_image)
  {
    float w = (float)m_width;
    float h = (float)m_height;
    bool activate2D = m_activate2D;
    Activate2D ();
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

    if (alpha < 1.0f)
      glEnable (GL_BLEND);

    glDisable (GL_DEPTH_TEST);
    glColor4f (1.0f, 1.0f, 1.0f, alpha);
    m_background_image->Bind ();
    if (m_background_image_flip)
    {
      glBegin (GL_QUADS);
      glTexCoord2f (0.0f, 1.0f);
      glVertex3f (0., 0., 0.0f);
      glTexCoord2f (1.0f, 1.0f);
      glVertex3f (w, 0., 0.0f);
      glTexCoord2f (1.0f, 0.0f);
      glVertex3f (w, h, 0.0f);
      glTexCoord2f (0.0f, 0.0f);
      glVertex3f (0., h, 0.0f);
      glEnd ();
    }
    else
    {
      glBegin (GL_QUADS);
      glTexCoord2f (0.0f, 0.0f);
      glVertex3f (0., 0., 0.0f);
      glTexCoord2f (1.0f, 0.0f);
      glVertex3f (w, 0., 0.0f);
      glTexCoord2f (1.0f, 1.0f);
      glVertex3f (w, h, 0.0f);
      glTexCoord2f (0.0f, 1.0f);
      glVertex3f (0., h, 0.0f);
      glEnd ();
    }
    glDisable (GL_TEXTURE_2D);
    glEnable (GL_DEPTH_TEST);
    glDisable (GL_BLEND);

    if (m_wireframe)
      glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

    if (!activate2D)
      Activate3D ();
  }
}

void
tgEngine::UnloadBackgroundImage ()
{
  m_background_image_show = false;
  if (m_background_image)
    delete (m_background_image);
  m_background_image = 0;
}

void
tgEngine::PrintText (std::string text, float x, float y, float r, float g, float b)
{
  Activate2D ();
  g_font->Print (text.c_str (), 15, x, y, r, g, b, 0.0f);
}

void
tgEngine::PrintText3D (std::string text, vec3 pos, int size, float r, float g, float b, float a)
{
  vec2 vPos = m_camera.ToImageSpace (pos);
  bool activate2D = m_activate2D;
  Activate2D ();
  g_font->Print (text.c_str (), size, vPos.x, vPos.y, r, g, b, a);
  if (!activate2D)
    Activate3D ();
}

vec3
tgEngine::Get3DPointFrom2D (int x, int y)
{
  vec3 vResult;
  int viewport[4];
  double modelview[16];
  double projection[16];
  float z;
  int y_new;
  double result[3];
  bool activate2D = m_activate2D;

  Activate3D ();
  m_camera.SetZRange (0.0, 1.0);

  glGetDoublev (GL_MODELVIEW_MATRIX, &modelview[0]); //Aktuelle Modelview Matrix in einer Variable ablegen
  glGetDoublev (GL_PROJECTION_MATRIX, &projection[0]); //Aktuelle Projection[s] Matrix in einer Variable ablegen
  glGetIntegerv (GL_VIEWPORT, &viewport[0]); // Aktuellen Viewport in einer Variable ablegen
  y_new = viewport[3] - y; // In OpenGL steigt Y von unten (0) nach oben

  // Auslesen des Tiefenpuffers an der Position (X/Y_new)
  glReadPixels (x, y_new, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

  // Errechnen des Punktes, welcher mit den beiden Matrizen multipliziert (X/Y_new/Z) ergibt:
  gluUnProject ((double)x, (double)y_new, (double)z, modelview, projection, viewport, &result[0], &result[1],
                &result[2]);

  vResult.x = (float)result[0];
  vResult.y = (float)result[1];
  vResult.z = (float)result[2];

  if (activate2D)
    Activate2D ();

  return vResult;
}

