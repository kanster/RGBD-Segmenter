/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file Kinect.cpp
 * @author Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the OpenNI Driver for the Kinect sensor
 */

#include "Kinect.h"
#include <XnCodecIDs.h>

#define Z_STEP_THRESHOLD 0.01

namespace Kinect {


/**
 * @brief Constructor of Class Kinect
 */
Kinect::Kinect(const char *kinect_xml_file)
{
  ni_pause = false;

  if( !Init(kinect_xml_file) ) {
    printf("Kinect: Error: Initialisation not successful!\n");
    exit(0);
  }
}

/**
 * @brief Constructor of Class Kinect
 */
Kinect::Kinect()
{
  ni_pause = false;

  if( !Init(KINECT_CONFIG) ) {
    printf("Kinect: Error: Initialisation not successful!\n");
    exit(0);
  }
}

/**
 * @brief Destructor of Class Kinect
 */
Kinect::~Kinect()
{
  StopCapture();
}

/**
 * @brief Initialisation of the device.
 * @param kinect_xml_path
 * @return Return true if initialization was successful.
 */
bool Kinect::Init(const char *kinect_xml_file)
{
  XnStatus rc = XN_STATUS_OK;

  // Initialisation from xml file
  EnumerationErrors errors;
  rc = kinect::openDeviceFromXml(kinect_xml_file, errors);
  if( rc != XN_STATUS_OK ) {
    printf("Kinect::Init: Error: Initialisation from xml-file failed. Check filename and connection to Kinect: %s\n", kinect_xml_file);
    return false;
  }

  if(kinect::isImageOn())
  {
    // Input format should be 6 ???
    rc = kinect::getImageGenerator()->SetIntProperty("InputFormat", 6);
    if( rc != XN_STATUS_OK )
      printf("Kinect::Init: Error: Changing input format failed.\n");

    // set pixel format to grayscale (bayer image)
    rc = kinect::getImageGenerator()->SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
    if( rc != XN_STATUS_OK )
      printf("Kinect::Init: Error: Changing pixel format to grayscale failed.\n");

    // Input format should be 2 for software registration
    if( kinect::getDepthGenerator()->SetIntProperty("RegistrationType", 2) != XN_STATUS_OK )
      if( rc != XN_STATUS_OK )
        printf("Kinect::Init: Error: Changing registration type failed.\n");
    
    // Change registration  
    rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().ResetViewPoint();
    if( rc != XN_STATUS_OK )
      printf("Kinect::Init: Error: Switching off depth stream registration failed.\n");

      rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().SetViewPoint(*kinect::getImageGenerator());
      if( rc != XN_STATUS_OK )
        printf("Kinect::Init: Error: Switching on depth stream registration failed.\n");
  }
  else if(kinect::isIROn())
  { 
    // Input format should be 2 for software registration
    if( kinect::getDepthGenerator()->SetIntProperty("RegistrationType", 2) != XN_STATUS_OK )
      if( rc != XN_STATUS_OK )
        printf("Kinect::Init: Error: Changing registration type failed.\n");
    
    // Change registration  
    rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().ResetViewPoint();
    if( rc != XN_STATUS_OK )
      printf("Kinect::Init: Error: Switching off depth stream registration failed.\n");

      rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().SetViewPoint(*kinect::getIRGenerator());
      if( rc != XN_STATUS_OK )
        printf("Kinect::Init: Error: Switching on depth stream registration failed.\n");
  }

  rc = kinect::getDepthGenerator()->GetIntProperty("ShadowValue", shadow_value);
  if( rc != XN_STATUS_OK )
    printf("Kinect::Init: Error: Could not read shadow value.\n");

  rc = kinect::getDepthGenerator()->GetIntProperty("ShadowValue", no_sample_value);
  if( rc != XN_STATUS_OK )
    printf("Kinect::Init: Error: Could not read \"no sample\" value.\n");

  rgbWidth = 640; /// TODO Get width and height from file!
  rgbHeight = 480;
  depWidth = 640;
  depHeight = 480;

  centerX = (depWidth >> 1) - 0.5f;
  centerY = (depHeight >> 1) - 0.5f;

  // PCL: This magic value is taken from a calibration routine, unless calibrated params are not supported we rely on thsi value!
  const float rgb_focal_length_SXGA_ = 1050;
  //   depth_focal_length_SXGA_ = (float)depth_focal_length_SXGA / pixel_size;
  depth_focal_length_SXGA_ = rgb_focal_length_SXGA_;

  float output_x_resolution = rgbWidth;
  depthScale = output_x_resolution / (float) XN_SXGA_X_RES; // XN_SXGA_X_RES = 1280;
  depthFocalLength = depth_focal_length_SXGA_ * depthScale;
  constant = 0.001 / depthFocalLength; // 0.001 (mm => m)

  return (rc == XN_STATUS_OK);
}

/**
 * @brief Start capturing from kinect sensor.
 * @param delay Delay in miliseconds.
 */
void Kinect::StartCapture(int delay)
{
  if( ni_pause )
    printf("Kinect::StartCapture: Warning: Cannot record when paused!\n");
  else
    kinect::captureStart(delay);
}

/**
 * @brief Stop capturing from kinect sensor.
 */
void Kinect::StopCapture()
{
  if( ni_pause )
    printf("Kinect::StopCapture: Warning: Cannot stop when paused!\n");
  else
    kinect::captureStop(0);
}

/**
 * @brief Convert the meta data map to an openCV ipl-image.
 * @param pImageMD Meta data map
 * @param iplImg Destination ipl-image
 */
void Kinect::MapMetaData2IplImage(const MapMetaData* pImageMD, IplImage **iplImg)
{
  if( *iplImg != 0 )
    if( (*iplImg)->width != (int) pImageMD->FullXRes() || (*iplImg)->height != (int) pImageMD->FullYRes() )
      cvReleaseImage(iplImg);
  if( *iplImg == 0 )
    *iplImg = cvCreateImage(cvSize(pImageMD->FullXRes(), pImageMD->FullYRes()), IPL_DEPTH_8U, pImageMD->BytesPerPixel());
  assert(*iplImg != 0);

  for( unsigned co = 0; co < pImageMD->DataSize(); co++ )
    (*iplImg)->imageData[co] = pImageMD->Data()[co];
}

/**
 * @brief Convert the meta data depth map to an openCV ipl-image and normalize it to 16 bit.
 * @param pDepthMD Meta data depth map
 * @param iplImg Destination ipl-image
 */
void Kinect::DepthMetaData2IplImage(const DepthMetaData* pDepthMD, IplImage **iplImg)
{
  printf("Kinect::DepthMetaData2IplImage: Not yet implemented!\n");
}

/**
 * @brief Get color image as openCV iplImage from the Kinect sensor
 * @param iplImg Video image as ipl image.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetColorImage(IplImage **iplImg)
{
  NextFrame();
  (*iplImg) = cvCreateImage(cvSize(rgbWidth, rgbHeight), IPL_DEPTH_8U, 3);
  IplImage tmp = rgbImage;
  cvCopy(&tmp, (*iplImg));
  return true;
}

/**
 * @brief Get the next frame from the Kinect sensor and copy to openCV matrices.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::NextFrame()
{
  if( !kinect::isCapturing() ) {
    printf("Kinect::NextFrame: Warning: Kinect is not capturing data.\n");
    return false;
  }

  kinect::readFrame(); // read next frame

  // get depth image
  const DepthMetaData* pDepthMD = kinect::getDepthMetaData();
  if( kinect::isDepthOn() ) {
    depImage = cv::Mat(depHeight, depWidth, CV_16S);
    short *d = depImage.ptr<short> (0);
    for( int co = 0; co < depHeight * depWidth; co++ )
      d[co] = pDepthMD->Data()[co];
  } else
    printf("Kinect::NextFrame: Warning: No depth data available!\n");

  // get image data (color or ir image data)
  const MapMetaData* pImageMD = NULL;
  if( kinect::isImageOn() ) {
    pImageMD = kinect::getImageMetaData();
    grayImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC1);
    uchar *d = grayImage.ptr<uchar> ();
    for( int co = 0; co < rgbHeight * rgbWidth; co++ )
      d[co] = pImageMD->Data()[co];
    rgbImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC3);
    cv::cvtColor(grayImage, rgbImage, CV_BayerGB2BGR/*CV_BayerGB2RGB*/, 3);
    return true;
  } 
  else if( kinect::isIROn() ) {
    pImageMD = kinect::getIRMetaData();
    grayImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC1);
    uchar *d = grayImage.ptr<uchar> ();
    for( int co = 0; co < rgbHeight * rgbWidth; co++ )
      d[co] = pImageMD->Data()[co];
    rgbImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC3);
    cv::cvtColor(grayImage, rgbImage, CV_GRAY2RGB, 3);
    return true;
  } else {
    printf("Kinect::NextFrame: Warning: No image data available!\n");
    return false;
  }
  return false;
}

/**
 * @brief Get undistorted and registered images from the kinect sensor.
 * @param rgbImg Video image as openCV matrix.
 * @param depImg Depth image as openCV matrix with 16 bit depth!
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetImages(cv::Mat &rgbImg, cv::Mat &depImg)
{
  rgbImg = rgbImage;
  depImg = depImage;
  return true;
}

cv::Vec4f Kinect::DepthToWorld_Vec4f(const int &x, const int &y, const int &depthValue)
{
  cv::Vec4f result;

  // convert depth
  result[0] = (x - centerX) * depthValue * constant;
  result[1] = (y - centerY) * depthValue * constant;
  result[2] = depthValue * 0.001;

  // convert color
  RGBValue col;
  uchar *ptr = rgbImage.data;
  col.r = ptr[(y * rgbWidth + x) * 3 + 2]; // change red and blue channel
  col.g = ptr[(y * rgbWidth + x) * 3 + 1];
  col.b = ptr[(y * rgbWidth + x) * 3];
  result[3] = col.float_value;

  return result;
}

/**
 * @brief Transform depth point to world coordinates.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param depthValue Raw depth value from the sensor.
 * @return Returns the 3d point in the world coordinate system.
 */
cv::Point3f Kinect::DepthToWorld(int x, int y, int depthValue)
{
  cv::Point3f result;
  result.x = (x - centerX) * depthValue * constant;
  result.y = (y - centerY) * depthValue * constant;
  result.z = depthValue * 0.001;
  return result;
}

/**
 * @brief Get the color for world points.
 * @param x x-coordinate in depth image
 * @param y y-coordinate in depth image
 * @return Returns the color as 3d point.
 */
cv::Point3f Kinect::WorldToColor(unsigned x, unsigned y)
{
  uchar *ptr = rgbImage.data;
  cv::Point3f col;
  col.x = ptr[(y * rgbWidth + x) * 3 + 2]; // change red and blue channel
  col.y = ptr[(y * rgbWidth + x) * 3 + 1];
  col.z = ptr[(y * rgbWidth + x) * 3];
  return col;
}

/**
 * @brief Get the color of world points.
 * @param x x-coordinate in depth image
 * @param y y-coordinate in depth image
 * @return Returns the color as float value.
 */
float Kinect::PointToColorFloat(unsigned x, unsigned y)
{
  RGBValue col;
  uchar *ptr = rgbImage.data;
  col.r = ptr[(y * rgbWidth + x) * 3 + 2]; // change red and blue channel
  col.g = ptr[(y * rgbWidth + x) * 3 + 1];
  col.b = ptr[(y * rgbWidth + x) * 3];
  return col.float_value;
}

/**
 * @brief Get a world point from the kinect image.
 * @param x x-coordinate of the point on the image plane.
 * @param y y-coordinate of the point on the image plane.
 * @return Returns the 3d world point.
 */
cv::Point3f Kinect::Get3dWorldPoint(unsigned x, unsigned y)
{
  short depth = depImage.at<short> (y, x);
  return DepthToWorld(x, y, (int) depth);
}

/**
 * @brief Get a world point from the kinect image.
 * @param x x-coordinate of the point on the image plane.
 * @param y y-coordinate of the point on the image plane.
 * @return Returns the 3d world point with the color.
 */
cv::Vec4f Kinect::Get3dWorldPointWithColor(unsigned x, unsigned y)
{
  short depth = depImage.at<short> (y, x);
  cv::Vec4f colPt = DepthToWorld_Vec4f(x, y, (int)depth);
  return colPt;
}

/**
 * @brief Get a world point cloud from the kinect image.
 * @param cloud 3D point cloud.
 * @param colCloud Cloud with the color values for each point in the cloud.
 * @param scale Get scaled (reduced) point cloud (Everey 2,3,4 ... point of the full cloud)
 */
void Kinect::Get3dWorldPointCloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud, int scale)
{
  cloud = cv::Mat_<cv::Point3f>(rgbHeight / scale, rgbWidth / scale);
  colCloud = cv::Mat_<cv::Point3f>(rgbHeight / scale, rgbWidth / scale);
  int rgb2depthRatio = rgbWidth / depWidth; /// TODO Bei 1280 Auflösung gibt es eine Verzerrung in z-Richtung?
  for( int row = 0; row < depHeight; row += scale ) {
    for( int col = 0; col < depWidth; col += scale ) {
      short depth = depImage.at<short> (row, col);
      if( (int) depth != (int) shadow_value && (int) depth != (int) no_sample_value ) {
        cloud.at<cv::Point3f> (row / scale, col / scale) = DepthToWorld(col, row, (int) depth);
        colCloud.at<cv::Point3f> (row / scale, col / scale) = WorldToColor(col * rgb2depthRatio, row * rgb2depthRatio);
      } else {
        /* Initialize points if we have no valid data (to transmit via ice-interface) */
        cloud.at<cv::Point3f> (row / scale, col / scale) = cv::Point3f(FLT_MAX, FLT_MAX, FLT_MAX);
        colCloud.at<cv::Point3f> (row / scale, col / scale) = WorldToColor(col * rgb2depthRatio, row * rgb2depthRatio);
      }
    }
  }
}

/**
 * @brief Get a world point cloud from the kinect image.
 * ATTENTION: We also do not get NANs, we get 0.-points, if no valid data is available!
 * @param cloud 3D point cloud with color.
 * @param scale Get scaled (reduced) point cloud
 */
void Kinect::Get3dWorldPointCloud(cv::Mat_<cv::Vec4f> &cloud, int scale)
{
  cloud = cv::Mat_<cv::Vec4f>(rgbHeight / scale, rgbWidth / scale);
  for( int row = 0; row < depHeight; row += scale )
    for( int col = 0; col < depWidth; col += scale )
      cloud.at<cv::Vec4f> (row / scale, col / scale) = Get3dWorldPointWithColor(col, row);
}

/**
 * @brief Get a smooth point cloud from the kinect image, averaged from the last three point clouds.
 * Delivers zeros, for unknown points.
 * @param cloud 3D point cloud with color.
 * @param scale Get scaled (reduced) point cloud (Everey 2,3,4 ... point of the full cloud)
 */
void Kinect::Get3dWorldPointCloudSmooth(cv::Mat_<cv::Vec4f> &cloud, int scale)
{
  cv::Mat_<cv::Vec4f> a_cloud_0, a_cloud_1, a_cloud_2;
  cloud = cv::Mat_<cv::Vec4f>(rgbHeight / scale, rgbWidth / scale);
  a_cloud_0 = cv::Mat_<cv::Vec4f>(rgbHeight / scale, rgbWidth / scale);
  a_cloud_1 = cv::Mat_<cv::Vec4f>(rgbHeight / scale, rgbWidth / scale);
  a_cloud_2 = cv::Mat_<cv::Vec4f>(rgbHeight / scale, rgbWidth / scale);

  NextFrame();
  Get3dWorldPointCloud(a_cloud_0);
  NextFrame();
  Get3dWorldPointCloud(a_cloud_1);
  NextFrame();
  Get3dWorldPointCloud(a_cloud_2);

  for( int row = 0; row < depHeight; row += scale )
    for( int col = 0; col < depWidth; col += scale ) {
      int r = row / scale;
      int c = col / scale;
      bool valid[3] = { false, false, false };
      int nr_valid = 0;
      if( a_cloud_0.at<cv::Vec4f> (r, c)[0] != 0. ) {
        valid[0] = true;
        nr_valid++;
      }
      if( a_cloud_1.at<cv::Vec4f> (r, c)[0] != 0. ) {
        valid[1] = true;
        nr_valid++;
      }
      if( a_cloud_2.at<cv::Vec4f> (r, c)[0] != 0. ) {
        valid[2] = true;
        nr_valid++;
      }

      cv::Vec4f &pt = cloud.at<cv::Vec4f> (r, c);
      pt = a_cloud_0.at<cv::Vec4f> (r, c);

      if( nr_valid == 0 )
        continue;
      else if( nr_valid == 1 ) {
        if( valid[0] ) {
          pt[0] = a_cloud_0.at<cv::Vec4f> (r, c)[0];
          pt[1] = a_cloud_0.at<cv::Vec4f> (r, c)[1];
          pt[2] = a_cloud_0.at<cv::Vec4f> (r, c)[2];
        }
        if( valid[1] ) {
          pt[0] = a_cloud_1.at<cv::Vec4f> (r, c)[0];
          pt[1] = a_cloud_1.at<cv::Vec4f> (r, c)[1];
          pt[2] = a_cloud_1.at<cv::Vec4f> (r, c)[2];
        }
        if( valid[2] ) {
          pt[0] = a_cloud_2.at<cv::Vec4f> (r, c)[0];
          pt[1] = a_cloud_2.at<cv::Vec4f> (r, c)[1];
          pt[2] = a_cloud_2.at<cv::Vec4f> (r, c)[2];
        }
      } else if( nr_valid == 2 ) {
        float depth_step = 0;
        if( valid[0] ) // 01 und 02
        {
          if( valid[1] ) { // 01
            depth_step = fabs(a_cloud_0.at<cv::Vec4f> (r, c)[2] - a_cloud_1.at<cv::Vec4f> (r, c)[2]);
            if( depth_step < Z_STEP_THRESHOLD ) {
              pt[0] = (a_cloud_0.at<cv::Vec4f> (r, c)[0] + a_cloud_1.at<cv::Vec4f> (r, c)[0]) / 2.;
              pt[1] = (a_cloud_0.at<cv::Vec4f> (r, c)[1] + a_cloud_1.at<cv::Vec4f> (r, c)[1]) / 2.;
              pt[2] = (a_cloud_0.at<cv::Vec4f> (r, c)[2] + a_cloud_1.at<cv::Vec4f> (r, c)[2]) / 2.;
            } else if( a_cloud_0.at<cv::Vec4f> (r, c)[2] > a_cloud_1.at<cv::Vec4f> (r, c)[2] ) { // we assign it to the foreground
              pt[0] = a_cloud_0.at<cv::Vec4f> (r, c)[0];
              pt[1] = a_cloud_0.at<cv::Vec4f> (r, c)[1];
              pt[2] = a_cloud_0.at<cv::Vec4f> (r, c)[2];
            } else {
              pt[0] = a_cloud_1.at<cv::Vec4f> (r, c)[0];
              pt[1] = a_cloud_1.at<cv::Vec4f> (r, c)[1];
              pt[2] = a_cloud_1.at<cv::Vec4f> (r, c)[2];
            }
          } else { // 02
            depth_step = fabs(a_cloud_0.at<cv::Vec4f> (r, c)[2] - a_cloud_2.at<cv::Vec4f> (r, c)[2]);
            if( depth_step < Z_STEP_THRESHOLD ) {
              pt[0] = (a_cloud_0.at<cv::Vec4f> (r, c)[0] + a_cloud_2.at<cv::Vec4f> (r, c)[0]) / 2.;
              pt[1] = (a_cloud_0.at<cv::Vec4f> (r, c)[1] + a_cloud_2.at<cv::Vec4f> (r, c)[1]) / 2.;
              pt[2] = (a_cloud_0.at<cv::Vec4f> (r, c)[2] + a_cloud_2.at<cv::Vec4f> (r, c)[2]) / 2.;
            } else if( a_cloud_0.at<cv::Vec4f> (r, c)[2] > a_cloud_2.at<cv::Vec4f> (r, c)[2] ) {
              pt[0] = a_cloud_0.at<cv::Vec4f> (r, c)[0];
              pt[1] = a_cloud_0.at<cv::Vec4f> (r, c)[1];
              pt[2] = a_cloud_0.at<cv::Vec4f> (r, c)[2];
            } else {
              pt[0] = a_cloud_2.at<cv::Vec4f> (r, c)[0];
              pt[1] = a_cloud_2.at<cv::Vec4f> (r, c)[1];
              pt[2] = a_cloud_2.at<cv::Vec4f> (r, c)[2];
            }
          }
        } else { // 12
          depth_step = fabs(a_cloud_1.at<cv::Vec4f> (r, c)[2] - a_cloud_2.at<cv::Vec4f> (r, c)[2]);
          if( depth_step < Z_STEP_THRESHOLD ) {
            pt[0] = (a_cloud_1.at<cv::Vec4f> (r, c)[0] + a_cloud_2.at<cv::Vec4f> (r, c)[0]) / 2.;
            pt[1] = (a_cloud_1.at<cv::Vec4f> (r, c)[1] + a_cloud_2.at<cv::Vec4f> (r, c)[1]) / 2.;
            pt[2] = (a_cloud_1.at<cv::Vec4f> (r, c)[2] + a_cloud_2.at<cv::Vec4f> (r, c)[2]) / 2.;
          } else if( a_cloud_1.at<cv::Vec4f> (r, c)[2] > a_cloud_2.at<cv::Vec4f> (r, c)[2] ) {
            pt[0] = a_cloud_1.at<cv::Vec4f> (r, c)[0];
            pt[1] = a_cloud_1.at<cv::Vec4f> (r, c)[1];
            pt[2] = a_cloud_1.at<cv::Vec4f> (r, c)[2];
          } else {
            pt[0] = a_cloud_2.at<cv::Vec4f> (r, c)[0];
            pt[1] = a_cloud_2.at<cv::Vec4f> (r, c)[1];
            pt[2] = a_cloud_2.at<cv::Vec4f> (r, c)[2];
          }
        }
      }

      else if( nr_valid == 3 ) {
        bool devs_01 = false;
        bool devs_02 = false;
        bool devs_12 = false;
        float z_dev_01 = fabs(a_cloud_0.at<cv::Vec4f> (r, c)[2] - a_cloud_1.at<cv::Vec4f> (r, c)[2]);
        float z_dev_02 = fabs(a_cloud_0.at<cv::Vec4f> (r, c)[2] - a_cloud_2.at<cv::Vec4f> (r, c)[2]);
        float z_dev_12 = fabs(a_cloud_1.at<cv::Vec4f> (r, c)[2] - a_cloud_2.at<cv::Vec4f> (r, c)[2]);
        int nr_devs = 0;
        if( z_dev_01 > Z_STEP_THRESHOLD ) {
          nr_devs++;
          devs_01 = true;
        }
        if( z_dev_02 > Z_STEP_THRESHOLD ) {
          nr_devs++;
          devs_02 = true;
        }
        if( z_dev_12 > Z_STEP_THRESHOLD ) {
          nr_devs++;
          devs_12 = true;
        }

        if( nr_devs == 0 ) // average
        {
          pt[0] = (a_cloud_0.at<cv::Vec4f> (r, c)[0] + a_cloud_1.at<cv::Vec4f> (r, c)[0] + a_cloud_2.at<cv::Vec4f> (r, c)[0]) / 3.;
          pt[1] = (a_cloud_0.at<cv::Vec4f> (r, c)[1] + a_cloud_1.at<cv::Vec4f> (r, c)[1] + a_cloud_2.at<cv::Vec4f> (r, c)[1]) / 3.;
          pt[2] = (a_cloud_0.at<cv::Vec4f> (r, c)[2] + a_cloud_1.at<cv::Vec4f> (r, c)[2] + a_cloud_2.at<cv::Vec4f> (r, c)[2]) / 3.;
        } else if( nr_devs == 2 ) {
          if( devs_01 && devs_02 ) {
            pt[0] = (a_cloud_1.at<cv::Vec4f> (r, c)[0] + a_cloud_2.at<cv::Vec4f> (r, c)[0]) / 2.;
            pt[1] = (a_cloud_1.at<cv::Vec4f> (r, c)[1] + a_cloud_2.at<cv::Vec4f> (r, c)[1]) / 2.;
            pt[2] = (a_cloud_1.at<cv::Vec4f> (r, c)[2] + a_cloud_2.at<cv::Vec4f> (r, c)[2]) / 2.;
          } else if( devs_01 && devs_12 ) {
            pt[0] = (a_cloud_0.at<cv::Vec4f> (r, c)[0] + a_cloud_2.at<cv::Vec4f> (r, c)[0]) / 2.;
            pt[1] = (a_cloud_0.at<cv::Vec4f> (r, c)[1] + a_cloud_2.at<cv::Vec4f> (r, c)[1]) / 2.;
            pt[2] = (a_cloud_0.at<cv::Vec4f> (r, c)[2] + a_cloud_2.at<cv::Vec4f> (r, c)[2]) / 2.;
          } else if( devs_02 && devs_12 ) {
            pt[0] = (a_cloud_0.at<cv::Vec4f> (r, c)[0] + a_cloud_1.at<cv::Vec4f> (r, c)[0]) / 2.;
            pt[1] = (a_cloud_0.at<cv::Vec4f> (r, c)[1] + a_cloud_1.at<cv::Vec4f> (r, c)[1]) / 2.;
            pt[2] = (a_cloud_0.at<cv::Vec4f> (r, c)[2] + a_cloud_1.at<cv::Vec4f> (r, c)[2]) / 2.;
          }
        }
      }
    }
}

}

