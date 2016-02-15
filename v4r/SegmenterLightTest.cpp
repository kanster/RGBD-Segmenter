/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
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
 * @file SegmenterLightTest.cpp
 * @author Andreas Richtsfeld
 * @date January 2013
 * @version 0.1
 * @brief Test program for the SegmenterLight.
 */


#include "SegmenterLightTest.h"

namespace segment
{

double timespec_diff(struct timespec *x, struct timespec *y)
{
  if(x->tv_nsec < y->tv_nsec)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double)(x->tv_sec - y->tv_sec) +
    (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
}
  
void SegmenterLightTest::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                                                cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  
  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[2] = pt.r;
      cvp[1] = pt.g;
      cvp[0] = pt.b;
    }
  }
}

void SegmenterLightTest::ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pcl_cloud, 
                                               std::vector<cv::Vec4f> &cvCloud/*,
                                               bool random_colors*/)
{
  int pcWidth = pcl_cloud->width;
  int pcHeight = pcl_cloud->height;
  unsigned position = 0;

  unsigned max_label = 0;
  for(unsigned i=0; i<pcl_cloud->points.size(); i++)
    if(pcl_cloud->points[i].label > max_label)
      max_label = pcl_cloud->points[i].label;
    
  
  RGBValue color[max_label];
  for(unsigned i=0; i<max_label; i++)
    color[i].float_value = GetRandomColor();

  for (int row = 0; row < pcHeight; row++) {
    for (int col = 0; col < pcWidth; col++) {
      cv::Vec4f p;
      position = row * pcWidth + col;
      p[0] = pcl_cloud->points[position].x;
      p[1] = pcl_cloud->points[position].y;
      p[2] = pcl_cloud->points[position].z;
      p[3] = color[pcl_cloud->points[position].label].float_value;
      cvCloud.push_back(p);
    }
  }
}


/* --------------- PreSegmenter --------------- */

SegmenterLightTest::SegmenterLightTest()
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = "/media/Daten/OSD/";
  rgbd_filename = "points2/test%1d.pcd";
  data_live = false;
  startIdx = 0;
  endIdx = 65;  
  overall_runtime = 0;
}

SegmenterLightTest::~SegmenterLightTest()
{
}

void SegmenterLightTest::init()
{
  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  
  // init kinect data reader
  kinect = new KinectData();
  kinect->setDatabasePath(database_path); 
  if(data_live)
    kinect->setReadDataLive();
  else
    kinect->setReadDataFromFile(rgbd_filename, rgbd_filename, startIdx, endIdx, data_depth);
  if(load_models)
    kinect->setReadModelsFromFile(sfv_filename, startIdx, endIdx);
}

void SegmenterLightTest::process()
{
  pcl_cloud_labeled.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);
  kinect->getImageData(pcl_cloud);
                     
  static struct timespec start, current;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  segment::SegmenterLight seg("");
  seg.setFast(true);
  seg.setDetail(2);
  pcl_cloud_labeled = seg.processPointCloud(pcl_cloud);
  
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  double runtime = segment::timespec_diff(&current, &start);
  printf("[processPointCloud] Runtime SegmenterLight: %4.3f\n", runtime);
  overall_runtime += runtime;
  printf("[processPointCloud] Runtime SegmenterLight OVERALL: %4.3f\n", overall_runtime);
}


void SegmenterLightTest::run(std::string _rgbd_filename,
                       int _startIdx, int _endIdx, 
                       bool _live)
{
  bool processed = false;
  database_path = "";
  rgbd_filename = _rgbd_filename;
  startIdx = _startIdx;
  endIdx = _endIdx;
  data_live = _live;
  init();
  
  // ######################## Setup TomGine ########################
  int width = 640;
  int height = 480;
  surface::View view;
  
#ifdef V4R_TOMGINE
  TomGine::tgTomGineThread dbgWin(width, height, "TomGine Render Engine");
#endif
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Vec3d rotCenter(0, 0, 1.0);

  cv::Mat intrinsic;
  intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  view.intrinsic = Eigen::Matrix3d::Zero();
  intrinsic.at<double> (0, 0) = intrinsic.at<double> (1, 1) = view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
  intrinsic.at<double> (0, 2) = view.intrinsic(0, 2) = 320;
  intrinsic.at<double> (1, 2) = view.intrinsic(1, 2) = 240;
  intrinsic.at<double> (2, 2) = view.intrinsic(2, 2) = 1.;

#ifdef V4R_TOMGINE
  dbgWin.SetClearColor(0.0, 0.0, 0.0);
//   dbgWin.SetCoordinateFrame();
  dbgWin.SetCamera(intrinsic);
  dbgWin.SetCamera(R, t);
  dbgWin.SetRotationCenter(rotCenter);
  dbgWin.SetInputSpeeds(0.5, 0.5, 0.5);
  dbgWin.Update();
#endif
  
  cv::Mat_<cv::Vec3b> kImage = cv::Mat_<cv::Vec3b>::zeros(480, 640);
  cv::imshow("Debug image", kImage);
  
  bool do_it = true;
  bool single_image = true;
  bool win_done = true;
  while(do_it) {
    if(!single_image) {
      process();
      ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage); 
      cvWaitKey(10);
#ifdef V4R_TOMGINE
      dbgWin.SetImage(kImage);
      dbgWin.Update();
#endif
      win_done = false;
    }

    int key = cvWaitKey(50);
    
    if((char) key == 'h') {
      printf("[SegmenterLightTest] Print help:\n");
      printf("  Debug window:\n");
      printf("    \'h\' - Print this help.\n");
      printf("    \'F9\' - Process single data file.\n");
      printf("    \'F10\' - Process data file sequence. \n");
      printf("    \'5\' - Show results.\n");
      printf("    \'q\' - Quit.\n");
      printf("  TomGine Render Engine:\n");
      printf("    \'z\' - To initial position.\n");
      printf("    \'i\' - Enable/disable background image.\n");
      printf("    \'t\' - Enable/disable text annotation.\n");
      printf("    \'p\' - Enable/disable displaying of point cloud.\n");
      printf("    \'F11\' - Save sceenshot.\n");
      printf("    \'q\' - Quit.\n");
    }
    
    if (key == 65478 || key == 1114054)  { // F9
      printf("[SegmenterLightTest] Process single image.\n");
      process();
      ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage);
#ifdef V4R_TOMGINE
      dbgWin.SetImage(kImage);
      dbgWin.Update();
#endif
      win_done = false;
      processed = true;
    }
    if (key == 65479 || key == 1114055)  { // F10
      printf("[SegmenterLightTest] Process images countiniously.\n");
      single_image = false;
      processed = true;
    }

    if((char) key == 'q') {
        printf("[SegmenterLightTest] Quit.\n");
        do_it = false;
    }
    
    if((char) key == '5' || !win_done) {
#ifdef V4R_TOMGINE
      dbgWin.Clear();
      std::vector<cv::Vec4f> vec_cloud;
      ConvertPCLCloud2CvVec(pcl_cloud_labeled, vec_cloud);
      dbgWin.AddPointCloud(vec_cloud);
      dbgWin.Update();
#endif
      win_done = true;
    }
  }
}

} // end segment


void printUsage(char *av)
{
  printf("Usage: %s [options] \n"
    " Options:\n"
    "   [-h] ... show this help.\n"
    "   [-f rgbd_filename] ... specify rgbd-image path and filename\n"
    "   [-idx start end] ... start and end index of files\n"
    "   [-l] ... live image from Kinect\n", av);
  std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -idx 0 10" << std::endl;
}


int main(int argc, char *argv[])
{
  std::string rgbd_filename = "points2/test%1d.pcd";
  int startIdx = 0;
  int endIdx = 65;
  bool live = false;
  
  for(int i=1; i<argc; i++) {
    if(strcmp (argv[i], "-h") == 0) {
      printUsage(argv[0]);
      exit(0);
    }
    if(strcmp (argv[i], "-l") == 0)
      live = true;
    if(i+1 < argc) {
      if(strcmp (argv[i], "-f") == 0)
        rgbd_filename = argv[i+1];
      if(strcmp (argv[i], "-idx") == 0) {
        startIdx = atoi(argv[i+1]);
        if(i+2 < argc)
          endIdx = atoi(argv[i+2]);
      }
    }
    else 
      printUsage(argv[0]);
  }
    
  segment::SegmenterLightTest seg;
  seg.run(rgbd_filename, startIdx, endIdx, live);
}


