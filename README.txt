SegmenterLight 0.1
##################
This software package contains software to segment data from RGBD-sensors, such as Microsoft Kinect or Asus Xtion. 


Related papers
##############
[Richtsfeld2012] Segmentation of Unknown Objects in Indoor Environments. Richtsfeld A., Mörwald T., Prankl J.,
Zillich M. and Vincze M. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012


Prerequisites
#############
The software package is designed and tested under Linux (11.10 and 12.04). Installation of OpenCV (> Vers. 2.4.1)
and installation of the pcl-trunk (www.pointclouds.org) is mandatory. 
The following Linux (Ubuntu) packages are required:
- Eigen3


Installation
############
Open a console session and go to the root of the software package. Then:
	$ make build
	$ cd build
	$ cmake ..
	$ make
To install the software to the system (default: /usr/local/):
	$ sudo make install
Uninstall the software:
	$ sudo make uninstall


Using the software
##################

We provide a interface to use the segmentation in your software package. Link the produced library 'v4rSegmenterLight'
(libv4rSegmenterLight.so) to your C++ project and add the folling to your code:

  #include "v4r/SegmenterLight/SegmenterLight.h"
  ...
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud;	// your pcl cloud to segment
  // get your pcl_cloud from the sensor or load it from disk (see pcl documentation for more information)
  ...
  std::string modelPath = "";
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
  segment::SegmenterLight seg(modelPath);
  seg.setFast(true);
  seg.setDetail(0);        
  pcl_cloud_labeled = seg.processPointCloud(pcl_cloud);


'pcl_cloud_labeled' contains now the segmentation results. You can use e.g. pcl-cloud viewer to visualize the results.
You have to define the model path with the SVM models. If the path is empty, the sofware uses the path provided from 
CMake (this requires installation of the package).

setFast: disables model-fitting and makes processing much faster but segmentation slightly worser.

setDetail: 0: Maximum details, 1: medium details, 2: minimum details. Set the degree of details. The more details, the 
more accurate, but also slower.

Demo-Application
################
When installing the package, a demo app will be constructed, called SegmenterLight.


Citation
########
If you are using the software, please cite:

[Richtsfeld2012a] Segmentation of Unknown Objects in Indoor Environments. Richtsfeld A., Mörwald T., Prankl J.,
Zillich M. and Vincze M. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012


Do not hesitate to contact me for further questions.

-------------------
DI Richtsfeld Andreas
Vienna University of Technology
Gusshausstraße 25
1170 Vienna
ari(at)acin.tuwien.ac.at
