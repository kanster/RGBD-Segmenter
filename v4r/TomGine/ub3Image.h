#ifndef UNSIGNED_BYTE_3_IMAGE_H
#define UNSIGNED_BYTE_3_IMAGE_H

#include <pcl/point_types.h>

namespace TomGine
{
  class ub3Image
  {
  private:
    ub3Image ();

  public:
    unsigned width;
    unsigned height;
    unsigned char* data;

    ub3Image (const ub3Image &img)
    {
      width = img.width;
      height = img.height;
      data = (unsigned char*)malloc (sizeof(unsigned char) * img.width * img.height * 3);
      for (unsigned i = 0; i < img.width * img.height * 3; i++)
        data[i] = img.data[i];
    }

    ub3Image (const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
    {
      width = cloud.width;
      height = cloud.height;
      data = (unsigned char*)malloc (sizeof(unsigned char) * cloud.width * cloud.height * 3);
      size_t j (0);
      for (size_t i = 0; i < cloud.size (); i++)
      {
        const pcl::PointXYZRGB &p = cloud.at (i);
        data[j + 0] = p.b;
        data[j + 1] = p.g;
        data[j + 2] = p.r;
        j += 3;
      }
    }

    ~ub3Image ()
    {
      free (data);
    }

    void
    operator= (const ub3Image &img)
    {
      free (data);
      width = img.width;
      height = img.height;
      data = (unsigned char*)malloc (sizeof(unsigned char) * img.width * img.height * 3);
      for (unsigned i = 0; i < img.width * img.height * 3; i++)
        data[i] = img.data[i];
    }

    void
    operator= (const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
    {
      free (data);
      width = cloud.width;
      height = cloud.height;
      data = (unsigned char*)malloc (sizeof(unsigned char) * cloud.width * cloud.height * 3);
      size_t j (0);
      for (size_t i = 0; i < cloud.size (); i++)
      {
        const pcl::PointXYZRGB &p = cloud.at (i);
        data[j + 0] = p.b;
        data[j + 1] = p.g;
        data[j + 2] = p.r;
        j += 3;
      }
    }
  };
}

#endif
