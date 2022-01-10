#ifndef LIDAR_H
#define LIDAR_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/io.h>
#include <sstream>
#include <pcl/range_image/range_image.h>
#include <opencv2/core/mat.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;

class lidar
{
private:
bool ready;
public:
  bool getReady();
  void clearReady();
  CloudType::Ptr cloud;
  lidar();
  void cb_cloud(const CloudType::ConstPtr & cloud_msg);
  int getBinaryImg(const char *axis,int lowerLimit,int uperLimit, std::vector<cv::Mat> &img, float pos[50][6]);
  int calibLidar(const char *axis,int lowerLimit,int uperLimit,float pos[6]);
};

#endif // LIDAR_H
