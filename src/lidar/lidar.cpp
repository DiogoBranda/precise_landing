#include "lidar.h"
/**
 * @brief lidar::lidar - class constructer
 */
lidar::lidar()
{
  this->cloud = boost::make_shared<CloudType>();// inicialize cloud to store data from lidar

}

/**
 * @brief lidar::cb_cloud - callback function. When use to subscrib a lidar topic,
 * automatic stores on variable cloud from this class
 * @param cloud_msg - point cloud from lidar
 */
void lidar::cb_cloud(const CloudType::ConstPtr & cloud_msg){
  pcl::copyPointCloud(*cloud_msg,*(this->cloud)) ; // copy point cloud from lidar to the point cloud param on the class
  this->ready = true; // lidar is returning data, so it can be read
  return;
}
/**
 * @brief lidar::getReady - Return true if the topic relative to point cloud from the lidar was inicialized
 * false other wise
 * @return status of inicialization
 */
bool lidar::getReady(){
  return this->ready;
}

/**
 * @brief lidar::getBinaryImg
 * @param axis
 * @param lowerLimit
 * @param uperLimit
 * @param img
 * @return
 */
int lidar::getBinaryImg(char axis[4],int lowerLimit,int uperLimit,  std::vector<cv::Mat> img){
  CloudType::Ptr cloudCopy;

  pcl::copyPointCloud(*(this->cloud),*cloudCopy) ;// create a point cloud copy, so that the point cloud stored in class dont change

  pcl::PassThrough<PointType> pass; // pass filter only the points in the interval define on the parameters will count


  /*#############################
   *                            #
   *    Point cloud filter      #
   *                            #
   * ###########################*/

  // From this point only the points on this interval [ lowerLimit, uperLimit ] will be used

  pass.setInputCloud(this->cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits (lowerLimit, uperLimit);
  pass.filter(*cloudCopy);


  /*#############################
   *                            #
   *    Point cloud clusters    #
   *                            #
   * ###########################*/

  //Euclidean Cluster

  pcl::EuclideanClusterExtraction<PointType> ec;
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  ec.setSearchMethod (tree);
  ec.setClusterTolerance (0.15); //15 cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudCopy);
  std::vector <pcl::PointIndices> clusters;
  ec.extract (clusters);

  /*#############################
   *                            #
   *     Clusters extraction    #
   *                            #
   * ###########################*/

  int j = 0;
  std::vector< pcl::PointCloud<pcl::PointXYZI> > vecclusters;

  if(clusters.size () > 0){


    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it){
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header.frame_id=cloud->header.frame_id;
      }
      vecclusters.push_back(*cloud_cluster);
      j=j+1;
    }

  }

 std::cout<<"vec size = "<< vecclusters.size()<<"\n size j = "<< j<< "\n";

  /*#############################
   *                            #
   *    Range image generate    #
   *                            #
   * ###########################*/

//  pcl::RangeImage rangeImage;
//  rangeImage.createFromPointCloud(*cloudCopy, pcl::deg2rad(0.35f), pcl::deg2rad(360.0f), pcl::deg2rad(45.0f), Eigen::Affine3f::Identity (), pcl::RangeImage::LASER_FRAME, 0.0f, 0.3f, 0 );


//  int rt = 0,tr = 0;

//  for(int i=0; i<rangeImage.points.size(); i++){
//    if(rangeImage.points[i].x!=rangeImage.points[i].x){
//      img.at<uchar>(rt,tr) = 0;
//      rt++;
//    }
//    else{
//      img.at<uchar>(rt,tr) = 255;
//      rt++;
//    }
//    if(i%rangeImage.width==0){
//      rt=0;
//      tr++;
//    }
//  }
//  return img;
}
