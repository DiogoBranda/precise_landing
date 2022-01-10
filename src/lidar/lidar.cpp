#include "lidar.h"
/**
 * @brief lidar::lidar - class constructer
 */
lidar::lidar()
{
  this->cloud = boost::make_shared<CloudType>();// inicialize cloud to store data from lidar
  this->ready = false;
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
 * @brief lidar::clearReady - set the ready variable to zero
 */
void lidar::clearReady(){
   this->ready = false;
}
/**
 * @brief lidar::getBinaryImg
 * @param axis
 * @param lowerLimit
 * @param uperLimit
 * @param img
 * @return
 */
int lidar::getBinaryImg(const char *axis,int lowerLimit,int uperLimit,  std::vector<cv::Mat> &img, float pos[50][6]){
  CloudType::Ptr cloudCopy= boost::make_shared<CloudType>();
 CloudType::Ptr cloudCopy2= boost::make_shared<CloudType>();
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

  std::cout<<"teste_01\n";
   pcl::RangeImage rangeImage;
    std::cout<<"teste_02\n";

    std::cout << "elementos " <<cloudCopy->size() <<"\n";
    std::cout<<"teste_03\n";//:CAMERA_FRAME,
    std::cout<<"asds "<<this->cloud->size()<<"\n";
    for (int r=0;r<cloudCopy->size();r++) {
      if((!(cloudCopy->points[r].x==0.0)) && (!(cloudCopy->points[r].y==0.0)) && (!(cloudCopy->points[r].z==0.0))){
        cloudCopy2->push_back(cloudCopy->points[r]);
      }
    }
    cloudCopy2->width=cloudCopy2->size();
    cloudCopy2->height=1;




  /*#############################
   *                            #
   *    Point cloud clusters    #
   *                            #
   * ###########################*/

  //Euclidean Cluster

  pcl::EuclideanClusterExtraction<PointType> ec;
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  ec.setSearchMethod (tree);
  ec.setClusterTolerance (0.05); //15 cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudCopy2);
  std::vector <pcl::PointIndices> clusters;
  ec.extract (clusters);

//  /*#############################
//   *                            #
//   *     Clusters extraction    #
//   *                            #
//   * ###########################*/
int retval=0;
  int j = 0;
  std::vector< pcl::PointCloud<pcl::PointXYZI> > vecclusters;
  if(clusters.size () > 0){

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it){
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        cloud_cluster->points.push_back (cloudCopy2->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header.frame_id=cloudCopy2->header.frame_id;
      }
      vecclusters.push_back(*cloud_cluster);
      j=j+1;
    }

  }
  /*#############################
   *                            #
   *    Range image generate    #
   *                            #
   * ###########################*/
  std::cout<<"j = " <<j <<"\n";

  for (int iter = 0;iter<j;iter++) {

    if(vecclusters[iter].points[0].x!=0.0){
      pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(vecclusters[iter], pcl::deg2rad(0.35f), pcl::deg2rad(0.7f), pcl::deg2rad(360.0f), pcl::deg2rad(45.0f), Eigen::Affine3f::Identity (), pcl::RangeImage::LASER_FRAME, 0.0f, 0.3f, 0 );

    std::cout<<"itTeste_1 \n";

    cv::Mat imgct= cv::Mat::zeros(500, 500, CV_8U);
    std::cout<<"itTeste_2 \n";

    float x1=0,y1=0,z1=0;
    std::cout<<"itTeste_3 \n";

    for ( unsigned int i = 0; i < vecclusters[iter].size(); i++ ){
      x1 += vecclusters[iter].points[i].x;
      y1 += vecclusters[iter].points[i].y;
      z1 += vecclusters[iter].points[i].z;
    }
    std::cout<<"itTeste_4 \n";

      x1 /=   vecclusters[iter].size();
      y1 /=  vecclusters[iter].size() ;
      z1 /=  vecclusters[iter].size() ;
      std::cout<<"itTeste_5 \n";

      pos[retval][0]=x1;
      pos[retval][1]=y1;
      pos[retval][2]=z1;
      std::cout<<"itTeste_6 \n";

      int rt = 0,tr = 0;

      for(int i=0; i<rangeImage.points.size(); i++){
        if(rangeImage.points[i].x!=rangeImage.points[i].x){
          imgct.at<uchar>(rt,tr) = 0;
          rt++;
        }
        else{
          imgct.at<uchar>(rt,tr) = 255;
          rt++;
        }
        if(i%rangeImage.width==0){
          rt=0;
          tr++;
        }
      }
      std::cout<<"itTeste_7 \n";

      img.push_back(imgct);
      std::cout<<"itTeste_8 \n";
      retval++;
      std::cout<<"itTeste_9 \n";

     // rangeImage.reset();

    }
    }
     return 1;//retval;
  }

int lidar::calibLidar(const char *axis,int lowerLimit,int uperLimit,float pos[6]){
  CloudType::Ptr cloudCopy= boost::make_shared<CloudType>();
 CloudType::Ptr cloudCopy2= boost::make_shared<CloudType>();
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

  double x1, y1,z1;

  for ( unsigned int i = 0; i <cloudCopy->size(); i++ ){
    x1 += cloudCopy->points[i].x;
    y1 += cloudCopy->points[i].y;
    z1 += cloudCopy->points[i].z;
  }
  x1 /=   cloudCopy->size();
  y1 /=  cloudCopy->size() ;
  z1 /=  cloudCopy->size() ;

  pos[0]=x1;
  pos[1]=y1;
  pos[2]=z1;

     return 1;//retval;
  }
