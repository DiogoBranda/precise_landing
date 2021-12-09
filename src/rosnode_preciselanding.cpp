/**
  Code developed by Diogo Silva

  **/

/*#####################
  ##     Includes    ##
  #####################*/

#include <ros/ros.h> ///Ros
#include <pcl_ros/point_cloud.h> ///PCL ROS
#include <sensor_msgs/Image.h> ///For camera image
#include <sensor_msgs/CompressedImage.h> ///For camera image when playing from a bag
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "uavControler/uavcontrol.h" /// Class with the functions to control the UAV
#include "camera/camera.h"
#include "arucoCras/arucoCras.h"
#include "sensorPose/sensorpose.h"
#include "lidar/lidar.h"

/*########################
  ##     Topic Names    ##
  ########################*/

#define VISUAL_CAMERA_IMG "videofile/image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define VISUAL_CAMERA_PARMETER  "videofile/camera_info" ///Name of the topic where the parameter of the termal camera are published
#define TERMAL_CAMERA_IMG "videofile/image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define TERMAL_CAMERA_PARMETER  "videofile/camera_info" ///Name of the topic where the parameter of the termal camera are published
#define N_VISUAL_ARUCOS 1
#define N_TERMAL_ARUCOS 1
#define VISUAL_CAMERA_LINK "cam_link"
#define THERMAL_CAMERA_LINK "cam_link"
#define PLATFORM_LINK "link"

/**
 * @brief main
 */
int main(int argc, char **argv){

  /*##################################
    ##     Ros parameters initi     ##
    ##################################*/

  ros::init(argc, argv, "rosnode_preciselanding"); // Node name
  ros::NodeHandle  n; // Node
  ros::Rate rate(20.0); // Publication rate

  /*#############################
    ##     Class instances     ##
    #############################*/

  uavControl *myUav = new uavControl();

  camera *visualCamera = new camera();
  camera *termalCamera = new camera();
  lidar *lidarOs = new lidar();
  arucoCras *visualAruco[N_VISUAL_ARUCOS] = {new arucoCras(0,0.2,"aruco_1")};
  arucoCras *termalAruco[N_TERMAL_ARUCOS] = {new arucoCras(0,0.2,"aruco_1")};

  sensorpose *visualpose = new sensorpose(VISUAL_CAMERA_LINK);
  sensorpose *termalpose = new sensorpose(THERMAL_CAMERA_LINK);

  /*############################
    ##     Ros Subscriber     ##
    ############################*/

  ros::Subscriber termalCamera_par_sub = n.subscribe<sensor_msgs::CameraInfo>(TERMAL_CAMERA_PARMETER, 10, &camera::cb_camPar,termalCamera);
  ros::Subscriber termalCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(TERMAL_CAMERA_IMG, 10, &camera::cb_camImg,termalCamera);

  ros::Subscriber visualCamera_par_sub = n.subscribe<sensor_msgs::CameraInfo>(VISUAL_CAMERA_PARMETER, 10, &camera::cb_camPar,visualCamera);
  ros::Subscriber visualCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(VISUAL_CAMERA_IMG, 10, &camera::cb_camImg,visualCamera);

  ros::Subscriber lidar_pointcloud_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZI>>("os_cloud_node/points", 10, &lidar::cb_cloud,lidarOs);

  /*###########################
    ##     Ros Publisher     ##
    ###########################*/

  ros::Publisher ArucoPose_pub = n.advertise<geometry_msgs::PoseStamped>("ArucoPose", 10);
  ros::Publisher ArucoDetect_pub = n.advertise<std_msgs::Bool>("VisualDetect", 10);
  ros::Publisher pub = n.advertise<pcl::PointCloud<pcl::PointXYZI>>("output1", 10);

  /*#######################
    ##     Variables     ##
    #######################*/

  tf::TransformListener* listener;
  listener = new tf::TransformListener;

  /*########################
    ##     Main cicle     ##
    ########################*/

  bool configDone = false; // if all sensores are ready to transmite data then the drone can procede to land

  while(ros::ok()){
    if(!configDone){
      if(!lidarOs->getReady()){//!termalCamera->getImageReady()
      }
      else {
        configDone = true;
      }
    }
    else {
//     cv::Mat termalImage;
//     cv::bitwise_not(termalCamera->getImage()->image,termalImage);
//     arucoCras::detector(termalImage,0,termalAruco,N_TERMAL_ARUCOS,termalCamera->getCameraMatrix(),termalCamera->getDistCoeffs());
//     arucoCras::arucoTfPosition(termalAruco,N_TERMAL_ARUCOS,listener,termalpose,PLATFORM_LINK);

//     arucoCras::detector(visualCamera->getImage()->image,0,visualAruco,N_TERMAL_ARUCOS,visualCamera->getCameraMatrix(),visualCamera->getDistCoeffs());
//     arucoCras::arucoTfPosition(visualAruco,N_TERMAL_ARUCOS,listener,visualpose,PLATFORM_LINK);

      std::vector<cv::Mat>img = {cv::Mat::zeros(500,500,CV_8U),cv::Mat::zeros(500,500,CV_8U),cv::Mat::zeros(500,500,CV_8U)};
      //lidarOs->getBinaryImg("intensity",5500,8000,img);


      pub.publish(lidarOs->cloud);
     ArucoPose_pub.publish(termalpose->getPose() );
    }
    ros::spinOnce();
    rate.sleep();
  }


  return 1;
}
