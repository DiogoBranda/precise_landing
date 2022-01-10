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

#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<eigen_conversions/eigen_msg.h>

/*########################
  ##     Topic Names    ##
  ########################*/
//"image_raw/compressed"
#define VISUAL_CAMERA_IMG "image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define VISUAL_CAMERA_PARMETER  "image_raw/camera_info" ///Name of the topic where the parameter of the termal camera are published
#define TERMAL_CAMERA_IMG "usb_cam/image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define TERMAL_CAMERA_PARMETER  "usb_cam/image_raw/camera_info" ///Name of the topic where the parameter of the termal camera are published
#define LIDAR_POINT_CLOUD "os_cloud_node/points"
#define N_VISUAL_ARUCOS 1
#define N_TERMAL_ARUCOS 1
#define N_LIDAR_ARUCOS 1
#define VISUAL_CAMERA_LINK "cam_link"
#define THERMAL_CAMERA_LINK "cam_link"
#define LIDAR_LINK "cam_link"

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
  //ros::Rate rate(20.0); // Publication rate
  ros::Rate rate(2.0); // Publication rate

  std::cout<<"Precise landing node starting\n" ;

  /*#############################
    ##     Class instances     ##
    #############################*/

  uavControl *myUav = new uavControl();

  camera *visualCamera = new camera();
  camera *termalCamera = new camera();
  lidar *lidarOs = new lidar();

  arucoCras *visualAruco[N_VISUAL_ARUCOS] = {new arucoCras(0,0.2,"aruco_1",VISUAL_CAMERA_LINK)};
  arucoCras *termalAruco[N_TERMAL_ARUCOS] = {new arucoCras(0,0.2,"aruco_1",THERMAL_CAMERA_LINK)};
  arucoCras *lidarAruco[N_LIDAR_ARUCOS] = {new arucoCras(0,0.2,"aruco_1",LIDAR_LINK)};

  sensorpose *visualpose = new sensorpose(VISUAL_CAMERA_LINK);
  sensorpose *termalpose = new sensorpose(THERMAL_CAMERA_LINK);
  sensorpose *lidarpose = new sensorpose(THERMAL_CAMERA_LINK);
  std::cout<<"Class instances done" ;

  /*############################
    ##     Ros Subscriber     ##
    ############################*/

  ros::Subscriber termalCamera_par_sub = n.subscribe<sensor_msgs::CameraInfo>(TERMAL_CAMERA_PARMETER, 10, &camera::cb_camPar,termalCamera);
  ros::Subscriber termalCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(TERMAL_CAMERA_IMG, 10, &camera::cb_camImg,termalCamera);

  ros::Subscriber visualCamera_par_sub = n.subscribe<sensor_msgs::CameraInfo>(VISUAL_CAMERA_PARMETER, 10, &camera::cb_camPar,visualCamera);
  ros::Subscriber visualCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(VISUAL_CAMERA_IMG, 10, &camera::cb_camImg,visualCamera);

  ros::Subscriber lidar_pointcloud_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZI>>("os_cloud_node/points", 10, &lidar::cb_cloud,lidarOs);
  std::cout<<" Ros Subscriber done" ;

  /*###########################
    ##     Ros Publisher     ##
    ###########################*/
ros::Publisher land_vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
  //ros::Publisher ArucoPose_pub = n.advertise<geometry_msgs::PoseStamped>("ArucoPose", 10);
  //ros::Publisher ArucoDetect_pub = n.advertise<std_msgs::Bool>("VisualDetect", 10);
//  std::cout<<" Ros Publisher done" ;

  /*#######################
    ##     Variables     ##
    #######################*/

  tf::TransformListener* listener;
  listener = new tf::TransformListener;
  std::cout<<" Tf listener done" ;

  /*########################
    ##     Main cicle     ##
    ########################*/

  bool configDone = false; // if all sensores are ready to transmite data then the drone can procede to land
  while(ros::ok()){
      double x = 0, y = 0, z = 0;
      double tcaliby = -0.0306408, tcalibx=-0.235605, tcalibz=-1.52046;
      double lcalibx = 0.148387, lcaliby =0.119842, lcalibz = -0.22668;

      int count = 0;
    if(termalCamera->getImageReady()==1  && termalCamera->getConfigReady() ){
      cv::Mat termalImage;
      cv::bitwise_not(termalCamera->getImage()->image,termalImage);
      arucoCras::detector(termalImage,termalAruco,N_TERMAL_ARUCOS,termalCamera->getCameraMatrix(),termalCamera->getDistCoeffs(),"termica");
      arucoCras::arucoTfPosition(termalAruco,N_TERMAL_ARUCOS,listener,termalpose,PLATFORM_LINK);
      std::cout << " Termica x =" <<termalpose->getPose().pose.position.x<< " y =" <<termalpose->getPose().pose.position.y<< " z = " <<termalpose->getPose().pose.position.z<<"\n";
      x = x + termalpose->getPose().pose.position.x + tcalibx;
      y = y + termalpose->getPose().pose.position.y + tcaliby;
      z = z + termalpose->getPose().pose.position.z + tcalibz;
      count++;
    }

    if(visualCamera->getImageReady()==1 && visualCamera->getConfigReady()){
      arucoCras::detector(visualCamera->getImage()->image,visualAruco,N_TERMAL_ARUCOS,visualCamera->getCameraMatrix(),visualCamera->getDistCoeffs(),"VISUAL");
      arucoCras::arucoTfPosition(visualAruco,N_TERMAL_ARUCOS,listener,visualpose,PLATFORM_LINK);
      std::cout << " Visual x =" <<visualpose->getPose().pose.position.x<< " y =" <<visualpose->getPose().pose.position.y<< " z = " <<visualpose->getPose().pose.position.z<<"\n";
      x = x + visualpose->getPose().pose.position.x ;
      y = y + visualpose->getPose().pose.position.y ;
      z = z + visualpose->getPose().pose.position.z ;
      count++;

    }
  if(lidarOs->getReady()){
    float pos[6];

    int nLidarArucos = lidarOs->calibLidar("intensity",5500,8000,pos);
    std::cout<<"lidar x = "<<pos[0]<<" y = "<<pos[1]<<" z = "<<pos[2]<<"\n";

    x = x + pos[0] + lcalibx;
    y = y + pos[2] + lcaliby;
    z = z - pos[1] + lcalibz;
    count++;

   }
    x /=count;
    y /=count;
    z /=count;

    std::cout << " fim x =" <<x<< " y =" <<y<< " z = " <<z<<"\n";
    float vx =0, vy=0, vz =0;
    myUav->p(x,y,(-1)*z,&vx,&vy,&vz,1/10);
    std::cout << " vel x =" <<vx<< " y =" <<vy<< " z = " <<vz<<"\n";

    geometry_msgs::TwistStamped vel;
    vel.header.stamp = ros::Time::now();
    vel.twist.linear.x = vx;
    vel.twist.linear.y = vy;
    vel.twist.linear.z = vz;
land_vel_pub.publish(vel);
    termalCamera->clearImageReady();
    visualCamera->clearImageReady();
    lidarOs->clearReady();

    ros::spinOnce();
    rate.sleep();
  }
  return 1;
}
