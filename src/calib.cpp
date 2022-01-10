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
int errx =0,erry =0,errz =0;
  /*##################################
    ##     Ros parameters initi     ##
    ##################################*/

  ros::init(argc, argv, "rosnode_preciselanding"); // Node name
  ros::NodeHandle  n; // Node
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
          float pos[6];
    double caliby = -0.0306408, calibx=-0.235605, calibz=-1.52046;
    double ty = 0, tx=0, tz=0;
    if(termalCamera->getImageReady()==1  && termalCamera->getConfigReady() ){
      cv::Mat termalImage;
      cv::bitwise_not(termalCamera->getImage()->image,termalImage);
      arucoCras::detector(termalImage,termalAruco,N_TERMAL_ARUCOS,termalCamera->getCameraMatrix(),termalCamera->getDistCoeffs(),"termica");
      arucoCras::arucoTfPosition(termalAruco,N_TERMAL_ARUCOS,listener,termalpose,PLATFORM_LINK);
      std::cout << " Termica x =" <<termalpose->getPose().pose.position.x<< " y =" <<termalpose->getPose().pose.position.y<< " z = " <<termalpose->getPose().pose.position.z<<"\n";
     // std::cout << " TERMAL CORR x =" <<(termalpose->getPose().pose.position.x + calibx)<< " y =" <<(termalpose->getPose().pose.position.y + caliby)<< " z = " <<(termalpose->getPose().pose.position.z +calibz)<<"\n";
      tx = (termalpose->getPose().pose.position.x + calibx);
      ty = (termalpose->getPose().pose.position.y + caliby);
      tz = (termalpose->getPose().pose.position.z + calibz);
    }

    if(visualCamera->getImageReady()==1 && visualCamera->getConfigReady()){
      arucoCras::detector(visualCamera->getImage()->image,visualAruco,N_TERMAL_ARUCOS,visualCamera->getCameraMatrix(),visualCamera->getDistCoeffs(),"VISUAL");
      arucoCras::arucoTfPosition(visualAruco,N_TERMAL_ARUCOS,listener,visualpose,PLATFORM_LINK);
      std::cout << " Visual x =" <<visualpose->getPose().pose.position.x<< " y =" <<visualpose->getPose().pose.position.y<< " z = " <<visualpose->getPose().pose.position.z<<"\n";
     if(((abs(visualpose->getPose().pose.position.x -tx)>0.02)) && visualpose->getDetect().data && termalpose->getDetect().data) {
//         std::cout << " erro x =" <<(visualpose->getPose().pose.position.x -tx)<<"\n";
//         std::cout << " Termica x =" <<termalpose->getPose().pose.position.x<< "\n";
//         std::cout << " Visual x =" <<visualpose->getPose().pose.position.x<<"\n";
         errx++;
//         std::cout << " err count x =" <<errx<<"\n";

     }
     if(((abs(visualpose->getPose().pose.position.y -ty)>0.02)) && visualpose->getDetect().data && termalpose->getDetect().data){
//         std::cout << " erro y =" <<(visualpose->getPose().pose.position.y -ty)<<"\n";
//         std::cout << " Termica y =" <<termalpose->getPose().pose.position.y<< "\n";
//         std::cout << " Visual y =" <<visualpose->getPose().pose.position.y<<"\n";
         erry++;
//         std::cout << " err count y =" <<erry<<"\n";
     }
     if(((abs(visualpose->getPose().pose.position.z -tz)>0.02)) && visualpose->getDetect().data && termalpose->getDetect().data){
//         std::cout << " erro z =" <<(visualpose->getPose().pose.position.z -tz)<<"\n";
//         std::cout << " Termica z =" <<termalpose->getPose().pose.position.z<< "\n";
//         std::cout << " Visual z =" <<visualpose->getPose().pose.position.z<<"\n";
         errz++;
//         std::cout << " err count z =" <<errz<<"\n";
     }

    }
  if(lidarOs->getReady()){
      std::vector<cv::Mat> img ;


      int nLidarArucos = lidarOs->calibLidar("intensity",5500,8000,pos);
      std::cout<<"lidar x = "<<pos[0]<<" y = "<<pos[1]<<" z = "<<pos[2]<<"\n";

//      std::cout<<"teste_1\n";

//      if (nLidarArucos>0){
//        std::cout<<"teste_2\n";

//        for (int y=0;y<N_LIDAR_ARUCOS;y++) {
//          lidarAruco[y]->setArucoPose(0,0,0,0,Eigen::Quaterniond());
//        }
//        std::cout<<"teste_3\n";

//        for (int y=0;y<nLidarArucos;y++) {
//          std::cout<<"teste_4_"<<y<<"\n";
//          std::cout<<"nlidar "<<nLidarArucos<<"\n";
//           std::cout<<"img size "<<img[y].size<<"\n";
//           std::cout<<"img pos "<<pos[y][0]<<"\n";
//          arucoCras::detectorId(img[y],lidarAruco,N_LIDAR_ARUCOS,pos[y]);
//          std::cout<<"teste_5_"<<y<<"\n";

//          arucoCras::arucoTfPosition(lidarAruco,N_LIDAR_ARUCOS,listener,lidarpose,PLATFORM_LINK);
//          std::cout<<"teste_6_"<<y<<"\n";

//        }
//      }
   }
     std::cout << " CALIB x =" <<visualpose->getPose().pose.position.x - pos[0]<< " y =" <<visualpose->getPose().pose.position.y - pos[2]<< " z = " <<visualpose->getPose().pose.position.z + pos[1]<<"\n";

    termalCamera->clearImageReady();
    visualCamera->clearImageReady();
    lidarOs->clearReady();

    ros::spinOnce();
    rate.sleep();
  }
  return 1;
}
