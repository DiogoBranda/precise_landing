#include "arucoCras.h"

/**
 * @brief myaruco - constructor
 * @param id - type int id of the aruco
 */
arucoCras::arucoCras(int id,float size,const char *nameFrame){
  this->id=id;
  this->size = size;
  this->setArucoPose(0,0,0,0,Eigen::Quaterniond());
  this->nameFrame=nameFrame;
}
/**
 * @brief setArucoPose - save the pose of aruco relative to something
 * @param x - float x cordenate value
 * @param y - float y cordenate value
 * @param z - float z cordenate value
 * @param state - state 1 if aruco is detect 0 if not
 */
void arucoCras::setArucoPose(float x, float y, float z, float state,const Eigen::Quaterniond &Q1){

  this->poseAruco.header.stamp=ros::Time();
  this->poseAruco.pose.position.x = x;
  this->poseAruco.pose.position.y = y;
  this->poseAruco.pose.position.z = z;
  tf::quaternionEigenToMsg(Q1, this->poseAruco.pose.orientation);
  this->state = state;
}
/**
 * @brief getArucoPose - provides the pose of aruco relative to something
 * @return geometry_msgs::PoseStamped type value
 */
geometry_msgs::PoseStamped arucoCras::getArucoPose(){
  return this->poseAruco;
}

int arucoCras::getId(){
  return this->id;
}

float arucoCras::getSize(){
  return this->size;
}
float arucoCras::getState(){
  return this->state;
}


 void arucoCras::detector(cv::Mat img, arucoCras **aruco, int n, cv::Mat cameraMatrix,cv::Mat distCoeffs){
  std::vector<cv::Vec3d> rvecs, tvecs;// rotation and translation matrix
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> markerIds;//Marker ID
  std::vector<std::vector<cv::Point2f>> corners;// Marker corners

  cv::aruco::detectMarkers(img, dictionary, corners, markerIds);


  if (markerIds.size()>0){

    if(corners.size()>0){

      bool foundId = false;
      for (int f=0;f<n;f++) {
        foundId = false;
        for(int i=0; i<markerIds.size(); i++){

          if(markerIds[i]==aruco[f]->getId()){
            foundId = true;
            cv::aruco::estimatePoseSingleMarkers(corners, aruco[f]->getSize(), cameraMatrix, distCoeffs, rvecs, tvecs);
            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvecs.data()[i], rot);
            // Convert to a tf2::Matrix3x3
            Eigen::Matrix3d tf2_rot;
            tf2_rot <<rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2);

            Eigen::Quaterniond Q1(tf2_rot);
            aruco[f]->setArucoPose((float) tvecs.data()[i][0],(float) tvecs.data()[i][1],(float) tvecs.data()[i][2],(float) 1,Q1);
            break;
          }
        }
        if(!foundId){
          aruco[f]->setArucoPose((float)0,(float) 0,(float) 0,0, Eigen::Quaterniond());
        }
      }

    }
    else {
        //Zero if no pose can be calculated
      for (int f=0;f<n;f++)
        aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, 0,Eigen::Quaterniond());

    }


  }
  else {
     //Zero if no marker was detected
    for (int f=0;f<n;f++)
      aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, 0,Eigen::Quaterniond());

  }
}

const char *arucoCras::getNameFrame(){
  return this->nameFrame;
}

void arucoCras::arucoTfPosition(arucoCras **aruco,int sizeVec,tf::TransformListener *listener,sensorpose *pose, const char * platformBaseLinkName){
  geometry_msgs::PoseStamped transform[40];
  tf::StampedTransform transformer[40];
  float peso = 0;
  bool flag = false;

  float x=0,y=0,z=0;
  float xq=0,yq=0,zq=0,w=1;

  for (int j=0;j<sizeVec;j++) {
    if(aruco[j]->getState()>0){
      peso += aruco[j]->getSize();
      flag =true;
      try {
          listener->waitForTransform(platformBaseLinkName, aruco[j]->getNameFrame(), ros::Time(0), ros::Duration(0.1));
          listener->lookupTransform(platformBaseLinkName, aruco[j]->getNameFrame(),ros::Time(0), transformer[j]);
      }catch (tf::TransformException e) {
        ROS_ERROR("%s",e.what());
        ros::Duration(1.0).sleep();
      }
      x = ((float)transform[j].pose.position.x + transformer[j].getOrigin().x())* aruco[j]->getSize() + x;
      y = ((float)transform[j].pose.position.y + transformer[j].getOrigin().y()) * aruco[j]->getSize() + y;
      z = ((float)transform[j].pose.position.z - transformer[j].getOrigin().z())* aruco[j]->getSize() + z;
      xq = transform[j].pose.orientation.x * aruco[j]->getSize() +  xq;
      yq = transform[j].pose.orientation.y * aruco[j]->getSize() +  yq;
      zq = transform[j].pose.orientation.z * aruco[j]->getSize() + zq;
      w = transform[j].pose.orientation.w * aruco[j]->getSize() +  w;
    }
  }
  if(flag){
    pose->setPose(x/peso, y/peso, z/peso, xq/peso, yq/peso, zq/peso, w/peso);
    pose->setDetect(true);
  }
  else {
    pose->setDetect(false);
  }
return;
}

