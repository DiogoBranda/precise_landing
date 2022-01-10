#include "arucoCras.h"

/**
 * @brief arucoCras  - Class constructor
 * @param id - id of aruco (INT)(in)
 * @param size - size of aruco (float)(in)
 * @param nameFrame - Frame name where the aruco is placed on the plataform (const char *nameFrame)(in)
 */
arucoCras::arucoCras(int id,float size,const char *nameFrame,const char *cameraFrame){
  this->id = id;
  this->size = size;
  this->nameFrame = nameFrame;
  this->cameraFrame = cameraFrame;
  this->setArucoPose(0,0,0,0,Eigen::Quaterniond());
}
/**
 * @brief setArucoPose - set aruco pos with the values post on the function
 * @param x - x cordenate value (float)(in)
 * @param y - y cordenate value (float)(in)
 * @param z - z cordenate value (float)(in)
 * @param state - state 1 if aruco is detect 0 if not (float)(in)
 * @param Q1 - Quaternion value (Eigen::Quaterniond)(in)
 */
void arucoCras::setArucoPose(float x, float y, float z, float state,const Eigen::Quaterniond &Q1){

  this->poseAruco.header.stamp=ros::Time();
  this->poseAruco.pose.position.x = x;
  this->poseAruco.pose.position.y = y;
  this->poseAruco.pose.position.z = z;
  tf::quaternionEigenToMsg(Q1, this->poseAruco.pose.orientation);
  this->state = state;
  this->poseAruco.header.frame_id =this->cameraFrame;
}

/**
 * @brief getArucoPose - provides the pose of aruco relative to something
 * @return returns the aruco pose (geometry_msgs::PoseStamped)
 */
geometry_msgs::PoseStamped arucoCras::getArucoPose(){
  return this->poseAruco;
}

/**
 * @brief getId - provides the aruco id
 * @return aruco id (int)
 */
int arucoCras::getId(){
  return this->id;
}

/**
 * @brief getSize - provides the aruco size
 * @return aruco size (float)
 */
float arucoCras::getSize(){
  return this->size;
}

/**
 * @brief getState - provides the aruco state
 * @return aruco state, 0 if not detect 1 if is detect (float)
 */
float arucoCras::getState(){
  return this->state;
}

/**
 * @brief getNameFrame - provides the aruco name framee
 * @return aruco name frame (const char *)
 */
const char *arucoCras::getNameFrame(){
 return this->nameFrame;
}



/**
 * @brief detector given an image it detects the arucos an estimate the position,
 * then stores on the variable aruco passe on the function.
 * @param img - image to try detect the arucos (cv::Mat)(in)
 * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
 * @param n - number of arucos on that exist (int)(in)
 * @param cameraMatrix - camera matrix (cv::Mat)(in)
 * @param distCoeffs -  camera distorcion values (cv::Mat)(in)
 */
void arucoCras::detector(cv::Mat img, arucoCras **aruco, int n, cv::Mat cameraMatrix,cv::Mat distCoeffs,const char *display){
cv::Mat imageCopy;
img.copyTo(imageCopy);
//std::cout <<"asdsad img size  =  "<<img.size<< " "<< display<<"\n";
  /*#####################
   *##    Variables    ##
   *#####################*/

  std::vector<cv::Vec3d> rvecs, tvecs;// rotation and translation matrix
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> markerIds;//Marker ID
  std::vector<std::vector<cv::Point2f>> corners;// Marker corners

  /*######################
   *##    Main cicle    ##
   *######################*/

  /**
   * @brief cv::aruco::detectMarkers return the coners an ids of the arucos presents on the image
   */
  cv::aruco::detectMarkers(img, dictionary, corners, markerIds);


  //If markerids.size()>0 then at list one aruco was detect
  if (markerIds.size()>0){
    cv::aruco::drawDetectedMarkers(imageCopy, corners, markerIds);
    if(corners.size()>0){

      bool foundId = false;// set to true when an aruco that existes on the plataform is detected on the image

      /**
       * check if any of the arucos found on the image are on the **aruco variable
       * */
      //Cicle for the **aruco variable
      for (int f=0;f<n;f++) {

        foundId = false;// everytime a new cicle begins set foundId flag to false

        //cicle for the arucos detect on the image
        for(int i=0; i<markerIds.size(); i++){

          //if the aruco on the image is equal to an aruco on the **aruco variable then it as found a valid aruco
          if(markerIds[i]==aruco[f]->getId()){

            foundId = true;
            /**
             * @brief cv::aruco::estimatePoseSingleMarkers estimates the relative position to the aruco seen by the camera
             */
            cv::aruco::estimatePoseSingleMarkers(corners, aruco[f]->getSize(), cameraMatrix, distCoeffs, rvecs, tvecs);

            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvecs.data()[i], rot);
            // Convert to a tf2::Matrix3x3
            Eigen::Matrix3d tf2_rot;
            tf2_rot <<rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2);

            //Create a quarternion from the rotation matrix
            Eigen::Quaterniond Q1(tf2_rot);

            //Set the pose of the **aruco variable to the value found on the aruco present in the image and set the state to 1
            aruco[f]->setArucoPose((float) tvecs.data()[i][0],(float) tvecs.data()[i][1],(float) tvecs.data()[i][2],(float) 1,Q1);
            break;
          }
        }

        // If not found set pos and state to 0
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
  cv::resizeWindow(display, 960, 540);
  cv::imshow(display, imageCopy);
  char key = (char) cv::waitKey(1);
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
          listener->waitForTransform("base_link_frd","cam_link", ros::Time(0), ros::Duration(0.1));
          listener->transformPose("base_link_frd",aruco[j]->getArucoPose(), transform[j]);
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


/**
 * @brief detectorId detects the id on the image, and associate the position(x,y,z) given
 * @param img - image to check if is there any aruco (cv::Mat)(in)
 * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
 * @param n - number of arucos on that exist (int)(in)
 * @param pos - position of the aruco that are present in the image
 */
void arucoCras::detectorId(cv::Mat img, arucoCras **aruco, int n,float pos[6]){
  cv::Mat imageCopy;
  img.copyTo(imageCopy);


  /*#####################
   *##    Variables    ##
   *#####################*/

  std::vector<cv::Vec3d> rvecs, tvecs;// rotation and translation matrix
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> markerIds;//Marker ID
  std::vector<std::vector<cv::Point2f>> corners;// Marker corners

  /*######################
   *##    Main cicle    ##
   *######################*/


  /**
   * @brief cv::aruco::detecstMarkers return the coners an ids of the arucos presents on the image
   */
  cv::aruco::detectMarkers(img, dictionary, corners, markerIds);

  //If markerids.size()>0 then at list one aruco was detect
  if (markerIds.size()>0){
    cv::aruco::drawDetectedMarkers(imageCopy, corners, markerIds);
    if(corners.size()>0){

      bool foundId = false;// set to true when an aruco that existes on the plataform is detected on the image

      /**
       * check if any of the arucos found on the image are on the **aruco variable
       * */
      //Cicle for the **aruco variable
      for (int f=0;f<n;f++) {
        foundId = false;// everytime a new cicle begins set foundId flag to false

        for(int i=0; i<markerIds.size(); i++){


          if(markerIds[i]==aruco[f]->getId()){

            foundId = true;

            // create cameraMatrix and distCoefs set to zero
            cv::Mat cameraMatrix(3, 3, CV_64FC1);
            cv::Mat distCoeffs(1,5, CV_64FC1);

            /**
             * @brief cv::aruco::estimatePoseSingleMarkers estimates the relative position to the aruco seen by the camera
             */
            cv::aruco::estimatePoseSingleMarkers(corners, aruco[f]->getSize(), cameraMatrix, distCoeffs, rvecs, tvecs);

            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvecs.data()[i], rot);
            // Convert to a tf2::Matrix3x3
            Eigen::Matrix3d tf2_rot;
            tf2_rot <<rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2);

            //Create a quarternion from the rotation matrix
            Eigen::Quaterniond Q1(tf2_rot);

            //Set the pose of the **aruco variable to the value on the pose variable and set the state to 1
            aruco[f]->setArucoPose((float) pos[0],(float) pos[1],(float) pos[2],(float) 1,Q1);
            break;
          }
        }
        if(!foundId){
          //aruco[f]->setArucoPose((float)0,(float) 0,(float) 0,0, Eigen::Quaterniond());
        }
      }

    }
    else {
        //Zero if no pose can be calculated
//      for (int f=0;f<n;f++)
//        aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, 0,Eigen::Quaterniond());

    }


  }
  else {
     //Zero if no marker was detected
//    for (int f=0;f<n;f++)
//      aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, 0,Eigen::Quaterniond());

  }
  cv::resizeWindow("display", 960, 540);
  cv::imshow("display", imageCopy);
  char key = (char) cv::waitKey(1);

}

