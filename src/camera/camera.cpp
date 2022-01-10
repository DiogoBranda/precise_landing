#include "camera.h"

/**
 * @brief camera - constructor
 * @param cameImgName - name of the topic that publish the camera image
 * @param cameInfoName - name of the topic that publish the camera info
 * @param n - Ros Node
 */
camera::camera()
{

}
/**
 * @brief cam_par - callback function use to save the camera parameters
 * @param camInfo_msg
 */
void camera::cb_camPar (const sensor_msgs::CameraInfoConstPtr& camInfo_msg){
  // If not config (No parameters set before)
  if(this->config==0){
    cv::Mat cameraMatrix_pt(3, 3, CV_64FC1, (void *) camInfo_msg->K.data());
    cv::Mat distCoeffs_pt(1,5, CV_64FC1, (void *) camInfo_msg->D.data());
    cameraMatrix_pt.copyTo( this->cameraMatrix);
    distCoeffs_pt.copyTo(this->distCoeffs);
    this->config=1; // config done
  }
}
/**
 * @brief cam_img - callback function use to save the image provides by the camera
 * @param image_msg
 */
//void camera::cam_img(const sensor_msgs::Image::ConstPtr & image_msg){
//  try{
//    this->image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
//    this->imageReady = 1;
//  }

//  catch (cv_bridge::Exception& e){
//    ROS_ERROR("cv_bridge exception in cam_cb function: %s", e.what());
//    return;
//  }
//}
void camera::cb_camImg(const sensor_msgs::CompressedImage::ConstPtr & image_msg){
  try{
    this->image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    this->imageReady = 1;
  }

  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception in cam_cb function: %s", e.what());
    return;
  }

}

/**
 * @brief getCameraMatrix - provides the camera parameters
 * @return cv::Mat type matrix with the camera parameters
 */
cv::Mat camera::getCameraMatrix(){
  return this->cameraMatrix;
}
/**
 * @brief getDistCoeffs - provides the camera distortion coefficients
 * @return cv::Mat type matrix with the camera distortion coefficients
 */
cv::Mat camera::getDistCoeffs(){
  return this->distCoeffs;
}
/**
 * @brief getImage - provides the image caped from the camera
 * @return  cv_bridge::CvImagePtr type variable  with the image caped from the camera
 */
cv_bridge::CvImagePtr camera::getImage(){
  return  this->image;
}
/**
 * @brief getImageReady - If the camera is capting image returns 1. If not it returns 0.
 * @return char type variable
 */
char camera::getImageReady(){
  return this->imageReady;
}
char camera::getConfigReady(){
  return this->config;
}
/**
 * @brief clearImageReady - Set imageReady variable to 0.
 */
void camera::clearImageReady(){
   this->imageReady=0;
}
