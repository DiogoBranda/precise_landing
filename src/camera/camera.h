#ifndef ROSCAM_H
#define ROSCAM_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <math.h>       /* sqrt */
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>


//using namespace cv;
/**
 * @brief - The camera class contains the camera parameters values and distortion coefficients.
 * Furthermore it provides the image capture by the camera.
 */
class camera
{

private:
  ros::Subscriber camera_pam;
  ros::Subscriber camera_sub;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  char imageReady =0;
  char config = 0;
  cv_bridge::CvImagePtr image;


public:

  /**
   * @brief cam_par - callback function use to save the camera parameters
   * @param camInfo_msg
   */
  void cb_camPar (const sensor_msgs::CameraInfoConstPtr& camInfo_msg);
  /**
   * @brief cam_img - callback function use to save the image provides by the camera
   * @param image_msg
   */
  //void cb_camImg(const sensor_msgs::Image::ConstPtr & image_msg);
  void cb_camImg(const sensor_msgs::CompressedImage::ConstPtr & image_msg);

  /**
   * @brief camera - constructor
   * @param cameImgName - name of the topic that publish the camera image
   * @param cameInfoName - name of the topic that publish the camera info
   * @param n - Ros Node
   */
  camera();
  /**
   * @brief getCameraMatrix - provides the camera parameters
   * @return cv::Mat type matrix with the camera parameters
   */
  cv::Mat getCameraMatrix();
  /**
   * @brief getDistCoeffs - provides the camera distortion coefficients
   * @return cv::Mat type matrix with the camera distortion coefficients
   */
  cv::Mat getDistCoeffs();
  /**
   * @brief getImage - provides the image caped from the camera
   * @return  cv_bridge::CvImagePtr type variable  with the image caped from the camera
   */
  cv_bridge::CvImagePtr getImage();
  /**
   * @brief getImageReady - If the camera is capting image returns 1. If not it returns 0.
   * @return char type variable
   */
  char getImageReady();

  /**
   * @brief clearImageReady - Set imageReady variable to 0.
   */
  void clearImageReady();

  /**
   * @brief getImageReady - If the camera parameters are ready returns 1. If not it returns 0.
   * @return char type variable
   */
  char getConfigReady();
};

#endif // ROSCAM_H
