#ifndef ARUCOCRAS_H
#define ARUCOCRAS_H

#include <geometry_msgs/PoseStamped.h>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<eigen_conversions/eigen_msg.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include "../sensorPose/sensorpose.h"
//using namespace cv;

/**
 * @brief The arucoCras class
 */
class arucoCras{
  private:
    int id;
    float size;
    const char *nameFrame;
    float state;
    geometry_msgs::PoseStamped poseAruco;
    std_msgs::Bool detect;
  public:

    /**
     * @brief arucoCras  - Class constructor
     * @param id - id of aruco (INT)
     * @param size - size of aruco (float)
     * @param nameFrame - Frame name where the aruco is placed on the plataform (const char *nameFrame)
     */
    arucoCras(int id,float size,const char *nameFrame);


    /**
     * @brief setArucoPose - set aruco pos with the values post on the function
     * @param x - x cordenate value (float)
     * @param y - y cordenate value (float)
     * @param z - z cordenate value (float)
     * @param state - state 1 if aruco is detect 0 if not (float)
     * @param Q1 - Quaternion value (Eigen::Quaterniond)
     */
    void setArucoPose(float x, float y, float z, float state,const Eigen::Quaterniond &Q1);

    /**
     * @brief getArucoPose - provides the pose of aruco relative to something
     * @return returns the aruco pose (geometry_msgs::PoseStamped)
     */
    geometry_msgs::PoseStamped getArucoPose();

    /**
     * @brief getId - provides the aruco id
     * @return aruco id (int)
     */
    int getId();

    /**
     * @brief getSize - provides the aruco size
     * @return aruco size (float)
     */
    float getSize();


    /**
     * @brief getState - provides the aruco state
     * @return aruco state, 0 if not detect 1 if is detect (float)
     */
    float getState();

    /**
     * @brief getNameFrame - provides the aruco name framee
     * @return aruco name frame (const char *)
     */
    const char *getNameFrame();

    /**
     * @brief detector given an image it detects the arucos an estimate the position,
     * then stores on the variable aruco passe on the function.
     * @param img - image to try detect the arucos (cv::Mat)
     * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)
     * @param n - number of arucos on that exist (int)
     * @param cameraMatrix - camera matrix (cv::Mat)
     * @param distCoeffs -  camera distorcion values (cv::Mat)
     */
    static void detector(cv::Mat img, arucoCras **aruco, int n, cv::Mat cameraMatrix,cv::Mat distCoeffs);


    /**
     * @brief arucoTfPosition
     * @param aruco
     * @param sizeVec
     * @param listener
     * @param pose
     * @param platformBaseLinkName
     */
    static void arucoTfPosition(arucoCras **aruco,int sizeVec,tf::TransformListener *listener,sensorpose *pose, const char * platformBaseLinkName);//

};

#endif // ARUCOCRAS_H
