#ifndef ARUCOCRAS_H
#define ARUCOCRAS_H
/*#####################
  ##     Includes    ##
  #####################*/

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

/**
 * @brief The arucoCras class
 */
class arucoCras{
  private:
    int id;
    float size;
    const char *nameFrame;
    const char *cameraFrame;
    float state;
    geometry_msgs::PoseStamped poseAruco;
    std_msgs::Bool detect;
  public:

    /**
     * @brief arucoCras  - Class constructor
     * @param id - id of aruco (INT)(in)
     * @param size - size of aruco (float)(in)
     * @param nameFrame - Frame name where the aruco is placed on the plataform (const char *nameFrame)(in)
     * @param cameraFrame - Frame name where the camera is placed on the drone (const char *nameFrame)(in)
     */
    arucoCras(int id,float size,const char *nameFrame,const char *cameraFrame);


    /**
     * @brief setArucoPose - set aruco pos with the values post on the function
     * @param x - x cordenate value (float)(in)
     * @param y - y cordenate value (float)(in)
     * @param z - z cordenate value (float)(in)
     * @param state - state 1 if aruco is detect 0 if not (float)(in)
     * @param Q1 - Quaternion value (Eigen::Quaterniond)(in)
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
     * @param img - image to try detect the arucos (cv::Mat)(in)
     * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
     * @param n - number of arucos on that exist (int)(in)
     * @param cameraMatrix - camera matrix (cv::Mat)(in)
     * @param distCoeffs -  camera distorcion values (cv::Mat)(in)
     */
    static void detector(cv::Mat img, arucoCras **aruco, int n, cv::Mat cameraMatrix,cv::Mat distCoeffs,const char *display);


    /**
     * @brief arucoTfPosition given the transform listener it convert the position of an aruco seen by the camera in a
     * relative position between the UAV and the center of the platform
     * @param aruco - arucos that exist in the plataform (arucoCras **)(in)
     * @param sizeVec - number of arucos on the plataform (int)(in)
     * @param listener - transform listener (tf::TransformListener)(in)
     * @param pose - local where the final position will be return (sensorpose *)(out)
     * @param platformBaseLinkName - plataform link name (const char *)(in)
     */
    static void arucoTfPosition(arucoCras **aruco,int sizeVec,tf::TransformListener *listener,sensorpose *pose, const char * platformBaseLinkName);//

    /**
     * @brief detectorId detects the id on the image, and associate the position(x,y,z) given
     * @param img - image to check if is there any aruco (cv::Mat)(in)
     * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
     * @param n - number of arucos on that exist (int)(in)
     * @param pos - position of the aruco that are present in the image
     */
    static void detectorId(cv::Mat img, arucoCras **aruco, int n,float pos[6]);
};

#endif // ARUCOCRAS_H
