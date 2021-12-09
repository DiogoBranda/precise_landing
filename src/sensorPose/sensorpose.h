#ifndef SENSORPOSE_H
#define SENSORPOSE_H
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<tf/tf.h>
#include<tf_conversions/tf_eigen.h>
#include<eigen_conversions/eigen_msg.h>

class sensorpose
{

private:
  geometry_msgs::PoseStamped outPose;
  std_msgs::Bool detect;
  char  frame_id [50];
public:
  sensorpose(const char *camFrameName);
  geometry_msgs::PoseStamped getPose();
  std_msgs::Bool getDetect();
  void setPose(float x, float y, float z, float qx, float qy, float qz, float qw);
  char* getFrameId();
  void setDetect(bool value);
};

#endif // SENSORPOSE_H
