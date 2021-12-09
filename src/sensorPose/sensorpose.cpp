#include "sensorpose.h"

sensorpose::sensorpose(const char *camFrameName){

  this->outPose.header.frame_id=camFrameName;

  strcpy(this->frame_id,camFrameName);

  this->outPose.header.stamp=ros::Time();
  this->outPose.pose.position.x = 0;
  this->outPose.pose.position.y = 0;
  this->outPose.pose.position.z = 0;
  tf::quaternionEigenToMsg(Eigen::Quaterniond(),this->outPose.pose.orientation);
  this->detect.data = false;
}

void sensorpose::setPose(float x, float y, float z, float qx, float qy, float qz, float qw){
  this->outPose.header.stamp=ros::Time();
  this->outPose.pose.position.x = x;
  this->outPose.pose.position.y = y;
  this->outPose.pose.position.z = z;
  this->outPose.pose.orientation.x = qx;
  this->outPose.pose.orientation.y = qy;
  this->outPose.pose.orientation.z = qz;
  this->outPose.pose.orientation.w = qw;
}

void sensorpose::setDetect (bool value){
  this->detect.data = value;
}

geometry_msgs::PoseStamped sensorpose::getPose(){
  return this->outPose;
}

std_msgs::Bool sensorpose::getDetect(){
  return this->detect;
}

char* sensorpose::getFrameId(){
  return this->frame_id;
}
