#include <ros/ROS.h>
#include <geometry_msgs/Pose.h>





geometry_msgs::Pose realPose;
void RealPoseCallBack(const geometry_msgs::Pose::ConstPtr& _realPose)
{
  realPose = (*_realPose);
}
