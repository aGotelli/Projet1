#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

/**
 * \file robot base file
 * \brief contains all the ros based features
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 28/05/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° /TwistToRobot
 *
 * Publishes to: <BR>
 *    ° /
 *
 * Description
          The aim of this file is to embed all the ros related functions, i.e.
          all the publishers, subscribers and operations. The idea is to separate
          what is related to the kinematics of the robot and what is needed to
          make the simulation to work.

 *
 */

 #include "simulation/utility.h"

 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <visualization_msgs/Marker.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <simulation_messages/Encoders.h>
 #include <simulation_messages/IRSensors.h>


class RobotBase
{
public:
  RobotBase();

  void isMoving();

protected:
  ros::Subscriber commandReceived;

  //  Virtual Callback
  virtual void TwistReceived(const geometry_msgs::Twist::ConstPtr& twist)=0;

  virtual void ComputeInput()=0;

  virtual void PerformMotion()=0;

  virtual void CheckSensorStatus()=0;

  //virtual void PrepareMessages()=0;

  simulation_messages::Encoders wheelsAngles;
  simulation_messages::IRSensors status;
  geometry_msgs::PoseStamped robotPosture;

  visualization_msgs::Marker robotMarker;

private:

  ros::NodeHandle nh_glob;

  ros::Rate robotFrameRate {100};

  ros::Publisher RobotMarker, Robot, Encoders, IRSensors;

};


RobotBase::RobotBase()
{
  commandReceived = nh_glob.subscribe<geometry_msgs::Twist>("/TwistToRobot", 1, &RobotBase::TwistReceived, this) ;

  RobotMarker = nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker", 1) ;

  Robot = nh_glob.advertise<geometry_msgs::PoseStamped>("RobotPosture", 1) ;
  Encoders = nh_glob.advertise<simulation_messages::Encoders>("RobotOdometry", 1) ;
  IRSensors = nh_glob.advertise<simulation_messages::IRSensors>("IRSensorsStatus", 1) ;

}


void RobotBase::isMoving()
{

  utility::InitMarker( robotMarker ) ;

  while (ros::ok()){
      ros::spinOnce();

      ComputeInput() ;

      PerformMotion() ;

      CheckSensorStatus() ;



      Robot.publish( robotPosture );

      Encoders.publish( wheelsAngles );

      IRSensors.publish( status );

      RobotMarker.publish( robotMarker ) ;


      robotFrameRate.sleep();
  }
}





#endif //ROBOT_BASE_H
