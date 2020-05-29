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
 *    ° /visualization_marker
 *    ° RobotPosture
 *    ° RobotOdometry
 *    ° IRSensorsStatus
 *
 * Description
          The aim of this file is to embed all the ros related functions, i.e.
          all the publishers, subscribers and operations. The idea is to separate
          what is related to the kinematics of the robot and what is needed to
          make the simulation to work.

          The function that have to be implemented are declared as pure virtual.
          The reason is that they have to be declared accordingly with the robot
          kinematics but they should be a RobotBase members functions, in order
          to be called in the isMoving() member function. Without any surprise,
          this memeber function executes all the others needed to make the
          simulation possible and accurate.

          The choice of have multiple member function was completely albitrary,
          as well as other assumptions in this code. However we were inspired by
          the Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
          ° C.2, C.48, C.120, C.128, C.132
          ° All the Nl section, especially NL.16 and NL.17 but not the NL.10
          ° F.1, F.2, F.3, F.15, F.16, F.17


 *
 */

 #include "simulation/utility.h"

 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <visualization_msgs/Marker.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <simulation_messages/Encoders.h>
 #include <simulation_messages/IRSensors.h>


class RobotBase {
public:

  RobotBase()=default;    //  Initialize all memeber to their default value i.e.
                          //  what is inside the {}

  void isMoving();

protected:

  //  Virtual Callback
  inline void TwistReceived(const geometry_msgs::Twist::ConstPtr& twist) { twistReceived = (*twist) ;}

  virtual void ComputeInput() const=0;

  virtual void PerformMotion() const=0;

  virtual void CheckSensorStatus() const=0;

  virtual void PrepareMessages()=0;

  simulation_messages::Encoders wheelsAngles; //  No need of default initialization
  simulation_messages::IRSensors status;      //  No need of default initialization
  geometry_msgs::PoseStamped robotPosture;    //  No need of default initialization
  geometry_msgs::Twist twistReceived;         //  No need of default initialization

  visualization_msgs::Marker robotMarker;     //  No need of default initialization

  ros::Time prevTime;
  ros::Time currentTime;
  ros::Duration timeElapsed;

private:

  ros::NodeHandle nh_glob;                    //  No need of default initialization

  ros::Rate robotFrameRate {100};

  ros::Subscriber commandReceived { nh_glob.subscribe<geometry_msgs::Twist>("/TwistToRobot", 1, &RobotBase::TwistReceived, this) } ;

  ros::Publisher RobotMarker { nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker", 1) } ;

  ros::Publisher Robot { nh_glob.advertise<geometry_msgs::PoseStamped>("RobotPosture", 1) } ;
  ros::Publisher Encoders { nh_glob.advertise<simulation_messages::Encoders>("RobotOdometry", 1) } ;
  ros::Publisher IRSensors { nh_glob.advertise<simulation_messages::IRSensors>("IRSensorsStatus", 1) } ;

};




void RobotBase::isMoving()
{

  utility::InitMarker( robotMarker ) ;

  prevTime = ros::Time::now() ;

  while (ros::ok()){

      ros::spinOnce();

      //  First account the time elapsed in between two loops
      currentTime = ros::Time::now() ;
      timeElapsed = currentTime - prevTime;
      prevTime = currentTime ;
      //  Then perform all the computations


      ComputeInput() ;

      PerformMotion() ;

      CheckSensorStatus() ;

      PrepareMessages() ;

      utility::UpdateMarker( robotMarker, robotPosture ) ;



      Robot.publish( robotPosture );

      Encoders.publish( wheelsAngles );

      IRSensors.publish( status );

      RobotMarker.publish( robotMarker ) ;


      robotFrameRate.sleep();
  }
}





#endif //ROBOT_BASE_H
