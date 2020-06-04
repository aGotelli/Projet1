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
          ° All the SF section.


 *
 */


//ROS
 #include <ros/ros.h>

#include "simulation/utility.h"

// Include here the ".h" files corresponding to the topic type you use.
 #include <geometry_msgs/Twist.h>
 #include <visualization_msgs/Marker.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <nav_msgs/Odometry.h>
 #include <tf/transform_broadcaster.h>
 #include <sensor_msgs/JointState.h>

 #include <simulation_messages/Encoders.h>
 #include <simulation_messages/IRSensors.h>
 #include <simulation_messages/SensorStatus.h>



class RobotBase {
public:

  RobotBase()=default;    //  Initialize all memeber to their default value i.e.
                          //  what is inside the {}

  virtual void isMoving();

protected:

  //  Virtual Callback
  inline virtual void TwistReceived(const geometry_msgs::Twist::ConstPtr& twist) { twistReceived = (*twist) ;}

  virtual void ComputeInput() const=0;

  virtual void PerformMotion() const=0;

  virtual void ComputeOdometry() const=0;

  virtual void PrepareMessages()=0;

  simulation_messages::Encoders wheelsAngles; //  No need of default initialization
  simulation_messages::IRSensors status;      //  No need of default initialization
  geometry_msgs::PoseStamped robotPosture;    //  No need of default initialization
  geometry_msgs::PoseStamped odomPosture;     //  No need of default initialization
  geometry_msgs::Twist twistReceived;         //  No need of default initialization
  tf::Transform movingPlatformFrame;

  //visualization_msgs::Marker robotMarker;     //  No need of default initialization
  visualization_msgs::Marker generatedPath;   //  No need of default initialization

  // Time handling
  ros::Time prevTime;
  ros::Time currentTime;
  ros::Duration timeElapsed;


  // jointstate for castor wheel
  sensor_msgs::JointState actuations;

  geometry_msgs::TransformStamped geometryStamp;



private:

  //Node definition
  ros::NodeHandle nh_glob;                    //  No need of default initialization

  ros::Rate robotFrameRate {150};

  //Publisher and subscriber definition
  ros::Subscriber commandReceived { nh_glob.subscribe<geometry_msgs::Twist>("/TwistToRobot", 1, &RobotBase::TwistReceived, this) } ;

  ros::ServiceClient sensorsServer { nh_glob.serviceClient<simulation_messages::SensorStatus>("CheckSensorStatus") } ;

  ros::Publisher RobotMarker { nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker", 1) } ;

  ros::Publisher Robot { nh_glob.advertise<geometry_msgs::PoseStamped>("RobotPosture", 1) } ;
  ros::Publisher Encoders { nh_glob.advertise<simulation_messages::Encoders>("EncodersReading", 1) } ;
  ros::Publisher IRSensors { nh_glob.advertise<simulation_messages::IRSensors>("IRSensorsStatus", 1) } ;
  ros::Publisher Odometry { nh_glob.advertise<nav_msgs::Odometry>("RobotOdometry", 10) } ;

  tf::TransformBroadcaster br;

  ros::Publisher jointsController { nh_glob.advertise<sensor_msgs::JointState>("/joint_states", 1) } ;

};




void RobotBase::isMoving()
{

  //utility::InitMarker( robotMarker ) ;

  utility::InitLineStrip( generatedPath ) ;

  prevTime = ros::Time::now() ;

  while (ros::ok()){

      ros::spinOnce();

      //  First account the time elapsed in between two loops
      currentTime = ros::Time::now() ;
      timeElapsed = currentTime - prevTime;
      prevTime = currentTime ;
      //  Then perform all the computations

      //  First compute the input for the robot actuators
      ComputeInput() ;

      //  Then compute the matematical integration (displacement)
      //  that the robot performed due to the input
      PerformMotion() ;

      //  Compute the odometry to show
      ComputeOdometry();

      //  Elaborate the data for being published
      PrepareMessages() ;
/*
      br.sendTransform(tf::StampedTransform(movingPlatformFrame,
                                                  ros::Time::now(),
                                                  "map", "moving_platform"));

*/
      //  Update the marker position for visualization
      //utility::UpdateMarker( robotPosture, robotMarker ) ;

      //  Update the line strip for visualization
      utility::UpdatePath( robotPosture, generatedPath ) ;

      //  Publish current robot posture
      Robot.publish( robotPosture );

      //  Publish current wheels orientations
      Encoders.publish( wheelsAngles );

      //  Publish the marker for visualization
      //RobotMarker.publish( robotMarker ) ;

      //  Publish the line strip for visualization
      RobotMarker.publish( generatedPath ) ;

      //  Publish the joint state
      jointsController.publish( actuations ) ;

/*


      geometryStamp.header.stamp = currentTime;
      geometryStamp.header.frame_id = "map";
      geometryStamp.child_frame_id = "moving_platform";
      geometryStamp.transform.translation.x = robotPosture.pose.position.x ;
      geometryStamp.transform.translation.y = robotPosture.pose.position.y ;
      geometryStamp.transform.translation.z = robotPosture.pose.position.z ;

      geometryStamp.transform.rotation = robotPosture.pose.orientation;

      br.sendTransform( geometryStamp );





    //  Declare the message
    nav_msgs::Odometry robotOdometry;

    //  Stamp the current time
    robotOdometry.header.stamp = currentTime;

    //  Set the frames
    robotOdometry.header.frame_id = "map";
    robotOdometry.child_frame_id = "moving_platform";

    //  Set the position
    robotOdometry.pose.pose = odomPosture.pose;

    //  Publish the computed odometry
    Odometry.publish( robotOdometry ) ;

*/
      //  Wait for next iteration
      robotFrameRate.sleep();

  }
}





#endif //ROBOT_BASE_H
