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
          this member function executes all the others needed to make the
          simulation possible and accurate.

            This class is mostly an interface. As for definition, it should
          contain only virtual member function and no member elements. The
          functions provided are all virtual, allowing some user to change them
          accordingly with the robot type. The memeber function isMoving()
          should not be modified, as it represents a coherent developing of the
          computation. However, if a user wants to costumize it, there is this
          possibility.

            The choice of having multiple member function was completely arbitrary,
          as well as other assumptions in this code. However, we were inspired by
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
 #include <sensor_msgs/JointState.h>

 #include <simulation_messages/Encoders.h>




class RobotBase {
public:

  RobotBase()=default;    //  Initialize all memeber to their default value i.e.
                          //  what is inside the {}

  virtual ~RobotBase()=default;

  virtual void isMoving();

protected:

  //  Virtual Callback
  inline virtual void TwistReceived(const geometry_msgs::Twist::ConstPtr& twist) { twistReceived = (*twist) ;}

  //  Function to convert the received twist from the controller into the
  //  input defined for the robot which is implemented
  virtual void ComputeInput() const=0;

  //  This function contains all the computations related to the robot kinematic
  virtual void PerformMotion() const=0;

  //  This function computed the encoders reading and create a posture from the
  //  computed data, to make it possible to check the computations consistency.
  virtual void ComputeOdometry() const=0;

  //  This function is meant to prepare the messages to be published. In other
  //  words, in this function the data obtained in the cycle are copied into
  //  the messages just before they are published.
  virtual void PrepareMessages()=0;


  //  The dots readed by the encoders mounted on the fixed wheels.
  simulation_messages::Encoders elapsedDots;  //  No need of default initialization

  //  The full robot pusture obtained with the kinematic model.
  geometry_msgs::PoseStamped robotPosture;    //  No need of default initialization

  //  The full robot posture obtained from the odometry
  nav_msgs::Odometry robotOdometry;           //  No need of default initialization

  //  The twist received from the controller
  geometry_msgs::Twist twistReceived;         //  No need of default initialization

  //  A merker containing all the robot position, in order to display the path
  //  that has been generated.
  visualization_msgs::Marker generatedPath;   //  No need of default initialization

  // Time handling
  ros::Time prevTime;
  ros::Time currentTime;
  ros::Duration timeElapsed;


  // jointstate for control the robot model posture and configuaration
  sensor_msgs::JointState actuations;


private:

  //Node handle
  ros::NodeHandle nh_glob;                    //  No need of default initialization

  //  The frame rate of this node
  ros::Rate robotFrameRate {150};

  //                    Publisher and subscriber definition

  //  Subscribe to the command received from the controller
  ros::Subscriber CommandReceived { nh_glob.subscribe<geometry_msgs::Twist>("/TwistToRobot", 1, &RobotBase::TwistReceived, this) } ;

  //  Publish the marker that has been generated
  ros::Publisher ShowMarker { nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker", 1) } ;

  //  Publish the robot posture as message
  ros::Publisher Robot { nh_glob.advertise<geometry_msgs::PoseStamped>("RobotPosture", 1) } ;

  //  Publish the encoders reading
  ros::Publisher Encoders { nh_glob.advertise<simulation_messages::Encoders>("EncodersReading", 1) } ;

  //  Publish the Odometry that has been computed
  ros::Publisher Odometry { nh_glob.advertise<nav_msgs::Odometry>("RobotOdometry", 10) } ;

  //  Publish a joint state message to control the URDF model
  ros::Publisher JointsController { nh_glob.advertise<sensor_msgs::JointState>("/joint_states", 1) } ;

};




void RobotBase::isMoving()
{

  //  Initialize the line of the path before proceeding
  utility::InitLineStrip( generatedPath ) ;

  //  Initialize the time
  prevTime = ros::Time::now() ;

  while (ros::ok()){
      //  Obtain the published messages
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

      //  Update the line strip for visualization
      utility::UpdatePath( robotPosture, generatedPath ) ;

      //  Publish current robot posture
      Robot.publish( robotPosture );

      //  Publish current wheels orientations
      Encoders.publish( elapsedDots );

      //  Publish the line strip for visualization
      if( generatedPath.points.size() >= 2)
        ShowMarker.publish( generatedPath ) ;

      //  Publish the joint state
      JointsController.publish( actuations ) ;

      //  Publish the computed odometry
      Odometry.publish( robotOdometry ) ;

      //  Wait for next iteration
      robotFrameRate.sleep();

  }
}





#endif //ROBOT_BASE_H
