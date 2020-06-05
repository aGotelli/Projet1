#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H

/**
 * \file robot 2_0 file
 * \brief contains the model of the robot
 * \author Bianca & Andrea
 * \version 0.3
 * \date 05/06/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
            This file contains all the functions related to the simulation of the
          robot motion. The aim is to get information regarding the robot position
          and velocity and regarding.

            First the input is computed from the twist message, and the max speed
          of the wheels in ensured to both. Then with the knowledge of the input,
          the matrix which represents the kinematic model is updated and used to
          obain the derivative of the generalized robot coordinates.

            Once the derivative of the robot generalized coordinates is computed,
          it is necessary to perform and integration over the time elapsed from
          the last one (so it is more like to compute a displacement). Then the
          displacement is added to the current value so all the coordinates are
          updated.

            Morover, in this file it is computed the odometry. Starting from the
          current value of phi angles of the two fixed wheels, the elementary
          rotation of the two wheel between two instant of times are computed
          taking into account the wheel resolution. In this way, the discretized
          input is obtained and,through it, the odometry coordinates are updated.

          Several coiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° Con.1, Con.2
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10

 *
 */


#include "simulation/robot_base.h"
#include "simulation/robot_2_0_generalizedcoord.h"

#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations



class Robot_2_0 : public RobotBase {
public:
  Robot_2_0()=default;

  Robot_2_0(const double _frontAxle, const double _wheelRadius,
            const double _jointOffSet, const double _castorArmLength,
            const double _wMax) :
            trackGauge(2*_frontAxle),
            wheelRadius(_wheelRadius),
            jointOffSet(_jointOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            RobotBase() { /* All the rest is initialized as default */
              ROS_DEBUG_STREAM("User-defined initialization called..."); }

  Robot_2_0(const utility::Pose2D& initial, const double _frontAxle,
            const double _wheelRadius, const double _jointOffSet,
            const double _castorArmLength, const double _wMax, const double _resolution) :
            q( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            q_odom( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            trackGauge(2*_frontAxle),
            wheelRadius(_wheelRadius),
            jointOffSet(_jointOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            encoder( Encoders(_resolution) ),
            RobotBase() {/* All the rest is initialized as default */
              ROS_DEBUG_STREAM("Fully user-defined initialization called...");     }



  // Function to set the velocieties as the input
  void ComputeInput() const override;

  // Function to compute the generalized coordinates
  void PerformMotion() const override;

  // Function to publish the robot position, the odometry, the joint state
  // message and the dots of the fixed wheels
  void PrepareMessages() override;

  // Function to update the J matrix elements
  void UpdateMatrix() const;

  // Function to control the max value of the velocities
  void EnsureMaxSpeed() const;

  // Function to compute the odometry
  void ComputeOdometry() const;



private:

  class Encoders {
  public:
    Encoders()=default;

    Encoders(const double _resolution) : resolution(_resolution) {
      ROS_INFO_STREAM("resolution : " << resolution );
    }

    inline const double ResolutionToRad() const {return resolution*M_PI/180; }
    inline const double Resolution() const {return resolution; }

  private:
    //  the resolution of the encoder
    const double resolution{0.1} ;   //  [dot/grad]

  };

  // Generalized coordinates
  mutable robot_2_0::GeneralizedCoordinates q, q_dot;

  // Generalized coordinates for odometry
  mutable robot_2_0::GeneralizedCoordinates q_odom, q_dot_odom;

  const Encoders encoder;

  // Robot parameters
  const double trackGauge {0.4} ;              //  [m]
  const double wheelRadius {0.05 };           //  [m]
  const double jointOffSet {0.4 };            //  [m]
  const double castorArmLength {0.15 };       //  [m]
  const double wMax { 10 };                   //  [RAD/s]

  // Input vector
  mutable Eigen::Vector2d u {0.0f, 0.0f} ;  //  [m/s, RAD/s]

  mutable Eigen::MatrixXd S{7, 2} ;         //  The kinematic model


  mutable Eigen::Vector2d currentReading{0, 0};
  mutable Eigen::Vector2d previusReading{0, 0};
};




///////////////////////////////////////////~ FUNCTIONS DECLARATION ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void Robot_2_0::ComputeInput() const
{
  UpdateMatrix();

  u(0) = twistReceived.linear.x;
  u(1) = twistReceived.angular.z;

  EnsureMaxSpeed();

}



void Robot_2_0::PerformMotion() const
{

  q_dot = S*u;

  q = q + q_dot.Integrate(timeElapsed);

}


void Robot_2_0::ComputeOdometry() const
{

  // Current value of phi_1f and phi_2f
  currentReading  = ( Eigen::Vector2d(q.phi_1f, q.phi_2f)*180/M_PI )*encoder.Resolution() ;
  currentReading[0] = std::floor( currentReading[0] ) ;
  currentReading[1] = std::floor( currentReading[1] ) ;

  Eigen::Vector2d rotation = ( (currentReading - previusReading)/encoder.Resolution() )*M_PI/180 ;

  if( isnan(rotation[0]))
    rotation[0] = 0;

  if( isnan(rotation[1]))
    rotation[1] = 0;

  // Compute input discretized
  const Eigen::Vector2d d_input = S.block<2,2>(4,0).inverse()*rotation ;

  // Compute odometry position
  q_odom.x = q_odom.x + d_input[0]*cos(q_odom.theta) ;
  q_odom.y = q_odom.y + d_input[0]*sin(q_odom.theta) ;
  q_odom.theta = q_odom.theta + d_input[1] ;

  previusReading = currentReading ;

}



void Robot_2_0::UpdateMatrix() const
{

  S <<            cos(q.theta)          ,                                    0                                ,
                  sin(q.theta)          ,                                    0                                ,
                      0                 ,                                    1                                ,
        -sin(q.beta_3c)/castorArmLength ,    -(castorArmLength + jointOffSet*cos(q.beta_3c))/castorArmLength  ,
                  1/wheelRadius         ,                        trackGauge/(2*wheelRadius)                   ,
                  1/wheelRadius         ,                       -trackGauge/(2*wheelRadius)                   ,
          -cos(q.beta_3c)/wheelRadius   ,              sin(q.beta_3c)*jointOffSet/wheelRadius                 ;

}




void Robot_2_0::EnsureMaxSpeed() const
{

  Eigen::Vector2d phi_dot = S.block<2,2>(4,0)*u;


  const double scaleFactor = Eigen::Vector2d(phi_dot[0], phi_dot[1]).cwiseAbs().maxCoeff() > wMax ?
                              Eigen::Vector2d(phi_dot[0], phi_dot[1]).cwiseAbs().maxCoeff()/wMax : 1 ;

  phi_dot /= scaleFactor;

  u = S.block<2,2>(4,0).inverse()*phi_dot ;

}


void Robot_2_0::PrepareMessages()
{

  // Time handling for robot posture
  robotPosture.header.stamp = currentTime ;

  // Publish the current robot posture
  robotPosture.pose.position.x = q.x ;
  robotPosture.pose.position.y = q.y ;
  robotPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q.theta) ;


  // Publish the current odometry position
  odomPosture.pose.position.x = q_odom.x ;
  odomPosture.pose.position.y = q_odom.y ;
  odomPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q_odom.theta) ;


  //  Publish the current reading in dots
  elapsedDots.phi_1f = currentReading[0] ;
  elapsedDots.phi_2f = currentReading[1] ;


  //  Clear the joint state message
  actuations.name.clear() ;
  actuations.position.clear() ;

  //  Set the joint state message new values
  actuations.header.stamp = ros::Time::now() ;
  actuations.name.push_back("move_along_x") ;
  actuations.position.push_back(q.x) ;
  actuations.name.push_back("move_along_y") ;
  actuations.position.push_back(q.y) ;
  actuations.name.push_back("heading") ;
  actuations.position.push_back(q.theta) ;
  actuations.name.push_back("castor_joint") ;
  actuations.position.push_back(q.beta_3c) ;
  actuations.name.push_back("fixed_wheel_1_joint") ;
  actuations.position.push_back(q.phi_1f) ;
  actuations.name.push_back("fixed_wheel_2_joint") ;
  actuations.position.push_back(q.phi_2f) ;
  actuations.name.push_back("castor_wheel_joint") ;
  actuations.position.push_back(q.phi_3c) ;


}


#endif //ROBOT_2_0_H
