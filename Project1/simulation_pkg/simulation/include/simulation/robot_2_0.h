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
          of the wheels is ensured to both. Then with the knowledge of the input,
          the matrix which represents the kinematic model is updated and used to
          obtain the derivative of the generalized robot coordinates.

            Once the derivative of the robot generalized coordinates is computed,
          it is necessary to perform an integration over the time elapsed from
          the last one (so it is more like to compute a displacement). Then the
          displacement is added to the current value so all the coordinates are
          updated.

            Morover, in this file it is computed the odometry. Starting from the
          current value of phi angles of the two fixed wheels, the elementary
          rotation of the two wheels between two instants of time are computed
          taking into account the wheel resolution. In this way, the discretized
          input is obtained and, through it, the odometry coordinates are updated.

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
            const double _trailingOffSet, const double _castorArmLength,
            const double _wMax) :
            trackGauge(2*_frontAxle),
            wheelRadius(_wheelRadius),
            trailingOffSet(_trailingOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            RobotBase() { /* All the rest is initialized as default */
              ROS_DEBUG_STREAM("User-defined initialization called..."); }

  Robot_2_0(const utility::Pose2D& initial, const double _frontAxle,
            const double _wheelRadius, const double _trailingOffSet,
            const double _castorArmLength, const double _wMax,
            const double _resolution, const double _wheel1Error,
            const double _trackGaugeError) :
            q( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            q_odom( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            trackGauge(2*_frontAxle),
            wheelRadius(_wheelRadius),
            trailingOffSet(_trailingOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            encoder( Encoders(_resolution) ),
            wheel1Error(_wheel1Error),
            trackGaugeError(_trackGaugeError),
            RobotBase() {/* All the rest is initialized as default */
              ROS_INFO_STREAM("trackGaugeError : " << trackGaugeError );
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

  void UpdateOdomMatrix() const;

  // Function to control the max value of the velocities
  void EnsureMaxSpeed() const;

  // Function to compute the odometry
  void ComputeOdometry() const;



private:

  class Encoders {
  public:
    Encoders()=default;

    Encoders(const double _resolution) : resolution(_resolution) {
      ROS_DEBUG_STREAM("resolution : " << resolution );
    }

    inline const double ResolutionToRad() const {return resolution/(2*M_PI); }
    inline const double Resolution() const {return resolution; }


  private:
    //  the resolution of the encoder
    const double resolution{360} ;   //  [dots/revolution]

  };

  // Generalized coordinates
  mutable robot_2_0::GeneralizedCoordinates q, q_dot;

  // Generalized coordinates for odometry
  mutable robot_2_0::GeneralizedCoordinates q_odom, q_dot_odom;

  const Encoders encoder;

  const double wheel1Error     { 0.0 };
  const double trackGaugeError { 0.0 };

  // Robot parameters
  const double trackGauge {0.4} ;             //  [m]
  const double wheelRadius {0.05 };           //  [m]
  const double trailingOffSet {0.4 };         //  [m]
  const double castorArmLength {0.15 };       //  [m]
  const double wMax { 10 };                   //  [RAD/s]

  // Input vector
  mutable Eigen::Vector2d u {0.0f, 0.0f} ;  //  [m/s, RAD/s]

  mutable Eigen::MatrixXd S{7, 2} ;         //  The kinematic model

  mutable Eigen::MatrixXd S_odom{7, 2} ;    //  The kinematic model for the odometry


  mutable Eigen::Vector2d currentReading{0, 0};
  mutable Eigen::Vector2d previusReading{0, 0};

  mutable Eigen::Vector2d currentAngles{0, 0};
  mutable Eigen::Vector2d previusAngles{0, 0};
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

  q = q + (*q_dot.Integrate(timeElapsed)) ;

}


// compute odometry with error in wheel radius and track gauge
void Robot_2_0::ComputeOdometry() const
{
  UpdateOdomMatrix();


  // Current value of phi_1f and phi_2f
  currentReading  = ( Eigen::Vector2d(q.phi_1f, q.phi_2f) )*encoder.ResolutionToRad() ;
  currentReading[0] = std::floor( currentReading[0] ) ;
  currentReading[1] = std::floor( currentReading[1] ) ;

  //  Define the wheels rotation in between two iterations
  Eigen::Vector2d rotation = ( (currentReading - previusReading)/encoder.ResolutionToRad() );


  Eigen::Matrix2d imperfection;
  imperfection <<  1/(wheelRadius*wheel1Error)     ,      (trackGauge*trackGaugeError)/(2*(wheelRadius*wheel1Error))         ,
                        1/wheelRadius              ,            -(trackGauge*trackGaugeError)/(2*wheelRadius)                ;


  // Compute input discretized
  const Eigen::Vector2d d_input = imperfection.inverse()*rotation ;


  q_odom = q_odom + S_odom*d_input;

  // Update
  previusReading = currentReading ;


}



void Robot_2_0::UpdateMatrix() const
{

  S <<            cos(q.theta)          ,                                    0                                   ,
                  sin(q.theta)          ,                                    0                                   ,
                      0                 ,                                    1                                   ,
        -sin(q.beta_3c)/castorArmLength ,    -(castorArmLength + trailingOffSet*cos(q.beta_3c))/castorArmLength  ,
                  1/wheelRadius         ,                        trackGauge/(2*wheelRadius)                      ,
                  1/wheelRadius         ,                       -trackGauge/(2*wheelRadius)                      ,
          -cos(q.beta_3c)/wheelRadius   ,              sin(q.beta_3c)*trailingOffSet/wheelRadius                 ;

}


void Robot_2_0::UpdateOdomMatrix() const
{

  S_odom <<            cos(q_odom.theta)          ,                                         0                                   ,
                       sin(q_odom.theta)          ,                                         0                                   ,
                              0                   ,                                         1                                   ,
             -sin(q_odom.beta_3c)/castorArmLength ,    -(castorArmLength + trailingOffSet*cos(q_odom.beta_3c))/castorArmLength  ,
                        1/wheelRadius             ,                             trackGauge/(2*wheelRadius)                      ,
                        1/wheelRadius             ,                            -trackGauge/(2*wheelRadius)                      ,
               -cos(q_odom.beta_3c)/wheelRadius   ,                  sin(q_odom.beta_3c)*trailingOffSet/wheelRadius             ;

}




void Robot_2_0::EnsureMaxSpeed() const
{
  //  Take the block, in the matrix S, related to the motorization
  Eigen::Vector2d phi_dot = S.block<2,2>(4,0)*u;

  //  Take the amount of saturation in wheels speed
  const double scaleFactor = Eigen::Vector2d(phi_dot[0], phi_dot[1]).cwiseAbs().maxCoeff() > wMax ?
                              Eigen::Vector2d(phi_dot[0], phi_dot[1]).cwiseAbs().maxCoeff()/wMax : 1 ;

  //  Scale the current velocity to be equal to the max (at the most)
  phi_dot /= scaleFactor;

  //  Use the newly computed wheels speed to obtain a feasible input
  u = S.block<2,2>(4,0).inverse()*phi_dot ;

}


void Robot_2_0::PrepareMessages()
{

  // Update the current robot posture
  robotPosture.position.x = q.x ;
  robotPosture.position.y = q.y ;
  robotPosture.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q.theta) ;


  //  Time handling for odometry posture
  robotOdometry.header.stamp = currentTime;

  //  Set the frames
  robotOdometry.header.frame_id = "map";
  robotOdometry.child_frame_id = "moving_platform";

  //  Update the position
  robotOdometry.pose.pose.position.x = q_odom.x ;
  robotOdometry.pose.pose.position.y = q_odom.y ;
  robotOdometry.pose.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q_odom.theta) ;


  //  Publish the current reading in dots
  wheelsRotations.phi_1f = q_odom.phi_1f ;
  wheelsRotations.phi_2f = q_odom.phi_2f ;


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
