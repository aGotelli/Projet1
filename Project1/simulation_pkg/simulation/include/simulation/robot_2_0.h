#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H

/**
 * \file robot 2_0 file
 * \brief contains the model of the robot
 * \author Bianca & Andrea
 * \version 0.1
 * \date 28/05/2020
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
          This file contains all the functions related to the simulation of the robot motion.
          The aim is to get information regarding the robot position and velocity and regarding
          the state of the sensors. So the kinematic of the robot is defined by the functions
          and method implemented here.

          Several coiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° Con.1, Con.2
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10

 *
 */


#include "simulation/robot_base.h"
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations




class Robot_2_0 : public RobotBase {
public:
  Robot_2_0()=default;

  Robot_2_0(const double _trackGauge, const double _wheelRadius,
            const double _jointOffSet, const double _castorArmLength,
            const double _wMax) :
            trackGauge(_trackGauge),
            wheelRadius(_wheelRadius),
            jointOffSet(_jointOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            RobotBase() { /* All the rest is initialized as default */
              ROS_INFO_STREAM("User-defined initialization called..."); }


  // Function to set the velocieties as the input
  void ComputeInput() const override;

  // Function to compute the generalized coordinates
  void PerformMotion() const override;

  // Function to publish the robot position and odometry
  void PrepareMessages() override;

  // Function to update the J matrix elements
  void UpdateMatrix() const;

  // Function to control the max value of the velocities
  void EnsureMaxSpeed() const;

  // Function to compute the odometry
  void ComputeOdometry() const;



private:

  friend class World;

  //  Powerfull structure for kinematic computations
  struct GeneralizedCorrdinates  {

    GeneralizedCorrdinates()=default;

    GeneralizedCorrdinates operator=(const GeneralizedCorrdinates& equal);
    //============================ AGGIUNGI REGOLA DEI CINQUE ==================
    GeneralizedCorrdinates operator=(const Eigen::VectorXd& result);

    GeneralizedCorrdinates operator+(const GeneralizedCorrdinates& addendum);


    GeneralizedCorrdinates Integrate(const ros::Duration& timeElapsed);

    double x{0.0}, y{0.0}, theta{0.0};

    double phi_1f{0.0}, phi_2f{0.0}, phi_3c{0.0};

    double beta_3c{0.0};

  };

  //  Matrix initialization
  const Eigen::Matrix2d InitMotorizationMatrix()
  {
    Eigen::Matrix2d f;

    f <<        wheelRadius/2,                  wheelRadius/2,
          wheelRadius/(2*trackGauge),   -wheelRadius/(2*trackGauge);

    return f;
  }

  // Generalized coordinates
  mutable GeneralizedCorrdinates q, q_dot;


  // Generalized coordinates for odometry
  mutable GeneralizedCorrdinates q_odom, q_dot_odom;

  // Robot parameters
  const double trackGauge {0.2} ;           //  [m]
  const double wheelRadius {0.05 };         //  [m]
  const double jointOffSet {0.4 };           //  [m]
  const double castorArmLength {0.08 };      //  [m]
  const double wMax { 10 };                 //  [RAD/s]

  // Input vector
  mutable Eigen::Vector2d u {0.0f, 0.0f} ;  //  [m/s, RAD/s]

  mutable Eigen::MatrixXd J{7, 2} ;         //  The kinematic model
  
  const Eigen::Matrix2d F { InitMotorizationMatrix() } ;


};





///////////////////////////////////////////~ FUNCTIONS DECLARATION ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void Robot_2_0::ComputeInput() const
{
  u(0) = twistReceived.linear.x;
  u(1) = twistReceived.angular.z;

  EnsureMaxSpeed();

}



void Robot_2_0::PerformMotion() const
{
  UpdateMatrix();

  q_dot = J*u;

  q = q + q_dot.Integrate(timeElapsed);

}




void Robot_2_0::PrepareMessages()
{
  robotPosture.header.stamp = currentTime ;

  robotPosture.pose.position.x = q.x ;
  robotPosture.pose.position.y = q.y ;

  //const auto quat = utility::ToQuaternion( q.theta, 0.0, 0.0 ) ;
  const auto quat = utility::ToQuaternion( utility::EulerAngles(q.theta) ) ;
  robotPosture.pose.orientation.w = quat.w ;
  robotPosture.pose.orientation.x = quat.x ;
  robotPosture.pose.orientation.y = quat.y ;
  robotPosture.pose.orientation.z = quat.z ;


  // Odometry : set the position
  robotOdometry.pose.pose.position.x = x_odom;
  robotOdometry.pose.pose.position.y = y_odom;
  robotOdometry.pose.pose.position.z = 0.0;

  // Odometry : set the orientation
  const auto quat_odom = utility::ToQuaternion( utility::EulerAngles(q_odom.theta) ) ;
  robotOdometry.pose.pose.orientation.x = quat_odom.x;
  robotOdometry.pose.pose.orientation.y = quat_odom.y;
  robotOdometry.pose.pose.orientation.z = quat_odom.z;
  robotOdometry.pose.pose.orientation.w = quat_odom.w;

  // Odometry : set the velocity
  
  robotOdometry.twist.twist.linear.x = q_dot_odom.x;
  robotOdometry.twist.twist.linear.y = q_dot_odom.y;
  robotOdometry.twist.twist.angular.z = q_dot_odom.theta;

  // Odometry : set header 
  robotOdometry.header.stamp = currentTime;
  robotOdometry.header.frame_id = "Odometry";


}




void Robot_2_0::UpdateMatrix() const
{

  J <<            cos(q.theta)          ,                                    0                                ,
                  sin(q.theta)          ,                                    0                                ,
                      0                 ,                                    1                                ,
        -sin(q.beta_3c)/castorArmLength ,    -(castorArmLength + jointOffSet*cos(q.beta_3c))/castorArmLength  ,
                  1/wheelRadius         ,                         trackGauge/wheelRadius                      ,
                  1/wheelRadius         ,                        -trackGauge/wheelRadius                      ,
          cos(q.beta_3c)/wheelRadius    ,              sin(q.beta_3c)*jointOffSet/wheelRadius                 ;


}




void Robot_2_0::EnsureMaxSpeed() const
{

  Eigen::Vector2d phi_dot = F*u;

  const double scaleFactor = std::max(phi_dot(0), phi_dot(1)) > wMax ? std::max(phi_dot(0), phi_dot(1)) : 1 ;

  phi_dot /= scaleFactor;

  u = F.inverse()*phi_dot ;
}




void Robot_2_0::ComputeOdometry() const
{
  // Discretization of phi_1f and phi_2f
  Eigen::Vector2d phi_discretized;
  phi_discretized(0) = std::floor(phi_1f/resolution)*resolution;
  phi_discretized(1) = std::floor(phi_2f/resolution)*resolution;

  // Discretization of the input
  Eigen::Vector2d U_discretized = F.inverse()*phi_discretized;

  // Discretization of the state vector
  q_dot_odom = J*U_discretized;

  // Next iteration
  q_odom = q_odom + q_dot_odom.Integrate(timeElapsed);
}


// Genralized coordinates' functions and operatos
Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator=(const Eigen::VectorXd& result)
{

  this->x       = result(0) ;
  this->y       = result(1) ;
  this->theta   = result(2) ;
  this->beta_3c = result(3) ;
  this->phi_1f  = result(4) ;
  this->phi_2f  = result(5) ;
  this->phi_3c  = result(6) ;

  return (*this) ;
}



Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator=(const GeneralizedCorrdinates& equal)
{
  this->x = equal.x ;
  this->y = equal.y ;
  this->theta = equal.theta ;

  return (*this) ;
}



Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator+(const GeneralizedCorrdinates& addendum)
{
  this->x += addendum.x ;
  this->y += addendum.y ;
  this->theta = utility::LimitAngle( this->theta + addendum.theta ) ;

  return (*this) ;
}



Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::Integrate(const ros::Duration& timeElapsed)
{
  GeneralizedCorrdinates delta;

  //  Function called from a q_dot object, actually x, y, theta represents velocities
  delta.x = x*timeElapsed.toSec() ;
  delta.y = y*timeElapsed.toSec() ;
  delta.theta = theta*timeElapsed.toSec() ;

  return delta;
}



#endif //ROBOT_2_0_H
