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
          the state of the sensors. 

          Several coiches have been made following the advices of t
          he Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° Con.1, Con.2
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10

 *
 */

// Include here the ".h" files corresponding to the topic type you use.
 #include "simulation/robot_base.h"
 #include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations


class Robot_2_0 : public RobotBase {
public:
  Robot_2_0() = default;

  Robot_2_0(const double _trackGauge, const double _wheelRadius) :
            trackGauge(_trackGauge),
            wheelRadius(_wheelRadius),
            RobotBase() {}


  // Function to set the velocieties as the input
  void ComputeInput() const override;

  // Function to compute the generalized coordinates
  void PerformMotion() const override;

  // Function to check the state of the sensors
  void CheckSensorStatus() const override;

  // Function to publish the robot position and odometry
  void PrepareMessages() override;

  // Function to update the J matrix elements 
  void UpdateMatrix() const;

  // Function to control the max value of the velocities 
  void EnsureMaxSpeed() const;



private:

  struct GeneralizedCorrdinates  {

    GeneralizedCorrdinates()=default;

    GeneralizedCorrdinates operator=(const GeneralizedCorrdinates& equal);
    //============================ AGGIUNGI REGOLA DEI CINQUE ==============0===
    GeneralizedCorrdinates operator=(const Eigen::VectorXd& result);

    GeneralizedCorrdinates operator+(const GeneralizedCorrdinates& addendum);


    GeneralizedCorrdinates Integrate(const ros::Duration& timeElapsed);

    double x{0.0}, y{0.0}, theta{0.0};

  };

  // Generalized coordinates
  mutable GeneralizedCorrdinates q, q_dot;

  // Robot parameters
  const double trackGauge {0.2} ;           //  [m]
  const double wheelRadius {0.05 };         //  [m]
  const double wMax { 10 };                 //  [RAD/s]
 
  // Input vector
  mutable Eigen::Vector2d u {0.0f, 0.0f} ;  //  [m/s, RAD/s]
  
  // J matrix
  mutable Eigen::MatrixXd J{3, 2} ;




};



void Robot_2_0::ComputeInput() const
{
  u(0) = twistReceived.linear.x;
  u(1) = twistReceived.angular.z;

  EnsureMaxSpeed();

}



void Robot_2_0::PerformMotion() const
{
  UpdateMatrix();

  ROS_INFO_STREAM("Velocities, v : " << u(0) << " omega : " << u(1) ) ;

  q_dot = J*u;

  q = q + q_dot.Integrate(timeElapsed);

}




void Robot_2_0::CheckSensorStatus() const
{
  //ROS_INFO_STREAM("4");
}




void Robot_2_0::PrepareMessages()
{
  robotPosture.pose.position.x = q.x;
  robotPosture.pose.position.y = q.y;

  ROS_INFO_STREAM("Positions, x : " << q.x << " y : " << q.y ) ;
}




void Robot_2_0::UpdateMatrix() const
{
  J <<  cos(q.theta),     0,
        sin(q.theta),     0,
            0,            1;
}




void Robot_2_0::EnsureMaxSpeed() const
{
  Eigen::Matrix2d F;
  F <<        wheelRadius/2,                  wheelRadius/2,
        wheelRadius/(2*trackGauge),   -wheelRadius/(2*trackGauge) ;

  Eigen::Vector2d phi_dot = F*u;

  const double scaleFactor = std::max(phi_dot(0), phi_dot(1)) > wMax ? std::max(phi_dot(0), phi_dot(1)) : 1 ;

  phi_dot /= scaleFactor;

  u = F.inverse()*phi_dot ;
}


// Genralized coordinates' functions and operatos

Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator=(const Eigen::VectorXd& result)
{

  this->x = result(0) ;
  this->y = result(1) ;
  this->theta = result(2) ;

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




Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator+(const GeneralizedCorrdinates& addendum)
{
  this->x += addendum.x ;
  this->y += addendum.y ;
  this->theta += addendum.theta ;

  return (*this) ;
}




Robot_2_0::GeneralizedCorrdinates Robot_2_0::GeneralizedCorrdinates::operator=(const GeneralizedCorrdinates& equal)
{
  this->x = equal.x ;
  this->y = equal.y ;
  this->theta = equal.theta ;

  return (*this) ;
}



#endif //ROBOT_2_0_H
