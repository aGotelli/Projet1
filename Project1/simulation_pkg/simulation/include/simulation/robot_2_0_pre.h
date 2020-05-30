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

class Robot_2_0;


class Sensor {
public:
 Sensor()=default;

 Sensor(const double _x, const double _y) : HCoord( Eigen::Vector3d(_x, _y, 1.0) ) {}


 inline void setState(bool stateSensor ){ stateSensor = state; }

 inline const bool getState(){ return state; }

 inline const Eigen::Vector3d& Coord() const { return HCoord; }

private:

  // Sensor parameters
  const Eigen::Vector3d HCoord {0.05, 0.0, 1.0};
  mutable bool state {false};


};

class World {
public:
 World()=default;

 const bool CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0& robot) const;

 const bool CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0::GeneralizedCorrdinates& q) const;

private:

 const double xSpacing { 0.2 };
 const double ySpacing { 0.2 };
 const double lineWidth { 0.0 };

};


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

  // Function to check the state of the sensors
  void CheckSensorStatus() const override;

  // Function to publish the robot position and odometry
  void PrepareMessages() override;

  // Function to update the J matrix elements
  void UpdateMatrix() const;

  // Function to control the max value of the velocities
  void EnsureMaxSpeed() const;

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

  // Robot parameters
  const double trackGauge {0.2} ;           //  [m]
  const double wheelRadius {0.05 };         //  [m]
  const double jointOffSet {0.4 };           //  [m]
  const double castorArmLength {0.08 };      //  [m]
  const double wMax { 10 };                 //  [RAD/s]

  // Input vector
  mutable Eigen::Vector2d u {0.0f, 0.0f} ;  //  [m/s, RAD/s]

  mutable Eigen::MatrixXd J{3, 2} ;         //  The kinematic model

  const Eigen::Matrix2d F { InitMotorizationMatrix() } ;

  mutable std::vector<Sensor> robotSensors;

  const World world;


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




void Robot_2_0::CheckSensorStatus() const
{
  for(const auto sensor : robotSensors) {

    sensor.setState( world.CheckWorldLines( sensor.Coord(), this->q ) );
  }

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

}




void Robot_2_0::UpdateMatrix() const
{
  J <<  cos(q.theta),     0 ,
        sin(q.theta),     0 ,
            0,            1 ;
}




void Robot_2_0::EnsureMaxSpeed() const
{

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










const bool World::CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0& robot) const
{

  const int nx = std::floor( robot.q.x/xSpacing )*xSpacing;
  const int ny = std::floor( robot.q.y/ySpacing )*ySpacing;

  const int next_nx = nx + xSpacing;
  const int next_ny = ny + ySpacing;

  //  In homogeneus coordinates a line has the equation: ax + by + c = 0.

  //  For a "vertical" line x = a -> x - a = 0.
  //  that is in homogeneus coordinates 1*x + 0*y -a*c = 0
  const Eigen::Vector3d leftLine(0.0f, 1.0f, -nx);
  const Eigen::Vector3d rightLine(0.0f, 1.0f, -next_nx);

  //  For a "horizontal" line y = b -> y - b = 0.
  //  that is in homogeneus coordinates 0*x + 1*y -b*c = 0
  const Eigen::Vector3d bottomLine(1.0f, 0.0f, -ny);
  const Eigen::Vector3d upperLine(1.0f, 0.0f, -next_ny);

  //  Comcatenate the line into a matrix
  Eigen::MatrixXd worldLines(3, 4);
  worldLines << leftLine, rightLine, bottomLine,upperLine ;

  const Eigen::Vector3d overLine = HCoord.transpose()*worldLines;

  return !overLine.minCoeff() ;

}





#endif //ROBOT_2_0_H
