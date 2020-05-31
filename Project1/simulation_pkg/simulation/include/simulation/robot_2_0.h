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

            First the imput is computed from the twist message, and the max speed
          of the wheels in ensured to both. Then with the knowledge of the input,
          the matrix which represents the kinematic model is updated and used to
          obain the derivative of the generalized robot coordinates.

            Once the derivative of the robot generalized coordinates is computed
          it is necessary to perform and integration over the time elapsed from
          the last one (so it is more like to compute a displacement). Then the
          displacement is added to the current value so all the coordinates are
          updated.

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

  // Function to compute the odometry
  void ComputeOdometry() const override;

  // Function to update the J matrix elements
  void UpdateMatrix() const;

  // Function to control the max value of the velocities
  void EnsureMaxSpeed() const;

private:

  //  Powerfull structure for kinematic computations
  struct GeneralizedCoordinates  {

    GeneralizedCoordinates()=default;

    //GeneralizedCoordinates(GeneralizedCoordinates& other); define it later

    GeneralizedCoordinates(const double _x, const double _y, const double _theta,
                            const double _phi_1f, const double _phi_2f,
                            const double _phi_3c, const double _beta_3c) :
                            x(_x), y(_y), theta(_theta), phi_1f(_phi_1f),
                            phi_2f(_phi_2f), beta_3c(_beta_3c) {}

    ~GeneralizedCoordinates() {/* no new objects to delete */}

    //GeneralizedCoordinates(GeneralizedCoordinates&& other)=delete;

    //GeneralizedCoordinates& operator=(GeneralizedCoordinates&& other)=delete;


    GeneralizedCoordinates operator=(const GeneralizedCoordinates& equal);
    //============================ AGGIUNGI REGOLA DEI CINQUE ==================
    GeneralizedCoordinates operator=(const Eigen::VectorXd& result);

    GeneralizedCoordinates operator+(const GeneralizedCoordinates& addendum);


    GeneralizedCoordinates Integrate(const ros::Duration& timeElapsed);

    double x{0.0}, y{0.0}, theta{0.0};

    double phi_1f{0.0}, phi_2f{0.0}, phi_3c{0.0};

    double beta_3c{0.0};

  };

  struct Encoder {
    Encoder()=default;

    //  dots per wheel rotation
    const int resolution {180};
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
  mutable GeneralizedCoordinates q, q_dot;

  //  Robot has its encoders
  const Encoder encoders;

  // Generalized coordinates for odometry
  mutable GeneralizedCoordinates q_odom, q_dot_odom;

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
  //  Set robot posture PoseStamped message
  robotPosture.header.stamp = currentTime ;
  robotPosture.pose.position.x = q.x ;
  robotPosture.pose.position.y = q.y ;
  robotPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q.theta) ;


  movingPlatformFrame.setOrigin( tf::Vector3(q.x, q.y, wheelRadius));
  tf::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, q.theta);
  movingPlatformFrame.setRotation( quaternion );


  //  Set the wheels angles message
  wheelsAngles.phi_1f = q_dot_odom.phi_1f;
  wheelsAngles.phi_2f = q_dot_odom.phi_2f;


  beta.header.stamp = currentTime ;
  beta.name.push_back("castor_joint") ;
  beta.position.push_back(q.beta_3c) ;

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
  const Eigen::Vector2d phi_discretized( std::floor(q.phi_1f/encoders.resolution)*encoders.resolution,
                                          std::floor(q.phi_2f/encoders.resolution)*encoders.resolution ) ;

  // Discretization of the input
  Eigen::Vector2d U_discretized = F.inverse()*phi_discretized;

  // Discretization of the state vector
  q_dot_odom = J*U_discretized;

  // Next iteration
  q_odom = q_odom + q_dot_odom.Integrate(timeElapsed);

}


// Genralized coordinates' functions and operatos
Robot_2_0::GeneralizedCoordinates Robot_2_0::GeneralizedCoordinates::operator=(const Eigen::VectorXd& result)
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



Robot_2_0::GeneralizedCoordinates Robot_2_0::GeneralizedCoordinates::operator=(const GeneralizedCoordinates& equal)
{

  this->x       = equal.x       ;
  this->y       = equal.y       ;
  this->theta   = equal.theta   ;
  this->beta_3c = equal.beta_3c ;
  this->phi_1f  = equal.phi_1f  ;
  this->phi_2f  = equal.phi_2f  ;
  this->phi_3c  = equal.phi_3c  ;

  return (*this) ;
}



Robot_2_0::GeneralizedCoordinates Robot_2_0::GeneralizedCoordinates::operator+(const GeneralizedCoordinates& addendum)
{
  this->x += addendum.x ;
  this->y += addendum.y ;
  this->theta   = utility::LimitAngle( this->theta    + addendum.theta    ) ;
  this->beta_3c = utility::LimitAngle( this->beta_3c  + addendum.beta_3c  ) ;
  this->phi_1f  = utility::LimitAngle( this->phi_1f   + addendum.phi_1f   ) ;
  this->phi_2f  = utility::LimitAngle( this->phi_2f   + addendum.phi_2f   ) ;
  this->phi_3c  = utility::LimitAngle( this->phi_3c   + addendum.phi_3c   ) ;

  return (*this) ;
}



Robot_2_0::GeneralizedCoordinates Robot_2_0::GeneralizedCoordinates::Integrate(const ros::Duration& timeElapsed)
{
  GeneralizedCoordinates delta;

  //  Function called from a q_dot object, actually x, y, theta represents velocities
  delta.x       = x       *timeElapsed.toSec() ;
  delta.y       = y       *timeElapsed.toSec() ;
  delta.theta   = theta   *timeElapsed.toSec() ;
  delta.beta_3c = beta_3c *timeElapsed.toSec() ;
  delta.phi_1f  = phi_1f  *timeElapsed.toSec() ;
  delta.phi_2f  = phi_2f  *timeElapsed.toSec() ;
  delta.phi_3c  = phi_3c  *timeElapsed.toSec() ;

  return delta;
}



#endif //ROBOT_2_0_H
