#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H

/**
 * \file robot 2_0 file
 * \brief contains the model of the robot
 * \author Bianca & Andrea
 * \version 0.2
 * \date 01/06/2020
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
#include "simulation/robot_2_0_generalizedcoord.h"

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

  struct Encoders {
    Encoders()=default;

    //  dots per wheel rotation
    const int resolution{180} ;
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
  mutable robot_2_0::GeneralizedCoordinates q, q_dot;

  // Generalized coordinates for odometry
  mutable robot_2_0::GeneralizedCoordinates q_odom, q_dot_odom;

  const Encoders encoder;

  // Robot parameters
  const double trackGauge {0.2} ;             //  [m]
  const double wheelRadius {0.05 };           //  [m]
  const double jointOffSet {0.4 };            //  [m]
  const double castorArmLength {0.08 };       //  [m]
  const double wMax { 10 };                   //  [RAD/s]

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


void Robot_2_0::ComputeOdometry() const
{
  //ROS_INFO_STREAM("Odometry, phi_1f : " << q.phi_1f << " phi_2f : " << q.phi_2f ) ;
  // Discretization of phi_1f and phi_2f

  const Eigen::Vector2d phi_discretized ( std::floor(q.phi_1f*180/M_PI/encoder.resolution*encoder.resolution)*M_PI/180,
                                          std::floor(q.phi_2f*180/M_PI/encoder.resolution*encoder.resolution)*M_PI/180 ) ;



  // Discretization of the input
  const Eigen::Vector2d U_discretized = F.inverse()*phi_discretized;

  // Discretization of the state vector
  q_dot_odom = J*U_discretized;

  // Next iteration
  q_odom = q_odom + q_dot_odom.Integrate(timeElapsed);
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












void Robot_2_0::PrepareMessages()
{
  robotPosture.header.stamp = currentTime ;

  robotPosture.pose.position.x = q.x ;
  robotPosture.pose.position.y = q.y ;
  robotPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q.theta) ;

  

  odomPosture.pose.position.x = q_odom.x ;
  odomPosture.pose.position.y = q_odom.y ;
  odomPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q_odom.theta) ;


  movingPlatformFrame.setOrigin( tf::Vector3(q.x, q.y, wheelRadius));
  tf::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, q.theta);
  movingPlatformFrame.setRotation( quaternion );


  //  Set the wheels angles message
  wheelsAngles.phi_1f = q_dot_odom.phi_1f;
  wheelsAngles.phi_2f = q_dot_odom.phi_2f;


  //  Set the angle of the castor joint
  beta.header.stamp = currentTime ;
  beta.name.push_back("castor_joint") ;
  beta.position.push_back(q.beta_3c) ;

}







#endif //ROBOT_2_0_H
