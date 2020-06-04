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
          The aim is to get information regarding the robot position and velocity and regarding.

            First the input is computed from the twist message, and the max speed
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

  Robot_2_0(const double _frontAxle, const double _wheelRadius,
            const double _jointOffSet, const double _castorArmLength,
            const double _wMax) :
            frontAxle(_frontAxle),
            wheelRadius(_wheelRadius),
            jointOffSet(_jointOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            RobotBase() { /* All the rest is initialized as default */
              ROS_INFO_STREAM("User-defined initialization called..."); }

  Robot_2_0(const utility::Pose2D& initial, const double _frontAxle,
            const double _wheelRadius, const double _jointOffSet,
            const double _castorArmLength, const double _wMax) :
            q( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            q_odom( robot_2_0::GeneralizedCoordinates( initial.x, initial.y, initial.theta) ),
            frontAxle(_frontAxle),
            wheelRadius(_wheelRadius),
            jointOffSet(_jointOffSet),
            castorArmLength(_castorArmLength),
            wMax(_wMax),
            RobotBase() {/* All the rest is initialized as default */
              ROS_INFO_STREAM("Fully user-defined initialization called...");     }



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

  class Encoders {
  public:
    Encoders()=default;

    inline const int ResolutionToRad() const {return resolution*M_PI/180; }
    inline const int Resolution() const {return resolution; }

  private:
    //  the resolution of the encoder
    const int resolution{2} ;   //  [dot/grad]

  };

  // Generalized coordinates
  mutable robot_2_0::GeneralizedCoordinates q, q_dot;

  // Generalized coordinates for odometry
  mutable robot_2_0::GeneralizedCoordinates q_odom, q_dot_odom;

  const Encoders encoder;

  // Robot parameters
  const double frontAxle {0.2} ;              //  [m]
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
//   /*
//   jointToCartesian = [ rwheel/2           rwheel/2          ;
//                        rwheel/trackGauge -rwheel/trackGauge ] ;
//
//
//    for i = 2 : length(tq)
//        dCart = jointToCartesian*[ qRight(i)-qRight(i-1) ; qLeft(i)-qLeft(i-1) ] ;
//        xOdo(i)     = xOdo(i-1)     + dCart(1)*cos(thetaOdo(i-1)) ;
//        yOdo(i)     = yOdo(i-1)     + dCart(1)*sin(thetaOdo(i-1)) ;
//        thetaOdo(i) = thetaOdo(i-1) + dCart(2)                    ;
//    end
//
//   */
//   Eigen::MatrixXd temp(2,2) ;
//   temp <<   1/wheelRadius ,   frontAxle/wheelRadius ,
//             1/wheelRadius ,  -frontAxle/wheelRadius ;
//
//   currentReading[0] = q.phi_1f ;
//   currentReading[1] = q.phi_2f ;
//
//   //ROS_INFO_STREAM(" Current rotation : "<< (currentReading - previusReading)*180/M_PI );
//
//   // Eigen::Vector2d elapsedDots  = (currentReading - previusReading)*encoder.ResolutionToRad() ;
//   // elapsedDots[0] = std::floor( elapsedDots[0] );
//   // elapsedDots[1] = std::floor( elapsedDots[1] );
//
//   // const Eigen::Vector2d d_input = S.block<2,2>(4,0).inverse()*elapsedDots/encoder.ResolutionToRad();
//
//   const Eigen::Vector2d rotation = currentReading - previusReading ;
//
//   const Eigen::Vector2d input = temp.inverse()*rotation ;
//
//   const Eigen::VectorXd result = S*input ;
//
//   ROS_INFO_STREAM("______________________________" ) ;
//   q_odom = q_odom + result;
// /*
//   const Eigen::Vector2d rotation = currentReading - previusReading ;
//
//   Eigen::Vector2d d_input;
//   d_input <<    wheelRadius*(rotation[0] + rotation[1])/2         ,
//             wheelRadius*(rotation[0] + rotation[1])/(2*frontAxle) ;
//
//   q_odom.x = q_odom.x + d_input[0]*cos(q_odom.theta) ;
//   q_odom.y = q_odom.y + d_input[0]*sin(q_odom.theta) ;
//   q_odom.theta = q_odom.theta + d_input[1] ;
//   */
//   ROS_INFO_STREAM(" Current rotation : "<< q.phi_1f*180/M_PI << " and : " << q.phi_2f*180/M_PI );
//   ROS_INFO_STREAM(" Current reading : "<< q_odom.phi_1f*180/M_PI << " and : " << q_odom.phi_2f*180/M_PI );
//   ROS_INFO_STREAM(" Current position  : "<< q_odom.x << " and : " << q_odom.y << " th: " << q_odom.theta*180/M_PI );
//   previusReading = currentReading ;


}



void Robot_2_0::UpdateMatrix() const
{

  S <<            cos(q.theta)          ,                                    0                                ,
                  sin(q.theta)          ,                                    0                                ,
                      0                 ,                                    1                                ,
        -sin(q.beta_3c)/castorArmLength ,    -(castorArmLength + jointOffSet*cos(q.beta_3c))/castorArmLength  ,
                  1/wheelRadius         ,                          frontAxle/wheelRadius                      ,
                  1/wheelRadius         ,                         -frontAxle/wheelRadius                      ,
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
  robotPosture.header.stamp = currentTime ;

  robotPosture.pose.position.x = q.x ;
  robotPosture.pose.position.y = q.y ;
  robotPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q.theta) ;



  odomPosture.pose.position.x = q_odom.x ;
  odomPosture.pose.position.y = q_odom.y ;
  odomPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(q_odom.theta) ;


  // movingPlatformFrame.setOrigin( tf::Vector3(q.x, q.y, wheelRadius));
  // tf::Quaternion quaternion;
  // quaternion.setRPY(0.0, 0.0, q.theta);
  // movingPlatformFrame.setRotation( quaternion );


  //  Set the wheels angles message
  currentReading[0] = q.phi_1f ;
  currentReading[1] = q.phi_2f ;
  ROS_INFO_STREAM("currentReading : " << currentReading*180/M_PI );
  Eigen::Vector2d elapsedDots  = (currentReading*180/M_PI)*encoder.Resolution() ;

  elapsedDots[0] = std::floor( elapsedDots[0] );
  elapsedDots[1] = std::floor( elapsedDots[1] );
  ROS_INFO_STREAM("dots : " << elapsedDots );


  const Eigen::Vector2d discrAgles = elapsedDots/encoder.Resolution() ;
  wheelsAngles.phi_1f = discrAgles[0];
  wheelsAngles.phi_2f = discrAgles[1];
  ROS_INFO_STREAM("algles : " << discrAgles );
  previusReading = currentReading ;

  //  Set the angle of the castor joint
  actuations.name.clear() ;
  actuations.position.clear() ;
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
