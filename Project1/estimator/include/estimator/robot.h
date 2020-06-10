#ifndef ROBOT_H
#define ROBOT_H

// ROS
#include <ros/ros.h>

// Include here the ".h" files corresponding to the topic type you use.
#include <eigen3/Eigen/Dense>

/*

% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

% Uncertainty on initial position of the robot.

sigmaX     = 8;         % Determined by student ( I did some exsperiments)
sigmaY     = 8;         % Determined by student ( I did some exsperiments)
sigmaTheta = 3*pi()/180 ;   % Determined by student (usually is easier to
%check parallelism and ortoghonality )
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;

*/


class Robot{
public:
  Robot()=default;

  Robot(Eigen::Matrix3d _A, Eigen::MatrixXd _B, Eigen::MatrixXd _C):
        A(_A), B(_B), C(_C) {}

  inline const Eigen::Matrix2d JointToCartesian() const {return jointToCartesian; }

  void ComputeErrorPropagation() const;
  void UpdateMatrix() const;

private:
  mutable Eigen::Matrix3d A;
  mutable Eigen::MatrixXd B;
  mutable Eigen::MatrixXd C;

  // Propagation error
  const double sigmaX = 8.0;
  const double sigmaX = 8.0;
  const double sigmaTheta = 3*M_PI/180;

  mutable Eigen::Matrix3d P;
  P << sigmaX,  0,        0,
         0,   sigmaY,     0,
         0,     0,   sigmaTheta;


  // Joint to Cartesian matrix
  const Eigen::Matrix2d jointToCartesian;

};

// Function declaration

void Robot::ComputeErrorPropagation() const
{
  //P = A*P*(A.') + B*Qbeta*(B.') + Qalpha ;

  P = A*P*

}


void Robot::UpdateMatrix() const{
  A << 1, 0, -u(0)sin(theta),
       0, 1,  u(0)cos(theta),
       0, 0,               1;

  B << cos(theta),  0,
       sin(theta),  0,
            0    ,  1;



}









#endif //ROBOT_H
