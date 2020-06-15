#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/**
 * \file kalman filter
 * \brief
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 9/06/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° /
 *
 * Publishes to: <BR>
 *    ° /
 *
 * Description

 *
 */
#include <geometry_msgs/PoseWithCovariance.h>

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations
#include "simulation/utility.h"
#include "simulation/sensor.h"
#include "simulation/world.h"




class KalmanFilter {
public:
  KalmanFilter(const Eigen::Matrix2d& _jointToCartesian, const double _sigmaMeasurement, const double _sigmaTuning) :
                  jointToCartesian(_jointToCartesian),
                  sigmaMeasurement(_sigmaMeasurement),
                  sigmaTuning(_sigmaTuning)
                  {
                    ROS_INFO_STREAM("Qwheels                            : " << Qwheels );
                    ROS_INFO_STREAM("Qbeta                              : " << Qbeta );
                    ROS_INFO_STREAM("Qgamma                             : " << Qgamma );
                  }

  //  Initialize the P matrix using the uncertainties on the robot posture
  const Eigen::Matrix3d Pinit() ;

  //  Propagate the error
  inline const Eigen::Matrix3d Propagation( Eigen::Matrix3d& P,
                                            const Eigen::Matrix3d& A,
                                            const Eigen::MatrixXd& B )
                                            {
                                              // ROS_INFO_STREAM("A       : " << A );
                                              // ROS_INFO_STREAM("B       : " << B );
                                              // ROS_INFO_STREAM("Qbeta   : " << Qbeta );
                                              // ROS_INFO_STREAM("Primo   : " << A*P*A.transpose() );
                                              // ROS_INFO_STREAM("Secondo : " << B*Qbeta*B.transpose() );
                                              P = A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha ;
                                            }

  //  Perform the estimation for the model
  void Estimation(Eigen::Matrix3d& P, Eigen::Vector3d& X, const Eigen::MatrixXd& C, const double innov);

  // Initialize sigmaTuning and sigmaMeasurement as variances
  void sigmaInit(const double sigmaMeasurement, const double lineThickness, const double unit);

  //  Compute the Mahalanobis distance given the current covariace matrix
  inline double ComputeMahalanobis(double innov,
                                    Eigen::MatrixXd& C,
                                    Eigen::Matrix3d& P)
                                    {
                                      // ROS_INFO_STREAM("product : " << (C*P*C.transpose()).value() );
                                      return std::pow(innov, 2 ) / ( (C*P*C.transpose()).value() + Qgamma ) ;
                                    }


private:

  //  Uncertainties on the robot initial posture
  const double sigmaX     { 8.0f };
  const double sigmaY     { 8.0f };
  const double sigmaTheta { 3*M_PI/180 };

  //  Uncertainty on the measurement
  const double sigmaMeasurement { 0.0 };  //  Default 0.0 not a good idea

  //  Inversely proportional on how accurate the model is
  const double sigmaTuning { 0.0 };  //  Default 0.0 not a good idea

  const Eigen::Matrix2d jointToCartesian;

  //  The covariance matrix for the wheels
  const Eigen::Matrix2d Qwheels { Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2) };

  //  The covariance matrix for the input
  const Eigen::Matrix2d Qbeta { jointToCartesian*Qwheels*jointToCartesian.transpose() };

  //  The covariance matrix of...
  const Eigen::Matrix3d Qalpha { Eigen::MatrixXd::Zero(3, 3) };

  //  The covariance on the measurement
  const double Qgamma { std::pow(sigmaMeasurement, 2) };
};




const Eigen::Matrix3d KalmanFilter::Pinit()
{
  Eigen::Matrix3d P;
  P <<  sigmaX*sigmaX,       0,                   0       ,
              0,      sigmaY*sigmaY,              0       ,
              0,             0,    sigmaTheta*sigmaTheta  ;

  return P;
}


void KalmanFilter::Estimation(Eigen::Matrix3d& P, Eigen::Vector3d& X, const Eigen::MatrixXd& C, const double innov)
{

  Eigen::MatrixXd K = ( P*C.transpose() )/( (C*P*C.transpose()).value() + Qgamma ) ;
  //  estimation phase
  X = X + K*innov ;
  P = (Eigen::MatrixBase<Eigen::Matrix3d>::Identity() - K*C)*P ;
}



// Update of matrix A and B using the current input and robot posture
void UpdateMatrix(const Eigen::Vector3d& X, const Eigen::Vector2d input, Eigen::Matrix3d& A, Eigen::MatrixXd& B)
{
  //  Fill A using definition
  A << 1, 0, -input(0)*cos(X(2)),
       0, 1,  input(0)*sin(X(2)),
       0, 0,           1        ;


  //  Fill B using definition
  B << cos(X(2)), 0,
       sin(X(2)), 0,
          0     , 1;

}



// Compute and update vector state
void EvolutionModel(Eigen::Vector3d& X, const Eigen::Vector2d input)
{
  X(0) = X(0) + input(0)*cos(X(2));
  X(1) = X(1) + input(0)*sin(X(2));
  X(2) = X(2) + input(1);
}



//  Publish estimated posture and standard deviations
geometry_msgs::PoseWithCovariance Estimation(const Eigen::Vector3d& X, const Eigen::Matrix3d& P)
{
  geometry_msgs::PoseWithCovariance estimatedPosture;

  //  Initialize the position
  estimatedPosture.pose.position.x = X(0);
  estimatedPosture.pose.position.y = X(1);

  //  Initialize orientation
  estimatedPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>(X(2));

  //  Define the covariance
  estimatedPosture.covariance[6*0 + 0] = P(0, 0);
  estimatedPosture.covariance[6*1 + 1] = P(1, 1);
  estimatedPosture.covariance[6*5 + 5] = P(2, 2);

  return estimatedPosture;
}

//  Publish estimated posture and standard deviations
geometry_msgs::Pose PostureError(const geometry_msgs::Pose& realPosture, const geometry_msgs::PoseWithCovariance& estimatedPosture)
{
  geometry_msgs::Pose postureError;

  postureError.position.x    = realPosture.position.x    - estimatedPosture.pose.position.x     ;
  postureError.position.y    = realPosture.position.y    - estimatedPosture.pose.position.y     ;
  postureError.orientation.x = realPosture.orientation.x - estimatedPosture.pose.orientation.x  ;
  postureError.orientation.y = realPosture.orientation.y - estimatedPosture.pose.orientation.y  ;
  postureError.orientation.z = realPosture.orientation.z - estimatedPosture.pose.orientation.z  ;
  postureError.orientation.w = realPosture.orientation.w - estimatedPosture.pose.orientation.w  ;

  return postureError;
}


estimator_messages::Measurement Accepted(const Measurement& measurement)
{
  //  Create a message to publish the current measurement
  estimator_messages::Measurement accepted;

  //  Indicate the index of the line
  accepted.line_index.data = measurement.lineIndex;

  //  Include the position
  accepted.pose.position.x = measurement.activeSensor->AbsolutePosition().x ;
  accepted.pose.position.y = measurement.activeSensor->AbsolutePosition().y ;

  //  Filtering based on the type
  if( measurement.lineType == utility::LINETYPE::HORIZONTAL )
    accepted.line_type.data = "HORIZONTAL" ;
  else
    accepted.line_type.data = "VERTICAL" ;

  return accepted;
}







#endif  //  KALMAN_FILTER_H
