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
          In this file there are defined all the function needed from the estimator
        to compute the estimated position and check if a measurement should be
        accepted or rejected based on the comparison between the Mahalanobis distance
        and the threshold.

          Moreover, it contains all the inizialization of the matrix and some parameters
        as they have always the same construction. In order to update them, some functions
        are implemented.
 *
 */
#include <geometry_msgs/PoseWithCovariance.h>

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>  //  useful for matrix vectors operations
#include "simulation/utility.h"
#include "simulation/sensor.h"
#include "simulation/world.h"




class KalmanFilter {
public:
  KalmanFilter(const Eigen::Matrix2d& _jointToCartesian, const double _sigmaMeasurement, const double _sigmaTuning,
                const double _sigmaX, const double _sigmaY, const double _sigmaTheta) :
                  jointToCartesian(_jointToCartesian),
                  sigmaMeasurement(_sigmaMeasurement),
                  sigmaTuning(_sigmaTuning),
                  sigmaX(_sigmaX),
                  sigmaY(_sigmaY),
                  sigmaTheta(_sigmaTheta)
                  {
                    ROS_INFO_STREAM("Qwheels :                                       " << Qwheels );
                    ROS_INFO_STREAM("Qbeta :                                         " << Qbeta );
                    ROS_INFO_STREAM("Qgamma :                                        " << Qgamma );
                  }

  //  Initialize the P matrix using the uncertainties on the robot posture
  const Eigen::Matrix3d Pinit() ;

  //  Propagate the error
  inline const Eigen::Matrix3d Propagation( Eigen::Matrix3d& P,
                                            const Eigen::Matrix3d& A,
                                            const Eigen::MatrixXd& B )
                                            {
                                              //  Compute the error propagation in the reference frame
                                              P = A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha ;
                                            }

  //  Perform the estimation for the model
  void Estimation(Eigen::Matrix3d& P, Eigen::Vector3d& X, const Eigen::MatrixXd& C, const double innov);

  //  Compute the Mahalanobis distance given the current covariace matrix
  inline double ComputeMahalanobis(const double innov,
                                    Eigen::MatrixXd& C,
                                    Eigen::Matrix3d& P)
                                    {
                                      return std::pow(innov, 2 ) / ( (C*P*C.transpose()).value() + Qgamma ) ;
                                    }



private:

  //  Uncertainties on the robot initial posture
  const double sigmaX     { 0.00f };
  const double sigmaY     { 0.00f };
  const double sigmaTheta { 0.00f };

  //  Uncertainty on the measurement
  const double sigmaMeasurement { 0.0 };  //  Default 0.0 not a good idea

  //  Inversely proportional of the accurancy of the model
  const double sigmaTuning { 0.0 };  //  Default 0.0 not a good idea

  const Eigen::Matrix2d jointToCartesian;

  //  The covariance matrix for the wheels
  const Eigen::Matrix2d Qwheels { Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2) };

  //  The covariance matrix for the input
  const Eigen::Matrix2d Qbeta { jointToCartesian*Qwheels*jointToCartesian.transpose() };

  //  The covariance matrix set to zero
  const Eigen::Matrix3d Qalpha { Eigen::MatrixXd::Zero(3, 3) };

  //  The covariance on the measurement
  const double Qgamma { std::pow(sigmaMeasurement, 2) };

  ros::NodeHandle nh_glob;

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


// Publish measurements accepted
estimator_messages::Measurement Accepted(const Sensor& sensor, const Measurement& _measurement, const double dMaha)
{
  //  Create a message to publish the current measurement
  estimator_messages::Measurement measurement;

  //  Set the state of the measurement
  measurement.accepted = true;

  //  Indicate the index of the line
  measurement.line_index = _measurement.lineIndex;

  //  Include the position
  measurement.pose.position.x = sensor.AbsolutePosition().x ;
  measurement.pose.position.y = sensor.AbsolutePosition().y ;

  //  Set the Mahalanobis distance of the measurement
  measurement.distance = dMaha ;
  //  Filtering based on the type
  if( _measurement.lineType == utility::LINETYPE::HORIZONTAL )
    measurement.line_type = "HORIZONTAL" ;
  else
    measurement.line_type = "VERTICAL" ;

  return measurement;
}


// Publish measurements rejected
estimator_messages::Measurement Rejected(const Sensor& sensor, const Measurement& _measurement, const double dMaha)
{
  //  Create a message to publish the current measurement
  estimator_messages::Measurement measurement;

  //  Set the state of the measurement
  measurement.accepted = false;

  //  Indicate the index of the line
  measurement.line_index = _measurement.lineIndex;

  //  Include the position
  measurement.pose.position.x = sensor.AbsolutePosition().x ;
  measurement.pose.position.y = sensor.AbsolutePosition().y ;

  //  Set the Mahalanobis distance of the measurement
  measurement.distance = dMaha ;
  //  Filtering based on the type
  if( _measurement.lineType == utility::LINETYPE::HORIZONTAL )
    measurement.line_type = "HORIZONTAL" ;
  else
    measurement.line_type = "VERTICAL" ;

  return measurement;
}





#endif  //  KALMAN_FILTER_H
