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


#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations
//#include "estimator/robot.h"




class KalmanFilter {
public:
  KalmanFilter()=default;

  //  Initialize the P matrix using the Uncertainties on the robot posture
  const Eigen::Matrix3d Pinit() ;

  //  Propagate the error
  inline const Eigen::Matrix3d Propagation(const Eigen::Matrix3d& P,
                                            const Eigen::Matrix3d& A,
                                            const Eigen::MatrixXd& B )
                                            {return A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha; }

  void Estimation(Eigen::Matrix3d& P, Eigen::Vector3d& X, const Eigen::Matrix3d& C);

private:

  const Eigen::Matrix2d JTCinit() ;

  //  Uncertainties on the robot initial posture
  const double sigmaX     { 8.0f };
  const double sigmaY     { 8.0f };
  const double sigmaTheta { 3*M_PI/180 };

  //  Uncertainty on the measurement
  const double sigmaMeasurement { 0.0 };  //  Default 0.0 not a good idea

  //  Inversely proportional on how accurate the model is
  const double sigmaTuning { 0.0 };  //  Default 0.0 not a good idea

  const double trackGauge {0.4} ;             //  [m]
  const double wheelRadius {0.05 };           //  [m]

  const Eigen::Matrix2d jointToCartesian { JTCinit() };

  //  The covariance matrix for the wheels
  const Eigen::Matrix2d Qwheels { Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2) };

  //  The covariance matrix for the input
  const Eigen::Matrix2d Qbeta { jointToCartesian*Qwheels*jointToCartesian.transpose() };

  //  The covariance matrix of...
  const Eigen::Matrix3d Qalpha { Eigen::MatrixXd::Zero(3, 3) };
};




const Eigen::Matrix3d KalmanFilter::Pinit()
{
  Eigen::Matrix3d P;
  P <<  sigmaX,     0,        0       ,
          0,      sigmaY,     0       ,
          0,        0,    sigmaTheta  ;

  return P;
}

const Eigen::Matrix2d KalmanFilter::JTCinit()
{
  Eigen::Matrix2d temp;
  temp <<       wheelRadius/2      ,      wheelRadius/2       ,
           wheelRadius/trackGauge  , -wheelRadius/trackGauge  ;
  return temp;
}

void KalmanFilter::Estimation(Eigen::Matrix3d& P, Eigen::Vector3d& X, const Eigen::Matrix3d& C)
{

  Eigen::MatrixXd K = P * C.transpose() /( (C*P*C.transpose()).value() + Qgamma) ;
  //  estimation phase
  X = X + K*innov ;
  P = (Eigen::MatrixBase<Eigen::Matrix3d>::Identity() - K*C) * P ;
}



// Update of matrix A and B using the current input and robot posture
void UpdateMatrix(const Eigen::Vector3d& X, const Eigen::Vector2d input, Eigen::Matrix3d& A, Eigen::Matrix3d& B)
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








/*



class KalmanFilter {
public:
  KalmanFilter(const double _sigmaTuning, const Robot& _robot);




private:
  //  Default fully rely on robot model (not a good choice)
  const double sigmaTuning{0.0f};

  const Eigen::Matrix2d Qwheels;
  const Eigen::Matrix2d Qbeta;
  const Eigen::Matrix2d Qalpha;

};





% Input noise

sigmaTuning = 0.01;
Qwheels = sigmaTuning^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ;

% State noise

Qalpha = zeros(3) ;

% Mahalanobis distance threshold

mahaThreshold =  sqrt(chi2inv(0.95,2)) ;  % Determined by student
% as the compartion is with a distance we need to take the squqre root of a
% squared distance

% the 0.95 probability is a common parameter for the chi2inv function and 2
% because wer have two parameters in the mesurament, "a" and "b" of the
% detected magnet; for wich we have more uncertainty in the x than in the y
%







KalmanFilter::KalmanFilter(const double _sigmaTuning, const Robot& _robot) :
    sigmaTuning(_sigmaTuning),
    Qwheels( Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2) ),
    Qbeta( _robot.JointToCartesian()*Qwheels*_robot.JointToCartesian().transpose() ),
    Qalpha( Eigen::MatrixXd::Zero(2, 2) ) {}






*/











#endif  //  KALMAN_FILTER_H
