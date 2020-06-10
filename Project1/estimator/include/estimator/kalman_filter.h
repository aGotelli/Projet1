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





class Sensor;

class SensorMeasurements {


private:

};






























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
