#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H


#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations
//#include "estimator/robot.h"



namespace utility
{
  struct Pose2D {
    Pose2D()=default;

    Pose2D(const double _x, const double _y, const double _theta=0) : x(_x), y(_y), theta(_theta) {}

    double x { 0.0 } ;
    double y { 0.0 } ;
    double theta { 0.0 } ;
  };
}

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
