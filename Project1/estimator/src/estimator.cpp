/**
 * \file estimator
 * \brief Kalman filter estimator
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 10/06/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° simulation/EncodersReading
 *    ° simulation/IRSensorsStatus
 *
 * Publishes to: <BR>
 *    ° /
 *
 * Description

 *
 */


// ROS
#include <ros/ros.h>

// Include here the ".h" files corresponding to the topic type you use.
#include <math.h>

#include "estimator/kalman_filter.h"
#include "simulation/utility.h"
#include "simulation/sensor.h"
#include "simulation_messages/Encoders.h"
#include "simulation_messages/IRSensors.h"





double sigmaX     = 8;
double sigmaY     = 8;
double sigmaTheta = 3*M_PI/180 ;

const Eigen::Matrix3d Pinit()
{
  Eigen::Matrix3d P;
  P <<  sigmaX,     0,        0       ,
          0,      sigmaY,     0       ,
          0,        0,    sigmaTheta  ;

  return P;
}


// Callback function for encoder reading
Eigen::Vector2d currentReading(0.0, 0.0);
void EncoderReading(const simulation_messages::Encoders::ConstPtr& encoders)
{
  currentReading[0] = encoders->phi_1f ;
  currentReading[1] = encoders->phi_2f ;
}

// Callback function for sensor reading
Sensor sensor1, sensor2;
std::vector<Measurement> measurements;
void IrSensorsReading(const simulation_messages::IRSensors::ConstPtr& state,
                      const geometry_msgs::Pose& robotPosture )
{

  //  Received sensrs status
  if( state->sens1 ) {
    sensor1.UpdateTransform( robotPosture );
    measurements.push_back( sensor1.getMeasurement() );
  }

  if( state->sens2 ) {
    sensor2.UpdateTransform( robotPosture );
    measurements.push_back( sensor2.getMeasurement() );
  }

}

// Update of matrix A and B
Eigen::Matrix3d A(3, 3);
Eigen::MatrixXd B(3, 2);
void UpdateMatrix(const Eigen::Vector3d& X, const Eigen::Vector2d input)
{

  A << 1, 0, -input(0)*cos(X(2)),
       0, 1,  input(0)*sin(X(2)),
       0, 0,           1        ;

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

// Compute mahalanobis distance
double sigmaMeasurement = 5.6;
double Qgamma = std::pow(sigmaMeasurement, 2);
double ComputeMahalanobis(double innov, Eigen::MatrixXd& C, Eigen::Matrix3d& P)
{
  double dMaha;

  dMaha = std::pow(innov, 2 ) / ( (C*P*C.transpose()).value() + Qgamma );

  return dMaha;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");



  //  Initialize intial pose
//  const utility::Pose2D robotPosture(0.0, 0.0, 0.0);


  // Declare your node's subscriptions and service clients
  ros::Subscriber readEncoders = nh_glob.subscribe<simulation_messages::Encoders>("simulation/EncodersReading", 1, EncoderReading);
  //ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("simulation/IRSensorsStatus", 1, boost::bind(IrSensorsReading, _1, robotPosture));

  // Declare you publishers and service servers
  //ros::Publisher Mahalanobis = nh_glob.advertise<double>("/", 1);

  //  How much we rely on the model
  double sigmaTuning = 0.0f;
  double rwheel = 0.05;
  double trackGauge = 0.4;

  // Definition of matrix
  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       rwheel/2      ,      rwheel/2       ,
                       rwheel/trackGauge  , -rwheel/trackGauge  ;

  const Eigen::Matrix2d Qwheels = Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2);

  const Eigen::Matrix2d Qbeta = jointToCartesian*Qwheels*jointToCartesian.transpose();

  const Eigen::Matrix3d Qalpha = Eigen::MatrixXd::Zero(3, 3) ;




  // Propagation error matrix
  Eigen::Matrix3d P = Pinit();

  // Encoder reading
  Eigen::Vector2d previousReading(0.0f, 0.0f);

  double xInit, yInit, thetaInit;
  nh_loc.param("x_init", xInit, 0.0);
  nh_loc.param("y_init", yInit, 0.0);
  nh_loc.param("theta_init", thetaInit, 0.0);

  // State vector
  Eigen::Vector3d X(xInit, yInit, thetaInit*M_PI/180);




  ros::Rate estimatorRate(5);

  while( ros::ok() ) {

    ros::spinOnce();

    // Compute rotation
    Eigen::Vector2d rotation = currentReading - previousReading;

    // Compute input
    Eigen::Vector2d input = jointToCartesian*rotation;

    // Compute evolution model
    EvolutionModel(X, input);

    // Update matrix A and B
    UpdateMatrix(X, input);

    // Update propagation error matrix
    P = A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha;

    // Check for any measurements
    for(const auto& measurement : measurements ) {

      double dMaha;
      Eigen::MatrixXd C(1, 3);
      double innov;
      //  First filter the type of line
      if( measurement.lineType == utility::LINETYPE::HORIZONTAL ) {

        C << 0, 1,  measurement.activeSensor->RelativePosition().x*cos(X(2)) - measurement.activeSensor->RelativePosition().y*sin(X(2)) ;

        innov =  measurement.activeSensor->AbsolutePosition().y - measurement.lineIndex ;

        dMaha = ComputeMahalanobis( innov, C, P );

      } else {

        C << 1, 0, - measurement.activeSensor->RelativePosition().x*sin(X(2)) - measurement.activeSensor->RelativePosition().y*cos(X(2)) ;

        innov =  measurement.activeSensor->AbsolutePosition().x - measurement.lineIndex ;

        dMaha = ComputeMahalanobis( innov, C, P );

      }
      double threshold = 0.02;
      if( dMaha <= threshold ) {
        //  Only if we referred to a good line, update
        Eigen::MatrixXd K = P * C.transpose() /( (C*P*C.transpose()).value() + Qgamma) ;
        //  estimation phase
        X = X + K*innov ;
        P = (Eigen::MatrixBase<Eigen::Matrix3d>::Identity() - K*C) * P ;

      }

    }




    previousReading = currentReading ;
    measurements.clear();

    estimatorRate.sleep();
  }






  return 0;
}
