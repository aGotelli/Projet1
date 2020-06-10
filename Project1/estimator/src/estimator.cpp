


#include <estimator/kalman_filter.h>
//include "simulation_pkg/simulation/utility.h"
#include <simulation_messages/Encoders.h>
#include <simulation_messages/IRSensors.h>


#include <ros/ros.h>

#include <math.h>


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


Eigen::Vector2d currentReading(0.0, 0.0);
void EncoderReading(const simulation_messages::Encoders::ConstPtr& encoders)
{
  currentReading[0] = encoders->phi_1f ;
  currentReading[1] = encoders->phi_2f ;
}


void IrSensorsReading(const simulation_messages::IRSensors::ConstPtr& state,
                      const utility::Pose2D& robotPosture )
{
  // currentReading[0] = state->sens1 ;
  // currentReading[1] = state->sens2 ;

  //  Nope, copy the message in the class sensor and use the concept of measurement
}


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

void EvolutionModel(Eigen::Vector3d& X, const Eigen::Vector2d input)
{
  X(0) = X(0) + input(0)*cos(X(2));
  X(1) = X(1) + input(0)*sin(X(2));
  X(2) = X(2) + input(1);
}


double ComputeMahalanobis()
{
  // double dMaha;
  // double innov;
  // Eigen::MatrixXd C(1, 3);
  // Eigen::Matrix3d P;
  // double Qgamma;
  //
  // dMaha = sqrt( innov * (C*P*C.transpose() + Qgamma)* innov );
  //
  // return dMaha;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");

  //  Initialize intial pose
  const utility::Pose2D robotPosture(0.0, 0.0, 0.0);


  ros::Subscriber readEncoders = nh_glob.subscribe<simulation_messages::Encoders>("simulation/EncodersReading", 1, EncoderReading);
  ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("simulation/IRSensorsStatus", 1, boost::bind(IrSensorsReading, _1, robotPosture));

  //  How much we rely on the model
  double sigmaTuning = 0.0f;
  double rwheel = 0.05;
  double trackGauge = 0.4;

  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       rwheel/2      ,      rwheel/2       ,
                       rwheel/trackGauge  , -rwheel/trackGauge  ;

  const Eigen::Matrix2d Qwheels = Eigen::MatrixBase<Eigen::Matrix2d>::Identity()*std::pow(sigmaTuning, 2);

  const Eigen::Matrix2d Qbeta = jointToCartesian*Qwheels*jointToCartesian.transpose();

  const Eigen::Matrix3d Qalpha = Eigen::MatrixXd::Zero(3, 3) ;





  Eigen::Matrix3d P = Pinit();

  Eigen::Vector2d previousReading(0.0f, 0.0f);
  Eigen::Vector3d X(2.0, 2.0, 45*M_PI/180);




  ros::Rate estimatorRate(5);

  while( ros::ok() ) {

    ros::spinOnce();

    Eigen::Vector2d rotation = currentReading - previousReading;


    Eigen::Vector2d input = jointToCartesian*rotation;

    EvolutionModel(X, input);

    UpdateMatrix(X, input);

    P = A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha;

    bool any_measurement;
    if( any_measurement) {

      //  Ranged loop in the vector of measurements

          //  Compute the C Matrix

          //  Perform the computation of the position of the sensor

          //  Check neigborns lines

          //  Compute innovation

          //  Compute Mahalanobis distance

          //  Evaluate threshold

          //  Eventually Perform estimation


      //ComputeMahalanobis();
    }


    previousReading = currentReading ;

    estimatorRate.sleep();
  }






  return 0;
}
