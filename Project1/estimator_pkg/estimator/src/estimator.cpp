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
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include "estimator_messages/Measurement.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <math.h>

#include "estimator/kalman_filter.h"
#include "simulation/utility.h"
#include "simulation/sensor.h"
#include "simulation/world.h"
#include "simulation_messages/Encoders.h"
#include "simulation_messages/IRSensors.h"






// Encoder reading
Eigen::Vector2d previousReading(0.0f, 0.0f);
Eigen::Vector2d currentReading(0.0f, 0.0f);

// Callback function for encoder reading
void EncoderReading(const simulation_messages::Encoders::ConstPtr& encoders)
{
  currentReading[0] = encoders->phi_1f ;
  currentReading[1] = encoders->phi_2f ;
}


// Callback function for sensor reading
RobotSensors robotSensors;
std::vector<Measurement> measurements;

// State vector
Eigen::Vector3d X;
void IRSensorsReading(const simulation_messages::IRSensors::ConstPtr& state)
{

  //  Received sensrs status
  if( state->sens1 ) {
    robotSensors.sens1().UpdateTransform( X );
    // robotSensors.sens1().getMeasurement( measurements ) ;
    measurements.push_back( robotSensors.sens1().getMeasurement() );

  }

  if( state->sens2 ) {
    robotSensors.sens2().UpdateTransform( X );
    // robotSensors.sens2().getMeasurement( measurements );
    measurements.push_back( robotSensors.sens2().getMeasurement() );

  }

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");

  //  chi2inv(0.95,2) = 5.9915
  //  chi2inv(0.95,1) = 3.8415
  double threshold;
  nh_loc.param("threshold", threshold, 0.0 );
  //  Forgetting to set the threshold is a big mistake, the penality is regetting all the measurements

  //  Global world parameters
	double xSpacing, ySpacing, lineThickness;					// [m]
	nh_glob.param("/sensor/x_spacing", xSpacing, 0.5) ;
	nh_glob.param("/sensor/y_spacing", ySpacing, 1.0) ;
	nh_glob.param("/sensor/line_thickness", lineThickness, 0.005) ;

  ROS_INFO_STREAM("World parameters : " << xSpacing << ", " << ySpacing << ", " << lineThickness );

	//	Create the world object
	const World world(xSpacing, ySpacing, lineThickness);

  // Initial pose
  double xInit, yInit, thetaInit;
  nh_glob.param("/robot_2_0/x_init", xInit, 0.25);
  nh_glob.param("/robot_2_0/y_init", yInit, 0.5);
  nh_glob.param("/robot_2_0/theta_init", thetaInit, 0.0);

  ROS_INFO_STREAM("xInit : " << xInit << " yInit : "<< yInit << " thetaInit : " << thetaInit ) ;


  // State vector
  X <<        xInit         ,
              yInit         ,
        thetaInit*M_PI/180  ;


  // Robot parameters
  double wheelRadius, a;
  nh_glob.param("/robot_2_0/wheel_radius", wheelRadius, 0.05);
  nh_glob.param("/robot_2_0/a", a, 0.2);

  double trackGauge = 2*a;

  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       wheelRadius/2      ,      wheelRadius/2       ,
                       wheelRadius/trackGauge  , -wheelRadius/trackGauge  ;

  double encodersResolution;
  nh_glob.param("/robot_2_0/encoders_resolution", encodersResolution, (double)360) ;

  // Filter parameters
  const double sigmaMeasurement = sqrt(pow(lineThickness, 2)/12);

  double sigmaTuning;
  nh_glob.param("sigma_tuning", sigmaTuning, 0.0);

  //  Initialize the Kalman filter with the parameters
  KalmanFilter kalman(jointToCartesian, sigmaMeasurement, sigmaTuning);


  //  Declaration of the kalman filter matrices (better to have them here)
  Eigen::Matrix3d A;
  Eigen::MatrixXd B(3,2);
  Eigen::Matrix3d P = kalman.Pinit();
  ROS_INFO_STREAM("covariance matrix :" << P);

  // Declare your node's subscriptions and service clients
  ros::Subscriber readEncoders = nh_glob.subscribe<simulation_messages::Encoders>("/EncodersReading", 1, EncoderReading);
  ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("/IRSensorsStatus", 1, IRSensorsReading);

  // Declare you publishers and service servers
  ros::Publisher Mahalanobis = nh_glob.advertise<std_msgs::Float32>("/Mahalanobis", 1);
  ros::Publisher estPosture = nh_glob.advertise<geometry_msgs::PoseWithCovariance>("/EstimatedPosture", 1);
  ros::Publisher shareMeasurements = nh_glob.advertise<estimator_messages::Measurement>("/Measurements", 1);


  // Initialize sensors
  double x1, y1;
  nh_glob.param("/sensor/x1_pos", x1, 0.0) ;
  nh_glob.param("/sensor/y1_pos", y1, -0.1) ;	// First one on the right of the robot

  double x2, y2;
  nh_glob.param("/sensor/x2_pos", x2, 0.0) ;
  nh_glob.param("/sensor/y2_pos", y2, 0.1) ;		// Second one on the left of the robot

  // Inizialize the robot' sesnors
  robotSensors.AddSensor( Sensor( x1, y1, world ) ) ;
  robotSensors.AddSensor( Sensor( x2, y2, world ) ) ;

  Eigen::Vector2d input;

  ros::Rate estimatorRate(150);

  while( ros::ok() ) {

    ros::spinOnce();

    // Compute rotation
    Eigen::Vector2d rotation = currentReading - previousReading;

    // Compute input
    input = jointToCartesian*rotation;

    // Compute evolution model
    EvolutionModel(X, input);


    // Update matrix A and B
    UpdateMatrix(X, input, A, B);


    // Update propagation error matrix
    kalman.Propagation(P, A, B);

    // Check for any measurements
    for(const auto& measurement : measurements ) {

      double dMaha;
      Eigen::MatrixXd C(1, 3);
      double innov;
      //  First filter the type of line
      if( measurement.lineType == utility::LINETYPE::HORIZONTAL ) {

        C << 0, 1,  measurement.activeSensor->RelativePosition().x*cos(X(2)) - measurement.activeSensor->RelativePosition().y*sin(X(2)) ;

        const double Y = measurement.activeSensor->AbsolutePosition().y;
        const double Yhat = measurement.lineIndex;

        innov = Y - Yhat ;

        dMaha = kalman.ComputeMahalanobis( innov, C, P );

      } else {  //  In this case the line detected is "vertical"

        C << 1, 0, - measurement.activeSensor->RelativePosition().x*sin(X(2)) - measurement.activeSensor->RelativePosition().y*cos(X(2)) ;

        const double Y =  measurement.activeSensor->AbsolutePosition().x;
        const double Yhat = measurement.lineIndex ;

        innov =  Y - Yhat ;

        dMaha = kalman.ComputeMahalanobis( innov, C, P );

      }

      if( dMaha <= threshold ) {

        //  Only if we referred to a good line, update
        kalman.Estimation(P, X, C, innov) ;

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

        //  Than publish the newly measurement
        shareMeasurements.publish( accepted ) ;


      }

      //  Publish result for the Mahalanobis distance
      std_msgs::Float32 currentDist;
      currentDist.data = dMaha;
      Mahalanobis.publish( currentDist );

      //  Publish estimated posture and standard deviations
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

      //  Publish the message
      estPosture.publish( estimatedPosture );


    }

    //  Update before next iteration
    previousReading = currentReading ;

    //  Clear measurements for not repeating
    measurements.clear();

    //  Wait remaining time
    estimatorRate.sleep();
  }






  return 0;
}
