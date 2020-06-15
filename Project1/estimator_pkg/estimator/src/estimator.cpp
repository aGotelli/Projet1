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
#include <geometry_msgs/Twist.h>
#include "estimator_messages/Measurement.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <math.h>

#include "estimator/kalman_filter.h"
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

geometry_msgs::Pose realPosture;
void RealPostureReceived(const geometry_msgs::Pose::ConstPtr& _realPosture)
{
  realPosture = (*_realPosture);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");

  double threshold;
  nh_loc.param("threshold", threshold, 0.0 );
  //  Forgetting to set the threshold is a big mistake, the penality is regetting all the measurements

  double sigmaTuning;
  nh_loc.param("sigma_tuning", sigmaTuning, 0.0);
  //  Default zero means fully relying on odometry

  ROS_INFO_STREAM("Sigma tuning  : " << sigmaTuning );

  int frequency;
  nh_loc.param<int>("estimator_frequency", frequency, 150 );

  ROS_INFO_STREAM("Estimator frequency : " << frequency );

  //  Global world parameters
	double xSpacing, ySpacing, lineThickness;					// [m]
	nh_glob.param("/sensor/x_spacing", xSpacing, 1.0) ;
	nh_glob.param("/sensor/y_spacing", ySpacing, 1.0) ;
	nh_glob.param("/sensor/line_thickness", lineThickness, 0.005) ;

  ROS_INFO_STREAM("World parameters : " << xSpacing << ", " << ySpacing << ", " << lineThickness );

	//	Create the world object
	const World world(xSpacing, ySpacing, lineThickness);

  // Initial pose
  double xInit, yInit, thetaInit;
  nh_glob.param("/robot_2_0/x_init", xInit, 0.0);
  nh_glob.param("/robot_2_0/y_init", yInit, 0.0);
  nh_glob.param("/robot_2_0/theta_init", thetaInit, 0.0);

  ROS_INFO_STREAM("Initial position : " << xInit << ", " << yInit << ", " << thetaInit );

  // Robot parameters
  double wheelRadius, a;
  nh_glob.param("/robot_2_0/wheel_radius", wheelRadius, 0.05);
  nh_glob.param("/robot_2_0/a", a, 0.2);
  const double trackGauge = 2*a;

  //  Obatain the encoders resolutions
  double encodersResolution;
  nh_glob.param("/robot_2_0/encoders_resolution", encodersResolution, (double)360) ;

  ROS_INFO_STREAM("Encoders resolution : " << encodersResolution );

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


  // Declare your node's subscriptions and service clients
  ros::Subscriber readEncoders = nh_glob.subscribe<simulation_messages::Encoders>("/EncodersReading", 1, EncoderReading);
  ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("/IRSensorsStatus", 10, IRSensorsReading);

  //  This last subscriber is not part of the estimator itself, but allows to do a simple comparison with the real posture
  ros::Subscriber obtainRealPosture = nh_glob.subscribe<geometry_msgs::Pose>("/RobotPosture", 1, RealPostureReceived);

  // Declare you publishers and service servers
  ros::Publisher Mahalanobis = nh_glob.advertise<std_msgs::Float32>("/Mahalanobis", 1);
  ros::Publisher estPosture = nh_glob.advertise<geometry_msgs::PoseWithCovariance>("/EstimatedPosture", 1);
  ros::Publisher shareMeasurements = nh_glob.advertise<estimator_messages::Measurement>("/Measurements", 1);
  ros::Publisher pubInput = nh_glob.advertise<geometry_msgs::Twist>("/Velocities", 1);

  //  This last publisher is not part of the estimator itself, but allows to do a simple comparison with the real posture
  ros::Publisher errorStreamer = nh_glob.advertise<geometry_msgs::Pose>("/EstimationError", 1);



  //  State vector
  X <<        xInit         ,
              yInit         ,
        thetaInit*M_PI/180  ;


  //   Joint to cartesian matrix
  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       wheelRadius/2      ,      wheelRadius/2       ,
                       wheelRadius/trackGauge  , -wheelRadius/trackGauge  ;

  //   Convert to millimeters
  //jointToCartesian = jointToCartesian*1000;


  // Filter parameters
  const double sigmaMeasurement = sqrt(pow(lineThickness, 2)/12);

  //  Initialize the Kalman filter with the parameters
  KalmanFilter kalman(jointToCartesian, sigmaMeasurement, sigmaTuning);


  //  Declaration of the kalman filter matrices (better to have them here)
  Eigen::Matrix3d A;
  Eigen::MatrixXd B(3,2);
  Eigen::Matrix3d P = kalman.Pinit();

  Eigen::Vector2d input;

  ros::Rate estimatorRate( frequency );

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

      ROS_INFO_STREAM("Innovation :" << innov );
      ROS_INFO_STREAM("dMaha      :" << dMaha );
      ROS_INFO_STREAM("" );
      if( dMaha <= threshold ) {

        //  Only if we referred to a good line, update
        kalman.Estimation(P, X, C, innov) ;

        //  Than publish the newly measurement
        shareMeasurements.publish( Accepted( measurement ) ) ;

        //  Publish result for the Mahalanobis distance
        std_msgs::Float32 currentDist;
        currentDist.data = dMaha;
        Mahalanobis.publish( currentDist );

      }



    }

    //  Publish estimated posture and standard deviations
    estPosture.publish( Estimation(X, P) );

    //ROS_INFO_STREAM("Covariance matrix : " << P );

    //  Publish velocities
    geometry_msgs::Twist vel;
    vel.linear.x = input[0];
    vel.angular.z = input[1];
    pubInput.publish( vel );


    // Publish the message with the error in the posture
    errorStreamer.publish( PostureError(realPosture, Estimation(X, P) ) ) ;

    //  Update before next iteration
    previousReading = currentReading ;

    //  Clear measurements for not repeating
    measurements.clear();

    //  Wait remaining time
    estimatorRate.sleep();
  }






  return 0;
}
