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
void IrSensorsReading(const simulation_messages::IRSensors::ConstPtr& state)
{

  //  Received sensrs status
  if( state->sens1 ) {
    robotSensors.sens1().UpdateTransform( X );
    robotSensors.sens1().getMeasurement( measurements ) ;
    // measurements.push_back( robotSensors.sens1().getMeasurement() );
    //  ROS_INFO_STREAM( "measurement 1 : " << robotSensors.sens1().getMeasurement().lineIndex << " " <<  robotSensors.sens1().getMeasurement().lineType );

  }

  if( state->sens2 ) {
    robotSensors.sens2().UpdateTransform( X );
    robotSensors.sens2().getMeasurement( measurements );
    // measurements.push_back( robotSensors.sens2().getMeasurement() );
    //  ROS_INFO_STREAM( "measurement 2 : " << robotSensors.sens2().getMeasurement().lineIndex << " " <<  robotSensors.sens2().getMeasurement().lineType );

  }

  //ROS_INFO_STREAM("How many measurements ? " << measurements.size() );
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");

  //  chi2inv(0.95,2) = 5.9915
  //  chi2inv(0.95,1) = 3.8415
  double threshold;
  nh_loc.param("threshold", threshold, 3.8415);

/*

/ground_generator/line_thickness
/ground_generator/x_spacing
/ground_generator/y_spacing

/simulation/robot_2_0/a
/simulation/robot_2_0/actuator_max_speed
/simulation/robot_2_0/b
/simulation/robot_2_0/c
/simulation/robot_2_0/encoders_resolution
/simulation/robot_2_0/theta_init
/simulation/robot_2_0/wheel_radius
/simulation/robot_2_0/x_init
/simulation/robot_2_0/y_init
/simulation/sensor/line_thickness
/simulation/sensor/x1_pos
/simulation/sensor/x2_pos
/simulation/sensor/x_spacing
/simulation/sensor/y1_pos
/simulation/sensor/y2_pos
/simulation/sensor/y_spacing







*/



  // Initial pose
  double xInit, yInit, thetaInit;
  nh_loc.param("/simulation/robot_2_0/x_init", xInit, 0.25);
  nh_loc.param("/simulation/robot_2_0/y_init", yInit, 0.5);
  nh_loc.param("/simulation/robot_2_0/theta_init", thetaInit, 0.0);

  // State vector
  X <<        xInit         ,
              yInit         ,
        thetaInit*M_PI/180  ;

  double wheelRadius, a;
  nh_loc.param("/simulation/robot_2_0/wheel_radius", wheelRadius, 0.05);
  nh_loc.param("/simulation/robot_2_0/a", a, 0.2);

  double trackGauge = 2*a;

  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       wheelRadius/2      ,      wheelRadius/2       ,
                       wheelRadius/trackGauge  , -wheelRadius/trackGauge  ;


  // Filter parameters
  KalmanFilter kalman(jointToCartesian);

  Eigen::Matrix3d A;
  Eigen::MatrixXd B(3,2);
  Eigen::Matrix3d P = kalman.Pinit();
  ROS_INFO_STREAM("covariance matrix :" << P);

  // Declare your node's subscriptions and service clients
  ros::Subscriber readEncoders = nh_glob.subscribe<simulation_messages::Encoders>("simulation/EncodersReading", 1, EncoderReading);
  // ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("simulation/IRSensorsStatus", 1, boost::bind(IrSensorsReading, _1, X ));
  ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("simulation/IRSensorsStatus", 1, IrSensorsReading);

  // Declare you publishers and service servers
  ros::Publisher Mahalanobis = nh_glob.advertise<std_msgs::Float32>("/Mahalanobis", 1);
  ros::Publisher estPosture = nh_glob.advertise<geometry_msgs::PoseWithCovariance>("estimatedPosture", 1);



  // Initialize sensors
  double x1, y1;
  nh_loc.param("/simulation/sensor/x1_pos", x1, 0.0) ;
  nh_loc.param("/simulation/sensor/y1_pos", y1, -0.1) ;	// First one on the right of the robot

  double x2, y2;
  nh_loc.param("/simulation/sensor/x2_pos", x2, 0.0) ;
  nh_loc.param("/simulation/sensor/y2_pos", y2, 0.1) ;		// Second one on the left of the robot

  //  Global world parameters
	double xSpacing, ySpacing, lineThickness;					// [m]
	nh_loc.param("/simulation/sensor/x_spacing", xSpacing, 0.5) ;
	nh_loc.param("/simulation/sensor/y_spacing", ySpacing, 1.0) ;
	nh_loc.param("/simulation/sensor/line_thickness", lineThickness, 0.005) ;

	//	Create the world object
	const World world(xSpacing, ySpacing, lineThickness);

  // Encoder resolution
  double encodersResolution;
  nh_loc.param("encoders_resolution", encodersResolution, (double)1.0) ;

  // Inizialize the vector
  robotSensors.AddSensor( Sensor( x1, y1, world ) ) ;
  robotSensors.AddSensor( Sensor( x2, y2, world ) ) ;

  // Initialize sigmaTuning and sigmaMeasurement
  sigmaInit( sigmaTuning, encodersResolution, 1 );
  sigmaInit( sigmaMeasurement, lineThickness, 0 );

  ros::Rate estimatorRate(150);

  while( ros::ok() ) {

    ros::spinOnce();

    // Compute rotation
    Eigen::Vector2d rotation = currentReading - previousReading;

    // Compute input
    Eigen::Vector2d input = jointToCartesian*rotation;

    // Compute evolution model
    EvolutionModel(X, input); //  SURE UNTILL HERE, EVOLUTION MODEL WORKS JUST FINE (BUT NO LIMITATION IN THETA)


    // Update matrix A and B
    UpdateMatrix(X, input, A, B); //  SURE ABOUT THE UPDATE


    // Update propagation error matrix
    kalman.Propagation(P, A, B);  //  PROPAGATIONS WORKS FINE (sigmatuning = 0.0)

    // Check for any measurements
    for(const auto& measurement : measurements ) {  //  SURE ABOUT THE MEASUREMENTS, THE FUNCTION RETURNS MEANINGFULL DATA

      double dMaha;
      Eigen::MatrixXd C(1, 3);
      double innov;
      //  First filter the type of line
      if( measurement.lineType == utility::LINETYPE::HORIZONTAL ) {

        C << 0, 1,  measurement.activeSensor->RelativePosition().x*cos(X(2)) - measurement.activeSensor->RelativePosition().y*sin(X(2)) ;

        const double Y = measurement.activeSensor->AbsolutePosition().y;
        const double Yhat = measurement.lineIndex;

        // ROS_INFO_STREAM("HORIZONTAL ");
        // ROS_INFO_STREAM(" Y    = " << Y );
        // ROS_INFO_STREAM(" Yhat = " << Yhat );

        innov = Y - Yhat ;

        dMaha = kalman.ComputeMahalanobis( innov, C, P );

      } else {

        C << 1, 0, - measurement.activeSensor->RelativePosition().x*sin(X(2)) - measurement.activeSensor->RelativePosition().y*cos(X(2)) ;

        const double Y =  measurement.activeSensor->AbsolutePosition().x;
        const double Yhat = measurement.lineIndex ;

        // ROS_INFO_STREAM("VERTICAL ");
        // ROS_INFO_STREAM(" Y    = " << Y );
        // ROS_INFO_STREAM(" Yhat = " << Yhat );

        innov =  Y - Yhat ;

        dMaha = kalman.ComputeMahalanobis( innov, C, P );

      }
      // ROS_INFO_STREAM("dMaha : " << dMaha);
      if( dMaha <= threshold ) {

        //  Only if we referred to a good line, update
        kalman.Estimation(P, X, C, innov) ;
        // ROS_INFO_STREAM("covariance matrix :" << P);


      }

      //  Publish result for the Mahalanobis distance
      std_msgs::Float32 currentDist;
      currentDist.data = dMaha;
      Mahalanobis.publish( currentDist );

      //  Publish estimated posture and standard deviations
/*
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
*/
      geometry_msgs::PoseWithCovariance estimatedPosture;
      estimatedPosture.pose.position.x = X(0);
      estimatedPosture.pose.position.y = X(1);
<<<<<<< HEAD
      estimatedPosture.pose.orientation = utility::ToQuaternion<geometry_msgs::Quaternion>( X(2) );

      // estimatedPosture.covariance.data[0] = sqrt(P(0,0));
      // estimatedPosture.covariance[1].data = sqrt(P(1,1));
      // estimatedPosture.covariance[2].data = sqrt(P(2,2));
=======
      estimatedPosture.pose.orientation = utility::ToQuaternion(X(2));
      estimatedPosture.covariance.data.push_back( sqrt(P(0,0)) );
      estimatedPosture.covariance.data.push_back( sqrt(P(1,1)) );
      estimatedPosture.covariance.data.push_back( sqrt(P(2,2)) );
>>>>>>> e84cc05951ea0e132041f3fef3fd690f588e9c94
      estPosture.publish( estimatedPosture );


    }

    previousReading = currentReading ;
    measurements.clear();

    estimatorRate.sleep();
  }






  return 0;
}
