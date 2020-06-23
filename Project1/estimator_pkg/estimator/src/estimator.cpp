/**
 * \file estimator
 * \brief Kalman filter estimator
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 10/06/2020
 *
 * \param[in]
 * ° threshold
 * ° /robot_2_0/encoders_resolution
 * ° sigma_tuning
 * ° estimator_frequency
 * ° /sensor/x_spacing
 * ° /sensor/y_spacing
 * ° /sensor/line_thickness
 * ° /robot_2_0/x_init
 * ° /robot_2_0/y_init
 * ° /robot_2_0/theta_init
 * ° x_scost
 * ° y_scost
 * ° theta_rot
 * ° sigma_x
 * ° sigma_y
 * ° sigma_theta
 * ° /robot_2_0/wheel_radius
 * ° /robot_2_0/a
 * ° /sensor/x1_pos
 * ° /sensor/y1_pos
 * ° /sensor/x2_pos
 * ° /sensor/y2_pos
 *
 * Subscribes to: <BR>
 *    ° simulation/EncodersReading
 *    ° simulation/IRSensorsStatus
 *
 * Publishes to: <BR>
 *    ° /EstimatedPosture
 *    ° /Measurements
 *    ° /Velocities
 *
 * Description
            The aim of this file is to create an estimator for localise the robot
          using the result of a simulation. The estimator works as an usual
          Kalman Filter based estimator. It is designed for having a ROS interface
          so it can be used in any real robot which satisfies the criteria of this
          project.

            The estimator obtains the current readings from the encoders with a
          simple callback function. It uses the knowledge of the wheels rotations
          to perform a prediction of the current state and the error propagation.

            If some sensors are active, in the callback they are storaged in a
          vector. This phase is important, because there are two sensors located
          in different positions and they muste be distinguished.

            After the prediction, for every sensor in the vector, information
          about the measurement are obtained.

            At this point the Kalman Filter perform a coherence test and if
          the measurement satisfies the test, then the estimation is performed.



            The callbacks for the sensors states and the encoders readind differs
          from the point of view of the buffer size. This comes from the
          consideration that the estimator can run at a lower frequency than the
          simulation. Practically, this estimator is to be implemented in a real
          robot, outside the context of a simulation. In a real robot, encoders
          have their own frquency and resolution, so the estimator takes the last
          input form the encoders to retrieve the current rotations.  On the other
          hand, the sensors usually have a specific component, that works at best
          effort to publish the sensor state. Here comes the need of a larger buffer
          size: the estimator must know if a sensor was active in between the
          current and the last iteration. So, it must storage all the messages in
          the buffer and process them in the next iteration. 

            Once, it gets all the measurements, it performs the estimation per se.


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

//  Sensors which were activated in the time between two iterations
std::vector<Sensor> activatedSensors;

// State vector
Eigen::Vector3d X;
void IRSensorsReading(const simulation_messages::IRSensors::ConstPtr& state)
{

  //  Received sensors status
  if( state->sens1 )
    activatedSensors.push_back( robotSensors.sens1() );

  if( state->sens2 )
    activatedSensors.push_back( robotSensors.sens2() );

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh_glob, nh_loc("~");

  //  Global world parameters
  double xSpacing, ySpacing, lineThickness;					// [m]
  nh_glob.param("/sensor/x_spacing", xSpacing, 1.0) ;
  nh_glob.param("/sensor/y_spacing", ySpacing, 1.0) ;
  nh_glob.param("/sensor/line_thickness", lineThickness, 0.005) ;

  ROS_INFO_STREAM("World parameters : " << xSpacing << ", " << ySpacing << ", " << lineThickness );

  //	Create the world object
  const World world(xSpacing, ySpacing, lineThickness);

  // Filter parameters
  double threshold;
  nh_loc.param("threshold", threshold, 0.0 );
  //  Forgetting to set the threshold is a big mistake, the penality is regetting all the measurements

  //  Obatain the encoders resolutions
  double encodersResolution;
  nh_glob.param("/robot_2_0/encoders_resolution", encodersResolution, (double)360) ;

  ROS_INFO_STREAM("Encoders resolution : " << encodersResolution );

  double sigmaTuning;
  nh_loc.param("sigma_tuning", sigmaTuning, sqrt(pow(2/encodersResolution*M_PI, 2)/12));
  //  Default only takes into account the encoders resolution, with a perfect an high freq. model


  double sigmaX, sigmaY, sigmaTheta;
  nh_loc.param("sigma_x", sigmaX, 0.0);
  nh_loc.param("sigma_y", sigmaY, 0.0);
  nh_loc.param("sigma_theta", sigmaTheta, 0.0);

  const double sigmaMeasurement = sqrt(pow(lineThickness, 2)/12);

  ROS_INFO_STREAM("Sigma tuning  : " << sigmaTuning );

  int frequency;
  nh_loc.param<int>("estimator_frequency", frequency, 150 );

  ROS_INFO_STREAM("Estimator frequency : " << frequency );


  // Initial pose
  double xInit, yInit, thetaInit;
  nh_glob.param("/robot_2_0/x_init", xInit, 0.0);
  nh_glob.param("/robot_2_0/y_init", yInit, 0.0);
  nh_glob.param("/robot_2_0/theta_init", thetaInit, 0.0);

  // Error in initial position
  double xScost, yScost, thetaRot;
  nh_loc.param("x_scost", xScost, 0.0);
  nh_loc.param("y_scost", yScost, 0.0);
  nh_loc.param("theta_rot", thetaRot, 0.0);


  ROS_INFO_STREAM("Initial position : " << xInit + xScost << ", " << yInit + yScost<< ", " << thetaInit + thetaRot);

  // Robot parameters
  double wheelRadius, a;
  nh_glob.param("/robot_2_0/wheel_radius", wheelRadius, 0.05);
  nh_glob.param("/robot_2_0/a", a, 0.2);
  const double trackGauge = 2*a;

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
  ros::Subscriber readIRSensors = nh_glob.subscribe<simulation_messages::IRSensors>("/IRSensorsStatus", 5, IRSensorsReading);

  // Declare your publishers and service servers
  ros::Publisher estPosture = nh_glob.advertise<geometry_msgs::PoseWithCovariance>("/EstimatedPosture", 1);
  ros::Publisher shareMeasurements = nh_glob.advertise<estimator_messages::Measurement>("/Measurements", 1);
  ros::Publisher pubInput = nh_glob.advertise<geometry_msgs::Twist>("/EstimatedVelocities", 1);



  //  State vector
  X <<        xInit+xScost              ,
              yInit+yScost              ,
        (thetaInit+thetaRot)*M_PI/180   ;


  //   Joint to cartesian matrix
  Eigen::Matrix2d jointToCartesian;
  jointToCartesian <<       wheelRadius/2      ,      wheelRadius/2       ,
                       wheelRadius/trackGauge  , -wheelRadius/trackGauge  ;



  //  Initialize the Kalman filter with the parameters
  KalmanFilter kalman(jointToCartesian, sigmaMeasurement, sigmaTuning, sigmaX, sigmaY, sigmaTheta*M_PI/180);


  //  Declaration of the kalman filter matrices
  Eigen::Matrix3d A;
  Eigen::MatrixXd B(3,2);
  Eigen::Matrix3d P = kalman.Pinit();

  Eigen::Vector2d input;

  ros::Rate estimatorRate( frequency );


  //  Initialize the time
  ros::Time prevTime = ros::Time::now() ;

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
    for(const auto& sensor : activatedSensors ) {

      kalman.EvaluateMeasurement(sensor, X, P);

    }

    //  Publish estimated posture and standard deviations
    estPosture.publish( Estimation(X, P) );

    // Time handling
    ros::Time currentTime = ros::Time::now() ;
    ros::Duration timeElapsed = currentTime - prevTime;
    prevTime = currentTime ;

    //  Publish velocities
    geometry_msgs::Twist vel;
    vel.linear.x = input(0)/timeElapsed.toSec() ;
    vel.angular.z = input(1)/timeElapsed.toSec() ;
    pubInput.publish( vel );

    //  Update before next iteration
    previousReading = currentReading ;

    //  Clear vector for not repeating
    activatedSensors.clear();

    //  Wait remaining time
    estimatorRate.sleep();
  }


  return 0;
}
