/**
 * \file sensor node
 * \brief file for sensor behaviour
 * \author Bianca & Andrea
 * \version 0.1
 * \date 30/05/2020
 *
 * \param[in]
    world param 	: x_spacing, y_spacing
		sensors param	: x1_pos, x1_pos
		sensors param	: x2_pos, x2_pos

 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
            The aim of this file is to describe the sensors' behaviour
 *
 */

#include "simulation/sensor.h"
#include "simulation_messages/SensorStatus.h"
#include "simulation/utility.h"
#include <vector>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>


//	Vector of sensors
std::vector<Sensor> robotSensor;

//	Positions of triggered sensors
visualization_msgs::MarkerArray sensorsActivations;


// Service Callback
bool ServiceCallback(simulation_messages::SensorStatus::Request& from,
					simulation_messages::SensorStatus::Response& current)
{

		for(const auto& sens : robotSensor) {
			sens.UpdateTransform( from.robotPosture );
			sens.CheckStatus();
		}

		if( robotSensor[0].GetState() ) {
			current.status.sens1 = true;
			sensorsActivations.markers.push_back(	utility::PlaceActiveSensor( robotSensor[0].AbsolutePosition(),
																																										utility::SENSOR::RIGHT ) );

		}

		if( robotSensor[1].GetState() ) {
			current.status.sens1 = true;
			sensorsActivations.markers.push_back(	utility::PlaceActiveSensor( robotSensor[1].AbsolutePosition(),
																																										utility::SENSOR::LEFT ) );
		}

	return true;
}

int main (int argc, char** argv)
{

  // ROS Initialization
  ros::init(argc, argv, "sensor");

  //  Required NodeHandles
  ros::NodeHandle nh_loc("~"), nh_glob;

  //  Global world parameters
  double xSpacing, ySpacing;                    // [m]
  nh_glob.param("x_spacing", xSpacing, 0.2) ;
  nh_glob.param("y_spacing", ySpacing, 0.2) ;

  // Sensor parameters
	double x1, y1;
	nh_loc.param("x1_pos", x1, 0.0) ;
	nh_loc.param("y1_pos", y1, -0.2) ;	// First one on the right of the robot

	double x2, y2;
	nh_loc.param("x2_pos", x2, 0.0) ;
	nh_loc.param("y2_pos", y2, 0.2) ;		// First one on the left of the robot

	// Inizialize the vector
	robotSensor.push_back( Sensor( x1, y1 ));
	robotSensor.push_back( Sensor( x2, y2 ));

  	// Declare your node's subscriptions and service clients
  	ros::ServiceServer SensorService = nh_glob.advertiseService("CheckSensorStatus", ServiceCallback);

		ros::Publisher markersPub = nh_glob.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

  	ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();

				markersPub.publish( sensorsActivations );

        rate.sleep();
    }



}
