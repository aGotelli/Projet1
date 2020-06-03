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
 *    ° CheckSensorStatus (service)
 *		° /visualization_marker_array
 *
 * Description
            	The aim of this file is to provide and interface to simulate the
			sensor behaviur. The node simply subscribes to the robot position.
			With the knowledge of this position and the coordinates of the sensor
			in the robot frame it simulates the sensors.

 *
 */

#include "simulation/sensor.h"
#include "simulation/utility.h"
#include <vector>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "simulation_messages/IRSensors.h"




//	Position of the robot
geometry_msgs::Pose robotPosture;

void RobotPostureReceived(const geometry_msgs::PoseStamped::ConstPtr& _robotPosture)
{
	robotPosture = _robotPosture->pose;
}


//	Vector of sensors
std::vector<Sensor> robotSensors;

//	Positions of triggered sensors
visualization_msgs::MarkerArray sensorsActivations;

//  Check the sensor status and return the filled message
simulation_messages::IRSensors SensorsStatus()
{
	simulation_messages::IRSensors status;
	sensorsActivations.markers.clear();

		for(const auto& sens : robotSensors) {
			sens.UpdateTransform( robotPosture );
			sens.CheckStatus();
		}

		//	check status of the sensor on the right
		if( robotSensors[0].GetState() ) {
			status.sens1 = true;

			//	Add the new marker
			sensorsActivations.markers.push_back(	utility::PlaceActiveSensor( robotSensors[0].AbsolutePosition(),
																																										utility::SENSOR::RIGHT ) );

		}
		//	check status of the sensor on the left
		if( robotSensors[1].GetState() ) {
			status.sens1 = true;

			//	Add the new marker
			sensorsActivations.markers.push_back(	utility::PlaceActiveSensor( robotSensors[1].AbsolutePosition(),
																																										utility::SENSOR::LEFT ) );
		}

		return status;

}

void EnsureSameWorld(std::vector<Sensor>& robotSensors)
{
	// const World reference = robotSensors[0].ItsWorld();
	//
	// for(const auto& sensor : robotSensors ) {
	// 	if(sensor.ItsWorld() == reference) {
	//
	// 	}
	// }
	//

}





int main (int argc, char** argv)
{

  // ROS Initialization
  ros::init(argc, argv, "sensor");

  //  Required NodeHandles
  ros::NodeHandle nh_loc("~"), nh_glob;

  //  Global world parameters
  double xSpacing, ySpacing, lineThickness;					// [m]
  nh_glob.param("x_spacing", xSpacing, 0.2) ;
  nh_glob.param("y_spacing", ySpacing, 0.2) ;
	nh_glob.param("line_thickness", lineThickness, 0.005) ;

	const World world(xSpacing, ySpacing, lineThickness);

  // Sensor parameters
	double x1, y1;
	nh_loc.param("x1_pos", x1, 0.0) ;
	nh_loc.param("y1_pos", y1, -0.2) ;	// First one on the right of the robot

	double x2, y2;
	nh_loc.param("x2_pos", x2, 0.0) ;
	nh_loc.param("y2_pos", y2, 0.2) ;		// First one on the left of the robot

	// Inizialize the vector
	robotSensors.push_back( Sensor( x1, y1, world ) ) ;
	robotSensors.push_back( Sensor( x2, y2, world ) ) ;



	//	Subscribe to the published robot position to obtain infromation about its pose
	ros::Subscriber RecivedPosture = nh_glob.subscribe<geometry_msgs::PoseStamped>("RobotPosture", 1, RobotPostureReceived);

	//	Publishes the sensors status
	ros::Publisher IRSensors = nh_glob.advertise<simulation_messages::IRSensors>("SensorStatus", 1);

	//	Publishes marker to help understand
	ros::Publisher SensorsDisplay = nh_glob.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);


	ros::Rate rate(150);

  while (ros::ok()) {
      ros::spinOnce();

			IRSensors.publish( SensorsStatus() ) ;

			if( sensorsActivations.markers.size() )
				SensorsDisplay.publish( sensorsActivations );

      rate.sleep();
  }



}
