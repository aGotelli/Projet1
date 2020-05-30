/**
 * \file sensor node
 * \brief file for sensor behaviour
 * \author Bianca & Andrea
 * \version 0.1
 * \date 30/05/2020
 *
 * \param[in]
    world param : x_spacing, y_spacing

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

//ROS
#include "ros/ros.h"
#include "simulation/sensor.h"
#include "simulation_messages/SensorStatus.h"



// Vector of sensors
std::vector<Sensor> robotSensor;



// Service Callback
bool ServiceCallback(simulation_messages::SensorStatus::Request &req,
					simulation_messages::SensorStatus::Response &res){

		for(const auto& sens : robotSensor){
			sens.UpdateTrasform();
			sens.CheckSatus();
		}

		res.status.sens1 = robotSensor[0].GetState();
		res.status.sens2 = robotSensor[1].GetState();

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
	nh_loc.param("x1_pos", x1, 0.2) ;
	nh_loc.param("y1_pos", y1, 0.2) ;

	double x2, y2;
	nh_loc.param("x2_pos", x2, 0.2) ;
	nh_loc.param("y2_pos", y2, 0.2) ;

	// Inizialize the vector
	robotSensor.push_back(sens1(x1_pos, y1_pos));
	robotSensor.push_back(sens2(x2_pos, y2_pos));



  	// Declare your node's subscriptions and service clients
  	ros::ServiceServer SensorService = nh_glob.advertiseService("sensor_status", ServiceCallback);



  	ros::Rate rate(100);
    while (ros::ok()){
        ros::spinOnce();


        rate.sleep();
    }



}
