/**
 * \file
 * \brief
 * \author
 * \version 0.1
 * \date
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <boost/filesystem.hpp>

//ROS
#include "ros/ros.h"




int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "file_handler");

    ros::NodeHandle nh_glob;


    if( argc <  3) {
      ROS_ERROR_STREAM(ros::this_node::getName() << " not enought input arguments! You must provide the folder path and the file names ");
    }




}
