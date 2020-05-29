#ifndef UTILITY_H
#define UTILITY_H

/**
 * \file marker init 
 * \brief contains the inizialization of the marker
 * \author Bianca & Andrea
 * \version 0.1
 * \date 28/05/2020
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
        In order to make a more readable and less complex code, it has been created
        this header file that inizializes the markers.
 *
 */

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace utility
{


  void InitMarker(visualization_msgs::Marker& robotMarker)
  {

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    robotMarker.header.frame_id = "/map";
    robotMarker.header.stamp = ros::Time::now();


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    robotMarker.ns = "RobotShape";
    robotMarker.id = 0;


    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    robotMarker.type = visualization_msgs::Marker::ARROW;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    robotMarker.action = visualization_msgs::Marker::ADD;

    //  Set the marker scale
    robotMarker.scale.x = 1.0;
    robotMarker.scale.y = 1.0;
    robotMarker.scale.z = 1.0;

    //  Set the marker color
    robotMarker.color.r = 0.0f;
    robotMarker.color.g = 1.0f;
    robotMarker.color.b = 0.0f;
    robotMarker.color.a = 1.0f;


    robotMarker.lifetime = ros::Duration();

  }


  void UpdateMarker(visualization_msgs::Marker& robotMarker, const geometry_msgs::PoseStamped& robotPosture)
  {
    robotMarker.pose.position.x = robotPosture.pose.position.x ;
    robotMarker.pose.position.y = robotPosture.pose.position.y ;

  }























}

#endif  //UTILITY_H
