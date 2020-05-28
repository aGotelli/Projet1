#ifndef UTILITY_H
#define UTILITY_H

#include <visualization_msgs/Marker.h>

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























}

#endif  //UTILITY_H
