#ifndef UTILITY_H
#define UTILITY_H

/**
 * \file utility unit
 * \brief contains useful functions, UDT and marker inizialization
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
        This .h file contains some functions and User Defined DataTypes needed
        by the other file in this package. However, even if the listed elements
        are needed by the code, they do not belong to any "concept". Meaning that
        the functions and UDT listed below must be present in this package but
        philosophically do not belong to any branch. For example, the marker
        must be initialized, and function are provided for this. However, ideogically
        this operation does not belong in the scope of the (2, 0) robot or any
        other file in this project.

        For this practical considerations, they are all here. The use of the namespace
        makes clear when thery are called around the code.

        Everything is first listed, like in an index in the form of a forward
        declaration, then all the elements are defined in the order they are presented.
 *
 */

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <iostream>
#include <string>

namespace utility
{

////////////////////////////////////////////////~ INDEX ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

  //  Forward declarations
  struct Quaternion;

  template<class QuaternionTemplated>
  struct item_return;

  struct EulerAngles;

  struct Pose2D;

  enum SENSOR { RIGHT, LEFT };

  enum COLOR { RED, GREEN };


  //  Constrain an angle in the range [-M_PI, M_PI]
  inline const double LimitAngle( double a) ;


  //  Converting specified Eulers angles to a Quaterion
  [[deprecated("use the templated one for more consistent code")]]
  Quaternion ToQuaternion(const double& yaw, const double& pitch, const double& roll); // yaw (Z), pitch (Y), roll (X)


  //  Converting specified Eulers angles to a Quaterion (generic)
  template<class QuaternionTemplated>
  typename item_return<QuaternionTemplated>::type ToQuaternion(const double yaw, const double pitch=0.0, const double roll=0.0);


  //  Converting a quatersion to Euler Angles
  [[deprecated("use the templated one for more consistent code")]]
  EulerAngles ToEulerAngles(const Quaternion q);


  //  Converting a quatersion to Euler Angles
  template<class QuaternionTemplated>
  EulerAngles ToEulerAngles(const QuaternionTemplated& q);

  //  Initialize a marker for the robot
  [[deprecated("You should not use the visualization marker as the URDF is now available to use")]]
  void InitMarker(visualization_msgs::Marker& robotMarker);


  //  Initialize a marker for the line strip (robot generated path)
  void InitLineStrip(visualization_msgs::Marker& generatedPath);


  //  Used to upate the robot marker (remember to deprecate)
  [[deprecated("You should not use the visualization marker as the URDF is now available to use")]]
  void UpdateMarker(const geometry_msgs::PoseStamped& robotPosture,
                            visualization_msgs::Marker& robotMarker );


  //  Used to upate the path generated by the robot in the simulation
  void UpdatePath(const geometry_msgs::PoseStamped& robotPosture,
                            visualization_msgs::Marker& generatedPath );

  //  Function that returns a marker for the active sensor
  visualization_msgs::Marker PlaceActiveSensor(const Pose2D& point, const SENSOR& activeSensor);

  //  Function creating a marker accordingly with the active sensor
  visualization_msgs::Marker PlaceMarker(const Pose2D& point, const COLOR& markerColor, const std::string side, const int index);






///////////////////////////////////////////~ DECLARATION ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


struct Quaternion {
Quaternion()=default;

Quaternion(const double _w, const double _x,
            const double _y, const double _z) : w(_w), x(_x), y(_y), z(_z) {}

  double w {1.0};
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

template<class QuaternionTemplated>
struct item_return {

  typedef QuaternionTemplated type;
};


struct EulerAngles {
  EulerAngles()=default;
  EulerAngles(const double _yaw) : yaw(_yaw) {/* The rest are defaults */}

  double roll {0.0};
  double pitch {0.0};
  double yaw {0.0};
};

struct Pose2D {
  Pose2D()=default;

  Pose2D(const double _x, const double _y, const double _theta=0) : x(_x), y(_y), theta(_theta) {}

  double x { 0.0 } ;
  double y { 0.0 } ;
  double theta { 0.0 } ;
};

  template<class QuaternionTemplated>
  typename item_return<QuaternionTemplated>::type ToQuaternion(const double yaw, const double pitch, const double roll)
  {

      // Abbreviations for the various angular functions
      double cy = cos(yaw * 0.5);
      double sy = sin(yaw * 0.5);
      double cp = cos(pitch * 0.5);
      double sp = sin(pitch * 0.5);
      double cr = cos(roll * 0.5);
      double sr = sin(roll * 0.5);

      QuaternionTemplated q;
      q.w = cr * cp * cy + sr * sp * sy;
      q.x = sr * cp * cy - cr * sp * sy;
      q.y = cr * sp * cy + sr * cp * sy;
      q.z = cr * cp * sy - sr * sp * cy;

      return q;

  }



  Quaternion ToQuaternion(const double& yaw, const double& pitch, const double& roll)
  // yaw (Z), pitch (Y), roll (X) converted to a Quaternion
  {
      // Abbreviations for the various angular functions
      double cy = cos(yaw * 0.5);
      double sy = sin(yaw * 0.5);
      double cp = cos(pitch * 0.5);
      double sp = sin(pitch * 0.5);
      double cr = cos(roll * 0.5);
      double sr = sin(roll * 0.5);

      Quaternion q;
      q.w = cr * cp * cy + sr * sp * sy;
      q.x = sr * cp * cy - cr * sp * sy;
      q.y = cr * sp * cy + sr * cp * sy;
      q.z = cr * cp * sy - sr * sp * cy;

      return q;
  }



  EulerAngles ToEulerAngles(const Quaternion q)
  // Quaternion converted to Euler Angles: yaw (Z), pitch (Y), roll (X)
  {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
  }



  template<class QuaternionTemplated>
  EulerAngles ToEulerAngles(const QuaternionTemplated& q)
  {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
  }



  inline const double LimitAngle(double a)
  {
      while( a >=  M_PI ) a -= 2*M_PI ;
      while( a <  -M_PI ) a += 2*M_PI ;
      return a ;
  }


  void InitMarker(visualization_msgs::Marker& robotMarker)
  //  More practical to have the initialization here, it is needed but not
  //  a key point the scope of the simulation
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

    //  Set identity as initial orientation
    robotMarker.pose.orientation.w = 1.0 ;
    robotMarker.pose.orientation.x = 0.0 ;
    robotMarker.pose.orientation.y = 0.0 ;
    robotMarker.pose.orientation.z = 0.0 ;

    //  Set the marker scale
    robotMarker.scale.x = 1.0;
    robotMarker.scale.y = 0.4;
    robotMarker.scale.z = 0.4;

    //  Set the marker color
    robotMarker.color.r = 0.0f;
    robotMarker.color.g = 1.0f;
    robotMarker.color.b = 0.0f;
    robotMarker.color.a = 1.0f;

    //  Set the lifetime untill the next message
    robotMarker.lifetime = ros::Duration();

  }


  void InitLineStrip(visualization_msgs::Marker& generatedPath)
  {

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    generatedPath.header.frame_id = "/map";
    generatedPath.header.stamp = ros::Time::now();


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    generatedPath.ns = "RobotPath";
    generatedPath.id = 0;


    // Set the marker type.  For generate a path we want a line strip
    generatedPath.type = visualization_msgs::Marker::LINE_STRIP;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    generatedPath.action = visualization_msgs::Marker::ADD;

    //  Set identity as initial orientation
    generatedPath.pose.orientation.w = 1.0 ;
    generatedPath.pose.orientation.x = 0.0 ;
    generatedPath.pose.orientation.y = 0.0 ;
    generatedPath.pose.orientation.z = 0.0 ;

    // LINE_STRIP markers use only the x component of scale, for the line width
    generatedPath.scale.x = 0.05;

    //  Set the line color
    generatedPath.color.r = 0.0f;
    generatedPath.color.g = 0.0f;
    generatedPath.color.b = 1.0f;
    generatedPath.color.a = 1.0f;

  }


  void UpdateMarker(const geometry_msgs::PoseStamped& robotPosture,
                            visualization_msgs::Marker& robotMarker )
  //  In order to not do all the computations twice, copy the values in the nearly
  //  published message
  {
    robotMarker.pose.position.x = robotPosture.pose.position.x ;
    robotMarker.pose.position.y = robotPosture.pose.position.y ;


    robotMarker.pose.orientation.w = robotPosture.pose.orientation.w ;
    robotMarker.pose.orientation.x = robotPosture.pose.orientation.x ;
    robotMarker.pose.orientation.y = robotPosture.pose.orientation.y ;
    robotMarker.pose.orientation.z = robotPosture.pose.orientation.z ;

  }


  void UpdatePath(const geometry_msgs::PoseStamped& robotPosture,
                            visualization_msgs::Marker& generatedPath )
  {
    generatedPath.points.push_back( robotPosture.pose.position );
  }


  int index_left = -1;
  int index_right = -1;
  visualization_msgs::Marker PlaceActiveSensor(const Pose2D& point, const SENSOR& activeSensor)
  {
    COLOR markerColor;

    if( activeSensor == SENSOR::RIGHT ) {

      index_right ++;
      markerColor = COLOR::RED ;
      return PlaceMarker(point, markerColor, "Right", index_right) ;
    }

    if( activeSensor == SENSOR::LEFT ) {

      index_left ++;
      markerColor = COLOR::GREEN ;
      return PlaceMarker(point, markerColor, "Left", index_left) ;
    }

  }



  visualization_msgs::Marker PlaceMarker(const Pose2D& point, const COLOR& markerColor, const std::string side, const int index)
  {
    //  Instantiate the marker
    visualization_msgs::Marker sensorMarker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    sensorMarker.header.frame_id = "/map";
    sensorMarker.header.stamp = ros::Time::now();


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    sensorMarker.ns = side + "SensorMarker";
    sensorMarker.id = index;



    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    sensorMarker.type = visualization_msgs::Marker::CYLINDER;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    sensorMarker.action = visualization_msgs::Marker::ADD;

    //  Set identity as orientation
    sensorMarker.pose.orientation.w = 1.0 ;
    sensorMarker.pose.orientation.x = 0.0 ;
    sensorMarker.pose.orientation.y = 0.0 ;
    sensorMarker.pose.orientation.z = 0.0 ;

    //  Set position
    sensorMarker.pose.position.x = point.x ;
    sensorMarker.pose.position.y = point.y ;

    //  Set the marker scale
    sensorMarker.scale.x = 0.01;
    sensorMarker.scale.y = 0.01;
    sensorMarker.scale.z = 0.5;

    //  Set the marker color
    switch ( markerColor ) {
      case COLOR::RED :
        sensorMarker.color.r = 1.0f;
        sensorMarker.color.g = 0.0f;
        break;

      case COLOR::GREEN :
      sensorMarker.color.r = 0.0f;
      sensorMarker.color.g = 1.0f;
      break;
    };
    sensorMarker.color.b = 0.0f;
    sensorMarker.color.a = 1.0f;

    //  Set the lifetime as no end
    sensorMarker.lifetime = ros::Duration();

    return sensorMarker;
  }





















}

#endif  //UTILITY_H
