#ifndef UTILITY_H
#define UTILITY_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>

namespace utility
{

////////////////////////////////////////////////~ INDEX ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  //  Wikipedia comes in help for solving the tedius angles conversions
  struct Quaternion {
  Quaternion()=default;

    double w {0.0};
    double x {0.0};
    double y {0.0};
    double z {0.0};
  };

  struct EulerAngles {
    EulerAngles()=default;
    EulerAngles(const double _yaw) : yaw(_yaw) {/* The rest are defaults */}

    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};
  };

  //  Converting specified Eulers angles to a Quaterion
  Quaternion ToQuaternion(const double& yaw, const double& pitch, const double& roll); // yaw (Z), pitch (Y), roll (X)

  //  Function delagation allowing more user friendly interface
  inline Quaternion ToQuaternion(const EulerAngles angles) {ToQuaternion( angles.yaw, angles.pitch, angles.roll); } ;

  //  Converting a quatersion to Euler Angles
  EulerAngles ToEulerAngles(const Quaternion q);

  //  Initialize a marker for the robot (remember to deprecate)
  void InitMarker(visualization_msgs::Marker& robotMarker);

  //  Used to upate the robot marker (remember to deprecate)
  void UpdateMarker(visualization_msgs::Marker& robotMarker,
                    const geometry_msgs::PoseStamped& robotPosture);



///////////////////////////////////////////~ FUNCTIONS DECLARATION ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

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
  //  In order to not do all the computations twice, copy the values in the nearly
  //  published message
  {
    robotMarker.pose.position.x = robotPosture.pose.position.x ;
    robotMarker.pose.position.y = robotPosture.pose.position.y ;

  }























}

#endif  //UTILITY_H
