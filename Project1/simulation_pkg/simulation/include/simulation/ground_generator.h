#ifndef GROUND_GENERATOR_H
#define GROUND_GENERATOR_H


// ROS
#include <ros/ros.h>

#include "simulation/utility.h"
#include "simulation/sensor.h"



class GroundGenerator{
public:
  GroundGenerator()=default;

  GroundGenerator(const int _xLines,
        const int _yLines) :
          xLines( _xLines ),
          yLines( _yLines ) {}

 

 // Provided function to access private data
 inline const int& XLines() const {return xTiles; }
 inline const int& YLines() const {return yTIles; }

private:

 const int xLines { 4 };
 const int yLines { 4 };


};

// nell'utility

void TileGeneration(visualization_msgs::Marker& tilesHLine, visualization_msgs::Marker& tilesVLine) {

	tilesVLine.header.frame_id = tilesHLine.header.frame_id =  "map";
	tilesVLine.header.stamp = tilesHLine.header.stamp = ros::Time::now();
	tilesVLine.ns = "vertical lines";
	tilesHLine.ns = "horizontal lines"; 

	 // Set the marker type.  For generate a path we want a line strip
    tilesVLine.type = tilesHLine.type = visualization_msgs::Marker::LINE_STRIP;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    tilesVLine.action = tilesHLine.action = visualization_msgs::Marker::ADD;

    //  Set identity as initial orientation
    tilesVLine.pose.orientation.w = tilesHLine.pose.orientation.w = 1.0 ;
    
    // LINE_STRIP markers use only the x component of scale, for the line width
    tilesVLine.scale.x = tilesHLine.scale.x =  lineThickness;

    //  Set the line color
    tilesVLine.color.r = tilesHLine.color.r = 0.0f;
    tilesVLine.color.g = tilesHLine.color.g = 1.0f;
    tilesVLine.color.b = tilesHLine.color.b = 0.0f;
    tilesVLine.color.a = tilesHLine.color.a = 1.0f;

   
    // vertical lines
    for (uint32_t i = 0; i < yLines ; ++i)
    {

      double x = world.xSpacing()*(i - (yLines/2));
      geometry_msgs::Point p;
      p.x = x;
      p.y = 0.0f;
      p.z = 0.0f;

      // The line list needs two points for each line
      tilesVLine.points.push_back(p);
      p.x += xSpacing;
      tilesVLine.points.push_back(p);
    }

    // horizontal lines
    for (uint32_t i = 0; i < xLines ; ++i)
    {

      double y = world.xSpacing()*(i - (xLines/2));

      geometry_msgs::Point p;
      p.x = 0.0f;
      p.y = y;
      p.z = 0.0f;

      // The line list needs two points for each line
      tilesHLine.points.push_back(p);
      p.y += ySpacing;
      tilesHLine.points.push_back(p);
    }
}

void UpdateTiles(const Eigen::Matrix3d& oTm,
                            visualization_msgs::Marker& tilesHLine,
                             visualization_msgs::Marker& tilesVLine)
  {
  	const Eigen::Vector3d robotPose = oTm.col(2) ;

  	const int update_x = std::floor( robotPose[0]/world.XSpacing() )*world.XSpacing();
  	const int update_y = std::floor( robotPose[1]/world.YSpacing() )*world.YSpacing();

  	geometry_msgs::Point update_p;
  	p.x = update_x;
  	p.y = update_y;
  	p.z = 0.0f;


    tilesHLine.points.push_back( update_p);
    tilesVLine.points.push_back( update_p);

  }

#endif //GROUND_GENERATOR_H