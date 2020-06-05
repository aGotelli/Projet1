#ifndef GROUND_H
#define GROUND_H


// ROS
#include <ros/ros.h>

#include "simulation/utility.h"
#include "simulation/sensor.h"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>


struct Size {
  Size(const double _x, const double _y) : x(_x), y(_y) {}

  const double x;
  const double y;
};

struct Ground {
  Ground(){}

  //  Copy Constructor
  Ground(const visualization_msgs::MarkerArray& _tile,
          const visualization_msgs::MarkerArray& _lines,
          const utility::Pose2D& _center) : tile(_tile), lines(_lines), center(_center) {}

  //  Copy Constructor
  Ground(const Ground& other) : tile(other.tile), lines(other.lines), center(other.center) {}

  visualization_msgs::MarkerArray tile;
  visualization_msgs::MarkerArray lines;
  utility::Pose2D center;
};


int t_index = 0;
const double TILE_THICKNESS = 0.01;
visualization_msgs::Marker PlaceTile( const utility::Pose2D& position, const World& world)
{
  visualization_msgs::Marker tile;

  tile.header.frame_id =  "map";
	tile.header.stamp = ros::Time::now();
  tile.lifetime = ros::Duration();
  tile.ns = "tiles";
  tile.id = t_index;
  t_index ++ ;


	// Set the marker type.
  tile.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  tile.action = visualization_msgs::Marker::ADD;

  //  Set identity as initial orientation
  tile.pose.orientation.w = 1.0 ;

  // LINE_STRIP markers use only the x component of scale, for the line width
  // tile.scale.x = world.XSpacing() - world.LineThickness();
  // tile.scale.y = world.YSpacing() - world.LineThickness();
  tile.scale.x = 5.0;
  tile.scale.y = 5.0;
  tile.scale.z = TILE_THICKNESS ;

  tile.pose.position.x = position.x ;
  tile.pose.position.y = position.y ;
  tile.pose.position.z = -TILE_THICKNESS ;

  //  Set the line color
  tile.color.r = 1.0f;
  tile.color.g = 1.0f;
  tile.color.b = 1.0f;
  tile.color.a = 1.0f;

  return tile;
}


enum TYPE{ HORIZONTAL, VERTICAL };
int h_index = 0;
int v_index = 0;
visualization_msgs::Marker PlaceLine( const utility::Pose2D& position, const Size& chunkSize, const TYPE& type, const World& world, const utility::Pose2D& chunkCenter)
{
  visualization_msgs::Marker line;

  line.header.frame_id =  "map";
	line.header.stamp = ros::Time::now();
  line.lifetime = ros::Duration();

  if(type == TYPE::VERTICAL) {
    line.ns = "vertical_line";
    line.id = v_index;
    v_index ++ ;
  }

  if(type == TYPE::HORIZONTAL) {
    line.ns = "horizontal_line";
    line.id = h_index;
    h_index ++ ;
  }

  line.type = visualization_msgs::Marker::LINE_STRIP;

  line.action = visualization_msgs::Marker::ADD;

  line.pose.orientation.w = 1.0 ;

  line.scale.x =  world.LineThickness();

  //  Set the marker color
  line.color.r = 0.0f;
  line.color.g = 0.0f;
  line.color.b = 0.0f;
  line.color.a = 1.0f;

  geometry_msgs::Point p1, p2;
  p1.x = position.x;
  p1.y = position.y;

  if(type == TYPE::VERTICAL) {
    p2.x = position.x;
    p2.y = position.y + copysign( chunkSize.y, chunkCenter.y);
  }

  if(type == TYPE::HORIZONTAL) {
    p2.x = position.x + copysign( chunkSize.x, chunkCenter.x);
    p2.y = position.y;
  }


  line.points.push_back(p1);
  line.points.push_back(p2);

  return line;

}



Ground Generation( const utility::Pose2D& chunkCenter, const Size& chunkSize, const World& world )
{
  //  First allocate the memory for the array of markers comupting the numbers of
  //  tiles along x and y
  const int tilesNormalX = chunkSize.x/world.XSpacing() ;
  const int tilesNormalY = chunkSize.y/world.YSpacing() ;
  const int tilesNumber = tilesNormalX*tilesNormalY;
  const int linesNumber = tilesNumber + 2;
  visualization_msgs::MarkerArray tile;
  tile.markers.push_back( PlaceTile( utility::Pose2D( chunkCenter.x, chunkCenter.y ), world ) );

  visualization_msgs::MarkerArray lines;
  lines.markers.reserve( linesNumber );

  for(int i= 0; i <= tilesNormalX; i ++) {  //  Creating the "vertical" lines
    lines.markers.push_back( PlaceLine( utility::Pose2D( copysign( i*world.XSpacing(), chunkCenter.x), 0.0), chunkSize, TYPE::VERTICAL, world, chunkCenter) );
  }

  for(int j= 0; j <= tilesNormalY; j ++) {  //  Creating the "horizontal" lines
    lines.markers.push_back( PlaceLine( utility::Pose2D( 0.0, copysign( j*world.YSpacing(), chunkCenter.y)), chunkSize, TYPE::HORIZONTAL, world, chunkCenter) ) ;
  }

  return Ground(tile, lines, chunkCenter);
}




class WorldGenerator {
public:
  WorldGenerator()=default;

  WorldGenerator(const World& _world) : world(_world) {}

  void InitWorld();

  void ChuckBelonging(const geometry_msgs::PoseStamped& robotPosture);

  inline const std::vector<Ground>& Chunks() const {return chunks;}



private:

  const World world;

  std::vector<Ground> chunks ;

  const Size chunkSize{ Size(5.0f, 5.0f) };
};

void WorldGenerator::InitWorld()
{
  //  Initialize the world with a chink in every quadrants
  chunks.push_back( Generation( utility::Pose2D( chunkSize.x/2, chunkSize.y/2 ), chunkSize, world) ) ;

  chunks.push_back( Generation( utility::Pose2D( -chunkSize.x/2, chunkSize.y/2 ), chunkSize, world) ) ;

  chunks.push_back( Generation( utility::Pose2D( -chunkSize.x/2, -chunkSize.y/2 ), chunkSize, world) ) ;

  chunks.push_back( Generation( utility::Pose2D( chunkSize.x/2, -chunkSize.y/2 ), chunkSize, world) ) ;

}


void WorldGenerator::ChuckBelonging(const geometry_msgs::PoseStamped& robotPosture)
{

}

#endif //GROUND_GENERATOR_H
