#ifndef GROUND_H
#define GROUND_H
/**
 * \file ground header file
 * \brief contains the functions used to provide the ground
 * \author Bianca & Andrea
 * \version 0.2
 * \date 05/06/2020
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
            This files contains the functions to generate e tiled floor. The floor is generated
          using chunk of size 5x5 meters. They consist of a white parallelepiped of the chunk
          size. Some lines are generated representing the separations line among the tiles.

            To avoid memory leaks, the use of smart pointers is strongly recommended. They are
          used in functions where we need to pass ownership of an object. (see guidelines)


          Several choiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            * C.21, C.22, C.60, C.80, C.81
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10
            ° R.20, R.21, R.23, R.30

 *
 */

// ROS
#include <ros/ros.h>

#include "simulation/utility.h"
#include "simulation/world.h"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <memory>


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
  Ground(const Ground& other)=default;

  Ground(const Ground&&)=delete;

  ~Ground()=default;

  Ground& operator=(const Ground&)=delete;

  Ground& operator=(Ground&&)=delete;

  visualization_msgs::MarkerArray tile;
  visualization_msgs::MarkerArray lines;
  utility::Pose2D center;

};


int t_index = 0;
const double TILE_THICKNESS = 0.01;
visualization_msgs::Marker PlaceTile( const utility::Pose2D& position, const Size& chunkSize)
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
  tile.scale.x = chunkSize.x;
  tile.scale.y = chunkSize.y;
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



int h_index = 0;
int v_index = 0;
visualization_msgs::Marker PlaceLine( const utility::Pose2D& position, const Size& chunkSize, const utility::LINETYPE& type, const World& world, const utility::Pose2D& chunkCenter)
{
  visualization_msgs::Marker line;

  line.header.frame_id =  "map";
	line.header.stamp = ros::Time::now();
  line.lifetime = ros::Duration();

  if(type == utility::LINETYPE::VERTICAL) {
    line.ns = "vertical_line";
    line.id = v_index;
    v_index ++ ;
  }

  if(type == utility::LINETYPE::HORIZONTAL) {
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

  if(type == utility::LINETYPE::VERTICAL) {
    p2.x = position.x;
    p2.y = position.y + copysign( chunkSize.y, chunkCenter.y);
  }

  if(type == utility::LINETYPE::HORIZONTAL) {
    p2.x = position.x + copysign( chunkSize.x, chunkCenter.x);
    p2.y = position.y;
  }


  line.points.push_back(p1);
  line.points.push_back(p2);

  return line;

}



std::unique_ptr<Ground> Generation( const utility::Pose2D& chunkCenter, const Size& chunkSize, const World& world )
{
  //  First allocate the memory for the array of markers comupting the numbers of
  //  tiles along x and y
  const int tilesNormalX = chunkSize.x/world.XSpacing() ;
  const int tilesNormalY = chunkSize.y/world.YSpacing() ;

  const int linesNumber = tilesNormalX + tilesNormalY + 2;

  visualization_msgs::MarkerArray tile;
  tile.markers.push_back( PlaceTile( utility::Pose2D( chunkCenter.x, chunkCenter.y ), chunkSize ) );

  visualization_msgs::MarkerArray lines;
  lines.markers.reserve( linesNumber );

  for(int i= 0; i <= tilesNormalX; i ++) {  //  Creating the "vertical" lines
    lines.markers.push_back( PlaceLine( utility::Pose2D( copysign( i*world.XSpacing(), chunkCenter.x), 0.0), chunkSize, utility::LINETYPE::VERTICAL, world, chunkCenter) );
  }

  for(int j= 0; j <= tilesNormalY; j ++) {  //  Creating the "horizontal" lines
    lines.markers.push_back( PlaceLine( utility::Pose2D( 0.0, copysign( j*world.YSpacing(), chunkCenter.y)), chunkSize, utility::LINETYPE::HORIZONTAL, world, chunkCenter) ) ;
  }

  return std::make_unique<Ground>(tile, lines, chunkCenter);
}




class WorldGenerator {
public:
  WorldGenerator()=default;

  WorldGenerator(const World& _world) : world(_world) {}

  void InitWorld();

  //void ChuckBelonging(const geometry_msgs::PoseStamped& robotPosture);

  inline const std::vector< std::unique_ptr<Ground> >& Chunks() const {return chunks;}



private:

  const World world;

  std::vector< std::unique_ptr<Ground> > chunks ;

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



#endif //GROUND_GENERATOR_H
