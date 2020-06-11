#ifndef   WORLD_H
#define   WORLD_H

/**
 * \file world header file
 * \brief contains declaration and properties of the world
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
            This file contains the definition of the World. As it is used in a
          multiple other files, it is more useful to have it declared in a
          specific header file to include where needed.


          Several choiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            * C.21, C.22, C.60, C.80, C.81
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10
            ° R.20, R.21, R.23, R.30

 *
 */
 #include <cassert>
 #include <ros/ros.h>

// Definition of World class
class World {
public:
  World()=default;

  World(const double _xSpacing,
        const double _ySpacing) :
          xSpacing( _xSpacing ),
          ySpacing( _ySpacing ) { /*  lineWidth has default value  */
            ROS_DEBUG_STREAM("User-defined world only for line offset") ;

            if( xSpacing < 0 || ySpacing < 0 ) {
              ROS_ERROR_STREAM("YOU MUST USE POSITIVE VALUES FOR WORLD PARAMETERS");

              ros::shutdown();
            }

          }

  World(const double _xSpacing,
        const double _ySpacing,
          const double _lineThickness) :
            xSpacing( _xSpacing ),
            ySpacing( _ySpacing ),
            lineThickness( _lineThickness ) {
              ROS_DEBUG_STREAM("Completely user-defined world. xSpacing : " <<
              xSpacing << " ySpacing : " << ySpacing << " lineThickness : " << lineThickness) ;

              if( xSpacing < 0 || ySpacing < 0 || lineThickness < 0) {
                ROS_ERROR_STREAM("YOU MUST USE POSITIVE VALUES FOR WORLD PARAMETERS");

                ros::shutdown();
              }
            }

  //  Copy contructor
  World(const World& other)=default;

  //  Move contructor [no need]
  World(const World&&)=delete;

  //  The destructor
  ~World()=default;

  //  Copy operator [no need]
  World& operator=(const World&)=delete;

  //  Move operator [no need]
  World& operator=(World&&)=delete;

  //  Compare operator
  inline const bool operator==(const World& other) const;



   // Provided function to access private data
   inline const double& XSpacing() const {return xSpacing; }
   inline const double& YSpacing() const {return ySpacing; }
   inline const double& LineThickness() const {return lineThickness; }

  private:

   const double xSpacing { 1.0 };
   const double ySpacing { 1.0 };
   const double lineThickness { 0.005 };

 };




inline const bool World::operator==(const World& other) const
{
 if(this->xSpacing != other.XSpacing() )
  return false;

 if(this->ySpacing != other.YSpacing() )
  return false;

 if(this->lineThickness != other.LineThickness() )
  return false;

 return true;
}










#endif    //WORLD_H
