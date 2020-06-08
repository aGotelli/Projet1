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
            The aim of this file is to save all the recorded data in the proper folder.
          Moreover, as the architecture is versatile and some parameters can be changes,
          based on the specification of the project, all the informations regarding this,
          are also saved. In this way, the details of the robot, of the world and of the
          sensor are always visible and available.

            This file has a build-in method to avoid to overwrite and existing file,
          causing to loose some precius data. However, it is recommanded to use a
          file name without spaces and that does not termine with a number. Even
          if this node will work, the name of the file will be strongly modified.

            This node has also a build in method to ensure the existency of the
          folder where to storage the files. If the folder does not exist then
          it is created in the path given. However if the path is not correct
          the node is shutted down and simulation goes on WITHOUT SAVIN ANY FILE.

            This node is designed to be used in the same group of the simulation.
          In the case where the simulation is not in a group the node can handle
          this case. However the user is encouraged to use the namespaces i.e.
          the groups properly. This will lead to a more clear and undestandable
          architecture for the simulation. In fact, the group allow grouping
          together componets that work on the same task or concempt.

 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <algorithm>


#define BOOST_FILESYSTEM_NO_DEPRECATED
#define BOOST_FILESYSTEM_VERSION 3

//ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>







void AddSuffix(boost::filesystem::path& _fileName)
{

    //  Transform the path to a string
    std::string fileName = _fileName.string();

    //  Take the last character
    char last = fileName.back();

    //  Chef if it ends with a letter
    if( isalpha(last) ) {
        // In the case it ends with a letter, just add 1 to the name
        fileName = fileName + "1" ;
    } else {
        //  In the case it ends with a digit first find the position of the last letter
        size_t lastLetter = fileName.find_last_not_of("0123456789");

        //  Obtain the suffix as array of character
        const char* result = fileName.substr(lastLetter + 1).c_str() ;

        //  Transform the array in a number and increment it of one unit
        int suffix = atoi(result);
        suffix ++;

        //  Erase the existing suffix
        fileName.erase(lastLetter + 1);

        //  Add in the end of the name the new suffix
        fileName = fileName + std::to_string(suffix) ;
    }

    //  Transform the name again in a path object
    _fileName = boost::filesystem::path(fileName) ;

}

bool IsEqual(const boost::filesystem::directory_entry& folderFile,
                                  const boost::filesystem::path& fileName)
{
  if( folderFile.path().filename().stem() == fileName )
    return true;
  else
    return false;
}



bool FindFile(const boost::filesystem::path& folderPath,
              boost::filesystem::path& fileName)
{

  //  Set iterator at the end of a file
  const boost::filesystem::recursive_directory_iterator end;  //  By default it refers to the directory end;

  //  Initialize the predicate to feed the find_if function
  boost::function< bool (const boost::filesystem::directory_entry&)> Predicate(boost::bind(IsEqual, _1, fileName));

/*
  //  Creation of the predicate to feed into std::find_if basically return true
  //  if the names are the same (without the extension)
  auto lambdaCorrespondence = [&fileName](const boost::filesystem::directory_entry& e) {
                              return e.path().filename().stem() == fileName;
                              };



  //  Find the position in the directory where the correspondance occour
  const auto it = std::find_if(boost::filesystem::recursive_directory_iterator(folderPath),
                                end, lambdaCorrespondence) ;

*/

 const auto it = std::find_if(boost::filesystem::recursive_directory_iterator(folderPath),
                              end, Predicate) ;
  //  Check the result
  if ( it != end ) {
    AddSuffix(fileName) ;
    FindFile(folderPath, fileName);
  }
}






int main (int argc, char** argv)
{

	//ROS Initialization
  ros::init(argc, argv, "file_handler");

  ros::NodeHandle nh_glob;

  //  The node must be provided with two args
  if( argc <  3 ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Not enought input arguments! You must provide the folder path and the file names ");
  }

  //  Declare the path to the folder and the file name
  boost::filesystem::path folderPath(argv[1]);
  boost::filesystem::path fileName(argv[2]);


  //  Avoid using a user defined extension which will cause a problem
  fileName = fileName.stem();


  //  Obtain the origin of the destination folder
  size_t lastLetter = folderPath.string().find_last_of("/");
  boost::filesystem::path origin( folderPath.string().substr(0, lastLetter) );


  //  Check if the origin exists
  if( !boost::filesystem::exists(origin) ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " The path to the folder does not exist !  You are not saving any data !");
    ros::shutdown();
    return 1;
  }


  //  Check if the folder is actually a directory and it exist.
  //  In the case is does not than create a folder
  if( !boost::filesystem::is_directory(folderPath) ) {
    ROS_WARN_STREAM(ros::this_node::getName() << " The target folder does not exist ! A folder is created at " << origin.string() );
    boost::filesystem::create_directory(folderPath);
  } else {
    ROS_INFO_STREAM("found the path : " << folderPath ) ;
  }


  //  Check if the file already exists in the folder
  FindFile(folderPath, fileName );


  //  Obtain the namespace
  std::string group = ros::this_node::getNamespace() ;

  //  If the simulation is not defined in a namespace then the group is incorrect
  if( group == "/" )
    group = "";

  //  Save all the parameters
  const std::string rosparamCommand = "rosparam dump " + folderPath.string() + "/" + fileName.string() + ".yaml " + group ;
  system( const_cast<char*>( rosparamCommand.c_str() ) );


  //  Record all the messages
  const std::string rosbagCommand = "rosbag record -O" + folderPath.string() + "/" + fileName.string() + ".bag " + group + "/EncodersReading " + group + "/IRSensorsStatus " + group + "/RobotPosture " ;
  ROS_INFO_STREAM(rosbagCommand);
  system( const_cast<char*>( rosbagCommand.c_str() ) );

  return 0;

}
