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
#include <algorithm>


#define BOOST_FILESYSTEM_NO_DEPRECATED
#define BOOST_FILESYSTEM_VERSION 3

//ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>

bool FindFile(const boost::filesystem::path& folderPath,
              const boost::filesystem::path& fileName)
{
  //  Set iterator at the end of a file
  const boost::filesystem::recursive_directory_iterator end;

  //  Creation of the predicate to feed into std::find_if
  auto lambdaCorrespondence = [&fileName](const boost::filesystem::directory_entry& e) {
                              return e.path().filename() == fileName;
                              };

  //  Find the position in the directory where the correspondance occour
  const auto it = std::find_if(boost::filesystem::recursive_directory_iterator(folderPath),
                                end, lambdaCorrespondence) ;
  //  Check the result
  if ( it == end ) {
    return false;
  } else {
    // path_found = it->path();
    return true;
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

  //  Check if the folder exists
  if( !boost::filesystem::exists(folderPath) ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Te Folder does not exist ! ");
  }

  //  Check integrity of the inputs
  if( !boost::filesystem::is_regular_file(fileName) ||
        !boost::filesystem::is_regular_file(fileName)) {

    ROS_ERROR_STREAM(ros::this_node::getName() << " Error input of a non regular file ! ");
  }

  //  Check if the folder is actually a directory
  if( !boost::filesystem::is_directory(folderPath) ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " The target folder does not exist ! ");
  } else {
    ROS_INFO_STREAM("found the path : " << folderPath ) ;
  }



  //  Check if the file already exists in the folder
  if( FindFile(folderPath, fileName) ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " The file : " << fileName.string() << " already exist ! ");
  }

  //  Obtain the namespace
  const std::string group = ros::this_node::getNamespace() ;

  //  Save all the parameters
  const std::string rosparamCommand = "rosparam dump " + folderPath.string() + "/" + fileName.string() + ".yaml " + group ;
  system( const_cast<char*>( rosparamCommand.c_str() ) );





  //  Record all the messages
  const std::string rosbagCommand = "rosbag record -O" + folderPath.string() + "/" + fileName.string() + ".bag " + group + "/EncodersReading " + group + "/IRSensorsStatus " + group + "/RobotPosture " ;
  ROS_INFO_STREAM(rosbagCommand);
  system( const_cast<char*>( rosbagCommand.c_str() ) );



}
