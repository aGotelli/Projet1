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
#include "ros/ros.h"

bool FindFile(const boost::filesystem::path& dir_path,
              const boost::filesystem::path& file_name)
{
  //  Set iterator at the end of a file
  const boost::filesystem::recursive_directory_iterator end;

  //  Creation of the predicate to feed into std::find_if
  auto lambdaCorrespondence = [&file_name](const boost::filesystem::directory_entry& e) {
                              return e.path().filename() == file_name;
                              };
  //  Find the position in the directory where the correspondance occour
  const auto it = std::find_if(boost::filesystem::recursive_directory_iterator(dir_path),
                                end, lambdaCorrespondence) ;
  //  Check the result
  if (it == end) {
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
  if( argc <  3) {
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

  //  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp
  ROS_INFO_STREAM("Founded the followed files : " ) ;
  for ( boost::filesystem::directory_entry& file : boost::filesystem::directory_iterator(folderPath) ) {
      ROS_INFO_STREAM( file.path() ) ;
      //ROS_INFO_STREAM( file.file_status.file_type() ) ;

  }
  //  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp  temp



  //  Check if the file already exists in the folder
  if( FindFile(folderPath, fileName) ) {
    ROS_ERROR_STREAM(ros::this_node::getName() << " The file already exist ! ");
  }






}
