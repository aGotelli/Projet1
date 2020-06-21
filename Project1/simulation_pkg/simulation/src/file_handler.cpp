/**
 * \file file handler
 * \brief save data
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.3
 * \date 14/06/2020
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
            The aim of this file is to save all the recorded data in the proper folder. Moreover,
          as the architecture is versatile and some parameters can be changed, based on the specification
          of the project, all the information regarding this, are also saved.

            This nodes works by taking some information from the user:
          1) If they want to save the parameters
          2) Which topics must be saved

            This node is designed to be used in the same level of the data that must be recorded. Meaning that,
          if the topics and parameters are inside some group, the node must be putted in the same group.

            In the case where the simulation is not in a group the node can handle this case.
          However, the user is encouraged to use the namespaces i.e. the groups properly as
          this will lead to a clearer and more understandable architecture for the simulation.
          In fact, the group allows to put together components that work on the same task or concept.

            This file has a build-in method to avoid to overwrite an existing file, causing the loss of
          precious data. However, it is recommended to use a file name without spaces and that does not end
          with a number. Even if this node will work, the name of the file will be strongly modified.

            This node has also a build-in method to ensure the existence of the folder where to storage the
          files. If the folder does not exist then it is created in the path given. However if the path is
          not correct, the node will be shutted down and simulation will go on WITHOUT SAVING ANY FILE.

            To check that the file is not already present in the folder, the function std::find_if is used. However,
          sometimes, it is not quite easy to understand how it works.

            The function std::find_if takes three arguments: a beginning iterator, an ending iterator and a predicate.
          The first one, the beginning iterator, tells the function where to start to search using the predicate as the
          On the other hand, the ending iterator tells where to stop. As said, the search corresponds to the use of a
          predicate, that is a function that outputs true or false depending on the inputs. In other words, std::find_if
          simply starts feeding the predicate with the elements founded from the beginning to the end. It returns the
          iterator corresponding to the position of the element for which the predicate returns true, or the end iterator
          if no element has been found.

            In this case the predicate should find a file using boost::filesystem that makes possible to iterate files
          in a folder as they are like the elements of a std::vector,  by using the class recursive_directory_iterator.
          The function std::find_if takes the beginning of the folder and iterates until the end. The predicate
          consists in a function :

          bool IsEqual(const boost::filesystem::directory_entry& folderFile,
                                            const boost::filesystem::path& fileName)

            It takes two arguments, a directory_entry that basically is a file contained in the folder, and the fileName
          which must be unique. This function is not suitable for being a predicate because generally it takes only the
          file founded iterating in the folder. For this reason, the predicate is made from this function, but using the
          boost::bind, as the fileName does not change in the iteration.

          boost::bind(IsEqual, _1, fileName)

            It returns a pointer to a function that has the body of IsEqual but takes only one argument. This is used
          as predicate for the std::find_if function.
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
#include <ros/master.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>







void AddSuffix(boost::filesystem::path& _fileName)
{

    //  Transform the path to a string
    std::string fileName = _fileName.string();

    //  Take the last character
    char last = fileName.back();

    //  Check if it ends with a letter
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



void FindFile(const boost::filesystem::path& folderPath,
              boost::filesystem::path& fileName)
{

  //  Set iterator at the end of a file
  const boost::filesystem::recursive_directory_iterator end;  //  By default it refers to the directory end;

  //  Initialize the predicate to feed the find_if function
  boost::function< bool (const boost::filesystem::directory_entry&)> Predicate(boost::bind(IsEqual, _1, fileName));


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

  ros::NodeHandle nh_glob, nh_loc("~");

  //  Check the user's choice of saving the parameters
  bool saveParams;
  nh_loc.param<bool>("save_params", saveParams, true );

  //  Read the topic to save
  std::string topisToSave;
  nh_loc.param<std::string>("topics_to_save", topisToSave, "" );

  if( topisToSave.empty() )
    ROS_WARN_STREAM("YOU ARE NOT RECORDING ANY TOPICS");

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


  //  Check if the folder is actually a directory and it exists.
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

  ROS_INFO_STREAM("Group : " << group);
  //  Save all the parameters ( if required )
  if( saveParams ) {

    const std::string rosparamCommand = "rosparam dump " + folderPath.string() + "/" + fileName.string() + ".yaml " + group ;
    system( const_cast<char*>( rosparamCommand.c_str() ) );
  }


  //  Record topis if any
  if( !topisToSave.empty() ) {

    //  Get the first character to ensure a correct format
    char first = topisToSave.at(0);

    //  Ensure a correct format
    if( first != 32 ) //  32 is the integer value for " "
        topisToSave.insert(0, " ");

    //  Start from the beginning
    std::size_t pos = 0;

    while( pos < topisToSave.size() ) {

      //  Check status of the group
      if( !group.empty() ) {

          //  Insert the correct suffix
          topisToSave.insert( pos + 1, (group + "/") );

          //  Find new position
          pos = topisToSave.find( " ", pos + group.size() );
      } else {

          //  Insert the correct suffix
          topisToSave.insert( pos + 1,  "/" );

          //  Find new position
          pos = topisToSave.find( " ", pos + 1 );
      }

    } //  Exiting this loop with correctly formatted topics names and beginning with a space


    //  Record all the messages
    const std::string rosbagCommand = "rosbag record -O" + folderPath.string() + "/" + fileName.string() + ".bag" + topisToSave ;
    ROS_INFO_STREAM(rosbagCommand);
    system( const_cast<char*>( rosbagCommand.c_str() ) );
  }






  return 0;

}
