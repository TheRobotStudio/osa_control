/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file bag_recorder.cpp
 * @author Cyril Jourdan
 * @date Sep 28, 2017
 * @version 0.1.0
 * @brief Implementation file for the bag recorder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Oct 19, 2012
 */

/*** Includes ***/
#include "ros/ros.h"
#include <ros/package.h>
//#include <topic_tools/shape_shifter.h>
//#include <ros_type_introspection/ros_introspection.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <osa_msgs/MotorDataMultiArray.h>
//#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>

using namespace std;

/*** Variables ***/
osa_msgs::MotorDataMultiArray motor_data_array;
//auto input_data;
//topic_tools::ShapeShifter::

//to wait for the msg from both posture and anglesArmDescription topics
bool motor_data_array_arrived = false;
//bool input_data_arrived = false;

/*** Callback functions ***/
void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array = *data;
	motor_data_array_arrived = true;
}

/*
void inputDataCallback(const topic_tools::ShapeShifter::ConstPtr& data)
{
	input_data = *data;
	input_data_arrived = true;

	data->getDataType();
	data->getMessageDefinition();
	data->size();
}*/
/*
// Note, you can recycle this callback and subscribe to multiple topics
void messageCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                     const std::string &topic_name )
{
  using namespace RosIntrospection;

  // Store multiple ROSTypeList in a map.
  static std::map<std::string,ROSTypeList> registered_type;
  // buffer and flat_container are static to avoid re-allocation of memory
  static std::vector<uint8_t> buffer;
  static ROSTypeFlat flat_container;

  const std::string& datatype   = msg->getDataType();
  const std::string& definition = msg->getMessageDefinition();

  if( registered_type.find( datatype ) == registered_type.end() )
  {
    ROSTypeList typelist = buildROSTypeMapFromDefinition(datatype, definition);
    registered_type[datatype] = typelist;
  }

  // Once again, it would be nice to access directly the
  // private member ShapeShifter::msgBuf, unfortunately we can't.
  // This oblige us to do a copy of the buffer using ShapeShifter::write
  buffer.resize( msg.size() );
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  msg->write(stream);

  buildRosFlatType( _ros_type_map,
                    datatype,
                    topic_name,
                    buffer.data(),
                    &flat_container // the output
                  );
  // Do something with flat_container
  // Your code goes here
}
*/
/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trs_bag_recorder_node");
	ros::NodeHandle nh;

	//Variables
	bool run = true;
	rosbag::Bag bag;

	// Parameters
	string bag_path_name;

	// Grab the parameters
	try
	{
		nh.param("bag_path", bag_path_name, string("~"));
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		return 0;
	}

	try
	{
		bag.open(bag_path_name, rosbag::bagmode::Write);
	}
	catch(rosbag::BagException const &e)
	{
		ROS_ERROR(e.what());
		return 0;
	}

	//Subscribers
	ros::Subscriber sub_motor_data_array = nh.subscribe ("/motor_data_array", 10, motorDataArrayCallback);

	//ros::Subscriber sub_input_data = nh.subscribe ("/input_data", 10, inputDataCallback);
/*
	// bind the name of the topic in the second argument of the callback
	boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
	callback = boost::bind(&DataStreamROS::topicCallback, this, _1, "/input_data");
	ros::Subscriber sub_input_data = nh.subscribe("/input_data", 10, callback);
*/

	//TODO make sure it records the node_id and name etc of each dof, for future compatibility

	char input_key = '0';
	int loop_nb = 0;

	while(run)
	{
		//wait for user keyboard input
		ROS_INFO("Enter 'r' to record a posture, or 'q' to exit:");
		input_key = getchar();

		if(input_key == 'r')
		{
			while(!motor_data_array_arrived )//|| !input_data_arrived)
			{
				ros::spinOnce(); //listen to topics
			}

			//get time
			ros::Time time = ros::Time::now();
			//write data to the bag
			bag.write("/motor_data_array", time, motor_data_array);
			//bag.write("/input_data", time, input_data);

			//but this back to false for the next record
			motor_data_array_arrived = false;
			//input_data_arrived = false;

			ROS_INFO("Posture number %d recorded OK !", loop_nb);
		}
		else if(input_key == 'q')
		{
			bag.close(); //close the bag
			run = false; //stop the while loop, quit the program
			ROS_INFO("Bag closed ok");
		}
		getchar(); //to grab carriage return

		loop_nb++;
	}

	return 0;
}
