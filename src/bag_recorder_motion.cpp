/*
 * Copyright (c) 2014, The Robot Studio
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
 *
 *  Created on: Mar 27, 2015
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <stdio.h>
#include "robot_defines.h"

 /*** Defines ***/
#define LOOP_RATE	HEART_BEAT

/*** Variables ***/
osa_msgs::MotorDataMultiArray motor_data_array;

//to wait for the msg from both posture and anglesArmDescription topics
bool motor_data_array_arrived = false;

/*** Callback functions ***/
void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array = *data;
	motor_data_array_arrived = true;
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_bag_recorder_motion_node");
	ros::NodeHandle nh;
	bool run = true;

	ros::Rate r(LOOP_RATE);

	rosbag::Bag bag;
	bag.open(ros::package::getPath("osa_control") + "/bag/arm/imuRawToShoulder_1.bag", rosbag::bagmode::Write); //imuRawToShoulder_1  //btnA

	//Subscribers
	ros::Subscriber sub_motorDataArray = nh.subscribe ("/motor_data_array", 10, motorDataArrayCallback);

	int loopNb = 0;

	ros::Duration(3).sleep();

	printf("Start\n");

	while(run)
	{
		ros::spinOnce(); //listen to topics

		if(motor_data_array_arrived)// && imuRaw_arrived)
		{
			//get time
			ros::Time time = ros::Time::now();
			//write data to the bag
			bag.write("/motor_data_array", time, motor_data_array);

			//but this back to false for the next record
			motor_data_array_arrived = false;

			//loopNb++;
		}

		loopNb++;
		if(loopNb == 100) run=false; //200=10s, 2400=2min //400

		r.sleep();
	}

	bag.close(); //close the bag
	run = false; //stop the while loop, quit the program
	printf("Finish\n");

	return 0;
}
