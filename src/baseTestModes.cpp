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
 *
 *  Created on: Mar 30, 2015
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*! Includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include "robotDefines.h"

/*! Defines */
#define LOOP_RATE	HEART_BEAT
#define MIN_VELOCITY	-5000
#define MAX_VELOCITY	5000


/*! Variables */
//ROS publisher
ros::Publisher pub_motorCmdArray; //this allows to send a cmd msg packet to the mbed and drive the motors
osa_msgs::MotorCmdMultiArray armCmd_ma;
osa_msgs::MotorCmdMultiArray baseCmd_ma;
sensor_msgs::Joy xboxJoy;
osa_msgs::MotorDataMultiArray motorData_ma;

bool joy_arrived = false;
bool motorData_ma_arrived = false;

/*! Callback functions */
/*
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}
*/

void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motorData_ma = *data;
	motorData_ma_arrived = true;
}

/*! \fn int main(int argc, char** argv)
 *  \brief main function.
 *  \param argc
 *  \param argv
 *  \return int
 */
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_baseTestModes_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	//ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	ros::Subscriber sub_motorDataArray = nh.subscribe ("/motor_data_array", 10, motorDataArray_cb);

	//Publishers
	pub_motorCmdArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//create the commands multi array
	armCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	armCmd_ma.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	armCmd_ma.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	armCmd_ma.layout.dim[0].label = "motors";
	armCmd_ma.layout.data_offset = 0;
	armCmd_ma.motorCmd.clear();
	armCmd_ma.motorCmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		armCmd_ma.motorCmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		armCmd_ma.motorCmd[i].nodeID = i+1;
		armCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
		armCmd_ma.motorCmd[i].value = 0;
	}

	baseCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	baseCmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	baseCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	baseCmd_ma.layout.dim[0].label = "motors";
	baseCmd_ma.layout.data_offset = 0;
	baseCmd_ma.motorCmd.clear();
	baseCmd_ma.motorCmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		baseCmd_ma.motorCmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		baseCmd_ma.motorCmd[i].nodeID = i+1;
		baseCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
		baseCmd_ma.motorCmd[i].value = 0;
	}

	int step_idx = 42; //default starting loop point

	while(ros::ok())
	{
		ros::spinOnce();

		switch(step_idx)
		{
			case 50:
				ROS_INFO("0-SET_PROFILE_ACCELERATION");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					armCmd_ma.motorCmd[i].command = SET_PROFILE_ACCELERATION;
					armCmd_ma.motorCmd[i].value = 10000;
				}

				//BASE
				baseCmd_ma.motorCmd[0].command = SET_PROFILE_ACCELERATION;
				baseCmd_ma.motorCmd[0].value = 10000;
				baseCmd_ma.motorCmd[1].command = SET_PROFILE_ACCELERATION;
				baseCmd_ma.motorCmd[1].value = 10000;
				break;

			case 51:
				ROS_INFO("1-SET_PROFILE_DECELERATION");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					armCmd_ma.motorCmd[i].command = SET_PROFILE_DECELERATION;
					armCmd_ma.motorCmd[i].value = 10000;
				}

				//BASE
				baseCmd_ma.motorCmd[0].command = SET_PROFILE_DECELERATION;
				baseCmd_ma.motorCmd[0].value = 10000;
				baseCmd_ma.motorCmd[1].command = SET_PROFILE_DECELERATION;
				baseCmd_ma.motorCmd[1].value = 10000;
				break;

			case 52:
				ROS_INFO("2-SET_PROFILE_VELOCITY");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					armCmd_ma.motorCmd[i].command = SET_PROFILE_VELOCITY;
					armCmd_ma.motorCmd[i].value = 4000;
				}

				//BASE
				baseCmd_ma.motorCmd[0].command = SET_PROFILE_VELOCITY;
				baseCmd_ma.motorCmd[0].value = 4000;
				baseCmd_ma.motorCmd[1].command = SET_PROFILE_VELOCITY;
				baseCmd_ma.motorCmd[1].value = 4000;
				break;

			case 53:
				ROS_INFO("3-SET_CONTINUOUS_CURRENT_LIMIT");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					armCmd_ma.motorCmd[i].command = SET_OUTPUT_CURRENT_LIMIT;
					armCmd_ma.motorCmd[i].value = 1000;
				}

				//BASE
				baseCmd_ma.motorCmd[0].command = SET_OUTPUT_CURRENT_LIMIT;
				baseCmd_ma.motorCmd[0].value = 1000;
				baseCmd_ma.motorCmd[1].command = SET_OUTPUT_CURRENT_LIMIT;
				baseCmd_ma.motorCmd[1].value = 1000;
				break;
/*
			case 54:
				ROS_INFO("4-SET_MODES_OF_OPERATION");
				baseCmd_ma.motorCmd[0].command = SET_MODES_OF_OPERATION;
				baseCmd_ma.motorCmd[0].value = PROFILE_POSITION_MODE;
				baseCmd_ma.motorCmd[1].command = SET_MODES_OF_OPERATION;
				baseCmd_ma.motorCmd[1].value = PROFILE_POSITION_MODE;
				break;

			case 55:
				ROS_INFO("5-SET_CONTROLWORD");
				baseCmd_ma.motorCmd[0].command = SET_CONTROLWORD;
				baseCmd_ma.motorCmd[0].value = 271;
				baseCmd_ma.motorCmd[1].command = SET_CONTROLWORD;
				baseCmd_ma.motorCmd[1].value = 271;
				break;

			case 56:
				ROS_INFO("6-SET_TARGET_POSITION");
				baseCmd_ma.motorCmd[0].command = SET_TARGET_POSITION;
				baseCmd_ma.motorCmd[0].value = 100000;
				baseCmd_ma.motorCmd[1].command = SET_TARGET_POSITION;
				baseCmd_ma.motorCmd[1].value = 80000;
				break;

			case 57:
				ROS_INFO("4-SET_CONTROLWORD");
				baseCmd_ma.motorCmd[0].command = SET_CONTROLWORD;
				baseCmd_ma.motorCmd[0].value = 63;
				baseCmd_ma.motorCmd[1].command = SET_CONTROLWORD;
				baseCmd_ma.motorCmd[1].value = 63;
				break;
*/
			default:


				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					armCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
					armCmd_ma.motorCmd[i].value = 0;
				}

				//BASE
				baseCmd_ma.motorCmd[0].command = SEND_DUMB_MESSAGE;
				baseCmd_ma.motorCmd[0].value = 0;
				baseCmd_ma.motorCmd[1].command = SEND_DUMB_MESSAGE;
				baseCmd_ma.motorCmd[1].value = 0;
				break;
		}

		step_idx++;

		//pub_motorCmdArray.publish(armCmd_ma);
		pub_motorCmdArray.publish(baseCmd_ma);

		r.sleep();
	}

	return 0;
}

