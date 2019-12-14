/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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

/*! Includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include "robot_defines.h"

/*! Defines */
#define LOOP_RATE	HEART_BEAT
#define MIN_VELOCITY	-5000
#define MAX_VELOCITY	5000

/*! Variables */
//ROS publisher
ros::Publisher pub_motor_cmd_array; //this allows to send a cmd msg packet to the mbed and drive the motors
osa_msgs::MotorCmdMultiArray arm_cmd_array;
osa_msgs::MotorCmdMultiArray base_cmd_array;
sensor_msgs::Joy xbox_joy;
osa_msgs::MotorDataMultiArray motor_data_array;

bool joy_arrived = false;
bool motor_data_array_arrived = false;

/*! Callback functions */
/*
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	joy_arrived = true;
}
*/

void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array = *data;
	motor_data_array_arrived = true;
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
	ros::init (argc, argv, "osa_base_test_modes_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	//ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	ros::Subscriber sub_motor_data_array = nh.subscribe ("/motor_data_array", 10, motorDataArrayCallback);

	//Publishers
	pub_motor_cmd_array = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//create the commands multi array
	arm_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	arm_cmd_array.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	arm_cmd_array.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	arm_cmd_array.layout.dim[0].label = "motors";
	arm_cmd_array.layout.data_offset = 0;
	arm_cmd_array.motor_cmd.clear();
	arm_cmd_array.motor_cmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		//arm_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		arm_cmd_array.motor_cmd[i].node_id = i+1;
		arm_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		arm_cmd_array.motor_cmd[i].value = 0;
	}

	base_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	base_cmd_array.layout.dim[0].size = NUMBER_MOTORS_BASE;
	base_cmd_array.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	base_cmd_array.layout.dim[0].label = "motors";
	base_cmd_array.layout.data_offset = 0;
	base_cmd_array.motor_cmd.clear();
	base_cmd_array.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		//base_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		base_cmd_array.motor_cmd[i].node_id = i+1;
		base_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		base_cmd_array.motor_cmd[i].value = 0;
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
					arm_cmd_array.motor_cmd[i].command = SET_PROFILE_ACCELERATION;
					arm_cmd_array.motor_cmd[i].value = 10000;
				}

				//BASE
				base_cmd_array.motor_cmd[0].command = SET_PROFILE_ACCELERATION;
				base_cmd_array.motor_cmd[0].value = 10000;
				base_cmd_array.motor_cmd[1].command = SET_PROFILE_ACCELERATION;
				base_cmd_array.motor_cmd[1].value = 10000;
				break;

			case 51:
				ROS_INFO("1-SET_PROFILE_DECELERATION");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					arm_cmd_array.motor_cmd[i].command = SET_PROFILE_DECELERATION;
					arm_cmd_array.motor_cmd[i].value = 10000;
				}

				//BASE
				base_cmd_array.motor_cmd[0].command = SET_PROFILE_DECELERATION;
				base_cmd_array.motor_cmd[0].value = 10000;
				base_cmd_array.motor_cmd[1].command = SET_PROFILE_DECELERATION;
				base_cmd_array.motor_cmd[1].value = 10000;
				break;

			case 52:
				ROS_INFO("2-SET_PROFILE_VELOCITY");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					arm_cmd_array.motor_cmd[i].command = SET_PROFILE_VELOCITY;
					arm_cmd_array.motor_cmd[i].value = 4000;
				}

				//BASE
				base_cmd_array.motor_cmd[0].command = SET_PROFILE_VELOCITY;
				base_cmd_array.motor_cmd[0].value = 4000;
				base_cmd_array.motor_cmd[1].command = SET_PROFILE_VELOCITY;
				base_cmd_array.motor_cmd[1].value = 4000;
				break;

			case 53:
				ROS_INFO("3-SET_CONTINUOUS_CURRENT_LIMIT");

				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					arm_cmd_array.motor_cmd[i].command = SET_OUTPUT_CURRENT_LIMIT;
					arm_cmd_array.motor_cmd[i].value = 1000;
				}

				//BASE
				base_cmd_array.motor_cmd[0].command = SET_OUTPUT_CURRENT_LIMIT;
				base_cmd_array.motor_cmd[0].value = 1000;
				base_cmd_array.motor_cmd[1].command = SET_OUTPUT_CURRENT_LIMIT;
				base_cmd_array.motor_cmd[1].value = 1000;
				break;
/*
			case 54:
				ROS_INFO("4-SET_MODES_OF_OPERATION");
				base_cmd_array.motor_cmd[0].command = SET_MODES_OF_OPERATION;
				base_cmd_array.motor_cmd[0].value = PROFILE_POSITION_MODE;
				base_cmd_array.motor_cmd[1].command = SET_MODES_OF_OPERATION;
				base_cmd_array.motor_cmd[1].value = PROFILE_POSITION_MODE;
				break;

			case 55:
				ROS_INFO("5-SET_CONTROLWORD");
				base_cmd_array.motor_cmd[0].command = SET_CONTROLWORD;
				base_cmd_array.motor_cmd[0].value = 271;
				base_cmd_array.motor_cmd[1].command = SET_CONTROLWORD;
				base_cmd_array.motor_cmd[1].value = 271;
				break;

			case 56:
				ROS_INFO("6-SET_TARGET_POSITION");
				base_cmd_array.motor_cmd[0].command = SET_TARGET_POSITION;
				base_cmd_array.motor_cmd[0].value = 100000;
				base_cmd_array.motor_cmd[1].command = SET_TARGET_POSITION;
				base_cmd_array.motor_cmd[1].value = 80000;
				break;

			case 57:
				ROS_INFO("4-SET_CONTROLWORD");
				base_cmd_array.motor_cmd[0].command = SET_CONTROLWORD;
				base_cmd_array.motor_cmd[0].value = 63;
				base_cmd_array.motor_cmd[1].command = SET_CONTROLWORD;
				base_cmd_array.motor_cmd[1].value = 63;
				break;
*/
			default:


				//ARM
				for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
				{
					arm_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
					arm_cmd_array.motor_cmd[i].value = 0;
				}

				//BASE
				base_cmd_array.motor_cmd[0].command = SEND_DUMB_MESSAGE;
				base_cmd_array.motor_cmd[0].value = 0;
				base_cmd_array.motor_cmd[1].command = SEND_DUMB_MESSAGE;
				base_cmd_array.motor_cmd[1].value = 0;
				break;
		}

		step_idx++;

		//pub_motor_cmd_array.publish(arm_cmd_array);
		pub_motor_cmd_array.publish(base_cmd_array);

		r.sleep();
	}

	return 0;
}

