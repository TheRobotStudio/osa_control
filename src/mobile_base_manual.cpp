/*
 * Copyright (c) 2016, The Robot Studio
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
 *  Created on: Sep 30, 2016
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "osa_control/switchNode.h"
#include "osa_control/getSlaveCmdArray.h"
//other
#include <stdio.h>
//ROS packages include 
#include "robot_defines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define DOF_DIM 				2//NUMBER_MOTORS_ARM

/*** Variables ***/
bool switch_node = false; //disable by default
osa_msgs::MotorCmdMultiArray motor_cmd_array;
sensor_msgs::Joy xbox_joy;
bool joy_arrived = false;

//ROS publisher
//ros::Publisher pub_motorBaseCmdMultiArray;

/*** Callback functions ***/
void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(osa_control::switchNode::Request  &req, osa_control::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmdArray(osa_control::getSlaveCmdArray::Request  &req, osa_control::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motor_cmd_multi_array = motor_cmd_array;
		return true;
	}
	else
	{
		return false;
	}
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_mobile_base_manual_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joyCallback);

	//Publishers	
	//pub_motorBaseCmdMultiArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_base_cmd", 100, true);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_mobile_base_manual_srv", switchNode);
	ros::ServiceServer srv_getMotorCmdArray = nh.advertiseService("get_mobile_base_manual_cmd_srv", getMotorCmdArray);

	//create the commands multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = NUMBER_MOTORS_BASE;
	motor_cmd_array.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	motor_cmd_array.layout.dim[0].label = "motors";
	motor_cmd_array.layout.data_offset = 0;
	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		//motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		motor_cmd_array.motor_cmd[i].node_id = i+1;
		motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array.motor_cmd[i].value = 0;
	}

	float base_lr_f = 0; //left right
	float base_ud_f = 0; //up down
	float left_wheel_f = 0;
	float right_wheel_f = 0;
	int left_wheel_i = 0;
	int right_wheel_i = 0;

	while(ros::ok())
	{	/*	
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			motor_cmd_array.motor_cmd[i].node_id = i+1;
			motor_cmd_array.motor_cmd[i].mode = NO_MODE; //VELOCITY_MODE;
			motor_cmd_array.motor_cmd[i].value = 0;
		}*/

		ros::spinOnce();

		if(switch_node)
		{
			//ROS_INFO("srv ON");

			if(joy_arrived)
			{
					base_lr_f = xbox_joy.axes[3]/4; //left right
					base_ud_f = xbox_joy.axes[4]; //up down
								
					left_wheel_f = -(base_lr_f - base_ud_f)*2000;
					right_wheel_f = (base_lr_f + base_ud_f)*2000;

					left_wheel_i = (int)left_wheel_f;
					right_wheel_i = (int)right_wheel_f;
				
					motor_cmd_array.motor_cmd[0].command = SET_TARGET_VELOCITY;
					motor_cmd_array.motor_cmd[1].command = SET_TARGET_VELOCITY;
					motor_cmd_array.motor_cmd[0].value =  left_wheel_i;
					motor_cmd_array.motor_cmd[1].value = right_wheel_i;
			}
			else
			{
				//ROS_INFO("no joy");
				//STOP base motors	
				//baseCmd_ma.motor_cmd[0].mode = VELOCITY_MODE;
				//baseCmd_ma.motor_cmd[1].mode = VELOCITY_MODE;
				//baseCmd_ma.motor_cmd[0].value = 0;
				//baseCmd_ma.motor_cmd[1].value = 0;
			}

			//joy_arrived = false;
		}//if(switch_node)
		//else ROS_INFO("srv OFF");

		//r.sleep();
	}

	return 0;
}
