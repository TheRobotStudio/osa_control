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
 *  Created on: Oct 2, 2016
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
//#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Bool.h>
//ROS services
#include "osa_control/switchNode.h"
#include "osa_control/getSlaveCmdArray.h"
//other
//#include <stdio.h>
//ROS packages include 
#include "robot_defines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT //50

/*** Variables ***/
bool joy_arrived = false;
sensor_msgs::Joy xbox_joy;
osa_msgs::MotorCmdMultiArray arm_motor_cmd_array;
osa_msgs::MotorCmdMultiArray mobile_base_motor_cmd_array;

//booleans
bool switch_node = false; //disable by default
bool arm_manual_enabled = false;
bool mobile_base_manual_enabled = false;
bool mobile_base_follower_enabled = false;

/*** Callback functions ***/
void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(osa_control::switchNode::Request  &req, osa_control::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_xbox_joy_conductor_client_node");
	ros::NodeHandle nh;

	ros::Rate r(LOOP_RATE);

	//Publishers
	ros::Publisher pub_set_right_arm_command = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100);
	ros::Publisher pub_set_mobile_base_command = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100); //set_mobile_base_cmd

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joyCallback);
	
	//TODO make it depending on parameters to link a node to a button

	//Services
	ros::ServiceServer srv_switch_node = nh.advertiseService("switch_xbox_joy_conductor", switchNode);
	ros::ServiceClient srv_clt_switch_arm_manual = nh.serviceClient<osa_control::switchNode>("switch_right_arm_manual_srv");
	ros::ServiceClient srv_clt_get_arm_manual_cmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_right_arm_manual_cmd_srv");
	ros::ServiceClient srv_clt_switch_mobile_base_manual = nh.serviceClient<osa_control::switchNode>("switch_mobile_base_manual_srv");
	ros::ServiceClient srv_clt_get_mobile_base_manual_cmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_mobile_base_manual_cmd_srv");
	ros::ServiceClient srv_clt_switch_mobile_base_follower = nh.serviceClient<osa_control::switchNode>("switch_mobile_base_follower_srv");
	ros::ServiceClient srv_clt_get_mobile_base_follower_cmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_mobile_base_follower_cmd_srv");
	//ros::ServiceClient srvClt_switchCaffe = nh.serviceClient<osa_control::getSlaveCmdArray>("switch_caffe");
	//ros::ServiceClient srvClt_switchSound = nh.serviceClient<osa_control::getSlaveCmdArray>("switch_sound");
	
	osa_control::getSlaveCmdArray srv_get_slave_cmd_array_arm_manual;
	osa_control::getSlaveCmdArray srv_get_slave_cmd_array_mobile_base_manual;
	osa_control::getSlaveCmdArray srv_get_slave_cmd_array_mobile_base_follower;

	osa_control::switchNode srv_switch_node_arm_manual;
	osa_control::switchNode srv_switch_node_mobile_base_manual;
	osa_control::switchNode srv_switch_node_mobile_base_follower;

	srv_switch_node_arm_manual.request.state = false;
	srv_switch_node_mobile_base_manual.request.state = false;
	srv_switch_node_mobile_base_follower.request.state = false;

	//create the commands multi array
	arm_motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	arm_motor_cmd_array.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	arm_motor_cmd_array.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	arm_motor_cmd_array.layout.dim[0].label = "motors";
	arm_motor_cmd_array.layout.data_offset = 0;
	arm_motor_cmd_array.motor_cmd.clear();
	arm_motor_cmd_array.motor_cmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		//arm_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		arm_motor_cmd_array.motor_cmd[i].node_id = i+1;
		arm_motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		arm_motor_cmd_array.motor_cmd[i].value = 0;
	}

	//create the commands multi array
	mobile_base_motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mobile_base_motor_cmd_array.layout.dim[0].size = NUMBER_MOTORS_BASE;
	mobile_base_motor_cmd_array.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	mobile_base_motor_cmd_array.layout.dim[0].label = "motors";
	mobile_base_motor_cmd_array.layout.data_offset = 0;
	mobile_base_motor_cmd_array.motor_cmd.clear();
	mobile_base_motor_cmd_array.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		//mobile_base_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		mobile_base_motor_cmd_array.motor_cmd[i].node_id = i+1;
		mobile_base_motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		mobile_base_motor_cmd_array.motor_cmd[i].value = 0;
	}

	//No conductor above, so switched on by default
	switch_node = true;

	while(ros::ok())
	{
		//Default arm value
		for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
		{
			//arm_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
			arm_motor_cmd_array.motor_cmd[i].node_id = i+1;
			arm_motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
			arm_motor_cmd_array.motor_cmd[i].value = 0;
		}
		
		//Default base value
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			//mobile_base_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
			mobile_base_motor_cmd_array.motor_cmd[i].node_id = i+1;
			mobile_base_motor_cmd_array.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
			mobile_base_motor_cmd_array.motor_cmd[i].value = 0;
		}

		//read the joystick inputs
		ros::spinOnce();

		if(switch_node)
		{
			if(joy_arrived)
			{
				//ROS_INFO("joy_arrived");
				//LB ON + RB OFF
				if((xbox_joy.buttons[4]==1) && (xbox_joy.buttons[5]==0))
				{
					arm_manual_enabled = true;
					//turn ON mode armManual
					srv_switch_node_arm_manual.request.state = true;
				}
				else 
				{	
					arm_manual_enabled = false;
					//turn OFF mode mobileBaseManual
					srv_switch_node_arm_manual.request.state = false;
				}

				//LB OFF + RB ON + A OFF
				if((xbox_joy.buttons[4]==0) && (xbox_joy.buttons[5]==1) && (xbox_joy.buttons[0]==0))
				{
					mobile_base_manual_enabled = true;
					//turn ON mode mobileBaseManual
					srv_switch_node_mobile_base_manual.request.state = true;
				}
				else 
				{	
					mobile_base_manual_enabled = false;
					//turn OFF mode mobileBaseManual
					srv_switch_node_mobile_base_manual.request.state = false;
				}

				//LB OFF + RB ON + A ON
				if((xbox_joy.buttons[4]==0) && (xbox_joy.buttons[5]==1) && (xbox_joy.buttons[0]==1))
				{
					mobile_base_follower_enabled = true;
					//turn ON mode mobileBaseFollower
					srv_switch_node_mobile_base_follower.request.state = true;
				}
				else 
				{	
					mobile_base_follower_enabled = false;
					//turn OFF mode mobileBaseFollower
					srv_switch_node_mobile_base_follower.request.state = false;
				}
				
				//btn RB and LB to enable homing and current
				if((xbox_joy.buttons[4]==1) && (xbox_joy.buttons[5]==1))
				{
					if((xbox_joy.buttons[6]==1) && (xbox_joy.buttons[7]==0)) //back button : zero homing mode
					{
						for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
						{
							//arm_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
							arm_motor_cmd_array.motor_cmd[i].node_id = i+1;
							arm_motor_cmd_array.motor_cmd[i].command = SET_TARGET_POSITION;
							arm_motor_cmd_array.motor_cmd[i].value = 0;
						}
					
						pub_set_right_arm_command.publish(arm_motor_cmd_array);
					}

					if((xbox_joy.buttons[6]==0) && (xbox_joy.buttons[7]==1)) //start button : current mode
					{
						int curr = 120; //150;

						for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
						{
							//arm_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
							arm_motor_cmd_array.motor_cmd[i].node_id = i+1;
							arm_motor_cmd_array.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							arm_motor_cmd_array.motor_cmd[i].value = curr;
						}

						arm_motor_cmd_array.motor_cmd[0].value = 180; // 250;
						arm_motor_cmd_array.motor_cmd[1].value = 180; // 250;
						arm_motor_cmd_array.motor_cmd[9].value = 180; // 250;
						
						//for the 2 inverted wrist motors
						//rightHandCmd_ma.motor_cmd[1].command = SET_TARGET_POSITION;
						arm_motor_cmd_array.motor_cmd[1+10].value = 0;

						arm_motor_cmd_array.motor_cmd[2+10].value = 60; //80;
						arm_motor_cmd_array.motor_cmd[3+10].value = 60; //80;
						arm_motor_cmd_array.motor_cmd[4+10].value = 150; //80;//hand
						arm_motor_cmd_array.motor_cmd[5+10].value = 50; //thumb
			
						pub_set_right_arm_command.publish(arm_motor_cmd_array);
					}
				}

				//Stop the mobile base
				if(xbox_joy.buttons[5]==0)
				{
					pub_set_mobile_base_command.publish(mobile_base_motor_cmd_array);
				}
						
				//ROS_INFO("Switch the services according to the Joystick inputs");
				//Switch the services according to the Joystick inputs
				srv_clt_switch_arm_manual.call(srv_switch_node_arm_manual);
				srv_clt_switch_mobile_base_manual.call(srv_switch_node_mobile_base_manual);
				srv_clt_switch_mobile_base_follower.call(srv_switch_node_mobile_base_follower);
				//ROS_INFO("Switch end");

				joy_arrived = false;
			}

			//armManual
			if(arm_manual_enabled)
			{	
				ROS_INFO("arm_manual_enabled");
											
				if(srv_clt_get_arm_manual_cmd.call(srv_get_slave_cmd_array_arm_manual))
				{
					//publish to the commandBuilder node
					pub_set_right_arm_command.publish(srv_get_slave_cmd_array_arm_manual.response.motor_cmd_multi_array);
				}
				else
				{
					ROS_ERROR("Failed to call service get_right_arm_manual_cmd");
				}
			}	

			//mobileBaseManual
			if(mobile_base_manual_enabled)
			{	
				ROS_INFO("mobile_base_manual_enabled");
											
				if(srv_clt_get_mobile_base_manual_cmd.call(srv_get_slave_cmd_array_mobile_base_manual))
				{
					//publish to the commandBuilder node
					pub_set_mobile_base_command.publish(srv_get_slave_cmd_array_mobile_base_manual.response.motor_cmd_multi_array);
				}
				else
				{
					ROS_ERROR("Failed to call service get_mobile_base_manual_cmd");
				}
			}
			//else ROS_INFO("mobileBaseManual_disabled");
		
			//mobileBaseFollower
			if(mobile_base_follower_enabled)
			{			
				ROS_INFO("mobile_base_follower_enabled");
				if(srv_clt_get_mobile_base_follower_cmd.call(srv_get_slave_cmd_array_mobile_base_follower))
				{
					//publish to the commandBuilder node
					pub_set_mobile_base_command.publish(srv_get_slave_cmd_array_mobile_base_follower.response.motor_cmd_multi_array);
				}
				else
				{
					ROS_ERROR("Failed to call service get_mobile_base_follower_cmd");
				}				
			}	
			//else ROS_INFO("mobileBaseFollower_disabled");				
		}
		else
		{
			//ROS_INFO("Conductor OFF");
		}

		//r.sleep();
	}//while ros ok

	return 0;
}
