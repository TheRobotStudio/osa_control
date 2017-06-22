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
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT //50

/*** Variables ***/
bool joy_arrived = false;
sensor_msgs::Joy xboxJoy;
osa_msgs::MotorCmdMultiArray armMotorCmd_ma;
osa_msgs::MotorCmdMultiArray mobileBaseMotorCmd_ma;

//booleans
bool switch_node = false; //disable by default
bool armManual_enabled = false;
bool mobileBaseManual_enabled = false;
bool mobileBaseFollower_enabled = false;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
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
	ros::Publisher pub_setRightArmCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100);
	ros::Publisher pub_setMobileBaseCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100); //set_mobile_base_cmd

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	
	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_xbox_joy_conductor", switchNode);
	ros::ServiceClient srvClt_switchArmManual = nh.serviceClient<osa_control::switchNode>("switch_right_arm_manual_srv");
	ros::ServiceClient srvClt_getArmManualCmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_right_arm_manual_cmd_srv");
	ros::ServiceClient srvClt_switchMobileBaseManual = nh.serviceClient<osa_control::switchNode>("switch_mobile_base_manual_srv");
	ros::ServiceClient srvClt_getMobileBaseManualCmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_mobile_base_manual_cmd_srv");
	ros::ServiceClient srvClt_switchMobileBaseFollower = nh.serviceClient<osa_control::switchNode>("switch_mobile_base_follower_srv");
	ros::ServiceClient srvClt_getMobileBaseFollowerCmd = nh.serviceClient<osa_control::getSlaveCmdArray>("get_mobile_base_follower_cmd_srv");
	//ros::ServiceClient srvClt_switchCaffe = nh.serviceClient<osa_control::getSlaveCmdArray>("switch_caffe");
	//ros::ServiceClient srvClt_switchSound = nh.serviceClient<osa_control::getSlaveCmdArray>("switch_sound");
	
	osa_control::getSlaveCmdArray srv_getSlaveCmdArrayArmManual;
	osa_control::getSlaveCmdArray srv_getSlaveCmdArrayMobileBaseManual;
	osa_control::getSlaveCmdArray srv_getSlaveCmdArrayMobileBaseFollower;

	osa_control::switchNode srv_switchNodeArmManual;
	osa_control::switchNode srv_switchNodeMobileBaseManual;
	osa_control::switchNode srv_switchNodeMobileBaseFollower;

	srv_switchNodeArmManual.request.state = false;
	srv_switchNodeMobileBaseManual.request.state = false;
	srv_switchNodeMobileBaseFollower.request.state = false;

	//create the commands multi array
	armMotorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	armMotorCmd_ma.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	armMotorCmd_ma.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	armMotorCmd_ma.layout.dim[0].label = "motors";
	armMotorCmd_ma.layout.data_offset = 0;
	armMotorCmd_ma.motorCmd.clear();
	armMotorCmd_ma.motorCmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		armMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		armMotorCmd_ma.motorCmd[i].nodeID = i+1;
		armMotorCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
		armMotorCmd_ma.motorCmd[i].value = 0;
	}

	//create the commands multi array
	mobileBaseMotorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mobileBaseMotorCmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	mobileBaseMotorCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	mobileBaseMotorCmd_ma.layout.dim[0].label = "motors";
	mobileBaseMotorCmd_ma.layout.data_offset = 0;
	mobileBaseMotorCmd_ma.motorCmd.clear();
	mobileBaseMotorCmd_ma.motorCmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		mobileBaseMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		mobileBaseMotorCmd_ma.motorCmd[i].nodeID = i+1;
		mobileBaseMotorCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
		mobileBaseMotorCmd_ma.motorCmd[i].value = 0;
	}

	//No conductor above, so switched on by default
	switch_node = true;

	while(ros::ok())
	{
		//Default arm value
		for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
		{
			armMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
			armMotorCmd_ma.motorCmd[i].nodeID = i+1;
			armMotorCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
			armMotorCmd_ma.motorCmd[i].value = 0;
		}
		
		//Default base value
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			mobileBaseMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
			mobileBaseMotorCmd_ma.motorCmd[i].nodeID = i+1;
			mobileBaseMotorCmd_ma.motorCmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
			mobileBaseMotorCmd_ma.motorCmd[i].value = 0;
		}

		//read the joystick inputs
		ros::spinOnce();

		if(switch_node)
		{
			if(joy_arrived)
			{
				//ROS_INFO("joy_arrived");
				//LB ON + RB OFF
				if((xboxJoy.buttons[4]==1) && (xboxJoy.buttons[5]==0)) 
				{
					armManual_enabled = true;
					//turn ON mode armManual
					srv_switchNodeArmManual.request.state = true;	
				}
				else 
				{	
					armManual_enabled = false;
					//turn OFF mode mobileBaseManual
					srv_switchNodeArmManual.request.state = false;
				}

				//LB OFF + RB ON + A OFF
				if((xboxJoy.buttons[4]==0) && (xboxJoy.buttons[5]==1) && (xboxJoy.buttons[0]==0)) 
				{
					mobileBaseManual_enabled = true;
					//turn ON mode mobileBaseManual
					srv_switchNodeMobileBaseManual.request.state = true;	
				}
				else 
				{	
					mobileBaseManual_enabled = false;
					//turn OFF mode mobileBaseManual
					srv_switchNodeMobileBaseManual.request.state = false;
				}

				//LB OFF + RB ON + A ON
				if((xboxJoy.buttons[4]==0) && (xboxJoy.buttons[5]==1) && (xboxJoy.buttons[0]==1)) 
				{
					mobileBaseFollower_enabled = true;
					//turn ON mode mobileBaseFollower
					srv_switchNodeMobileBaseFollower.request.state = true;
				}
				else 
				{	
					mobileBaseFollower_enabled = false;				
					//turn OFF mode mobileBaseFollower
					srv_switchNodeMobileBaseFollower.request.state = false;
				}
				
				//btn RB and LB to enable homing and current
				if((xboxJoy.buttons[4]==1) && (xboxJoy.buttons[5]==1)) 
				{
					if((xboxJoy.buttons[6]==1) && (xboxJoy.buttons[7]==0)) //back button : zero homing mode
					{
						for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
						{
							armMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
							armMotorCmd_ma.motorCmd[i].nodeID = i+1;
							armMotorCmd_ma.motorCmd[i].command = SET_TARGET_POSITION;
							armMotorCmd_ma.motorCmd[i].value = 0;
						}
					
						pub_setRightArmCommand.publish(armMotorCmd_ma);								
					}

					if((xboxJoy.buttons[6]==0) && (xboxJoy.buttons[7]==1)) //start button : current mode
					{
						int curr = 120; //150;

						for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
						{
							armMotorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
							armMotorCmd_ma.motorCmd[i].nodeID = i+1;
							armMotorCmd_ma.motorCmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							armMotorCmd_ma.motorCmd[i].value = curr;
						}

						armMotorCmd_ma.motorCmd[0].value = 180; // 250;
						armMotorCmd_ma.motorCmd[1].value = 180; // 250;
						armMotorCmd_ma.motorCmd[9].value = 180; // 250;
						
						//for the 2 inverted wrist motors
						//rightHandCmd_ma.motorCmd[1].command = SET_TARGET_POSITION;
						armMotorCmd_ma.motorCmd[1+10].value = 0;

						armMotorCmd_ma.motorCmd[2+10].value = 60; //80;
						armMotorCmd_ma.motorCmd[3+10].value = 60; //80;
						armMotorCmd_ma.motorCmd[4+10].value = 150; //80;//hand
						armMotorCmd_ma.motorCmd[5+10].value = 50; //thumb
			
						pub_setRightArmCommand.publish(armMotorCmd_ma);				
					}
				}

				//Stop the mobile base
				if(xboxJoy.buttons[5]==0)
				{
					pub_setMobileBaseCommand.publish(mobileBaseMotorCmd_ma);
				}
						
				//ROS_INFO("Switch the services according to the Joystick inputs");
				//Switch the services according to the Joystick inputs
				srvClt_switchArmManual.call(srv_switchNodeArmManual);
				srvClt_switchMobileBaseManual.call(srv_switchNodeMobileBaseManual);
				srvClt_switchMobileBaseFollower.call(srv_switchNodeMobileBaseFollower);
				//ROS_INFO("Switch end");

				joy_arrived = false;
			}

			//armManual
			if(armManual_enabled)
			{	
				ROS_INFO("armManual_enabled");	
											
				if(srvClt_getArmManualCmd.call(srv_getSlaveCmdArrayArmManual))
				{
					//publish to the commandBuilder node
					pub_setRightArmCommand.publish(srv_getSlaveCmdArrayArmManual.response.motorCmdMultiArray);
				}
				else
				{
					ROS_ERROR("Failed to call service get_right_arm_manual_cmd");
				}
			}	

			//mobileBaseManual
			if(mobileBaseManual_enabled)
			{	
				ROS_INFO("mobileBaseManual_enabled");	
											
				if(srvClt_getMobileBaseManualCmd.call(srv_getSlaveCmdArrayMobileBaseManual))
				{
					//publish to the commandBuilder node
					pub_setMobileBaseCommand.publish(srv_getSlaveCmdArrayMobileBaseManual.response.motorCmdMultiArray);
				}
				else
				{
					ROS_ERROR("Failed to call service get_mobile_base_manual_cmd");
				}
			}
			//else ROS_INFO("mobileBaseManual_disabled");
		
			//mobileBaseFollower
			if(mobileBaseFollower_enabled)
			{			
				ROS_INFO("mobileBaseFollower_enabled");						
				if(srvClt_getMobileBaseFollowerCmd.call(srv_getSlaveCmdArrayMobileBaseFollower))
				{
					//publish to the commandBuilder node
					pub_setMobileBaseCommand.publish(srv_getSlaveCmdArrayMobileBaseFollower.response.motorCmdMultiArray);
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
