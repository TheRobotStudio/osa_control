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
//#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "osa_control/switchNode.h"
#include "osa_control/getSlaveCmdArray.h"
//other
#include <stdio.h>
//ROS packages include 
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define DOF_DIM 				2//NUMBER_MOTORS_ARM

/*** Variables ***/

//ROS publisher
//ros::Publisher pub_motorBaseCmd_ma;

osa_msgs::MotorCmdMultiArray motorCmd_ma;
//sensor_msgs::Joy xboxJoy;
std_msgs::Int16MultiArray objectCoords;

bool switch_node = false; //disable by default
//bool joy_arrived = false;
bool objectCoords_arrived = false;
//bool stateBtn_RB = false;
//bool stateBtn_A = false;
//bool motorData_ma_arrived = false;

/*** Callback functions ***/
/*
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;

	stateBtn_RB = (bool)xboxJoy.buttons[5];
	stateBtn_A = (bool)xboxJoy.buttons[0];
}*/

void objectCoords_cb(const std_msgs::Int16MultiArrayConstPtr& oc)
{	
	if((oc->data[0] != -1) && (oc->data[1] != -1) && (oc->data[2] < 500)) //if object detected
	{
		objectCoords = *oc;
		objectCoords_arrived = true;		
	}
	else	objectCoords_arrived = false;
}

/*** Services ***/
bool switchNode(osa_control::switchNode::Request  &req, osa_control::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmd_ma(osa_control::getSlaveCmdArray::Request  &req, osa_control::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdMultiArray = motorCmd_ma;
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
	ros::init (argc, argv, "osa_mobileBaseFollower_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	ros::Subscriber sub_headCoords = nh.subscribe ("/headCoords", 10, objectCoords_cb);

	//Publishers
	//pub_motorBaseCmd_ma = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_base_cmd", 100, true);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_mobile_base_follower_srv", switchNode);
	ros::ServiceServer srv_getMotorCmd_ma = nh.advertiseService("get_mobile_base_follower_cmd_srv", getMotorCmd_ma);
	
	//create the commands multi array
	motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorCmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	motorCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	motorCmd_ma.layout.dim[0].label = "motors";
	motorCmd_ma.layout.data_offset = 0;
	motorCmd_ma.motorCmd.clear();
	motorCmd_ma.motorCmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		motorCmd_ma.motorCmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		motorCmd_ma.motorCmd[i].nodeID = i+1;
		motorCmd_ma.motorCmd[i].command = SEND_DUMB_MESSAGE;
		motorCmd_ma.motorCmd[i].value = 0;
	}

	float baseLR_f = 0; //left right
	float baseUD_f = 0; //up down
	float leftWheel_f = 0;
	float rightWheel_f = 0;
	int leftWheel_i = 0;
	int rightWheel_i = 0;

	while(ros::ok())
	{	/*	
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			motorCmd_ma.motorCmd[i].nodeID = i+1;
			motorCmd_ma.motorCmd[i].mode = NO_MODE; //VELOCITY_MODE;
			motorCmd_ma.motorCmd[i].value = 0;
		}*/

		ros::spinOnce();

		if(switch_node)
		{
			//ROS_INFO("srv ON");

			if(objectCoords_arrived)
			{			
				//to drive the mobile base, 2 wheels //vel from 0 to 5000			
				//use object coords and size in the image to simulate a joystick motion, then use the same code as for the XBox Controller
				baseLR_f = ((-(float)(objectCoords.data[0])/640) + 1)/2;//xboxJoy.axes[3]; //left right
				baseUD_f = (-(float)(objectCoords.data[2])/250) + 1;//xboxJoy.axes[4]; //up down

				//ROS_INFO("LR=%f, UD=%f", baseLR_f, baseUD_f);
							
				leftWheel_f = -(baseLR_f - baseUD_f)*1000;
				rightWheel_f = (baseLR_f + baseUD_f)*1000;

				leftWheel_i = (int)leftWheel_f;
				rightWheel_i = (int)rightWheel_f;
			
				motorCmd_ma.motorCmd[0].command = SET_TARGET_VELOCITY;
				motorCmd_ma.motorCmd[1].command = SET_TARGET_VELOCITY;
				motorCmd_ma.motorCmd[0].value = rightWheel_i;
				motorCmd_ma.motorCmd[1].value = leftWheel_i;

				//objectCoords_arrived = false;		
			}
			else
			{
				//ROS_INFO("no joy");
				//STOP base motors	
				motorCmd_ma.motorCmd[0].command = SET_CURRENT_MODE_SETTING_VALUE;
				motorCmd_ma.motorCmd[1].command = SET_CURRENT_MODE_SETTING_VALUE;
				motorCmd_ma.motorCmd[0].value = 0;
				motorCmd_ma.motorCmd[1].value = 0;
			}

			

			
		}//if(switch_node)
		//else ROS_INFO("srv OFF");

		//r.sleep();
	}

	return 0;
}
