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
#define NB_MOTORS 12

/*** Variables ***/
osa_msgs::MotorCmdMultiArray arm_motor_cmd_array;

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_i2c_arm_test_node");
	ros::NodeHandle nh;

	ros::Rate r(0.2);

	//Publishers
	ros::Publisher pub_set_right_arm_command = nh.advertise<osa_msgs::MotorCmdMultiArray>("/i2c/motor_cmd_array", 100);

	//create the commands multi array
	arm_motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	arm_motor_cmd_array.layout.dim[0].size = NB_MOTORS;
	arm_motor_cmd_array.layout.dim[0].stride = NB_MOTORS;
	arm_motor_cmd_array.layout.dim[0].label = "motors";
	arm_motor_cmd_array.layout.data_offset = 0;
	arm_motor_cmd_array.motor_cmd.clear();
	arm_motor_cmd_array.motor_cmd.resize(NB_MOTORS);

	int loop_nb = 1;

	while(ros::ok())
	{
		//Default arm value
		for(int i=0; i<NB_MOTORS; i++)
		{
			//arm_motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
			arm_motor_cmd_array.motor_cmd[i].node_id = i+8;
			arm_motor_cmd_array.motor_cmd[i].command = SET_TARGET_POSITION;
			arm_motor_cmd_array.motor_cmd[i].value = 50000*loop_nb;
		}

		pub_set_right_arm_command.publish(arm_motor_cmd_array);
		loop_nb++;

		r.sleep();
	}//while ros ok

	return 0;
}
