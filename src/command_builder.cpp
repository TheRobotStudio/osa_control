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
 * @file command_builder.cpp
 * @author Cyril Jourdan
 * @date Sep 11, 2017
 * @version OSA 0.1.0
 * @brief Implementation file for the command builder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 15, 2013
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "robot_defines.h"

/*! Defines */
#define LOOP_RATE	HEART_BEAT
#define DOFS	2

/*! Variables */
osa_msgs::MotorCmdMultiArray motor_cmd_array;

bool cmdIgnored[DOFS] = {false};
int modeOfOperation[DOFS] = {NO_MODE};
int profilePositionCmdStep[DOFS] = {0}; //every Profile Position command is followed by a rising edge on the controlword
int profileVelocityCmdStep[DOFS] = {0}; //every Profile Velocity command is followed by setting the controlword

/*! \fn void resetMotorCmdMultiArray()
 *  \brief
 *  \return void
 */
void resetMotorCmdMultiArray()
{
	for(int i=0; i<DOFS; i++)
	{
		motor_cmd_array.motor_cmd[i].node_id = 0; //j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
		motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array.motor_cmd[i].value = 0;
	}
}

/*! \fn void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
 *  \brief
 *  \return void
 */
void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
{
	//#ifdef TRS_DEBUG
	ROS_INFO("Motor commands received");
	//#endif

	//ROS_INFO("Nb EPOS = %d", cmds->layout.dim[0].stride);

	//for(int i=0; i<cmds->layout.dim[0].stride; i++)
	for(int i=0; i<DOFS; i++)
	{
		//ROS_INFO("EPOS[%d]", i);

		//int i = cmds->motor_cmd[i].node_id - 1; //(cmds->motor_cmd[i].slaveBoardID - 1)*NUMBER_MAX_EPOS2_PER_SLAVE + (cmds->motor_cmd[i].node_id - 1);

		motor_cmd_array.motor_cmd[i].node_id = cmds->motor_cmd[i].node_id;

		//other cases just apply the command //TODO optimize because it sometimes doesn't need to go through this if
		if((cmds->motor_cmd[i].command != SET_TARGET_POSITION) && (cmds->motor_cmd[i].command != SET_TARGET_VELOCITY)) //TODO add current mode
		{
			motor_cmd_array.motor_cmd[i].command = cmds->motor_cmd[i].command;
			motor_cmd_array.motor_cmd[i].value = cmds->motor_cmd[i].value;

/*			if(motor_cmd_array.motor_cmd[i].command == SET_MODES_OF_OPERATION)
			{

				//ROS_INFO("motor[%d][%d] - SET_MODES_OF_OPERATION command[%d] - value[%d]",
						//motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command, motor_cmd_array.motor_cmd[i].value);
			}
*/
			//if(motor_cmd_array.motor_cmd[i].slaveBoardID == 2 && motor_cmd_array.motor_cmd[i].node_id == 1) ROS_INFO("momo");

			//ROS_INFO("motor[%d][%d] - command[%d] - value[%d]",
			//		motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, cmds->motor_cmd[i].command, cmds->motor_cmd[i].value);
		}

		//check if the controlword needs to be set in order to send a Profile Position command.
		//This also ignore the current command.
		if(profilePositionCmdStep[i] == 1) //send the lower state of the controlword bit or set the operation mode
		{
			//check if Profile Position mode is activated
			if(modeOfOperation[i] == PROFILE_POSITION_MODE)
			{
				motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array.motor_cmd[i].value = 0x002F;
				profilePositionCmdStep[i] = 3; //jump to step 3 to send the upper state
			}
			else
			{
				motor_cmd_array.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array.motor_cmd[i].value = PROFILE_POSITION_MODE;
				profilePositionCmdStep[i] = 2; //increase to send the upper state
				modeOfOperation[i] = PROFILE_POSITION_MODE;
			}

			cmdIgnored[i] = true;
		}
		else if(profilePositionCmdStep[i] == 2) //send the lower state of the controlword bit
		{
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x002F;
			profilePositionCmdStep[i] = 3; //increase to send the upper state
			cmdIgnored[i] = true;

			//ROS_INFO("motor[%d][%d] - Profile Position - send the lower state of the controlword bit",
					//motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id);
		}
		else if(profilePositionCmdStep[i] == 3) //send the lower state of the controlword bit
		{
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x003F;
			profilePositionCmdStep[i] = 0; //reset to 0 after the new position has been applied
			cmdIgnored[i] = true;

			//ROS_INFO("motor[%d][%d] - Profile Position - send the upper state of the controlword bit",
					//motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_POSITION)
			{
				motor_cmd_array.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profilePositionCmdStep[i] = 1;

				//ROS_INFO("motor[%d][%d] - Profile Position - send target position [%d]",
						//motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].value);
			}

			cmdIgnored[i] = false;
		}

		//Do the same for Profile Velocity
		if(profileVelocityCmdStep[i] == 1) //send the controlword bit or set the operation mode
		{
			//check if Profile Velocity mode is activated
			if(modeOfOperation[i] == PROFILE_VELOCITY_MODE)
			{
				motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array.motor_cmd[i].value = 0x000F;
				profileVelocityCmdStep[i] = 0; //reset to 0 after the new velocity has been applied
			}
			else
			{
				motor_cmd_array.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array.motor_cmd[i].value = PROFILE_VELOCITY_MODE;
				profileVelocityCmdStep[i] = 2; //increase to send the controlword
				modeOfOperation[i] = PROFILE_VELOCITY_MODE;
			}

			cmdIgnored[i] = true;
		}
		else if(profileVelocityCmdStep[i] == 2) //set the controlword bit
		{
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x000F;
			profileVelocityCmdStep[i] = 0; //reset to 0 after the new velocity has been applied
			cmdIgnored[i] = true;

			ROS_INFO("motor[%d] - Profile Velocity [%d] - set the controlword bit",
					motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_VELOCITY)
			{
				motor_cmd_array.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profileVelocityCmdStep[i] = 1;

				ROS_INFO("motor[%d] - Profile Velocity [%d] - send target velocity [%d]",
						motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command, motor_cmd_array.motor_cmd[i].value);
			}

			cmdIgnored[i] = false;
		}

		//TODO Do the same for Current mode

		//TODO Do the same for Torque mode

		//check if the command was not replaced by a controlword cmd
		if(!cmdIgnored[i])
		{
			if(cmds->motor_cmd[i].command == SET_MODES_OF_OPERATION)
			{
				//update the mode of operation
				modeOfOperation[i] = cmds->motor_cmd[i].value;

				//if(motor_cmd_array.motor_cmd[i].slaveBoardID == 1 && motor_cmd_array.motor_cmd[i].node_id == 1)
				//	ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command, motor_cmd_array.motor_cmd[i].value);

			}
/*
			if(cmds->motor_cmd[i].command == SET_CURRENT_MODE_SETTING_VALUE)
			{
				if(motor_cmd_array.motor_cmd[i].slaveBoardID == 1 && motor_cmd_array.motor_cmd[i].node_id == 1)
									ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command, motor_cmd_array.motor_cmd[i].value);
			}
*/
		}

		//if(motor_cmd_array.motor_cmd[i].slaveBoardID == 2 && motor_cmd_array.motor_cmd[i].node_id == 1)
		//ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motor_cmd[i].slaveBoardID, motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command, motor_cmd_array.motor_cmd[i].value);

		//reset to false for the next cycle
		cmdIgnored[i] = false;
	}//for

	//SPECIAL CASES FOR CURRENT MODE MOTORS (with broken encoders)
/*
	//Right Shoulder, Arm - Triceps
	if((cmds->motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].slaveBoardID == 3) && (cmds->motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].node_id == 3))
	{
		motor_cmd_array.motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].slaveBoardID = 3;
		motor_cmd_array.motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].node_id = 3; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
		motor_cmd_array.motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].command = SET_CURRENT_MODE_SETTING_VALUE;
		motor_cmd_array.motor_cmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].value = 150;
	}
*/
}

/*! \fn int main(int argc, char** argv)
 *  \brief
 *  \param argc
 *  \param argv
 *  \return int
 */
int main(int argc, char** argv)
{
	//Initialize ROS
    ros::init(argc, argv, "osa_command_builder_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers TODO faire une classe
 /*   ros::Subscriber sub_setSBCommands[NUMBER_SLAVE_BOARDS];

    for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
    {
    	sub_setSBCommands[i] = nh.subscribe("/setSBCommands", 1, setSBCommands_cb);
    }
*/

    ros::Subscriber sub_set_motor_commands = nh.subscribe("/set_motor_commands", 1, setMotorCommandsCallback);

	//Publishers
	ros::Publisher pub_send_motor_cmd_array = nh.advertise<osa_msgs::MotorCmdMultiArray>("/motor_cmd_array", 1);

	//create the cmd multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = DOFS; //NUMBER_SLAVE_BOARDS;
	motor_cmd_array.layout.dim[0].stride = DOFS; //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].label = "epos";

	motor_cmd_array.layout.data_offset = 0;

	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(DOFS); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	ROS_INFO("Main loop");

	while(ros::ok())
	{
		resetMotorCmdMultiArray(); //reset cmd set, and write the correct nodeIDs with slave Nb offset
		ros::spinOnce(); //grab msg and update cmd

		//erase some commands with controlword commands to apply the previous motor cmds

		//publish the final motor command package
		pub_send_motor_cmd_array.publish(motor_cmd_array); //publish it

		r.sleep();
	}

	return 0;
}
