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
 * @file commandBuilder.cpp
 * @author Cyril Jourdan
 * @date Feb 24, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for the command builder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 15, 2013
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "robotDefines.h"

/*! Defines */
#define LOOP_RATE		HEART_BEAT

/*! Variables */
osa_msgs::MotorCmdMultiArray motor_cmd_array;
bool cmdIgnored[NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE] = {false};
int modeOfOperation[NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE] = {NO_MODE};
int profilePositionCmdStep[NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE] = {0}; //every Profile Position command is followed by a rising edge on the controlword
int profileVelocityCmdStep[NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE] = {0}; //every Profile Velocity command is followed by setting the controlword

/*! \fn void resetMotorCmdMultiArray()
 *  \brief
 *  \return void
 */
void resetMotorCmdMultiArray()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motor_cmd_array.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			motor_cmd_array.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
	}
}

/*! \fn void setMotorCommands_cb(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
 *  \brief
 *  \return void
 */
void setMotorCommands_cb(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
{
	//cmds is expected to be dimension of 1 by NUMBER_MOTORS_ARM
	#ifdef TRS_DEBUG
	ROS_INFO("Motor commands received");
	#endif


	for(int i=0; i<cmds->layout.dim[0].stride; i++)
	{
		int motorIdx = (cmds->motorCmd[i].slaveBoardID - 1)*NUMBER_MAX_EPOS2_PER_SLAVE + (cmds->motorCmd[i].nodeID - 1);

		//check that motorIdx is within the range of the motor_cmd_array array
		if((motorIdx>=0) && (motorIdx<motor_cmd_array.layout.dim[0].stride))
		{
			motor_cmd_array.motorCmd[motorIdx].slaveBoardID = cmds->motorCmd[i].slaveBoardID;
			motor_cmd_array.motorCmd[motorIdx].nodeID = cmds->motorCmd[i].nodeID;

			//other cases just apply the command //TODO optimize because it sometimes doesn't need to go through this if
			if((cmds->motorCmd[i].command != SET_TARGET_POSITION) && (cmds->motorCmd[i].command != SET_TARGET_VELOCITY)) //TODO add current mode
			{
				motor_cmd_array.motorCmd[motorIdx].command = cmds->motorCmd[i].command;
				motor_cmd_array.motorCmd[motorIdx].value = cmds->motorCmd[i].value;

	/*			if(motor_cmd_array.motorCmd[motorIdx].command == SET_MODES_OF_OPERATION)
				{

					//ROS_INFO("motor[%d][%d] - SET_MODES_OF_OPERATION command[%d] - value[%d]",
							//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command, motor_cmd_array.motorCmd[motorIdx].value);
				}
	*/
				//if(motor_cmd_array.motorCmd[motorIdx].slaveBoardID == 2 && motor_cmd_array.motorCmd[motorIdx].nodeID == 1) ROS_INFO("momo");

				//ROS_INFO("motor[%d][%d] - command[%d] - value[%d]",
				//		motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, cmds->motorCmd[i].command, cmds->motorCmd[i].value);
			}

			//check if the controlword needs to be set in order to send a Profile Position command.
			//This also ignore the current command.
			if(profilePositionCmdStep[motorIdx] == 1) //send the lower state of the controlword bit or set the operation mode
			{
				//check if Profile Position mode is activated
				if(modeOfOperation[motorIdx] == PROFILE_POSITION_MODE)
				{
					motor_cmd_array.motorCmd[motorIdx].command = SET_CONTROLWORD;
					motor_cmd_array.motorCmd[motorIdx].value = 0x002F;
					profilePositionCmdStep[motorIdx] = 3; //jump to step 3 to send the upper state
				}
				else
				{
					motor_cmd_array.motorCmd[motorIdx].command = SET_MODES_OF_OPERATION;
					motor_cmd_array.motorCmd[motorIdx].value = PROFILE_POSITION_MODE;
					profilePositionCmdStep[motorIdx] = 2; //increase to send the upper state
					modeOfOperation[motorIdx] = PROFILE_POSITION_MODE;
				}

				cmdIgnored[motorIdx] = true;
			}
			else if(profilePositionCmdStep[motorIdx] == 2) //send the lower state of the controlword bit
			{
				motor_cmd_array.motorCmd[motorIdx].command = SET_CONTROLWORD;
				motor_cmd_array.motorCmd[motorIdx].value = 0x002F;
				profilePositionCmdStep[motorIdx] = 3; //increase to send the upper state
				cmdIgnored[motorIdx] = true;

				//ROS_INFO("motor[%d][%d] - Profile Position - send the lower state of the controlword bit",
						//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID);
			}
			else if(profilePositionCmdStep[motorIdx] == 3) //send the lower state of the controlword bit
			{
				motor_cmd_array.motorCmd[motorIdx].command = SET_CONTROLWORD;
				motor_cmd_array.motorCmd[motorIdx].value = 0x003F;
				profilePositionCmdStep[motorIdx] = 0; //reset to 0 after the new position has been applied
				cmdIgnored[motorIdx] = true;

				//ROS_INFO("motor[%d][%d] - Profile Position - send the upper state of the controlword bit",
						//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID);
			}
			else // == 0
			{
				if(cmds->motorCmd[i].command == SET_TARGET_POSITION)
				{
					motor_cmd_array.motorCmd[motorIdx].command = cmds->motorCmd[i].command;
					motor_cmd_array.motorCmd[motorIdx].value = cmds->motorCmd[i].value;
					profilePositionCmdStep[motorIdx] = 1;

					//ROS_INFO("motor[%d][%d] - Profile Position - send target position [%d]",
							//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].value);
				}

				cmdIgnored[motorIdx] = false;
			}

			//Do the same for Profile Velocity
			if(profileVelocityCmdStep[motorIdx] == 1) //send the controlword bit or set the operation mode
			{
				//check if Profile Velocity mode is activated
				if(modeOfOperation[motorIdx] == PROFILE_VELOCITY_MODE)
				{
					motor_cmd_array.motorCmd[motorIdx].command = SET_CONTROLWORD;
					motor_cmd_array.motorCmd[motorIdx].value = 0x000F;
					profileVelocityCmdStep[motorIdx] = 0; //reset to 0 after the new velocity has been applied
				}
				else
				{
					motor_cmd_array.motorCmd[motorIdx].command = SET_MODES_OF_OPERATION;
					motor_cmd_array.motorCmd[motorIdx].value = PROFILE_VELOCITY_MODE;
					profileVelocityCmdStep[motorIdx] = 2; //increase to send the controlword
					modeOfOperation[motorIdx] = PROFILE_VELOCITY_MODE;
				}

				cmdIgnored[motorIdx] = true;
			}
			else if(profileVelocityCmdStep[motorIdx] == 2) //set the controlword bit
			{
				motor_cmd_array.motorCmd[motorIdx].command = SET_CONTROLWORD;
				motor_cmd_array.motorCmd[motorIdx].value = 0x000F;
				profileVelocityCmdStep[motorIdx] = 0; //reset to 0 after the new velocity has been applied
				cmdIgnored[motorIdx] = true;

				//ROS_INFO("motor[%d][%d] - Profile Velocity [%d] - set the controlword bit",
						//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command);
			}
			else // == 0
			{
				if(cmds->motorCmd[i].command == SET_TARGET_VELOCITY)
				{
					motor_cmd_array.motorCmd[motorIdx].command = cmds->motorCmd[i].command;
					motor_cmd_array.motorCmd[motorIdx].value = cmds->motorCmd[i].value;
					profileVelocityCmdStep[motorIdx] = 1;

					//ROS_INFO("motor[%d][%d] - Profile Velocity [%d] - send target velocity [%d]",
							//motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command, motor_cmd_array.motorCmd[motorIdx].value);
				}

				cmdIgnored[motorIdx] = false;
			}

			//TODO Do the same for Current mode

			//TODO Do the same for Torque mode

			//check if the command was not replaced by a controlword cmd
			if(!cmdIgnored[motorIdx])
			{
				if(cmds->motorCmd[i].command == SET_MODES_OF_OPERATION)
				{
					//update the mode of operation
					modeOfOperation[motorIdx] = cmds->motorCmd[i].value;

					//if(motor_cmd_array.motorCmd[motorIdx].slaveBoardID == 1 && motor_cmd_array.motorCmd[motorIdx].nodeID == 1)
					//	ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command, motor_cmd_array.motorCmd[motorIdx].value);

				}
	/*
				if(cmds->motorCmd[i].command == SET_CURRENT_MODE_SETTING_VALUE)
				{
					if(motor_cmd_array.motorCmd[motorIdx].slaveBoardID == 1 && motor_cmd_array.motorCmd[motorIdx].nodeID == 1)
										ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command, motor_cmd_array.motorCmd[motorIdx].value);
				}
	*/
			}

			//if(motor_cmd_array.motorCmd[motorIdx].slaveBoardID == 2 && motor_cmd_array.motorCmd[motorIdx].nodeID == 1)
			//ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array.motorCmd[motorIdx].slaveBoardID, motor_cmd_array.motorCmd[motorIdx].nodeID, motor_cmd_array.motorCmd[motorIdx].command, motor_cmd_array.motorCmd[motorIdx].value);

			//reset to false for the next cycle
			cmdIgnored[motorIdx] = false;
		}//if
	}//for

	//SPECIAL CASES FOR CURRENT MODE MOTORS (with broken encoders)
/*
	//Right Shoulder, Arm - Triceps
	if((cmds->motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].slaveBoardID == 3) && (cmds->motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].nodeID == 3))
	{
		motor_cmd_array.motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].slaveBoardID = 3;
		motor_cmd_array.motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].nodeID = 3; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
		motor_cmd_array.motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].command = SET_CURRENT_MODE_SETTING_VALUE;
		motor_cmd_array.motorCmd[2*NUMBER_MAX_EPOS2_PER_SLAVE + 2].value = 150;
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
    ros::init(argc, argv, "osa_commandBuilder_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers TODO faire une classe
 /*   ros::Subscriber sub_setSBCommands[NUMBER_SLAVE_BOARDS];

    for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
    {
    	sub_setSBCommands[i] = nh.subscribe("/setSBCommands", 1, setSBCommands_cb);
    }
*/
    ros::Subscriber sub_setMotorCommands = nh.subscribe("/set_motor_commands", 1, setMotorCommands_cb);

	//Publishers
	ros::Publisher pub_sendMotorCmdMultiArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/motor_cmd_array", 1);

	//create the cmd multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motor_cmd_array.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].label = "slaves";

	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[1].label = "motors";

	motor_cmd_array.layout.data_offset = 0;

	motor_cmd_array.motorCmd.clear();
	motor_cmd_array.motorCmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	while(ros::ok())
	{
		resetMotorCmdMultiArray(); //reset cmd set, and write the correct nodeIDs with slave Nb offset
		ros::spinOnce(); //grab msg and update cmd

		//erase some commands with controlword commands to apply the previous motor cmds


		//publish the final motor command package
		pub_sendMotorCmdMultiArray.publish(motor_cmd_array); //publish it

		r.sleep();
	}

	return 0;
}

