/*
 * Copyright (c) 2018, The Robot Studio
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
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Apr 18, 2018
 * @date Created on Mar 15, 2013
 * @version 0.1.1
 * @brief Implementation file for the class CommandBuilder
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"

#include <osa_common/enums.h>

#include "osa_control/command_builder.h"

using namespace std;
using namespace osa_common;

namespace osa_control
{

CommandBuilder::CommandBuilder()
{
}

CommandBuilder::~CommandBuilder(void)
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // This is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

bool CommandBuilder::init()
{
	ROS_INFO("*** CommandBuilder Init ***\n");

	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_command_builder_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("~");

	ptr_robot_description_ = new osa_common::RobotDescription(&nh);

	ROS_INFO("*** Grab the parameters from the Parameter Server ***");

	try
	{
		ptr_robot_description_->grabRobotNamespaceFromParameterServer();
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR("Invalid Robot Namespace parameter!");
		return false;
	}

	try
	{
		ptr_robot_description_->grabRobotFromParameterServer();
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		return false;
	}
	catch(runtime_error const &e)
	{
		ROS_ERROR("Robot Namespace parameter not defined!");
		return false;
	}

	try
	{
		ptr_robot_description_->grabDOFFromParameterServer();
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		return false;
	}
	catch(runtime_error const &e)
	{
		ROS_ERROR("Robot Namespace parameter not defined!");
		return false;
	}

	//set the size for the arrays
	cmd_ignored_.resize(ptr_robot_description_->getRobotDof(), false);
	mode_of_operation_.resize(ptr_robot_description_->getRobotDof(), ActivatedModeOfOperation(CURRENT_MODE));
	map_index_node_id_.resize(ptr_robot_description_->getRobotDof(), 0);
	profile_position_cmd_step_.resize(ptr_robot_description_->getRobotDof(), 0);
	profile_velocity_cmd_step_.resize(ptr_robot_description_->getRobotDof(), 0);

	//Subsriber, need the number of EPOS for the FIFO
	sub_motor_cmd_to_build_ = nh.subscribe(ptr_robot_description_->getRobotNamespace() + "/motor_cmd_to_build", 1, &CommandBuilder::motorCmdToBuildCallback, this);

	//Publishers
	pub_send_motor_cmd_array_ = nh.advertise<osa_msgs::MotorCmdMultiArray>(ptr_robot_description_->getRobotNamespace() + "/motor_cmd_array", 1);

	//create the cmd multi array
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = ptr_robot_description_->getRobotDof(); //NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = ptr_robot_description_->getRobotDof(); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "epos";

	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(ptr_robot_description_->getRobotDof());

	//then start the main loop
	ROS_INFO("*** Command builder Start main loop ***");
	run();

	return true;
}

void CommandBuilder::run()
{
	ros::Rate r(ptr_robot_description_->getRobotHeartbeat());

	while(ros::ok())
	{
		resetMotorCmdArray(); //reset cmd set, and write the correct nodeIDs with slave Nb offset
		ros::spinOnce(); //grab msg and update cmd

		//erase some commands with controlword commands to apply the previous motor cmds

		//publish the final motor command package
		pub_send_motor_cmd_array_.publish(motor_cmd_array_); //publish it

		r.sleep();
	}
}

void CommandBuilder::motorCmdToBuildCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
{
	//ROS_INFO("Nb EPOS = %d", cmds->layout.dim[0].stride);

	for(int i=0; i<cmds->layout.dim[0].stride; i++)
	//for(int i=0; i<ptr_robot_description_->getRobotDof(); i++)
	{
		//ROS_INFO("EPOS[%d]", i);

		//int i = cmds->motor_cmd[i].node_id - 1; //(cmds->motor_cmd[i].slaveBoardID - 1)*NUMBER_MAX_EPOS2_PER_SLAVE + (cmds->motor_cmd[i].node_id - 1);

		motor_cmd_array_.motor_cmd[i].node_id = cmds->motor_cmd[i].node_id;

		//other cases just apply the command //TODO optimize because it sometimes doesn't need to go through this if
		if((cmds->motor_cmd[i].command != SET_TARGET_POSITION) && (cmds->motor_cmd[i].command != SET_TARGET_VELOCITY)) //TODO add current mode
		{
			motor_cmd_array_.motor_cmd[i].command = cmds->motor_cmd[i].command;
			motor_cmd_array_.motor_cmd[i].value = cmds->motor_cmd[i].value;

/*			if(motor_cmd_array_.motor_cmd[i].command == SET_MODES_OF_OPERATION)
			{

				//ROS_INFO("motor[%d][%d] - SET_MODES_OF_OPERATION command[%d] - value[%d]",
						//motor_cmd_array_.motor_cmd[i].slaveBoardID, motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command, motor_cmd_array_.motor_cmd[i].value);
			}
*/
			//if(motor_cmd_array_.motor_cmd[i].slaveBoardID == 2 && motor_cmd_array_.motor_cmd[i].node_id == 1) ROS_INFO("momo");

			//ROS_INFO("motor[%d][%d] - command[%d] - value[%d]",
			//		motor_cmd_array_.motor_cmd[i].slaveBoardID, motor_cmd_array_.motor_cmd[i].node_id, cmds->motor_cmd[i].command, cmds->motor_cmd[i].value);
		}

		//check if the controlword needs to be set in order to send a Profile Position command.
		//This also ignore the current command.
		if(profile_position_cmd_step_[i] == 1) //send the lower state of the controlword bit or set the operation mode
		{
			//check if Profile Position mode is activated
			if(mode_of_operation_[i] == PROFILE_POSITION_MODE)
			{
				motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
				motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array_.motor_cmd[i].value = 0x002F;
				profile_position_cmd_step_[i] = 3; //jump to step 3 to send the upper state
			}
			else
			{
				motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
				motor_cmd_array_.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array_.motor_cmd[i].value = PROFILE_POSITION_MODE;
				profile_position_cmd_step_[i] = 2; //increase to send the upper state
				mode_of_operation_[i] = PROFILE_POSITION_MODE;
			}

			cmd_ignored_[i] = true;
		}
		else if(profile_position_cmd_step_[i] == 2) //send the lower state of the controlword bit
		{
			motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
			motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array_.motor_cmd[i].value = 0x002F;
			profile_position_cmd_step_[i] = 3; //increase to send the upper state
			cmd_ignored_[i] = true;

			ROS_DEBUG("motor[%d] - Profile Position - send the lower state of the controlword bit", motor_cmd_array_.motor_cmd[i].node_id);
		}
		else if(profile_position_cmd_step_[i] == 3) //send the lower state of the controlword bit
		{
			motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
			motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array_.motor_cmd[i].value = 0x003F;
			profile_position_cmd_step_[i] = 0; //reset to 0 after the new position has been applied
			cmd_ignored_[i] = true;

			ROS_DEBUG("motor[%d] - Profile Position - send the upper state of the controlword bit", motor_cmd_array_.motor_cmd[i].node_id);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_POSITION)
			{
				map_index_node_id_[i] = motor_cmd_array_.motor_cmd[i].node_id; // Save the NodeID for later steps.
				motor_cmd_array_.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array_.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profile_position_cmd_step_[i] = 1;

				ROS_DEBUG("motor[%d] - Profile Position - send target position [%d]", motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].value);
			}

			cmd_ignored_[i] = false;
		}

		//Do the same for Profile Velocity
		if(profile_velocity_cmd_step_[i] == 1) //send the controlword bit or set the operation mode
		{
			//check if Profile Velocity mode is activated
			if(mode_of_operation_[i] == PROFILE_VELOCITY_MODE)
			{
				motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
				motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array_.motor_cmd[i].value = 0x000F;
				profile_velocity_cmd_step_[i] = 0; //reset to 0 after the new velocity has been applied
			}
			else
			{
				motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
				motor_cmd_array_.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array_.motor_cmd[i].value = PROFILE_VELOCITY_MODE;
				profile_velocity_cmd_step_[i] = 2; //increase to send the controlword
				mode_of_operation_[i] = PROFILE_VELOCITY_MODE;
			}

			cmd_ignored_[i] = true;
		}
		else if(profile_velocity_cmd_step_[i] == 2) //set the controlword bit
		{
			motor_cmd_array_.motor_cmd[i].node_id =  map_index_node_id_[i];
			motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array_.motor_cmd[i].value = 0x000F;
			profile_velocity_cmd_step_[i] = 0; //reset to 0 after the new velocity has been applied
			cmd_ignored_[i] = true;

			ROS_DEBUG("motor[%d] - Profile Velocity [%d] - set the controlword bit",
					motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_VELOCITY)
			{
				map_index_node_id_[i] = motor_cmd_array_.motor_cmd[i].node_id; // Save the NodeID for later steps.
				motor_cmd_array_.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array_.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profile_velocity_cmd_step_[i] = 1;

				ROS_DEBUG("motor[%d] - Profile Velocity [%d] - send target velocity [%d]",
						motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command, motor_cmd_array_.motor_cmd[i].value);
			}

			cmd_ignored_[i] = false;
		}

		//TODO Do the same for Current mode

		//TODO Do the same for Torque mode

		//check if the command was not replaced by a controlword cmd
		if(!cmd_ignored_[i])
		{
			if(cmds->motor_cmd[i].command == SET_MODES_OF_OPERATION)
			{
				//update the mode of operation
				mode_of_operation_[i] = cmds->motor_cmd[i].value;

				//if(motor_cmd_array_.motor_cmd[i].slaveBoardID == 1 && motor_cmd_array_.motor_cmd[i].node_id == 1)
				//	ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array_.motor_cmd[i].slaveBoardID, motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command, motor_cmd_array_.motor_cmd[i].value);

			}
/*
			if(cmds->motor_cmd[i].command == SET_CURRENT_MODE_SETTING_VALUE)
			{
				if(motor_cmd_array_.motor_cmd[i].slaveBoardID == 1 && motor_cmd_array_.motor_cmd[i].node_id == 1)
									ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array_.motor_cmd[i].slaveBoardID, motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command, motor_cmd_array_.motor_cmd[i].value);
			}
*/
		}

		//if(motor_cmd_array_.motor_cmd[i].slaveBoardID == 2 && motor_cmd_array_.motor_cmd[i].node_id == 1)
		//ROS_INFO("motor[%d][%d] cmd[%d] val[%d]", motor_cmd_array_.motor_cmd[i].slaveBoardID, motor_cmd_array_.motor_cmd[i].node_id, motor_cmd_array_.motor_cmd[i].command, motor_cmd_array_.motor_cmd[i].value);

		//reset to false for the next cycle
		cmd_ignored_[i] = false;
	}//for
}

void CommandBuilder::resetMotorCmdArray()
{
	for(int i=0; i<ptr_robot_description_->getRobotDof(); i++)
	{
		motor_cmd_array_.motor_cmd[i].node_id = 0;
		motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array_.motor_cmd[i].value = 0;
	}
}

} // namespace osa_control

/** @fn int main(int argc, char** argv)
 *  @brief
 *  @param argc
 *  @param argv
 *  @return int
 */
int main(int argc, char** argv)
{
	osa_control::CommandBuilder *command_builder = new osa_control::CommandBuilder();
	if(command_builder->init()) return -1;

	return 0;
}
