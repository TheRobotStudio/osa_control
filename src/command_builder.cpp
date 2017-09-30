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
 * @version 0.1.0
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
//#define DOFS	2

/*! Variables */
osa_msgs::MotorCmdMultiArray motor_cmd_array;

std::vector<bool> cmdIgnored; //[DOFS] = {false};
std::vector<int> modeOfOperation; //[DOFS] = {NO_MODE};
std::vector<int> mapIndexNodeID; //[DOFS] = {0}; // Maps the array index with the actual NodeID on the CAN bus.
std::vector<int> profilePositionCmdStep; //[DOFS] = {0}; // Every Profile Position command is followed by a rising edge on the controlword.
std::vector<int> profileVelocityCmdStep; //[DOFS] = {0}; // Every Profile Velocity command is followed by setting the controlword.

/**< ros parameters*/
std::string robot_name_;
std::string robot_can_device_;
int number_epos_boards_;

/**< TODO DRY Move this in a class */
std::vector<std::string> dof_name_list_;
std::vector<std::string> dof_type_list_;
std::vector<int> dof_node_id_list_;
std::vector<std::string> dof_controller_list_;
std::vector<std::string> dof_motor_list_;
std::vector<bool> dof_inverted_list_;
std::vector<std::string> dof_mode_list_;
std::vector<int> dof_value_list_;

/*
bool cmdIgnored[DOFS] = {false};
int modeOfOperation[DOFS] = {NO_MODE};
int mapIndexNodeID[DOFS] = {0}; // Maps the array index with the actual NodeID on the CAN bus.
int profilePositionCmdStep[DOFS] = {0}; // Every Profile Position command is followed by a rising edge on the controlword.
int profileVelocityCmdStep[DOFS] = {0}; // Every Profile Velocity command is followed by setting the controlword.
*/

/*! \fn void resetMotorCmdMultiArray()
 *  \brief
 *  \return void
 */
void resetMotorCmdMultiArray()
{
	for(int i=0; i<number_epos_boards_; i++)
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
	//ROS_INFO("Motor commands received");
	//#endif

	//ROS_INFO("Nb EPOS = %d", cmds->layout.dim[0].stride);

	for(int i=0; i<cmds->layout.dim[0].stride; i++)
	//for(int i=0; i<number_epos_boards_; i++)
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
				motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
				motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array.motor_cmd[i].value = 0x002F;
				profilePositionCmdStep[i] = 3; //jump to step 3 to send the upper state
			}
			else
			{
				motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
				motor_cmd_array.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array.motor_cmd[i].value = PROFILE_POSITION_MODE;
				profilePositionCmdStep[i] = 2; //increase to send the upper state
				modeOfOperation[i] = PROFILE_POSITION_MODE;
			}

			cmdIgnored[i] = true;
		}
		else if(profilePositionCmdStep[i] == 2) //send the lower state of the controlword bit
		{
			motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x002F;
			profilePositionCmdStep[i] = 3; //increase to send the upper state
			cmdIgnored[i] = true;

			ROS_DEBUG("motor[%d] - Profile Position - send the lower state of the controlword bit", motor_cmd_array.motor_cmd[i].node_id);
		}
		else if(profilePositionCmdStep[i] == 3) //send the lower state of the controlword bit
		{
			motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x003F;
			profilePositionCmdStep[i] = 0; //reset to 0 after the new position has been applied
			cmdIgnored[i] = true;

			ROS_DEBUG("motor[%d] - Profile Position - send the upper state of the controlword bit", motor_cmd_array.motor_cmd[i].node_id);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_POSITION)
			{
				mapIndexNodeID[i] = motor_cmd_array.motor_cmd[i].node_id; // Save the NodeID for later steps.
				motor_cmd_array.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profilePositionCmdStep[i] = 1;

				ROS_DEBUG("motor[%d] - Profile Position - send target position [%d]", motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].value);
			}

			cmdIgnored[i] = false;
		}

		//Do the same for Profile Velocity
		if(profileVelocityCmdStep[i] == 1) //send the controlword bit or set the operation mode
		{
			//check if Profile Velocity mode is activated
			if(modeOfOperation[i] == PROFILE_VELOCITY_MODE)
			{
				motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
				motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
				motor_cmd_array.motor_cmd[i].value = 0x000F;
				profileVelocityCmdStep[i] = 0; //reset to 0 after the new velocity has been applied
			}
			else
			{
				motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
				motor_cmd_array.motor_cmd[i].command = SET_MODES_OF_OPERATION;
				motor_cmd_array.motor_cmd[i].value = PROFILE_VELOCITY_MODE;
				profileVelocityCmdStep[i] = 2; //increase to send the controlword
				modeOfOperation[i] = PROFILE_VELOCITY_MODE;
			}

			cmdIgnored[i] = true;
		}
		else if(profileVelocityCmdStep[i] == 2) //set the controlword bit
		{
			motor_cmd_array.motor_cmd[i].node_id =  mapIndexNodeID[i];
			motor_cmd_array.motor_cmd[i].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i].value = 0x000F;
			profileVelocityCmdStep[i] = 0; //reset to 0 after the new velocity has been applied
			cmdIgnored[i] = true;

			ROS_DEBUG("motor[%d] - Profile Velocity [%d] - set the controlword bit",
					motor_cmd_array.motor_cmd[i].node_id, motor_cmd_array.motor_cmd[i].command);
		}
		else // == 0
		{
			if(cmds->motor_cmd[i].command == SET_TARGET_VELOCITY)
			{
				mapIndexNodeID[i] = motor_cmd_array.motor_cmd[i].node_id; // Save the NodeID for later steps.
				motor_cmd_array.motor_cmd[i].command = cmds->motor_cmd[i].command;
				motor_cmd_array.motor_cmd[i].value = cmds->motor_cmd[i].value;
				profileVelocityCmdStep[i] = 1;

				ROS_DEBUG("motor[%d] - Profile Velocity [%d] - send target velocity [%d]",
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

    // Grab the parameters //TODO avoid WET code, do DRY code, same as in class CANLayer od osa_communication package
	try
	{
		//load robot parameters
		if(!nh.param("/robot/name", robot_name_, std::string("my_robot")))
		{
			ROS_WARN("No /robot/name found in YAML config file");
		}

		if(!nh.param("/robot/dof", number_epos_boards_, int(2)))
		{
			ROS_WARN("No /robot/dof found in YAML config file");
		}

		if(!nh.param("/robot/can_device", robot_can_device_, std::string("can0")))
		{
			ROS_WARN("No /robot/can_device found in YAML config file");
		}

		ROS_DEBUG("Robot name=%s, dof=%d, can=%s", robot_name_.c_str(), number_epos_boards_, robot_can_device_.c_str());
/*
		//load mobile_base parameters
		if(nh.searchParam("/mobile_base", mobile_base_str))
		{
			ROS_INFO("/mobile_base found in YAML config file");
		}
		else
		{
			ROS_WARN("No /mobile_base found in YAML config file");
		}
*/
		//load controllers parameters
		//Example:
		//controller1: {node_id: 1, name: 'right wheel', type: 'EPOS4', inverted: true, motor: 'EC90', mode: 'PROFILE_VELOCITY_MODE', value: 0}

		bool dof_exist = true;
		//start with controller 1
		int dof_idx = 1;
		std::string rad_str = "dof"; //common radical name

		while(dof_exist)
		{
			//create the string "controller+index" to search for the controller parameter with that index number
			std::ostringstream dof_idx_path;
			dof_idx_path << rad_str << dof_idx;

			std::string absolute_str = "absolute_str";

			//ROS_INFO("string=%s", dof_idx_path.str().c_str());

			if(nh.searchParam(dof_idx_path.str(), absolute_str))
			{
				//ROS_INFO("%s found in YAML config file", dof_idx_path.str().c_str());
				//ROS_INFO("absolute_str = %s", absolute_str.c_str());

				//create variables to store the controller parameters:
				std:: string name;
				std:: string type;
				int node_id = 0;
				std:: string controller;
				std:: string motor;
				bool inverted;
				std:: string mode;
				int value;

				//grab the parameters of the current controller

				//name
				std::ostringstream name_path;
				name_path << absolute_str << "/name";
				if(!nh.getParam(name_path.str(), name))
				{
					ROS_ERROR("Can't grab param name for %s", dof_idx_path.str().c_str());
					return false;
				}

				//type
				std::ostringstream type_path;
				type_path << absolute_str << "/type";
				if(!nh.getParam(type_path.str(), type))
				{
					ROS_ERROR("Can't grab param type for %s", dof_idx_path.str().c_str());
					return false;
				}

				//node_id
				std::ostringstream node_id_path;
				node_id_path << absolute_str << "/node_id";
				if(!nh.getParam(node_id_path.str(), node_id))
				{
					ROS_ERROR("Can't grab param node_id for %s", dof_idx_path.str().c_str());
					return false;
				}

				//controller
				std::ostringstream controller_path;
				controller_path << absolute_str << "/controller";
				if(!nh.getParam(controller_path.str(), controller))
				{
					ROS_ERROR("Can't grab param controller for %s", dof_idx_path.str().c_str());
					return false;
				}

				//motor
				std::ostringstream motor_path;
				motor_path << absolute_str << "/motor";
				if(!nh.getParam(motor_path.str(), motor))
				{
					ROS_ERROR("Can't grab param motor for %s", dof_idx_path.str().c_str());
					return false;
				}

				//inverted
				std::ostringstream inverted_path;
				inverted_path << absolute_str << "/inverted";
				if(!nh.getParam(inverted_path.str(), inverted))
				{
					ROS_ERROR("Can't grab param inverted for %s", dof_idx_path.str().c_str());
					return false;
				}

				//mode
				std::ostringstream mode_path;
				mode_path << absolute_str << "/mode";
				if(!nh.getParam(mode_path.str(), mode))
				{
					ROS_ERROR("Can't grab param mode for %s", dof_idx_path.str().c_str());
					return false;
				}

				//value
				std::ostringstream value_path;
				value_path << absolute_str << "/value";
				if(!nh.getParam(value_path.str(), value))
				{
					ROS_ERROR("Can't grab param value for %s", dof_idx_path.str().c_str());
					return false;
				}

				//print the dof parameters
				ROS_INFO("%s : name[%s], type[%s], node_id[%d], controller[%s], motor[%s], inverted[%d], mode[%s], value[%d]", dof_idx_path.str().c_str(),
						name.c_str(), type.c_str(), node_id, controller.c_str(), motor.c_str(), inverted, mode.c_str(), value);

				//save the dof data in the attributes
				//number_epos_boards_
				dof_name_list_.push_back(name);
				dof_type_list_.push_back(type);
				dof_node_id_list_.push_back(node_id);
				dof_controller_list_.push_back(controller);
				dof_motor_list_.push_back(motor);
				dof_inverted_list_.push_back(inverted);
				dof_mode_list_.push_back(mode);
				dof_value_list_.push_back(value);

				//increment to search for the next controller
				dof_idx++;
			}
			else
			{
				dof_exist = false;
				//ROS_INFO("No more controllers found in YAML config file");
			}

			//dof_exist = false;
		}

		dof_idx--;
		if(number_epos_boards_ == dof_idx) ROS_INFO("Same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
		else
		{
			ROS_WARN("Not the same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
			throw 1;
		}

		/*
		nh.param("can_device", can_device_str, std::string("can0"));
		//mot1
		nh.param("controller1_type", controller1_type_str, std::string("EPOS4"));
		nh.param("motor1_type", motor1_type_str, std::string("EC90"));
		nh.param("motor1_inverted", motor1_inverted_bool, bool(true));
		nh.param("mode1", mode1_str, std::string("PROFILE_VELOCITY_MODE"));
		nh.param("value1", value1_int, int(0));
		//mot2
		nh.param("controller2_type", controller2_type_str, std::string("EPOS4"));
		nh.param("motor2_type", motor2_type_str, std::string("EC90"));
		nh.param("motor2_inverted", motor2_inverted_bool, bool(false));
		nh.param("mode2", mode2_str, std::string("PROFILE_VELOCITY_MODE"));
		//nh.param("value2", value2_int, int(0));
		nh.param("value2", value2_int);
		*/

		ROS_INFO("Parameters loaded successfully!\n");
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		ROS_ERROR("Parameters didn't load correctly!");
		ROS_ERROR("Please modify your YAML config file and try again.");

		return false;
	}

	//set the size for the arrays
	cmdIgnored.resize(number_epos_boards_, false);
	modeOfOperation.resize(number_epos_boards_, NO_MODE);
	mapIndexNodeID.resize(number_epos_boards_, 0);
	profilePositionCmdStep.resize(number_epos_boards_, 0);
	profileVelocityCmdStep.resize(number_epos_boards_, 0);

    ros::Subscriber sub_set_motor_commands = nh.subscribe("/set_motor_commands", 1, setMotorCommandsCallback);

	//Publishers
	ros::Publisher pub_send_motor_cmd_array = nh.advertise<osa_msgs::MotorCmdMultiArray>("/motor_cmd_array", 1);

	//create the cmd multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = number_epos_boards_; //NUMBER_SLAVE_BOARDS;
	motor_cmd_array.layout.dim[0].stride = number_epos_boards_; //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].label = "epos";

	motor_cmd_array.layout.data_offset = 0;

	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(number_epos_boards_); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

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
