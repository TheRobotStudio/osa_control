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
 * @file command_builder.h
 * @author Cyril Jourdan
 * @date Oct 3, 2017
 * @version 0.1.0
 * @brief Header file for the class CommandBuilder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Oct 3, 2017
 */

#ifndef OSA_CONTROL_COMMAND_BUILDER_H
#define OSA_CONTROL_COMMAND_BUILDER_H

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
//ROS messages
#include "osa_msgs/MotorCmdMultiArray.h"
//others
#include <string>

#include "osa_common/robot_description.h"

namespace osa_control
{

/**
 * @brief This is the class for CommandBuilder.
 */
class CommandBuilder
{
public:
	/**
	 * @brief Constructor.
	 */
	CommandBuilder();

	/**
	 * @brief Destructor.
	 */
	~CommandBuilder();

	/** @brief Initialize the ROS node. */
	bool init();

	/** @brief Run the ROS node. */
	void run();

	void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds);
	void resetMotorCmdArray();

protected:
	//ros::NodeHandle nh_;
	osa_common::RobotDescription* ptr_robot_description_;
	ros::Subscriber sub_set_motor_commands_;
	ros::Publisher pub_send_motor_cmd_array_;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
	std::vector<bool> cmd_ignored_; //[DOFS] = {false};
	std::vector<int> mode_of_operation_; //[DOFS] = {NO_MODE};
	std::vector<int> map_index_node_id_; //[DOFS] = {0}; // Maps the array index with the actual NodeID on the CAN bus.
	std::vector<int> profile_position_cmd_step_; //[DOFS] = {0}; // Every Profile Position command is followed by a rising edge on the controlword.
	std::vector<int> profile_velocity_cmd_step_; //[DOFS] = {0}; // Every Profile Velocity command is followed by setting the controlword.
	//std::vector<int> step_;
};

} // namespace osa_control

#endif // OSA_CONTROL_COMMAND_BUILDER_H
