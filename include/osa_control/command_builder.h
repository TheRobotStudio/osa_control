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
 * @brief Header file for the class CommandFilter
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
//ROS actionlib
#include <actionlib/server/simple_action_server.h>
#include <osa_control/PlaySequenceAction.h>
//ROS bag
#include <rosbag/bag.h>
//ROS messages
#include "osa_msgs/MotorCmdMultiArray.h"
//Flann
#include <flann/flann.hpp>
//others
#include <string>

namespace osa_control
{

typedef actionlib::SimpleActionServer<PlaySequenceAction> ActionServer;

/**
 * @brief This is the class for CommandFilter.
 */
class CommandFilter
{
public:
	/**
	 * @brief Constructor.
	 */
	CommandFilter();

	/**
	 * @brief Destructor.
	 */
	~CommandFilter();

	void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds);
	void resetMotorCmdArray();

protected:
	ros::NodeHandle nh_;
};

} // namespace osa_control

#endif // OSA_CONTROL_COMMAND_BUILDER_H
