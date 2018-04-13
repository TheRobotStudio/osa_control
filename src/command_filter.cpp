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
 * @file command_filter.cpp
 * @author Cyril Jourdan
 * @date Sep 11, 2017
 * @version 0.1.0
 * @brief Implementation file for the class CommandFilter
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 15, 2013
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"

#include <osa_common/enums.h>

#include "command_filter.h"

namespace osa_control
{

CommandFilter::CommandFilter()
{
}

CommandFilter::~CommandFilter(void)
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // This is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

/**
 * @brief Initialize the ROS node.
 * @return bool Returns true if the initialization has completed successfully and false otherwise.
 */
bool CommandFilter::init()
{
	ROS_INFO("*** CommandFilter Init ***\n");

	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_command_filter_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("~");

	//then start the main loop
	ROS_INFO("*** Command filter Start main loop ***");
	run();

	return true;
}

/**
 * @brief Run the ROS node.
 * @return void
 */
void CommandFilter::run()
{
	ros::Rate r(50);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

/*! \fn void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
 *  \brief
 *  \return void
 */
void CommandFilter::setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
{

}

/*! \fn void resetMotorCmdArray()
 *  \brief
 *  \return void
 */
void CommandFilter::resetMotorCmdArray()
{

}

} // namespace osa_control

/*! \fn int main(int argc, char** argv)
 *  \brief
 *  \param argc
 *  \param argv
 *  \return int
 */
int main(int argc, char** argv)
{
	osa_control::CommandFilter *command_filter = new osa_control::CommandFilter();
	if(command_filter->init()) return -1; //Or call that from the constructor ?

	//Main loop
	while(ros::ok())
	{
		//TODO wait ?
	}

	return 0;
}
