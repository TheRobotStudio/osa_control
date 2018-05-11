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
 * @file command_filter.cpp
 * @author Cyril Jourdan
 * @date Apr 19, 2018
 * @version 0.1.0
 * @brief Implementation file for the class CommandFilter
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Oct 3, 2017
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"

#include <osa_common/enums.h>

#include "osa_control/command_filter.h"

using namespace std;
using namespace osa_common;

namespace osa_control
{

CommandFilter::CommandFilter()
{
}

CommandFilter::~CommandFilter(void)
{
	//TODO delete vector of pointers

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
/*
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_command_filter_node");
*/
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

	ROS_INFO("Setup dynamic_reconfigure parameters");

	//Init the motor dyn config list
	for(int i=0; i<ptr_robot_description_->getRobotDof(); i++)
	{
		motor_param_list_.push_back(osa_control::MotorDynConfig::__getDefault__());
	}

	for(int i=0; i<ptr_robot_description_->getRobotDof(); i++)
	{
		const std::string dof_name = ptr_robot_description_->getControllerList().at(i)->getName();
		ros::NodeHandle node_handle(dof_name.c_str());

		dynamic_reconfigure::Server<osa_control::MotorDynConfig> *s = new dynamic_reconfigure::Server<osa_control::MotorDynConfig>(node_handle);
		dynamic_reconfigure::Server<osa_control::MotorDynConfig>::CallbackType f;
		f = boost::bind(&CommandFilter::motorDynConfigCallback, this, _1, _2, i);
		s->setCallback(f);
		motor_dyn_config_server_list_.push_back(s);
	}

	//Subsriber, need the number of EPOS for the FIFO
	sub_motor_cmd_to_filter_ = nh.subscribe(ptr_robot_description_->getRobotNamespace() + "/motor_cmd_to_filter", 1, &CommandFilter::motorCmdToFilterCallback, this);

	//Publishers
	pub_motor_cmd_to_build_ = nh.advertise<osa_msgs::MotorCmdMultiArray>(ptr_robot_description_->getRobotNamespace() + "/motor_cmd_to_build", 1);

	//create the cmd multi array
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = ptr_robot_description_->getRobotDof(); //NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = ptr_robot_description_->getRobotDof(); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "epos";

	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(ptr_robot_description_->getRobotDof());

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
	ros::Rate r(ptr_robot_description_->getRobotHeartbeat());

	while(ros::ok())
	{
		resetMotorCmdArray();

		//Check motor cmds and dynamic reconfiguration
		ros::spinOnce();

		r.sleep();
	}
}

/*! \fn void setMotorCommandsCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
 *  \brief
 *  \return void
 */
void CommandFilter::motorCmdToFilterCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& cmds)
{
	motor_cmd_array_ = *cmds;

	//filter the cmds
	for(int i=0; i<motor_cmd_array_.layout.dim[0].stride; i++)
	{
		int node_id = motor_cmd_array_.motor_cmd[i].node_id;

		//find the index in the array which correspond to the node-id
		auto it = find_if(ptr_robot_description_->getControllerList().begin(), ptr_robot_description_->getControllerList().end(),
				[&node_id](const osa_common::Controller* obj)-> bool {return obj->getNodeID() == node_id;});

		if(it != ptr_robot_description_->getControllerList().end())
		{
			auto index = std::distance(ptr_robot_description_->getControllerList().begin(), it);

			//Now we have the index to get the right parameters in motor_param_list_
			if(motor_param_list_[index].enable)
			{
				if(motor_cmd_array_.motor_cmd[i].command == SET_TARGET_POSITION)
				{
//TODO
/*
					//clip the value
					if(motor_cmd_array_.motor_cmd[i].value > motor_param_list_[index].max_pos)
						motor_cmd_array_.motor_cmd[i].value = motor_param_list_[index].max_pos;
					if(motor_cmd_array_.motor_cmd[i].value < motor_param_list_[index].min_pos)
						motor_cmd_array_.motor_cmd[i].value = motor_param_list_[index].min_pos;

					//add the offset //TODO decide whether this apply before or after clipping
					motor_cmd_array_.motor_cmd[i].value += motor_param_list_[index].offset_pos;
*/
				}
			}
			else
			{
				ROS_DEBUG_STREAM("Motor " << ptr_robot_description_->getControllerList()[index]->getName() << " id disabled.");

				//send dumb message instead, which will result in sending nothing on the bus
				motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;

				//TODO in this case send a controlword to really disable the EPOS controller.
			}
		}
	}

	//publish the final motor command package after filtering
	pub_motor_cmd_to_build_.publish(motor_cmd_array_);
}

/*! \fn void resetMotorCmdArray()
 *  \brief
 *  \return void
 */
void CommandFilter::resetMotorCmdArray()
{
	for(int i=0; i<ptr_robot_description_->getRobotDof(); i++)
	{
		motor_cmd_array_.motor_cmd[i].node_id = 0;
		motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array_.motor_cmd[i].value = 0;
	}
}

void CommandFilter::motorDynConfigCallback(osa_control::MotorDynConfig &config, uint32_t level, const int idx)//const std::string dof_name)
{
	ROS_DEBUG("Dynamic Reconfigure Request for dof%d [%s]: %s %d %d %d", idx+1, ptr_robot_description_->getControllerList().at(idx)->getName().c_str(),
			config.enable?"True":"False", config.min_pos, config.max_pos, config.offset_pos);

	motor_param_list_[idx] = config;
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
	ros::init(argc, argv, "osa_command_filter_node");
	osa_control::CommandFilter *command_filter = new osa_control::CommandFilter();
	if(command_filter->init()) return -1;

	//When run() finishes
	delete command_filter;

	return 0;
}
