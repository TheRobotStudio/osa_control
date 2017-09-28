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
 * @file hsa_balance.cpp
 * @author Cyril Jourdan
 * @author Rob Knight
 * @date Sept 18, 2017
 * @version 0.1.0
 * @brief Implementation file for the High Speed Android balance algorithm
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Sept 18, 2017
 */

#include <exception>
#include <stdexcept>
//ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <osa_control/hsa_balance_dyn_Config.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <razor_imu_9dof/RazorImu.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>

#include "robot_defines.h"

/*** Defines ***/
#define LOOP_RATE	15 //HEART_BEAT
#define NUMBER_OF_WHEELS	2
#define NUMBER_OF_MOTORS	10

using namespace std;

/*** Global variables ***/
osa_control::hsa_balance_dyn_Config pid_param;
sensor_msgs::Joy xbox_joy;
razor_imu_9dof::RazorImu razor_imu;
osa_msgs::MotorDataMultiArray motor_data_array;
osa_msgs::MotorCmdMultiArray motor_cmd_array;
bool joy_arrived = false;
bool imu_arrived = false;
bool motor_data_array_arrived = true;

/*** Callback functions ***/
void HSABalanceDynCallback(osa_control::hsa_balance_dyn_Config &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %f %f %f %f", config.p_double_param, config.i_double_param, config.d_double_param, config.pt_double_param);
	pid_param = config;
}

void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	joy_arrived = true;
}

void imuRawCallback(const razor_imu_9dof::RazorImuConstPtr& imu)
{
	razor_imu = *imu;
	imu_arrived = true;
}

void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array = *data;
	motor_data_array_arrived = true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_hsa_balance_node");
	ros::NodeHandle nh("~");

	ros::Rate r(LOOP_RATE);

	// Parameters
	string dof_wheel_name[NUMBER_OF_WHEELS];
	int joy_axis_left_right_idx, joy_axis_up_down_idx;

	ROS_INFO("OSA High Speed Android balance node.");

	ROS_INFO("Setup dynamic_reconfigure parameters.");

	dynamic_reconfigure::Server<osa_control::hsa_balance_dyn_Config> hsa_balance_dyn_server;
	dynamic_reconfigure::Server<osa_control::hsa_balance_dyn_Config>::CallbackType f;

	f = boost::bind(&HSABalanceDynCallback, _1, _2);
	hsa_balance_dyn_server.setCallback(f);

	ROS_INFO("Grab the parameters.");

	// Grab the parameters
	try
	{
		nh.param("dof_right_wheel", dof_wheel_name[0], string("/dof1"));
		nh.param("dof_left_wheel", dof_wheel_name[1], string("/dof2"));
		nh.param("joy_axis_left_right", joy_axis_left_right_idx, 3);
		nh.param("joy_axis_up_down", joy_axis_up_down_idx, 4);
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
	}

	string name[NUMBER_OF_WHEELS];
	string type[NUMBER_OF_WHEELS];
	int node_id[NUMBER_OF_WHEELS] = {0};
	string controller[NUMBER_OF_WHEELS];
	string motor[NUMBER_OF_WHEELS];
	bool inverted[NUMBER_OF_WHEELS];
	string mode[NUMBER_OF_WHEELS];
	int value[NUMBER_OF_WHEELS] = {0};

	//Publishers
	ros::Publisher pub_motor_cmd_array = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100);
	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joyCallback);
	ros::Subscriber sub_imu = nh.subscribe ("/imuRaw", 10, imuRawCallback);
	ros::Subscriber sub_motor_data_array = nh.subscribe("/motor_data_array", 10, motorDataArrayCallback);
	
	// Grab the parameters
	try
	{
		//start with controller 1
		//int dof_idx = 1;
		//string rad_str = "dof"; //common radical name

		for(int i=0; i<NUMBER_OF_WHEELS; i++)
		{
			//create the string "controller+index" to search for the controller parameter with that index number
			ostringstream dof_idx_path;
			dof_idx_path << dof_wheel_name[i]; //rad_str << dof_idx;

			string absolute_str = "absolute_str";

			ROS_INFO("string=%s", dof_idx_path.str().c_str());

			if(nh.searchParam(dof_idx_path.str(), absolute_str))
			{
				//grab the parameters of the current controller

				//name
				ostringstream name_path;
				name_path << absolute_str << "/name";
				if(!nh.getParam(name_path.str(), name[i]))
				{
					ROS_ERROR("Can't grab param name for %s", dof_idx_path.str().c_str());
					return false;
				}

				//type
				ostringstream type_path;
				type_path << absolute_str << "/type";
				if(!nh.getParam(type_path.str(), type[i]))
				{
					ROS_ERROR("Can't grab param type for %s", dof_idx_path.str().c_str());
					return false;
				}
/*
				//check that the type is "WHEEL"
				if(type[i] == string("WHEEL"))
				{
					throw runtime_error("Selected DOF is not of type WHEEL.");
				}
*/
				//node_id
				ostringstream node_id_path;
				node_id_path << absolute_str << "/node_id";
				if(!nh.getParam(node_id_path.str(), node_id[i]))
				{
					ROS_ERROR("Can't grab param node_id for %s", dof_idx_path.str().c_str());
					return false;
				}

				//controller
				ostringstream controller_path;
				controller_path << absolute_str << "/controller";
				if(!nh.getParam(controller_path.str(), controller[i]))
				{
					ROS_ERROR("Can't grab param controller for %s", dof_idx_path.str().c_str());
					return false;
				}

				//motor
				ostringstream motor_path;
				motor_path << absolute_str << "/motor";
				if(!nh.getParam(motor_path.str(), motor[i]))
				{
					ROS_ERROR("Can't grab param motor for %s", dof_idx_path.str().c_str());
					return false;
				}

				//inverted
				ostringstream inverted_path;
				inverted_path << absolute_str << "/inverted";
				if(!nh.getParam(inverted_path.str(), inverted[i]))
				{
					ROS_ERROR("Can't grab param inverted for %s", dof_idx_path.str().c_str());
					return false;
				}

				//mode
				ostringstream mode_path;
				mode_path << absolute_str << "/mode";
				if(!nh.getParam(mode_path.str(), mode[i]))
				{
					ROS_ERROR("Can't grab param mode for %s", dof_idx_path.str().c_str());
					return false;
				}

				//value
				ostringstream value_path;
				value_path << absolute_str << "/value";
				if(!nh.getParam(value_path.str(), value[i]))
				{
					ROS_ERROR("Can't grab param value for %s", dof_idx_path.str().c_str());
					return false;
				}

				//print the dof parameters
				ROS_INFO("%s : name[%s], type[%s], node_id[%d], controller[%s], motor[%s], inverted[%d], mode[%s], value[%d]", dof_idx_path.str().c_str(),
						name[i].c_str(), type[i].c_str(), node_id[i], controller[i].c_str(), motor[i].c_str(), inverted[i], mode[i].c_str(), value[i]);
			}
			else
			{
				//dof_exist = false;
				ROS_WARN("Controllers not found in YAML config file");
			}

			//dof_exist = false;
		}

		ROS_INFO("Wheels parameters found successfully!\n");
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		ROS_ERROR("Wrong parameters in config file or launch file!");
		ROS_ERROR("Please modify your YAML config file or launch file and try again.");

		return false;
	}

	//create the command array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = NUMBER_OF_MOTORS;
	motor_cmd_array.layout.dim[0].stride = NUMBER_OF_MOTORS;
	motor_cmd_array.layout.dim[0].label = "motors";
	motor_cmd_array.layout.data_offset = 0;
	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(NUMBER_OF_MOTORS);

	//Initialization
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		motor_cmd_array.motor_cmd[i].node_id = i+1;
		motor_cmd_array.motor_cmd[i].command = SET_TARGET_POSITION;
		motor_cmd_array.motor_cmd[i].value = 0;
	}

	motor_cmd_array.motor_cmd[8].command = SET_TARGET_VELOCITY;			//For the drive wheels
	motor_cmd_array.motor_cmd[9].command = SET_TARGET_VELOCITY;

	/* Main loop */
	ROS_INFO("Main loop");

	while(ros::ok())
	{
		// Get imu and motor data through callbacks.
		ros::spinOnce();

		if(joy_arrived)								//Will only be true if values have changed
		{
			//Joystick control
			//xbox_joy.buttons[0];
			//xbox_joy.axes[0];
		}

		// Check that both imu and motor data has arrived.
		if(imu_arrived && motor_data_array_arrived)
		{
			//--------------------- PID loop ---------------------
			// Variables
			float angle = razor_imu.pitch; //in rad
			float velocity_f = 0.0;
			int velocity_i = 0;
			float dt = 1/LOOP_RATE;

			//PID parameters are accessed with config.p_double_param, config.i_double_param, config.d_double_param.
			//Pitch Trim parameter is accessed with config.pt_double_param

			// Computation
			velocity_f = -(angle*4000)/M_PI;
			velocity_i = (int)velocity_f;

			// Print velocity value
			ROS_INFO("angle = %f, velocity = %d", angle, velocity_i);

			// Set final motor velocity
			motor_cmd_array.motor_cmd[8].value = velocity_i;
			motor_cmd_array.motor_cmd[9].value = velocity_i;

			// Publish the motor commands topic, caught by the command_builder node
			// which send it to the CAN bus via topic_to_socketcan_node
			pub_motor_cmd_array.publish(motor_cmd_array);
		}

		imu_arrived = false;
		motor_data_array_arrived = false;
		joy_arrived = false;

		if(!r.sleep()) ROS_WARN("sleep: desired rate %dhz not met!", LOOP_RATE);
	}//while ros ok

	return 0;
}
