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
 * @file play_sequence_action_server.cpp
 * @author Cyril Jourdan
 * @date Sep 30, 2017
 * @version 0.1.0
 * @brief Implementation file for the class JoystickActionClient
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Sep 30, 2017
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
//#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
//ROS services
#include "osa_control/switchNode.h"
#include "osa_control/getSlaveCmdArray.h"
//ROS actionlib
#include <actionlib/client/simple_action_client.h>
#include <osa_control/PlaySequenceAction.h>
//OSA
#include <enums.h>
#include "osa_control/play_sequence_action_client.h"

using namespace osa_control;

class JoystickActionClient
{
public:
	JoystickActionClient(std::string name) : play_sequence_ac_(name)
	{
		//TODO subscribe joy
	}
/*
	PlaySequenceActionClient getPlaySequenceAC()
	{
		return play_sequence_ac_;
	}
*/
private:
	ros::NodeHandle nh_;
public:
	PlaySequenceActionClient play_sequence_ac_;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "osa_joystick_action_client_node");

  JoystickActionClient joy_client(ros::this_node::getName());

  PlaySequenceGoal goal;
  goal.package_name = "bibot_apps";
  goal.sequence_bag_path = "/bag/bibot_1.bag";
  goal.loop_rate = 1;

 // joy_client.getPlaySequenceAC().sendGoal(goal);
  joy_client.play_sequence_ac_.sendPlaySequenceGoal(goal);

  ros::spin();

  return 0;
}
