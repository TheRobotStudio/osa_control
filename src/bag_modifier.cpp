/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 * @file bag_modifier.cpp
 * @author Cyril Jourdan <contact@therobotstudio.com>
 * @date Modified on Apr 13, 2018
 * @date Created on Apr 12, 2018
 * @version 0.1.1
 * @brief Source file for the OSA Bag Modifier program
 */

#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <std_msgs/UInt32MultiArray.h>
#include <stdio.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace std;

// Parameters
string package_name;
string bag_path;

rosbag::Bag bag;
rosbag::View view;
std::vector<std::string> topics;

std_msgs::UInt32MultiArray input_data;
std_msgs::UInt32MultiArray output_data;

void print_menu()
{
	ROS_INFO("--- Menu ---");
	ROS_INFO("Commands start with ':' and end with <ENTER>");
	ROS_INFO(":h - print this help menu");
	ROS_INFO(":o - open a bag");
	//ROS_INFO(":l - list topic names");
	//ROS_INFO(":r <old topic name> <new topic name> - rename a topic. Example: :r /motor_position /output");
	ROS_INFO(":p - print the whole bag");
	ROS_INFO(":p <topic> - print all the messages of a specific topic, /input or /output. Example:	:p /input");
	ROS_INFO(":p <topic> <index> - print the message from <topic> at <index>. Example:	:p /input 42");
	ROS_INFO(":d <index> - delete messages at <index> in all topics. Example:	:d 42");
	ROS_INFO(":e <topic> <index> <column> - edit a specific data. Example:	:e /output 42 7");
	ROS_INFO(":a <topic> <column> <offset> - add an offset to a specific topic column. Example:	:a /output 7 1000");
	//ROS_INFO(":i - perform data interpolation");
	ROS_INFO(":m <package> <path> - merge current bag with the bag located in <package> at the specified <path>. Example:	:m osa_control bag/my_data.bag");
	ROS_INFO(":waq <file name> - write as a new bag with <filename> in same directory and quit the program");
	ROS_INFO(":wq - write the bag and quit the program");
	ROS_INFO(":q! - discard the changes and quit the program");
}

void check_bag()
{
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	ROS_INFO("View size = %d", view.size());

	if(view.size() != 2) throw runtime_error("Topic /input or /output is not present in the bag!");
}

void load_bag_in_memory()
{
	foreach(rosbag::MessageInstance const m, view)
	{
		std_msgs::UInt32MultiArray::ConstPtr i = m.instantiate<std_msgs::UInt32MultiArray>();
		if (i != NULL)
			ROS_INFO_STREAM(i->data); //TODO Check print of multi array

		std_msgs::UInt32MultiArray::ConstPtr o = m.instantiate<std_msgs::UInt32MultiArray>();
		if (o != NULL)
			ROS_INFO_STREAM(o->data); //TODO Check print of multi array
	}
}

void open_bag(string pkg_name, string path)
{
	if(!pkg_name.empty() && !path.empty())
	{
		try
		{
			bag.open(ros::package::getPath(pkg_name) + path, rosbag::bagmode::Write);
		}
		catch(rosbag::BagException const &e)
		{
			ROS_ERROR(e.what());
			throw e;
		}
	}
	else
	{
		ROS_ERROR("Invalid package or path name!");
		throw runtime_error("Invalid package or path name!");
	}

	//check that there are 2 topics only with data type UInt32MultiArray
	try
	{
		check_bag();
	}
	catch(runtime_error const &e)
	{
		ROS_ERROR(e.what());
		throw e;
	}

	load_bag_in_memory();

	ROS_INFO_STREAM("The bag [" << pkg_name << path << "] has been opened successfuly!");
}

/*
void list_topic_names()
{
	rosbag::View view(bag, rosbag::TopicQuery(topics));
}
*/

void print_bag()
{
	foreach(rosbag::MessageInstance const m, view)
	{
		std_msgs::UInt32MultiArray::ConstPtr i = m.instantiate<std_msgs::UInt32MultiArray>();
		if (i != NULL)
			ROS_INFO_STREAM(i->data); //TODO Check print of multi array

		std_msgs::UInt32MultiArray::ConstPtr o = m.instantiate<std_msgs::UInt32MultiArray>();
		if (o != NULL)
			ROS_INFO_STREAM(o->data); //TODO Check print of multi array
	}
}

void print_topic(string topic)
{
	rosbag::View view_topic(bag, rosbag::TopicQuery(topic));

	foreach(rosbag::MessageInstance const m, view_topic)
	{
		std_msgs::UInt32MultiArray::ConstPtr t = m.instantiate<std_msgs::UInt32MultiArray>();
		if (t != NULL)
			ROS_INFO_STREAM(t->data); //TODO Check print of multi array
	}
}

void print_topic_at_index(string topic, int index)
{
	rosbag::View view_topic(bag, rosbag::TopicQuery(topic));

	foreach(rosbag::MessageInstance const m, view_topic)
	{
		std_msgs::UInt32MultiArray::ConstPtr t = m.instantiate<std_msgs::UInt32MultiArray>();
		if (t != NULL)
		{
			//check index exist
			if(index<t->data.size())
			ROS_INFO_STREAM(t->data[index]);
		}
	}
}

void delete_topics_at_index(int index)
{

}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_bag_modifier_node");
	ros::NodeHandle nh;

	//Variables
	bool run = true;

	ROS_INFO("*** OSA Bag Modifier ***");

	//2 topics are expected in the bag
	topics.push_back(std::string("/input"));
	topics.push_back(std::string("/output"));

	// Grab the parameters
	try
	{
		nh.param("package_name", package_name, string(""));
		nh.param("bag_path", bag_path, string(""));
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		return 0;
	}

	try
	{
		open_bag(package_name, bag_path);
	}
	catch(rosbag::BagException const &e)
	{
		ROS_ERROR(e.what());
	}
	catch(runtime_error const &e)
	{
		ROS_ERROR(e.what());
	}

	print_menu();

	if(bag.getFileName().empty())
		ROS_WARN("No bag has been opened yet.");

	std::string delimiter = " "; //white space between a command and its argument(s)

	while(run)
	{
		std::string whole_cmd; //made of cmd and arg(s) separated with spaces
		vector<string> cmd_split;

		ROS_INFO("Enter a command below");
		std::getline(cin, whole_cmd);

		if(whole_cmd[0] == ":") //command
		{
			//Search for the ":" and delete it
			whole_cmd = whole_cmd.substr(whole_cmd.find(":") + 1);

			size_t pos = 0;
			std::string token;

			//search for space(s), that means a command with arguments, and split it
			while((pos = whole_cmd.find(delimiter)) != std::string::npos)
			{
			    token = whole_cmd.substr(0, pos);
			    ROS_INFO_STREAM(token);
			    whole_cmd.erase(0, pos + delimiter.length());

			    cmd_split.push_back(token);
			}

			if(cmd_split.size() == 0) cmd_split.push_back(whole_cmd); //if no args

			//cmd = whole_cmd.substr(0, whole_cmd.find(delimiter));

			if(cmd_split[0].compare("h") == 0) //help
			{
				print_menu();
			}
			else if(cmd_split[0].compare("o") == 0) //open bag
			{
				if(cmd_split.size() == 3)
				{
					try
					{
						open_bag(cmd_split[1], cmd_split[2]);
					}
					catch(rosbag::BagException const &e)
					{
						ROS_ERROR(e.what());
					}
					catch(runtime_error const &e)
					{
						ROS_ERROR(e.what());
					}
				}
			}
			else if((cmd_split.size() == 1) && (cmd_split[0].compare("p") == 0)) //print whole bag
			{
				print_bag();
			}
			else if((cmd_split.size() == 2) && (cmd_split[0].compare("p") == 0)) //print topic
			{
				if(cmd_split[1].compare("/input") || cmd_split[1].compare("/output"))
				print_topic(cmd_split[1]);
			}
			else if((cmd_split.size() == 3) && (cmd_split[0].compare("p") == 0)) //print topic at index
			{
				if(cmd_split[1].compare("/input") || cmd_split[1].compare("/output"))
				{
					int idx = atoi(cmd_split[2].c_str());

					print_topic_at_index(cmd_split[1], idx);
				}
			}
			//TODO finish others


			else if(cmd_split[0].compare("q!") == 0) //discard and quit
			{
				fclose (ptr_file); //close the file
				run = false; //stop the while loop, quit the program
				printf("File closed ok\n");
			}
		}
		else
		{
			ROS_WARN("Incorrect command.");
		}
	}

	return 0;
}
