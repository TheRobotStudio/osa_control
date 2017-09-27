/*
 * tabletCommand.cpp
 *
 *  Created on: Apr 4, 2013
 *      Author: Cyril Jourdan
 */

/*** Include ***/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <boost/algorithm/string.hpp>

/*** Variables ***/
ros::Publisher cmd_vision_pub;
ros::Publisher cmd_handshake_pub;
ros::Publisher cmd_kinect_pub;
ros::Publisher cmd_current_pub;

std_msgs::Int16MultiArray visionVal;
std_msgs::Bool handshakeVal;
std_msgs::Bool kinectVal;
std_msgs::Bool currentVal;

/*** Callback functions ***/
void tabletCmd_cb(const std_msgs::StringConstPtr& tablet_cmd)
{
	std::vector<std::string> strs;
	boost::split(strs, tablet_cmd->data, boost::is_any_of("_"));

	if(strs.size() == 10)
	{
		visionVal.data[0] = std::atoi(strs[0].c_str());
		visionVal.data[1] = std::atoi(strs[4].c_str()); //eyeball roll
		visionVal.data[2] = std::atoi(strs[5].c_str()); //eyeball iris
		visionVal.data[3] = std::atoi(strs[6].c_str()); //eyeball X coord
		visionVal.data[4] = std::atoi(strs[7].c_str()); //eyeball Y coord
		visionVal.data[5] = std::atoi(strs[8].c_str()); //head X coord
		visionVal.data[6] = std::atoi(strs[9].c_str()); //head Y coord

		if(std::atoi(strs[1].c_str()) == 0) handshakeVal.data = false;
		else handshakeVal.data = true;

		if(std::atoi(strs[2].c_str()) == 0) kinectVal.data = false;
		else kinectVal.data = true;

		if(std::atoi(strs[3].c_str()) == 0) currentVal.data = false;
		else currentVal.data = true;

		cmd_vision_pub.publish(visionVal);
		cmd_handshake_pub.publish(handshakeVal);
		cmd_kinect_pub.publish(kinectVal);
		cmd_current_pub.publish(currentVal);
	}
}

/*** Main ***/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trs_tabletCommand_node");
    ros::NodeHandle nh;
    ros::Rate r(15);

    //Subscribers
    ros::Subscriber sub_tabletCmd = nh.subscribe("/tablet_cmd", 1, tabletCmd_cb);

    //Publishers
    cmd_vision_pub = nh.advertise<std_msgs::Int16MultiArray>("/tabletCmd_vision", 1);
    cmd_handshake_pub = nh.advertise<std_msgs::Bool>("/tabletCmd_handshake", 1);
    cmd_kinect_pub = nh.advertise<std_msgs::Bool>("/tabletCmd_kinect", 1);
    cmd_current_pub = nh.advertise<std_msgs::Bool>("/tabletCmd_current", 1);

    //init
    visionVal.data.clear();
    visionVal.data.push_back(0); //enable face tracking
    visionVal.data.push_back(50); //eyeball roll
	visionVal.data.push_back(50); //eyeball iris
    visionVal.data.push_back(100); //eyeball X coord
    visionVal.data.push_back(100); //eyeball Y coord
    visionVal.data.push_back(100); //head X coord
    visionVal.data.push_back(100); //head Y coord

    handshakeVal.data = false;
    kinectVal.data = false;
    currentVal.data = false;

    while(ros::ok())
	{
    	ros::spinOnce();
    	r.sleep();
	}
}



