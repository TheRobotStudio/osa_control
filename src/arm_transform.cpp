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
 * @file arm_transform.cpp
 * @author Cyril Jourdan
 * @date Sep 26, 2017
 * @version 0.1.0
 * @brief Implementation file for the arm transform
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Jul 17, 2012
 */

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
//#include <boost/lexical_cast.hpp>
#include "std_msgs/Int16.h"
#include <std_msgs/Float32.h>

#include <math.h>

using namespace std;

double calculateAlpha(tf::StampedTransform tf_hand, tf::StampedTransform tf_elbow, tf::StampedTransform tf_shoulder)
{
	double rightHand[3] = {tf_hand.getOrigin().x(), tf_hand.getOrigin().y(), tf_hand.getOrigin().z()};
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};

	//Al-Kashi triangle formula
	double num =  pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)
				+ pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2)
				- pow(rightShoulder[0]-rightHand[0], 2) - pow(rightShoulder[1]-rightHand[1], 2) - pow(rightShoulder[2]-rightHand[2], 2);

	double den = 2 * sqrt(pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)) * sqrt(pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2));

	return acos(num/den);
}

double calculateBeta(tf::StampedTransform tf_elbow, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder)
{
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double neck[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};

	//Al-Kashi triangle formula
	double num =  pow(rightElbow[0]-rightShoulder[0], 2) + pow(rightElbow[1]-rightShoulder[1], 2) + pow(rightElbow[2]-rightShoulder[2], 2)
						+ pow(rightShoulder[0]-neck[0], 2) + pow(rightShoulder[1]-neck[1], 2) + pow(rightShoulder[2]-neck[2], 2)
						- pow(neck[0]-rightElbow[0], 2) - pow(neck[1]-rightElbow[1], 2) - pow(neck[2]-rightElbow[2], 2);

	double den = 2 * sqrt(pow(rightElbow[0]-rightShoulder[0], 2) + pow(rightElbow[1]-rightShoulder[1], 2) + pow(rightElbow[2]-rightShoulder[2], 2)) * sqrt(pow(rightShoulder[0]-neck[0], 2) + pow(rightShoulder[1]-neck[1], 2) + pow(rightShoulder[2]-neck[2], 2));

	return acos(num/den);
}

double calculateTheta(tf::StampedTransform tf_elbow, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder,  tf::StampedTransform tf_torso)
{
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double neck[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};
	double torso[3] = {tf_torso.getOrigin().x(), tf_torso.getOrigin().y(), tf_torso.getOrigin().z()};

	////Theta is the angle between two planes : TNE and SNE
	double a1 = (rightShoulder[1]-torso[1])*(neck[2]-torso[2]) - (rightShoulder[2]-torso[2])*(neck[1]-torso[1]);
	double b1 = (rightShoulder[2]-torso[2])*(neck[0]-torso[0]) - (rightShoulder[0]-torso[0])*(neck[2]-torso[2]);
	double c1 = (rightShoulder[0]-torso[0])*(neck[1]-torso[1]) - (rightShoulder[1]-torso[1])*(neck[0]-torso[0]);

	double a2 = (rightShoulder[1]-rightElbow[1])*(neck[2]-rightElbow[2]) - (rightShoulder[2]-rightElbow[2])*(neck[1]-rightElbow[1]);
	double b2 = (rightShoulder[2]-rightElbow[2])*(neck[0]-rightElbow[0]) - (rightShoulder[0]-rightElbow[0])*(neck[2]-rightElbow[2]);
	double c2 = (rightShoulder[0]-rightElbow[0])*(neck[1]-rightElbow[1]) - (rightShoulder[1]-rightElbow[1])*(neck[0]-rightElbow[0]);

	double num = a1*a2 + b1*b2 + c1*c2;

	double den = sqrt((a1*a1+b1*b1+c1*c1)*(a2*a2+b2*b2+c2*c2));

	return acos(num/den);
}

double calculatePhi(tf::StampedTransform tf_hand, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder,  tf::StampedTransform tf_elbow)
{
	double H[3] = {tf_hand.getOrigin().x(), tf_hand.getOrigin().y(), tf_hand.getOrigin().z()};
	double N[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double S[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};
	double E[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};

	//Phi is the angle between two planes : SNE and SHE
	float n1[3] = {(N[1]-S[1])*(E[2]-S[2])-(N[2]-S[2])*(E[1]-S[1]),
					(N[2]-S[2])*(E[0]-S[0])-(N[0]-S[0])*(E[2]-S[2]),
					(N[0]-S[0])*(E[1]-S[1])-(N[1]-S[1])*(E[0]-S[0])}; //normal plane vector : SNE

	float n2[3] = {(H[1]-S[1])*(E[2]-S[2])-(H[2]-S[2])*(E[1]-S[1]),
						(H[2]-S[2])*(E[0]-S[0])-(H[0]-S[0])*(E[2]-S[2]),
						(H[0]-S[0])*(E[1]-S[1])-(H[1]-S[1])*(E[0]-S[0])}; //normal plane vector : SHE

	float a1,b1,c1;
	a1 = n1[0];
	b1 = n1[1];
	c1 = n1[2];

	float a2,b2,c2;
	a2 = n2[0];
	b2 = n2[1];
	c2 = n2[2];

	double angle_phi = acos(fabs((a1*a2 + b1*b2 + c1*c2)/(sqrt((a1*a1+b1*b1+c1*c1)*(a2*a2+b2*b2+c2*c2)))));

	//std::cerr << "angle_phi" << angle_phi << std::endl;

	return angle_phi;

	//right
	/*
	double a1 = (rightShoulder[1]-rightHand[1])*(rightElbow[2]-rightHand[2]) - (rightShoulder[2]-rightHand[2])*(rightElbow[1]-rightHand[1]);
	double b1 = (rightShoulder[2]-rightHand[2])*(rightElbow[0]-rightHand[0]) - (rightShoulder[0]-rightHand[0])*(rightElbow[2]-rightHand[2]);
	double c1 = (rightShoulder[0]-rightHand[0])*(rightElbow[1]-rightHand[1]) - (rightShoulder[1]-rightHand[1])*(rightElbow[0]-rightHand[0]);

	double a2 = (rightShoulder[1]-rightHand[1])*(neck[2]-rightHand[2]) - (rightShoulder[2]-rightHand[2])*(neck[1]-rightHand[1]);
	double b2 = (rightShoulder[2]-rightHand[2])*(neck[0]-rightHand[0]) - (rightShoulder[0]-rightHand[0])*(neck[2]-rightHand[2]);
	double c2 = (rightShoulder[0]-rightHand[0])*(neck[1]-rightHand[1]) - (rightShoulder[1]-rightHand[1])*(neck[0]-rightHand[0]);

	double num = a1*a2 + b1*b2 + c1*c2;

	double den = sqrt((a1*a1+b1*b1+c1*c1)*(a2*a2+b2*b2+c2*c2));

	return acos(num/den);
	*/
}
/*
double calculateLambda(tf::StampedTransform tf_hand, tf::StampedTransform tf_elbow, tf::StampedTransform tf_shoulder)
{
	double leftHand[3] = {tf_hand.getOrigin().x(), tf_hand.getOrigin().y(), tf_hand.getOrigin().z()};
	double leftElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double leftShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};

	//Al-Kashi triangle formula
	double num =  pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)
				+ pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2)
				- pow(rightShoulder[0]-rightHand[0], 2) - pow(rightShoulder[1]-rightHand[1], 2) - pow(rightShoulder[2]-rightHand[2], 2);

	double den = 2 * sqrt(pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)) * sqrt(pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2));

	return acos(num/den);
}
*/
//sensor_msgs::JointState prev_joint_state;
/*
string playerNb_str = "1";
string frame_str = "/openni_depth_frame";
*/
string playerNb_str = "0";
string frame_str = "/camera_depth_optical_frame";
/*
void playerNumber_cb(const std_msgs::Int16ConstPtr& number)
{
	playerNb_str = boost::lexical_cast<string>(number->data);

	//ROS_INFO("Player Number is %s", playerNb_str);
	//cout << "number = " << playerNb_str << endl;
	ROS_INFO("number = %d", number->data);

	if(number->data == 0)
	{
		frame_str = "/camera_depth_optical_frame";
	}
	else
	{
		frame_str = "/openni_depth_frame";
	}
}
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_transform");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states_kinect", 100);
	ros::Publisher test_pub = n.advertise<std_msgs::Float32>("/test", 1);

	ros::Rate loop_rate(50);
	//ros::Rate loop_rate(10);

	tf::TransformListener listener_neck, listener_torso, listener_shoulder, listener_elbow, listener_hand;
	tf::StampedTransform transform_neck, transform_torso, transform_shoulder, transform_elbow, transform_hand;

	//left part
	tf::TransformListener listener_leftShoulder, listener_leftElbow, listener_leftHand;
	tf::StampedTransform transform_leftShoulder, transform_leftElbow, transform_leftHand;

	//read the player number : 0 used for the posture generated by IR markers, other numbers used by openni_tracker
	//ros::Subscriber sub_playerNumber = n.subscribe("/playerNumber", 1, &playerNumber_cb);

	bool error = false;

	//tf::TransformBroadcaster transform;
	//transform.sendTransform(tf::StampedTransform(null, ros::Time::now(), ));

	// message declarations
	sensor_msgs::JointState joint_state;
/*
	//init joint state
	//update joint_state
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.name[0] ="elbow_alpha_joint";
	joint_state.position[0] = M_PI/2;
	joint_state.name[1] ="shoulder_beta_joint";
	joint_state.position[1] = M_PI/2;
	joint_state.name[2] ="shoulder_theta_joint";
	joint_state.position[2] = M_PI/2;
	joint_state.name[3] ="shoulder_phi_joint";
	joint_state.position[3] = 3*M_PI/4;
	joint_state.name[4] ="elbow_twist_joint";
	joint_state.position[4] = M_PI/2;
	joint_state.name[5] ="wrist_joint";
	joint_state.position[5] = M_PI/2;

	prev_joint_state = joint_state;
*/
	//uint32_t begin, end, loopTime;

	while (ros::ok())
	{/*
		begin = ros::WallTime::now().nsec;
		error = false;
*/


		//grab tf skeleton joints
		try
		{
			listener_neck.lookupTransform(frame_str, "/neck_" + playerNb_str, ros::Time(0), transform_neck);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_torso.lookupTransform(frame_str, "/torso_" + playerNb_str, ros::Time(0), transform_torso);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_shoulder.lookupTransform(frame_str, "/left_shoulder_" + playerNb_str, ros::Time(0), transform_shoulder);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_elbow.lookupTransform(frame_str, "/left_elbow_" + playerNb_str, ros::Time(0), transform_elbow);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_hand.lookupTransform(frame_str, "/left_hand_" + playerNb_str, ros::Time(0), transform_hand);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}
/*
		//get the left arm transform
		try
		{
			listener_leftShoulder.lookupTransform(frame_str, "/right_shoulder_" + playerNb_str, ros::Time(0), transform_leftShoulder);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_leftElbow.lookupTransform(frame_str, "/right_elbow_" + playerNb_str, ros::Time(0), transform_leftElbow);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}

		try
		{
			listener_leftHand.lookupTransform(frame_str, "/right_hand_" + playerNb_str, ros::Time(0), transform_leftHand);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			error = true;
		}
*/
		//ROS_INFO("transform_han %f", transform_hand.getOrigin().x());

		//update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(4);//6
		joint_state.position.resize(4);//6
		joint_state.name[0] ="elbow_alpha_joint";
		joint_state.position[0] = M_PI - calculateAlpha(transform_hand, transform_elbow, transform_shoulder);
		joint_state.name[1] ="shoulder_beta_joint";
		joint_state.position[1] = M_PI - calculateBeta(transform_elbow, transform_neck, transform_shoulder);
		joint_state.name[2] ="shoulder_theta_joint";
		joint_state.position[2] = calculateTheta(transform_elbow, transform_neck, transform_shoulder, transform_torso) - M_PI/2;
		joint_state.name[3] ="shoulder_phi_joint";
		joint_state.position[3] = calculatePhi(transform_hand, transform_neck, transform_shoulder, transform_elbow);

		/*
		joint_state.name[4] ="elbow_twist_joint";
		joint_state.position[4] = 2*calculatePhi(transform_leftHand, transform_neck, transform_leftShoulder, transform_leftElbow) - 3*M_PI/4;
		joint_state.name[5] ="wrist_joint";
		joint_state.position[5] = M_PI - calculateAlpha(transform_leftHand, transform_leftElbow, transform_leftShoulder);
*/
		//test
		std_msgs::Float32 test;
		test.data = M_PI;//(float)transform_hand.getOrigin().x();

		test_pub.publish(test);

		//send the joint state and transform
		joint_pub.publish(joint_state);
/*
			prev_joint_state = joint_state;
		}
		else
		{
			//std::cerr << "Error" << std::endl;
			joint_pub.publish(prev_joint_state);
		}
*/

		ros::spinOnce(); //reasign a new player if need

		// This will adjust as needed per iteration
		loop_rate.sleep();

		//if(error) ROS_INFO("Error !!!");
/*
		end = ros::WallTime::now().nsec;
		loopTime = end - begin;
		ROS_INFO("loopTime =  %d", loopTime);
	*/
	}

	return 0;
}

