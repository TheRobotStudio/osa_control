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
 * @file arm_manual.cpp
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 0.1.0
 * @brief Implementation file for the arm manual
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Oct 3, 2016
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//ROS messages
#include <razor_imu_9dof/RazorImu.h>
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "osa_control/switchNode.h"
#include "osa_control/getSlaveCmdArray.h"
//Others
#include <boost/foreach.hpp>
#include <flann/flann.hpp> //used for the kdtree search
#include <stdio.h>
//ROS packages include 
#include "robot_defines.h"

/*** Defines ***/
#define LOOP_RATE			HEART_BEAT
#define SPACE_DIM  			3  //roll pitch yaw
#define REF_DATA_DIM 			1  //only one reference point in kd space to find its nearest neighboor.
#define NUM_NN 				5  //number of nearest neighboors
#define DOF_DIM 			NUMBER_MOTORS_ARM
#define BICEPS_INC			32 //divide the kdtree

//MIN MAX
#define MIN_POS_BICEPS			6000
#define MIN_POS_SUPRA			0
#define MIN_POS_SUBSCAP			0
#define MIN_POS_INFRA			0
#define MIN_POS_TMIN			0
#define MIN_POS_LATDELT			0
#define MIN_POS_ANTDELT			0
#define MIN_POS_POSTDELT		0
#define MIN_POS_TRICEPS			37000
#define MIN_POS_BRACHI			18000
#define MIN_POS_HAND			10000
#define MIN_POS_ROTATOR			0 //-12000
#define MIN_POS_WRIST_IN		66000 //66000
#define MIN_POS_WRIST_UP		50000 //-146000
#define MIN_POS_WRIST_DOWN		40000 //-141000
#define MIN_POS_THUMB			30000 //70000

#define MAX_POS_BICEPS			78700
#define MAX_POS_SUPRA			0
#define MAX_POS_SUBSCAP			0
#define MAX_POS_INFRA			0
#define MAX_POS_TMIN			0
#define MAX_POS_LATDELT			0
#define MAX_POS_ANTDELT			0
#define MAX_POS_POSTDELT		0
#define MAX_POS_TRICEPS			73000
#define MAX_POS_BRACHI			40000
#define MAX_POS_HAND			60000
#define MAX_POS_ROTATOR			24000 //12000
#define MAX_POS_WRIST_IN		145000 //145000
#define MAX_POS_WRIST_UP		146000 //146000 //-50000
#define MAX_POS_WRIST_DOWN		190000 //141000 //-40000
#define MAX_POS_THUMB			125000 //180000

using namespace flann;

/*** Variables ***/
bool switch_node = false; //disable by default
osa_msgs::MotorCmdMultiArray motor_cmd_array;
//razor_imu_9dof::RazorImu razorImu;
//razor_imu_9dof::RazorImu playerImuRef;
sensor_msgs::Joy xbox_joy;

int selected_kd_tree = 0;
int dataset_dim = 0; //this will depend on the size of the bag, number of lines of data

//right arm
//Matrix<float> dataset_R_angles; //matrix for the angles
Matrix<int> dataset_r_positions; //matrix for the positions
Matrix<float> query_r(new float[SPACE_DIM*REF_DATA_DIM], REF_DATA_DIM, SPACE_DIM); //matrix for the query, just one line
Matrix<int> indices_r(new int[query_r.rows*SPACE_DIM], query_r.rows, NUM_NN);
Matrix<float> dists_r(new float[query_r.rows*SPACE_DIM], query_r.rows, NUM_NN);

float hand_val = 0;
float rotator_val = 0;
float elbow_val = 0;

//bool imu_arrived = false;
bool joy_arrived = false;
bool vel_test = false;

//bool enableImu = false;

//debug function to display a matrix
void displayMatrixFloat(const Matrix<float> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}

void displayMatrixInt(const Matrix<int> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}


int rotVal = 0;


/*** Callback functions ***/
/*
void imuRaw_cb(const razor_imu_9dof::RazorImuConstPtr& imu)
{
	razorImu = *imu;
	imu_arrived = true;
}
*/

void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(osa_control::switchNode::Request  &req, osa_control::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmdArray(osa_control::getSlaveCmdArray::Request  &req, osa_control::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motor_cmd_multi_array = motor_cmd_array;
		return true;
	}
	else
	{
		return false;
	}
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_arm_manual_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	//ros::Subscriber sub_imuRaw = nh.subscribe ("/imuRaw", 10, imuRaw_cb);
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joyCallback);

	//Services
	ros::ServiceServer srv_switch_node = nh.advertiseService("switch_right_arm_manual_srv", switchNode);
	ros::ServiceServer srv_get_motor_cmd_array = nh.advertiseService("get_right_arm_manual_cmd_srv", getMotorCmdArray);

	//various kdtrees drives the shoulder and biceps
	//Initialization of the kd tree
	//ROS_INFO("Initialization of the kd-tree :");

	//read a bag to generate the data matrix
	rosbag::Bag bag(ros::package::getPath("osa_control") + "/bag/arm/imuRawToShoulder_1.bag"); //change this bag
	//rosbag::View view_joint(bag, rosbag::TopicQuery("/imuRaw")); //angle info
	rosbag::View view_posture(bag, rosbag::TopicQuery("/motor_data_array")); //motor data position info

	int line = 0;
	//create the dataset_angle matrix
	dataset_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file
/*	Matrix<float> tempR1(new float[SPACE_DIM*dataset_dim], dataset_dim, SPACE_DIM);
	dataset_R_angles = tempR1;

	//ROS_INFO("dataset_dim = %d", dataset_dim);
	ROS_INFO("Create the angles matrix");

	BOOST_FOREACH(rosbag::MessageInstance const m, view_joint)
	{
		razor_imu_9dof::RazorImu::Ptr i = m.instantiate<razor_imu_9dof::RazorImu>();

		if(i != NULL)
		{
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+0] = i->roll;
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+1] = i->pitch;
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+2] = i->yaw;
		}
		else
			std::cout << "null" << std::endl;

	    	line++;
	}
*/
	ROS_INFO("Create posture matrix");
	//create the dataset_positions matrix
	int dataset_post_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file
	Matrix<int> tempR2(new int[DOF_DIM*dataset_post_dim], dataset_post_dim, DOF_DIM);
	dataset_r_positions = tempR2;

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_posture) //error compiles ok
	{
		osa_msgs::MotorDataMultiArray::Ptr i = m.instantiate<osa_msgs::MotorDataMultiArray>();

		if(i != NULL)
		{
			//Build the dataset_positions matrix
			for(int j=0; j<DOF_DIM; j++)
			{
				dataset_r_positions.ptr()[dataset_r_positions.cols*line+j] = i->motor_data[j].position;
			}
		}
		else
			std::cout << "null" << std::endl;

		line++;
	}

	bag.close();

	displayMatrixInt(dataset_r_positions, "postures");

	//create the commands multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].label = "motors";
	motor_cmd_array.layout.data_offset = 0;
	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);
	
	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		//motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		motor_cmd_array.motor_cmd[i].node_id = i+1;
		motor_cmd_array.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array.motor_cmd[i].value = 0;
	}

	//speed of sequences
	int speed_a = 1;
	int speed_b = 1;
	int speed_x = 2;
	int speed_y = 1;

	float reset_yaw = 0;

	//line to play in the database
	int data_r_idx = 42; //start somewhere in the database, line number 42

	bool cross_top = false;
	bool cross_bottom = false;

	while(ros::ok())
	{	
		ros::spinOnce();

		if(switch_node)
		{

			if(joy_arrived)
			{
					
				//if((xbox_joy.buttons[4]==1) && (xbox_joy.buttons[5]==0)) //btn LB to enable manual control
				//{
/*
					if(!enableImu) //each time the player reactivate the manual control, it's YAW position is subsctracted
					{
						reset_yaw = razorImu.yaw;
						ROS_INFO("reset_yaw = %f", reset_yaw);
					}

					enableImu = true; //then set to true so the previous if is no longer appied when the player keep pressing
*/
					//ROS_INFO("move shoulder");
					//move shoulder
/*
					if(query_r.cols == SPACE_DIM)
					{
						query_r.ptr()[0] = razorImu.roll;
						query_r.ptr()[1] = razorImu.pitch;
						query_r.ptr()[2] = razorImu.yaw-reset_yaw;
					}

					//ROS_INFO("%f %f %f", razorImu.roll, razorImu.pitch, razorImu.yaw-reset_yaw);

					//ROS_INFO("construct a randomized kd-tree index using 4 kd-trees");
					// construct a randomized kd-tree index using 4 kd-trees
					Index<L2<float> > data_R_index(dataset_R_angles, flann::KDTreeIndexParams(SPACE_DIM));
					data_R_index.buildIndex();

					//ROS_INFO("do a knn search, using 128 checks");
					// do a knn search, using 128 checks
					data_R_index.knnSearch(query_r, indices_r, dists_r, SPACE_DIM, flann::SearchParams(128));

					int data_r_idx = indices_r.ptr()[0];
*/
					//skip the IMU here and use the cross buttons on the xBox controller					
					
					if((!cross_top) && (xbox_joy.buttons[13]==1))
					{
						data_r_idx -= 10;
						cross_top = true;
					}

					if(cross_top && (xbox_joy.buttons[13]==0))
					{
						cross_top = false;
					}

					if((!cross_bottom) && (xbox_joy.buttons[14]==1))
					{
						data_r_idx += 10;
						cross_bottom = true;
					}

					if(cross_bottom && (xbox_joy.buttons[14]==0))
					{
						cross_bottom = false;
					}

					//if(cross_top) data_r_idx--;
					//if(cross_bottom) data_r_idx++;
					
					//clip
					if(data_r_idx<0) data_r_idx=0;
					if(data_r_idx>=dataset_dim) data_r_idx=dataset_dim-1;

					//ROS_INFO("init of the packet");
					//init of the packet
					for(int i=1; i<DOF_DIM-2; i++) //no biceps, no brachi/triceps
					{
						//R
						//motor_cmd_array.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
						motor_cmd_array.motor_cmd[i].node_id = i+1;
						motor_cmd_array.motor_cmd[i].command = SET_TARGET_POSITION;
						motor_cmd_array.motor_cmd[i].value = dataset_r_positions.ptr()[dataset_r_positions.cols*data_r_idx+i];
					}

					//ROS_INFO("data_r_idx=%d", data_r_idx);

					//Move Hand
					float hand_val_f = xbox_joy.axes[5];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					hand_val_f *= -(MAX_POS_HAND-MIN_POS_HAND)/2; // 2 = 1 - (-1)
					hand_val_f += MAX_POS_HAND-(MAX_POS_HAND-MIN_POS_HAND)/2;
					int hand_val_i = (int)hand_val_f;

					//ROS_INFO("hand_val_i = %d", hand_val_i);

					//clip
					if(hand_val_i>MAX_POS_HAND) hand_val_i = MAX_POS_HAND;
					if(hand_val_i<MIN_POS_HAND) hand_val_i = MIN_POS_HAND;

					motor_cmd_array.motor_cmd[0+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[0+10].value = hand_val_i;

					//Move Thumb
					float thumbVal_f = xbox_joy.axes[2];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					thumbVal_f *= -(MAX_POS_THUMB-MIN_POS_THUMB)/2; // 2 = 1 - (-1)
					thumbVal_f += MAX_POS_THUMB-(MAX_POS_THUMB-MIN_POS_THUMB)/2;
					int thumb_val_i = (int)thumbVal_f;

					//ROS_INFO("thumbVal_f = %d", thumb_val_i);

					//clip
					if(thumb_val_i>MAX_POS_THUMB) thumb_val_i = MAX_POS_THUMB;
					if(thumb_val_i<MIN_POS_THUMB) thumb_val_i = MIN_POS_THUMB;

					motor_cmd_array.motor_cmd[5+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[5+10].value = thumb_val_i;

					//Move Rotator
					float rotator_val_f = xbox_joy.axes[0];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					rotator_val_f *= (MAX_POS_ROTATOR-MIN_POS_ROTATOR)/2; // 2 = 1 - (-1)
					rotator_val_f += MAX_POS_ROTATOR-(MAX_POS_ROTATOR-MIN_POS_ROTATOR)/2;
					int rotator_val_i = (int)rotator_val_f;

					//ROS_INFO("rotator_val_i = %d", rotator_val_i);

					//clip
					if(rotator_val_i>MAX_POS_ROTATOR) rotator_val_i = MAX_POS_ROTATOR;
					if(rotator_val_i<MIN_POS_ROTATOR) rotator_val_i = MIN_POS_ROTATOR;

					motor_cmd_array.motor_cmd[1+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[1+10].value = rotator_val_i;

					//Move Brachi
					float brachi_val_f = xbox_joy.axes[1];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					brachi_val_f *= -(MAX_POS_BRACHI-MIN_POS_BRACHI)/2; // 2 = 1 - (-1)
					brachi_val_f += MAX_POS_BRACHI-(MAX_POS_BRACHI-MIN_POS_BRACHI)/2;
					int brachi_val_i = (int)brachi_val_f;

					//ROS_INFO("brachi_val_i = %d", brachi_val_i);

					//clip
					if(brachi_val_i>MAX_POS_BRACHI) brachi_val_i = MAX_POS_BRACHI;
					if(brachi_val_i<MIN_POS_BRACHI) brachi_val_i = MIN_POS_BRACHI;

					motor_cmd_array.motor_cmd[9].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[9].value = brachi_val_i;

					//Move triceps
					float tricepsVal_f = xbox_joy.axes[1];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					tricepsVal_f *= (MAX_POS_TRICEPS-MIN_POS_TRICEPS)/2; // 2 = 1 - (-1)
					tricepsVal_f += MAX_POS_TRICEPS-(MAX_POS_TRICEPS-MIN_POS_TRICEPS)/2;
					int triceps_val_i = (int)tricepsVal_f;

					//ROS_INFO("triceps_val_i = %d", triceps_val_i);

					//clip
					if(triceps_val_i>MAX_POS_TRICEPS) triceps_val_i = MAX_POS_TRICEPS;
					if(triceps_val_i<MIN_POS_TRICEPS) triceps_val_i = MIN_POS_TRICEPS;

					motor_cmd_array.motor_cmd[8].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[8].value = triceps_val_i;

					//Move Wrist left right
					float wrist_lr_val_f = xbox_joy.axes[3]; //left right
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wrist_lr_val_f *= -(MAX_POS_WRIST_UP-MIN_POS_WRIST_UP)/2; // 2 = 1 - (-1)
					wrist_lr_val_f += MAX_POS_WRIST_UP-(MAX_POS_WRIST_UP-MIN_POS_WRIST_UP)/2;
					int wrist_lr_val_i = (int)wrist_lr_val_f;

					//ROS_INFO("wrist_lr_val_i = %d", wrist_lr_val_i);

					//clip
					if(wrist_lr_val_i>MAX_POS_WRIST_UP) wrist_lr_val_i = MAX_POS_WRIST_UP;
					if(wrist_lr_val_i<MIN_POS_WRIST_UP) wrist_lr_val_i = MIN_POS_WRIST_UP;

					motor_cmd_array.motor_cmd[3+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[3+10].value = wrist_lr_val_i;

					//Move Wrist up down
					float wrist_ud_val_f = xbox_joy.axes[4]; //up down
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wrist_ud_val_f *= (MAX_POS_WRIST_IN-MIN_POS_WRIST_IN)/2; // 2 = 1 - (-1)
					wrist_ud_val_f += MAX_POS_WRIST_IN-(MAX_POS_WRIST_IN-MIN_POS_WRIST_IN)/2;
					int wrist_ud_val_i = (int)wrist_ud_val_f;

					//ROS_INFO("wrist_ud_val_i = %d", wrist_ud_val_i);

					//clip
					if(wrist_ud_val_i>MAX_POS_WRIST_IN) wrist_ud_val_i = MAX_POS_WRIST_IN;
					if(wrist_ud_val_i<MIN_POS_WRIST_IN) wrist_ud_val_i = MIN_POS_WRIST_IN;

					motor_cmd_array.motor_cmd[2+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[2+10].value = wrist_ud_val_i;

					//Move Wrist left right
					float wrist_ud_val2_f = xbox_joy.axes[4]; //up down
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wrist_ud_val2_f *= (MAX_POS_WRIST_DOWN-MIN_POS_WRIST_DOWN)/2; // 2 = 1 - (-1)
					wrist_ud_val2_f += MAX_POS_WRIST_DOWN-(MAX_POS_WRIST_DOWN-MIN_POS_WRIST_DOWN)/2;
					int wrist_ud_val2_i = (int)wrist_ud_val2_f;

					//ROS_INFO("wrist_ud_val_i = %d", wrist_ud_val_i);

					//clip
					if(wrist_ud_val2_i>MAX_POS_WRIST_DOWN) wrist_ud_val2_i = MAX_POS_WRIST_DOWN;
					if(wrist_ud_val2_i<MIN_POS_WRIST_DOWN) wrist_ud_val2_i = MIN_POS_WRIST_DOWN;

					motor_cmd_array.motor_cmd[4+10].command = SET_TARGET_POSITION;
					motor_cmd_array.motor_cmd[4+10].value = wrist_ud_val2_i;

					//by default apply current on biceps
					motor_cmd_array.motor_cmd[0].command = SET_CURRENT_MODE_SETTING_VALUE;
					motor_cmd_array.motor_cmd[0].value = 250;

/*
					if(xbox_joy.buttons[0]) //A
					{
						//ROS_INFO("butA_pressed");

						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnA_positions.rows/speed_a; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnA_positions.ptr()[btnA_positions.cols*i*speed_a+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnA_positions.ptr()[btnA_positions.cols*i*speed_a+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}
					}

					if(xbox_joy.buttons[1]) //B
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnB_positions.rows/speed_b; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnB_positions.ptr()[btnB_positions.cols*i*speed_b+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnB_positions.ptr()[btnB_positions.cols*i*speed_b+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butB_pressed = false;
					}

					if(xbox_joy.buttons[2]) //X
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnX_positions.rows/speed_x; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnX_positions.ptr()[btnX_positions.cols*i*speed_x+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnX_positions.ptr()[btnX_positions.cols*i*speed_x+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butX_pressed = false;
					}

					if(xbox_joy.buttons[3]) //Y
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnY_positions.rows/speed_y; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnY_positions.ptr()[btnY_positions.cols*i*speed_y+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnY_positions.ptr()[btnY_positions.cols*i*speed_y+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butY_pressed = false;
					}
*/

				//}
				//else
				//{
				//	enableImu = false;
				//}
/*
				if((xbox_joy.buttons[4]==0) && (xbox_joy.buttons[5]==1)) //btn RB to enable homing and current
				{
					if(xbox_joy.buttons[6]) //back button : zero homing mode
					{
						for(int i=0; i<NUMBER_MOTORS_ARM; i++)
						{
							rightArmCmd_ma.motor_cmd[i].node_id = i+1;
							rightArmCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
							rightArmCmd_ma.motor_cmd[i].value = 0;
						}

						for(int i=0; i<NUMBER_MOTORS_HAND; i++)
						{
							rightHandCmd_ma.motor_cmd[i].node_id = i+1;
							rightHandCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
							rightHandCmd_ma.motor_cmd[i].value = 0;
						}					
					}

					if(xbox_joy.buttons[7]) //start button : current mode
					{
						int curr = 120; //150;

						for(int i=0; i<NUMBER_MOTORS_ARM; i++)
						{
							rightArmCmd_ma.motor_cmd[i].node_id = i+1;
							rightArmCmd_ma.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							rightArmCmd_ma.motor_cmd[i].value = curr;
						}

						rightArmCmd_ma.motor_cmd[0].value = 180; // 250;
						rightArmCmd_ma.motor_cmd[1].value = 180; // 250;
						rightArmCmd_ma.motor_cmd[9].value = 180; // 250;

						for(int i=0; i<NUMBER_MOTORS_HAND; i++)
						{
							rightHandCmd_ma.motor_cmd[i].node_id = i+1;
							rightHandCmd_ma.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							rightHandCmd_ma.motor_cmd[i].value = curr;
						}

						//for the 2 inverted wrist motors
						//rightHandCmd_ma.motor_cmd[1].command = SET_TARGET_POSITION;
						rightHandCmd_ma.motor_cmd[1].value = 0;

						rightHandCmd_ma.motor_cmd[2].value = 60; //80;
						rightHandCmd_ma.motor_cmd[3].value = 60; //80;
						rightHandCmd_ma.motor_cmd[4].value = 60; //80;
						rightHandCmd_ma.motor_cmd[5].value = 50;
					}
				}*/	
			}
			else
			{
				//ROS_INFO("no joy");
			}

			//pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
			//pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);		

			//joy_arrived = false;
		}//if(switch_node)

		//r.sleep();
	}

	//free matrix pointers from memory
	//delete[] dataset_R_angles.ptr();
	delete[] dataset_r_positions.ptr();
/*
	delete[] btnA_positions.ptr();
	delete[] btnB_positions.ptr();
	delete[] btnX_positions.ptr();
	delete[] btnY_positions.ptr();
*/
	delete[] query_r.ptr();
	delete[] indices_r.ptr();
	delete[] dists_r.ptr();

	return 0;
}
