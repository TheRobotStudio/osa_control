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

/*** Includes ***/
#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <stdio.h>

#define NUMBER_OF_DIN			8 //digital in
#define NUMBER_OF_AIN			5 //analog in
#define NUMBER_OF_MOTORS		16

/*** Variables ***/
osa_msgs::MotorDataMultiArray motorDataArray;
std_msgs::UInt16MultiArray rigInputsArray;

//to wait for the msg from both posture and joint_states_kinect topics
bool motorDataArray_arrived = false;
bool rigInputsArray_arrived = false;

/*** Callback functions ***/
void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motorDataArray = *data;
	motorDataArray_arrived = true;
}

void rigInputsArray_cb(const std_msgs::UInt16MultiArrayConstPtr& data)
{
	rigInputsArray = *data;
	rigInputsArray_arrived = true;
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	bool run = true;
	ros::init(argc, argv, "osa_bag_exporter_node");
	ros::NodeHandle nh;

	//open a file
	FILE * ptr_file;
	//ptr_file = fopen("/database/database.txt", "w");
	const char* packagePath = ros::package::getPath("trs_battlebots").c_str();
	const char* relativeFilePath = "/database/database.txt";

	char * filePath = (char*)malloc(1 + strlen(packagePath) + strlen(relativeFilePath));
	strcpy(filePath, packagePath);
	strcat(filePath, relativeFilePath);

	const char* constFilePath = filePath;

	ptr_file = fopen(filePath, "w");

	//Subscribers
	ros::Subscriber sub_motorDataArray = nh.subscribe ("/motor_data_array", 10, motorDataArray_cb);
	ros::Subscriber sub_objectCoord = nh.subscribe ("/rig_inputs_array", 10, rigInputsArray_cb);

	char inputKey = '0';
	int loopNb = 0;

	while(run)
	{
		//wait for user keyboard input
		printf("Enter 'r' to record a posture, or 'q' to exit :\n");
		inputKey = getchar();

		if(inputKey == 'r')
		{
			while(!motorDataArray_arrived || !rigInputsArray_arrived)
			{
				ros::spinOnce(); //listen to topics
			}
			//write data to the file

			if (ptr_file!=NULL)
			{

				/*
				//create char* variables
				char *rigInputs[NUMBER_OF_AIN];
				char *motorPos[NUMBER_OF_MOTORS];
*/
				//load the char* with values

				//calculate the length of the final string
				//int lineSize = 1; //1 for the final character
				for(int i=NUMBER_OF_DIN; i<NUMBER_OF_DIN+NUMBER_OF_AIN ; i++) //skip the first 8 data that are digital in
				{
					//lineSize += strlen(rigInputs[i]);
					fputs(std::to_string(rigInputsArray.data[i]).c_str(), ptr_file);
					fputs("_",ptr_file);
				}

				for(int i=0; i<NUMBER_OF_MOTORS ; i++)
				{
					//lineSize += strlen(motorPos[i]);
					fputs(std::to_string(motorDataArray.motorData[i].encPosition).c_str(), ptr_file);
					if(i != NUMBER_OF_MOTORS-1) fputs("_", ptr_file);
					else fputs("\n",ptr_file);
				}

				//char *databaseLine = (char*)malloc(lineSize);

				//rigInputsArray;
				//fputs(databaseLine, ptr_file);

			}
			//bag.write("/motor_data_array", time, motorDataSet);
			//bag.write("/objectCoord", time, objectCoord);

			//but this back to false for the next record
			motorDataArray_arrived = false;
			rigInputsArray_arrived = false;

			printf("Posture number %d recorded OK !\n", loopNb);
		}
		else if(inputKey == 'q')
		{
			fclose (ptr_file); //close the file
			run = false; //stop the while loop, quit the program
			printf("File closed ok\n");
		}
		getchar(); //to grab carriage return

		loopNb++;
	}

	return 0;
}
