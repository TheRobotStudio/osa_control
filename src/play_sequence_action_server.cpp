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
 * @date Sep 29, 2017
 * @version 0.1.0
 * @brief Implementation file for the class PlaySequenceActionServer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Sep 29, 2017
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
//ROS actionlib
#include <actionlib/server/simple_action_server.h>
#include <osa_control/PlaySequenceAction.h>
//ROS bag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
//#include <boost/bind.hpp>
//ROS messages
#include <std_msgs/String.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
//OSA
#include <enums.h>
//Flann
#include <flann/flann.hpp> //used for the kdtree search
//others
#include <stdio.h>
#include <sstream>
#include <string>

using namespace flann;
using namespace osa_control;
typedef actionlib::SimpleActionServer<PlaySequenceAction> ActionServer;

class PlaySequenceActionServer
{
public:

	PlaySequenceActionServer(std::string name) :
		play_sequence_as_(nh_, name, false),
		action_name_(name)
	{
		// Register the goal and feeback callbacks
		play_sequence_as_.registerGoalCallback(boost::bind(&PlaySequenceActionServer::goalCallback, this));
		play_sequence_as_.registerPreemptCallback(boost::bind(&PlaySequenceActionServer::preemptCallback, this));

		// Subscribe to the data topic of interest
		//sub_ = nh_.subscribe("/random_number", 1, &PlaySequenceAction::analysisCallback, this);

		//Publishers
		pub_motor_cmd_array_ = nh_.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

		// Get the number of dof defined
		try
		{
			//load robot parameters
			if(!nh_.param("/robot/dof", number_epos_boards_, int(2)))
			{
				ROS_WARN("No /robot/dof found in YAML config file");
			}
		}
		catch(ros::InvalidNameException const &e)
		{
			ROS_ERROR(e.what());
		}

		//create the commands multi array
		motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		motor_cmd_array_.layout.dim[0].size = number_epos_boards_;
		motor_cmd_array_.layout.dim[0].stride = number_epos_boards_;
		motor_cmd_array_.layout.dim[0].label = "motors";
		motor_cmd_array_.layout.data_offset = 0;
		motor_cmd_array_.motor_cmd.clear();
		motor_cmd_array_.motor_cmd.resize(number_epos_boards_);

		initCmdSet();

		// Start the action server
		ROS_DEBUG("Start the action server");
		play_sequence_as_.start();
	}

	~PlaySequenceActionServer(void)
	{
	}

	void goalCallback()
	{
		// Get the goal parameters
	/*	//std_msgs::String sequence_bag_path = goal.get()->goal.sequence_bag_path.;
		//int32_t loop_rate = goal.get()->goal.loop_rate;

		// Parameters
		std::string sequence_bag_path_name = ""; //sequence_bag_path.data;
	*/
		// accept the new goal
		goal_ = *play_sequence_as_.acceptNewGoal();

		//char* sequence_bag_path = goal_.sequence_bag_path.data();
		int32_t loop_rate = goal_.loop_rate;

		// Parameters
		std::string package_name(goal_.package_name.data());
		std::string sequence_bag_path(goal_.sequence_bag_path.data());

		// try to open the bag file
		try
		{
			sequence_bag_.open(ros::package::getPath(package_name) + sequence_bag_path, rosbag::bagmode::Read);
		}
		catch(rosbag::BagException const &e)
		{
			ROS_ERROR(e.what());

			// Abort goal
			ROS_ERROR("Abort goal!");
			play_sequence_as_.setAborted();
		}

		rosbag::View view(sequence_bag_, rosbag::TopicQuery("/motor_data_array"));

		//create the matrix
		int dataset_post_dim = view.size(); //set the dataset dim equal to the number of lines in the bag file

		Matrix<int> temp_node_id_mat(new int[number_epos_boards_*dataset_post_dim], dataset_post_dim, number_epos_boards_);
		node_id_mat_ = temp_node_id_mat;

		Matrix<int> temp_position_mat(new int[number_epos_boards_*dataset_post_dim], dataset_post_dim, number_epos_boards_);
		positions_mat_ = temp_position_mat;

		int line = 0;
		BOOST_FOREACH(rosbag::MessageInstance const m, view) //error compiles ok
		{
			osa_msgs::MotorDataMultiArray::Ptr i = m.instantiate<osa_msgs::MotorDataMultiArray>();

			if(i != NULL)
			{
				//Build the dataset_positions matrix
				for(int j=0; j<number_epos_boards_; j++)
				{
					node_id_mat_.ptr()[node_id_mat_.cols*line+j] = i->motor_data[j].node_id;
					positions_mat_.ptr()[positions_mat_.cols*line+j] = i->motor_data[j].position;
				}
			}
			else
				std::cout << "null" << std::endl;

			line++;
		}

		sequence_bag_.close();

		//Display for debug
		displayMatrix(node_id_mat_, "node_id_mat");
		displayMatrix(positions_mat_, "positions_mat");

		//feedback_.percent_complete = 0;
		//publish the feedback
		//play_sequence_as_.publishFeedback(feedback_);

		playBag(goal_.loop_rate);

		// Free matrices pointers from memory
		delete[] temp_node_id_mat.ptr();
		delete[] temp_position_mat.ptr();
		//delete[] node_id_mat_.ptr();
		//delete[] positions_mat_.ptr();

		result_.total_postures_played = positions_mat_.rows;

		play_sequence_as_.setSucceeded(result_);
	}

	void preemptCallback()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		play_sequence_as_.setPreempted();
	}
/*
	void analysisCallback(const std_msgs::Float32::ConstPtr& msg)
	{
		// make sure that the action hasn't been canceled
		if(!play_sequence_as_.isActive())
			return;

		data_count_++;
		feedback_.sample = data_count_;
		feedback_.data = msg->data;
		//compute the std_dev and mean of the data
		sum_ += msg->data;
		feedback_.mean = sum_ / data_count_;
		sum_sq_ += pow(msg->data, 2);
		feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
		play_sequence_as_.publishFeedback(feedback_);

		if(data_count_ > goal_)
		{
			result_.mean = feedback_.mean;
			result_.std_dev = feedback_.std_dev;

			if(result_.mean < 5.0)
			{
				ROS_INFO("%s: Aborted", action_name_.c_str());
				//set the action state to aborted
				play_sequence_as_.setAborted(result_);
			}
			else
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				// set the action state to succeeded
				play_sequence_as_.setSucceeded(result_);
			}
		}
	}
*/
	void initCmdSet()
	{
		for(int i=0; i<number_epos_boards_; i++)
		{
			motor_cmd_array_.motor_cmd[i].node_id = 0;
			motor_cmd_array_.motor_cmd[i].command = SET_TARGET_POSITION;
			motor_cmd_array_.motor_cmd[i].value = 0;
		}
	}

	void playBag(int loop_rate)
	{
		ros::Rate r(loop_rate);

		for(int i=0; i<positions_mat_.rows; i++)
		{
			// Check for action preempt or abort
			ros::spinOnce();

			for(int j=0; j<number_epos_boards_; j++)
			{
				motor_cmd_array_.motor_cmd[j].node_id = node_id_mat_.ptr()[node_id_mat_.cols*i+j];
				motor_cmd_array_.motor_cmd[j].command = SET_TARGET_POSITION;
				motor_cmd_array_.motor_cmd[j].value = positions_mat_.ptr()[positions_mat_.cols*i+j];

				ROS_DEBUG("motor_cmd_array[%d][%d][%d][%d]", j,
						motor_cmd_array_.motor_cmd[j].node_id,
						motor_cmd_array_.motor_cmd[j].command,
						motor_cmd_array_.motor_cmd[j].value);
			}

			//pub_motor_cmd_array_.publish(motor_cmd_array_);

			// publish the feedback
			feedback_.percent_complete = (i+1)*100/positions_mat_.rows;
			play_sequence_as_.publishFeedback(feedback_);

			if(!r.sleep()) ROS_WARN("sleep: desired rate %dhz not met!", loop_rate);
		}
	}
/*
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
	}*/

	template <typename T>
	void displayMatrix(const Matrix<T> matrix, char* name)
	{
		//std::cout << name << " matrix :" << std::endl;
		ROS_DEBUG_STREAM(name << " matrix:");

		int count = 0;
		for(int i=0; i<matrix.rows; i++)
		{
			std::ostringstream matrix_line;
			//std::cout << count << " | ";
			//ROS_DEBUG_STREAM(count << " | ");
			matrix_line << " | ";

			for(int j=0; j<matrix.cols; j++)
			{
				//std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
				matrix_line << matrix.ptr()[matrix.cols*i+j] << "\t";
			}

			ROS_DEBUG_STREAM(matrix_line.str());

			count += 1;
			//std::cerr << std::endl;
		}
	}

protected:
	ros::NodeHandle nh_;
	ActionServer play_sequence_as_;
	std::string action_name_;
	//int data_count_,
	osa_control::PlaySequenceGoal goal_;
	osa_control::PlaySequenceFeedback feedback_;
	osa_control::PlaySequenceResult result_;
	ros::Subscriber sub_;
	ros::Publisher pub_motor_cmd_array_;
	rosbag::Bag sequence_bag_;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
	Matrix<int> node_id_mat_; //matrix for the positions
	Matrix<int> positions_mat_; //matrix for the positions
	int number_epos_boards_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "osa_play_sequence_action_server_node");

	//TODO server name parameter to transmit to the client so they have the same name

	//PlaySequenceActionServer play_sequence_as(ros::this_node::getName());
	PlaySequenceActionServer play_sequence_as("play_sequence");
	ros::spin();

	return 0;
}
