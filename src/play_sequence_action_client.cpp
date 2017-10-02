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
 * @file play_sequence_action_client.cpp
 * @author Cyril Jourdan
 * @date Sep 30, 2017
 * @version 0.1.0
 * @brief Implementation file for the class PlaySequenceActionClient
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Sep 30, 2017
 */

/*** Includes ***/

//OSA
//#include <enums.h>
#include "osa_control/play_sequence_action_client.h"

using namespace osa_control;

PlaySequenceActionClient::PlaySequenceActionClient(std::string name) :
		play_sequence_ac_(name, true)
{
	ROS_INFO("Waiting for action server to start.");
	play_sequence_ac_.waitForServer();
	ROS_INFO("Action server started, ready to send goals.");
}

PlaySequenceActionClient::~PlaySequenceActionClient()
{

}

void PlaySequenceActionClient::sendPlaySequenceGoal(PlaySequenceGoal goal)
{
	// Need boost::bind to pass in the 'this' pointer
	play_sequence_ac_.sendGoal(goal,
				boost::bind(&PlaySequenceActionClient::doneCb, this, _1, _2),
				boost::bind(&PlaySequenceActionClient::activeCb, this),
				boost::bind(&PlaySequenceActionClient::feedbackCb, this, _1));
}

void PlaySequenceActionClient::doneCb(const actionlib::SimpleClientGoalState& state, const PlaySequenceResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: total postures played: %d", result->total_postures_played);

	// Stop the client node
	//ros::shutdown();
}

// Called once when the goal becomes active
void PlaySequenceActionClient::activeCb()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal from the server
void PlaySequenceActionClient::feedbackCb(const PlaySequenceFeedbackConstPtr& feedback)
{
	ROS_INFO("Feedback: %f %% complete", feedback->percent_complete);
}
