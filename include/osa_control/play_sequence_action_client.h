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

#ifndef OSA_CONTROL_PLAY_SEQUENCE_ACTION_CLIENT_H
#define OSA_CONTROL_PLAY_SEQUENCE_ACTION_CLIENT_H

//ROS
#include <ros/ros.h>
//ROS actionlib
#include <actionlib/client/simple_action_client.h>
#include <osa_control/PlaySequenceAction.h>
#include <boost/bind.hpp>

namespace osa_control
{

typedef actionlib::SimpleActionClient<PlaySequenceAction> ActionClient;

/**
 * @brief This is the class for PlaySequenceActionClient.
 */
class PlaySequenceActionClient
{
public:
	/**
	 * @brief Constructor.
	 */
	PlaySequenceActionClient(std::string name);

	/**
	 * @brief Destructor.
	 */
	virtual ~PlaySequenceActionClient();

	void sendPlaySequenceGoal(PlaySequenceGoal goal);
	void doneCb(const actionlib::SimpleClientGoalState& state, const PlaySequenceResultConstPtr& result);
	void activeCb();
	void feedbackCb(const PlaySequenceFeedbackConstPtr& feedback);

protected:
	ActionClient play_sequence_ac_;
};

} // namespace osa_control

#endif // OSA_CONTROL_PLAY_SEQUENCE_ACTION_CLIENT_H
