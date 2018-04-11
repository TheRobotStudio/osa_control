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
 * @file robot_defines.h
 * @author Cyril Jourdan
 * @date Feb 24, 2017
 * @version OSA 0.1.0
 * @brief Header file for the defines related to the robot
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Jun 10, 2014
 */

#ifndef OSA_CONTROL_ROBOT_DEFINES_H
#define OSA_CONTROL_ROBOT_DEFINES_H

//#define OSA_DEBUG
//#define BUILD_FOR_NEXUS_7
//#define BUILD_FOR_SURFACE_PRO_2
//#define BUILD_FOR_MINT_BOX_2
//#define BUILD_FOR_DELL_LAPTOP

//Quantities
#define NUMBER_MAX_EPOS2_PER_SLAVE 		16
#define NUMBER_SLAVE_BOARDS         	6 //maximum 128
#define NUMBER_OF_MOTORS				NUMBER_MAX_EPOS2_PER_SLAVE*NUMBER_SLAVE_BOARDS //96

//New robot arm
#define NUMBER_MOTORS_NEW_ARM_DCX10		16
#define NUMBER_MOTORS_NEW_ARM_DCX16		9
#define NUMBER_MOTORS_NEW_ARM	    	NUMBER_MOTORS_NEW_ARM_DCX10 + NUMBER_MOTORS_NEW_ARM_DCX16


#define NUMBER_MOTORS_BASE	    2
#define NUMBER_MOTORS_ARM	    10
#define NUMBER_MOTORS_HAND		6
#define NUMBER_MOTORS_LEG		16
#define NUMBER_MOTORS_PELVIS	16
#define NUMBER_MOTORS_HEAD		8

#define SLAVE_DCX10_ID			1
#define SLAVE_DCX16_ID			2

#define SLAVE_LEFT_ARM_ID		2//1
#define SLAVE_RIGHT_ARM_ID		1//2
#define SLAVE_LEFT_LEG_ID		3
#define SLAVE_RIGHT_LEG_ID		4
#define SLAVE_PELVIS_ID			5
#define SLAVE_HEAD_ID			6

#define HEART_BEAT				100 //15 //20 //30 //20 //(20 for 6 slaves with 16 nodes each) //(25 for 5 slaves with 15 nodes each) //TODO dynamic reconfigure

//Commands
#define SET_TARGET_POSITION 				0
#define SET_TARGET_VELOCITY 				1
#define SET_PROFILE_ACCELERATION 			2
#define SET_PROFILE_DECELERATION 			3
#define SET_PROFILE_VELOCITY 				4
#define SET_OUTPUT_CURRENT_LIMIT 			5
#define SET_CONTROLWORD 					6
#define SET_CURRENT_MODE_SETTING_VALUE 		7
#define SET_MAXIMAL_SPEED_IN_CURRENT_MODE 	8
#define SET_MODES_OF_OPERATION 				9
#define SEND_DUMB_MESSAGE 					15

//CONTROL MODES
#define INTERPOLATED_POSITION_MODE 		0x00
#define PROFILE_VELOCITY_MODE 			0x01
#define PROFILE_POSITION_MODE 			0x02
#define POSITION_MODE 					0x03
#define VELOCITY_MODE					0x04
#define CURRENT_MODE 					0x05
#define CYCLIC_SYNCHRONOUS_TORQUE_MODE	0x05
#define NO_MODE 						0xFF

//HEAD DEFINES //eyeball calibrated centered
#define DCX_EYEBALL_PITCH_MIN		-8000 //0 //-22000
#define DCX_EYEBALL_PITCH_MAX		8000 //22000 //0 //calibrated
#define DCX_EYEBALL_YAW_MIN			-14000 //0 //-18000
#define DCX_EYEBALL_YAW_MAX			14000 //18000 //0 //calibrated
#define DCX_EYEBALL_IRIS_MIN		0 //calibrated
#define DCX_EYEBALL_IRIS_MAX		16000

#define EYEBALL_PITCH_CENTER		(DCX_EYEBALL_PITCH_MIN + (DCX_EYEBALL_PITCH_MAX - DCX_EYEBALL_PITCH_MIN)/2);
#define EYEBALL_YAW_CENTER			(DCX_EYEBALL_YAW_MIN + (DCX_EYEBALL_YAW_MAX - DCX_EYEBALL_YAW_MIN)/2);

#define DCX_NECK_FRONT_LEFT_MIN		0 //calibrated
#define DCX_NECK_FRONT_LEFT_MAX		0 //
#define DCX_NECK_FRONT_RIGHT_MIN	(-DCX_NECK_FRONT_LEFT_MIN) //0 //calibrated
#define DCX_NECK_FRONT_RIGHT_MAX	(-DCX_NECK_FRONT_LEFT_MAX)
#define DCX_NECK_BACK_LEFT_MIN		12000 //-65000
#define DCX_NECK_BACK_LEFT_MAX		80000 //-20000
#define DCX_NECK_BACK_RIGHT_MIN		12000 //(-DCX_NECK_BACK_LEFT_MAX)
#define DCX_NECK_BACK_RIGHT_MAX		80000 //(-DCX_NECK_BACK_LEFT_MIN)

#define HEAD_PITCH_DOWN_LIMIT		0 //
#define HEAD_PITCH_UP_LIMIT			0 //
#define HEAD_YAW_LEFT_LIMIT			0 //
#define HEAD_YAW_RIGHT_LIMIT		0 //

#define DCX_NECK_FRONT_LEFT_CURR	50
#define DCX_NECK_FRONT_RIGHT_CURR	50 //(-DCX_NECK_FRONT_LEFT_CURR)

#define HEAD_DETECTED_COUNTER		30	//number of frames with a face after which the iris moves to "focus" position
#define NO_HEAD_DETECTED_COUNTER	30	//number of frames without face after which the head and eyeball recenter and the iris moves to "aware" position

#define IRIS_AWARE_POSITION			5000
#define IRIS_FOCUS_POSITION			14500

#define MIDLE_POSITION				46000 //32000 //TODO
#define EYE_SPEED_FACTOR			8
#define NECK_SPEED_FACTOR			1000

#define BIBOT_ARM_SLAVEBOARD_ID		1
#define BIBOT_BASE_SLAVEBOARD_ID	2

/*
#define NO_MODE					-1 //0xFF
#define POSITION_MODE			0
#define CURRENT_MODE			1
#define VELOCITY_MODE			2
#define PROFILE_POSITION_MODE	3
#define FORCE_MODE				4
#define SERVO_POS_MODE			5
*/

//NODE IDs
//NEW ARM
#define RIGHT_ARM_LITTLE_WHOLE_FLEX_ID			1 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_RING_WHOLE_FLEX_ID			2 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_INDEX_WHOLE_FLEX_ID			3 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_FORE_WHOLE_FLEX_ID			4 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_WHOLE_FLEX_ID			5 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_IN_ID					6 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_UP_ID					7 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_DOWN_ID					8 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_META_EXTEND_ID			9 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_EXTEND_ID				10 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_FORE_EXTEND_ID				11 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_LOWER_3_EXTEND_ID				12 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_FORE_KNUCKLE_FLEX_1_ID		13 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_FORE_KNUCKLE_FLEX_2_ID		14 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_META_FLEX_1_ID			15 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_META_FLEX_2_ID			16 + (SLAVE_DCX10_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE

#define RIGHT_ARM_WRIST_ROTATOR_ID				1 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_BRACHII_ID					2 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_TRICEPS_ID					3 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_BICEPS_ID						4 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_DELTOID_ANTERIOR_ID			5 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_DELTOID_POSTERIOR_ID			6 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_SUPRA_ID						7 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_INFRA_ID						8 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_SUBSCAP_ID					9 + (SLAVE_DCX16_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE

/*
//Slave : Left Arm
#define LEFT_ARM_BICEPS_ID			1 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_SUPRA_ID			2 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_INFRA_ID			3 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_TMIN_ID			4 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_5_ID				5 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_SUBSCAP_ID			6 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_ANTDELT_ID			7 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_LATDELT_ID			8 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_POSTDELT_ID		9 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_BRACHI_ID			10 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_TRICEPS_ID			11 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_12_ID				12 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_HAND_ID			13 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_14_ID				14 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_ROTATOR_ID			15 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_16_ID				16 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
*/
/*
//Slave : Left Arm
#define LEFT_ARM_BICEPS_ID			1 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_SUPRA_ID			2 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_SUBSCAP_ID			3 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_INFRA_ID			4 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_TMIN_ID			5 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_LATDELT_ID			6 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_ANTDELT_ID			7 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_POSTDELT_ID		8 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_TRICEPS_ID			9 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_BRACHI_ID			10 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_HAND_ID			11 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_ROTATOR_ID			12 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_WRIST_IN_ID		13 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_WRIST_UP_ID		14 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_WRIST_DOWN_ID		15 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define LEFT_ARM_THUMB_ID			16 + (SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE

//Slave : Right Arm
#define RIGHT_ARM_BICEPS_ID			1 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_SUPRA_ID			2 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_INFRA_ID			3 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_TMIN_ID			4 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_SUBSCAP_ID		5 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_LATDELT_ID		6 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_ANTDELT_ID		7 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_POSTDELT_ID		8 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_TRICEPS_ID		9 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_BRACHI_ID			10 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_HAND_ID			11 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_ROTATOR_ID		12 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_IN_ID		13 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_UP_ID		14 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_WRIST_DOWN_ID		15 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define RIGHT_ARM_THUMB_ID			16 + (SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE

//Slave5 : Pelvis

//Slave6 : Head
#define HEAD_1_ID			1 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_2_ID			2 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_3_ID			3 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_4_ID			4 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_5_ID			5 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_6_ID			6 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_7_ID			7 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_EYE_PITCH_ID	8 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_EYE_YAW_ID		9 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_EYE_ROLL_ID	10 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_EYE_IRIS_ID	11 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_NECK7_ID		12 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_NECK8_ID		13 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_NECK9_ID		14 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_NECK10_ID		15 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE
#define HEAD_16_ID			16 + (SLAVE_HEAD_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE

#define FIRST_SERVO_ID					38
*/

//MIN MAX
#define MIN_POS_RIGHT_ARM_LITTLE_WHOLE_FLEX			0
#define MIN_POS_RIGHT_ARM_RING_WHOLE_FLEX			0
#define MIN_POS_RIGHT_ARM_INDEX_WHOLE_FLEX			0
#define MIN_POS_RIGHT_ARM_FORE_WHOLE_FLEX			0		//to check
#define MIN_POS_RIGHT_ARM_THUMB_WHOLE_FLEX			10000	//to check
#define MIN_POS_RIGHT_ARM_WRIST_IN					17000
#define MIN_POS_RIGHT_ARM_WRIST_UP					0
#define MIN_POS_RIGHT_ARM_WRIST_DOWN				0
#define MIN_POS_RIGHT_ARM_THUMB_META_EXTEND			37000
#define MIN_POS_RIGHT_ARM_THUMB_EXTEND				0
#define MIN_POS_RIGHT_ARM_FORE_EXTEND				16000
#define MIN_POS_RIGHT_ARM_LOWER_3_EXTEND			3000
#define MIN_POS_RIGHT_ARM_FORE_KNUCKLE_FLEX_1		33000
#define MIN_POS_RIGHT_ARM_FORE_KNUCKLE_FLEX_2		29000
#define MIN_POS_RIGHT_ARM_THUMB_META_FLEX_1			1000
#define MIN_POS_RIGHT_ARM_THUMB_META_FLEX_2			30000

#define MIN_POS_RIGHT_ARM_WRIST_ROTATOR				-18000
#define MIN_POS_RIGHT_ARM_BRACHII					10000
#define MIN_POS_RIGHT_ARM_TRICEPS					21000
#define MIN_POS_RIGHT_ARM_BICEPS					10000	//to check
#define MIN_POS_RIGHT_ARM_DELTOID_ANTERIOR			12000
#define MIN_POS_RIGHT_ARM_DELTOID_POSTERIOR			11000
#define MIN_POS_RIGHT_ARM_SUPRA						12000
#define MIN_POS_RIGHT_ARM_INFRA						0
#define MIN_POS_RIGHT_ARM_SUBSCAP					1300

#define MAX_POS_RIGHT_ARM_LITTLE_WHOLE_FLEX			215000
#define MAX_POS_RIGHT_ARM_RING_WHOLE_FLEX			250000
#define MAX_POS_RIGHT_ARM_INDEX_WHOLE_FLEX			253000
#define MAX_POS_RIGHT_ARM_FORE_WHOLE_FLEX			194000 	//to check
#define MAX_POS_RIGHT_ARM_THUMB_WHOLE_FLEX			216000 	//to check
#define MAX_POS_RIGHT_ARM_WRIST_IN					143000
#define MAX_POS_RIGHT_ARM_WRIST_UP					168000
#define MAX_POS_RIGHT_ARM_WRIST_DOWN				159000
#define MAX_POS_RIGHT_ARM_THUMB_META_EXTEND			176000
#define MAX_POS_RIGHT_ARM_THUMB_EXTEND				217000
#define MAX_POS_RIGHT_ARM_FORE_EXTEND				243000
#define MAX_POS_RIGHT_ARM_LOWER_3_EXTEND			178000
#define MAX_POS_RIGHT_ARM_FORE_KNUCKLE_FLEX_1		88000
#define MAX_POS_RIGHT_ARM_FORE_KNUCKLE_FLEX_2		90000
#define MAX_POS_RIGHT_ARM_THUMB_META_FLEX_1			207000
#define MAX_POS_RIGHT_ARM_THUMB_META_FLEX_2			241000

#define MAX_POS_RIGHT_ARM_WRIST_ROTATOR				12000
#define MAX_POS_RIGHT_ARM_BRACHII					47000
#define MAX_POS_RIGHT_ARM_TRICEPS					60000
#define MAX_POS_RIGHT_ARM_BICEPS					100000 //to check
#define MAX_POS_RIGHT_ARM_DELTOID_ANTERIOR			84000
#define MAX_POS_RIGHT_ARM_DELTOID_POSTERIOR			75000
#define MAX_POS_RIGHT_ARM_SUPRA						38000
#define MAX_POS_RIGHT_ARM_INFRA						50000
#define MAX_POS_RIGHT_ARM_SUBSCAP					45000

#endif // OSA_CONTROL_ROBOT_DEFINES_H
