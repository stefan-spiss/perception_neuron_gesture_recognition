/*
 * PerceptionGestures.h
 *
 *  Created on: Jan 29, 2016
 *      Author: steve
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <iostream>


// HEAD Thresholds
#define NOD_THRESHOLD 0.2
#define SHAKE_THRESHOLD 0.2
#define BASE_POSITION_THRESHOLD 0.1
#define HEAD_PITCH_THRESHOLD 0.4 //Turn head left or right (negative)

// MYO Initial Pose (for mean value take initial values and add to roll + PI/2)
#define MYO_STANDARD_INITIAL_ROLL_THRESHOLD 0.5
#define MYO_STANDARD_INITIAL_PITCH_THRESHOLD 0.4
#define MYO_STANDARD_INITIAL_YAW_THRESHOLD 0.4

// Additional MYO Pose parameters, due to change according to calibration Pose
#define MYO_ROLL M_PI_2 + 0.55
#define MYO_PITCH 0.2

// MYO Swipe Right
#define MYO_STANDARD_SWIPE_RIGHT_ROLL_THRESHOLD 0.5
#define MYO_STANDARD_SWIPE_RIGHT_PITCH_THRESHOLD 0.4
#define MYO_STANDARD_SWIPE_RIGHT_YAW_THRESHOLD 0.4

// Flat Hand Swipe Right
#define FLAT_HAND_SWIPE_RIGTH_ROLL_THRESHOLD 0.4
#define FLAT_HAND_SWIPE_RIGTH_PITCH_THRESHOLD 0.2
#define FLAT_HAND_SWIPE_RIGTH_YAW_THRESHOLD 0.4

// Flat Hand
#define FLAT_HAND_ROLL_THRESHOLD 0.4
#define FLAT_HAND_PITCH_THRESHOLD 0.3
#define FLAT_HAND_YAW_THRESHOLD 0.4

// FIST
#define INDEX_X 0.0
#define MIDDLE_X 0.0
#define RING_X 0.0
#define PINKY_X 0.1

struct Sample {
	unsigned long long timestamp;
	float roll, pitch, yaw;

//	Sample(geometry_msgs::Quaternion::ConstPtr& sensor_data,
//			unsigned long long time) {
//		x = sensor_data->x;
//		y = sensor_data->y;
//		z = sensor_data->z;
//		w = sensor_data->w;
//		timestamp = time;
//	}
	Sample(float roll, float pitch, float yaw, unsigned long long time) {
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->timestamp = time;
	}
};

/**
 * Class detects gestures from the Perception Neuron and publishes them to following topics:
 * 		/perception/hand_gestures 			std_msgs::Int32
 * 			Values:	-1	Nothing detected
 * 					 0	Myo initial gesture
 * 					 1	Fist
 * 					 3	Myo swipe right
 * 					10	Flat hand
 * 					13	Flat hand swipe right
 * 		/perception/head_gestures			std_msgs::Int32
 * 			Values:	-1	Nothing detected
 * 					 1	Head nod
 * 					 2	Head shake
 * 					 3	Head turn left
 * 					 4	Head turn right
 * 		/perception/raw/head_orientation	geometry_msgs::Quaternion
 * 			Raw data of head orientation.
 */
class PerceptionGestures {
public:
	PerceptionGestures(ros::NodeHandle node);
	void detectGestures(ros::Rate &rate);
private:
	float INIT_HAND_ROLL, INIT_HAND_PITCH, INIT_HAND_YAW;
	float INIT_HEAD_ROLL, INIT_HEAD_PITCH, INIT_HEAD_YAW;

	double hand_roll, hand_pitch, hand_yaw;
	double head_roll, head_pitch, head_yaw;

	bool myo_initial, swipe_right, fist, nothing_detected, flat_hand,
			flat_hand_swipe_right;

	ros::NodeHandle node;
	ros::Publisher pub_hand_gestures;
	ros::Publisher pub_head_gestures;
	ros::Publisher pub_head_orientation;
	ros::ServiceServer srv_init;
	tf::TransformListener listener;

	std::deque<Sample> head_data;
	int head_gesture;
	unsigned long long start_time_wait;

	bool srv_init_callback(std_srvs::Empty::Request& request,
				std_srvs::Empty::Response& response);
	bool detecteNod();
	bool detecteHeadShake();
	std::vector<int> getRange(unsigned long long start_time,
			unsigned long long end_time) const;

};
