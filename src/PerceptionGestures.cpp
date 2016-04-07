#include <perception_neuron_gesture_recognition/PerceptionGestures.h>
#include <perception_neuron_gesture_recognition/time_ms.h>

PerceptionGestures::PerceptionGestures(ros::NodeHandle node) :
		node(node), INIT_HAND_ROLL(0.0), INIT_HAND_PITCH(0.0), INIT_HAND_YAW(
				0.0), INIT_HEAD_PITCH(0.0), INIT_HEAD_ROLL(0.0), INIT_HEAD_YAW(
				0.0), hand_roll(0.0), hand_pitch(0.0), hand_yaw(0.0), head_roll(
				0.0), head_pitch(0.0), head_yaw(0.0), myo_initial(false), swipe_right(
				false), fist(false), nothing_detected(false), flat_hand(false), flat_hand_swipe_right(
				false), head_gesture(0) {
	pub_hand_gestures = node.advertise<std_msgs::Int32>(
			"/perception/hand_gestures", 10);
	pub_head_gestures = node.advertise<std_msgs::Int32>(
			"/perception/head_gestures", 10);
	pub_head_orientation = node.advertise<geometry_msgs::Quaternion>(
			"/perception/raw/head_orientation", 10);
	srv_init = node.advertiseService("perception/init",
			&PerceptionGestures::srv_init_callback, this);
}

void PerceptionGestures::detectGestures(ros::Rate &rate) {
	while (ros::ok()) {
		std_msgs::Int32 gesture_msg;
		/***************************************** HEAD ******************************************/
		tf::StampedTransform transform_head;

		try {
			listener.lookupTransform("/Head", "/Neck", ros::Time(0),
					transform_head);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		tf::Matrix3x3(transform_head.getRotation()).getRPY(head_roll,
				head_pitch, head_yaw);

		tf::Quaternion tfQuat = transform_head.getRotation();
		geometry_msgs::Quaternion head_orientation_msg;

		tf::quaternionTFToMsg(tfQuat, head_orientation_msg);

		pub_head_orientation.publish(head_orientation_msg);

		Sample tmp(head_roll, head_pitch, head_yaw, time_ms());

		head_data.push_front(tmp);

		if (head_data.size() > 120) {
			head_data.pop_back();
		}

		std_msgs::Int32 head_gest;

		if (time_ms() - start_time_wait > 500) {
			if (detecteNod()) {
				if (head_gesture != 1) {
//					std::cout << "Head nod" << std::endl;
					head_gesture = 1;
					head_gest.data = 1;
					pub_head_gestures.publish(head_gest);
				}
			} else if (detecteHeadShake()) {
				if (head_gesture != 2 && head_gesture != 3
						&& head_gesture != 4) {
//					std::cout << "Head shake" << std::endl;
					head_gesture = 2;
					head_gest.data = 2;
					pub_head_gestures.publish(head_gest);
				}
			} else if (head_pitch < INIT_HEAD_PITCH - HEAD_PITCH_THRESHOLD) {
				if (head_gesture != 2 && head_gesture != 3) {
//					std::cout << "Head turn left" << std::endl;
					head_gesture = 3;
					head_gest.data = 3;
					pub_head_gestures.publish(head_gest);
				}
			} else if (head_pitch > INIT_HEAD_PITCH + HEAD_PITCH_THRESHOLD) {
				if (head_gesture != 2 && head_gesture != 4) {
//					std::cout << "Head turn right" << std::endl;
					head_gesture = 4;
					head_gest.data = 4;
					pub_head_gestures.publish(head_gest);
				}
			} else {
				if (head_gesture != -1) {
//					std::cout << "Head nothing" << std::endl;
					head_gesture = -1;
					head_gest.data = -1;
					pub_head_gestures.publish(head_gest);
				}
			}
		}

		// yaw: 	seitwärts
		// pitch:	drehen
		// roll:	nach vorne/hinten

//				std::cout << "roll: " << head_roll * 360 / M_PI << ", pitch: "
//						<< head_pitch * 360 / M_PI << ", yaw: " << head_yaw * 360 / M_PI
//						<< std::endl;

		/***************************************** FIST ******************************************/
		tf::StampedTransform transform_index, transform_middle, transform_ring,
				transform_pinky;
		try {
			listener.lookupTransform("/RightHandIndex3", "/RightHand",
					ros::Time(0), transform_index);
			listener.lookupTransform("/RightHandMiddle3", "/RightHand",
					ros::Time(0), transform_middle);
			listener.lookupTransform("/RightHandRing3", "/RightHand",
					ros::Time(0), transform_ring);
			listener.lookupTransform("/RightHandPinky3", "/RightHand",
					ros::Time(0), transform_pinky);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		tf::Vector3 tf_pose_index = transform_index.getOrigin();
		tf::Vector3 tf_pose_middle = transform_middle.getOrigin();
		tf::Vector3 tf_pose_ring = transform_ring.getOrigin();
		tf::Vector3 tf_pose_pinky = transform_pinky.getOrigin();

		//		std::cout << count << " index:\t x: " << tf_pose_index.getX() << ",\t y: "
		//				<< tf_pose_index.getY() << ",\t z: " << tf_pose_index.getZ()
		//				<< std::endl;
		//		std::cout << count << " middle:\t x: " << tf_pose_middle.getX() << ",\t y: "
		//				<< tf_pose_middle.getY() << ",\t z: " << tf_pose_middle.getZ()
		//				<< std::endl;
		//		std::cout << count << " ring:\t x: " << tf_pose_ring.getX() << ",\t y: "
		//				<< tf_pose_ring.getY() << ",\t z: " << tf_pose_ring.getZ()
		//				<< std::endl;
		//		std::cout << count << " pinky:\t x: " << tf_pose_pinky.getX() << ",\t y: "
		//				<< tf_pose_pinky.getY() << ",\t z: " << tf_pose_pinky.getZ()
		//				<< std::endl;

		if (!fist && tf_pose_index.getX() < INDEX_X
				&& tf_pose_middle.getX() < MIDDLE_X
				&& tf_pose_ring.getX() < RING_X
				&& tf_pose_pinky.getX() < PINKY_X) {
//			std::cout << "fist" << std::endl;
			gesture_msg.data = 1;
			pub_hand_gestures.publish(gesture_msg);
			fist = true;
			nothing_detected = false;
		} else if (tf_pose_index.getX() > INDEX_X
				&& tf_pose_middle.getX() > MIDDLE_X
				&& tf_pose_ring.getX() > RING_X) {
			fist = false;
		}

		/***************************************** HAND SWIPE ******************************************/
		tf::StampedTransform transform_swipes;

		try {
			listener.lookupTransform("/RightArm", "/RightHand", ros::Time(0),
					transform_swipes);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		tf::Matrix3x3(transform_swipes.getRotation()).getRPY(hand_roll,
				hand_pitch, hand_yaw);

//		std::cout << "roll: " << hand_roll * 360 / M_PI << ", pitch: "
//				<< hand_pitch * 360 / M_PI << ", yaw: " << hand_yaw * 360 / M_PI
//				<< std::endl;
//		std::cout << "roll: " << hand_roll << ", pitch: " << hand_pitch
//				<< ", yaw: " << hand_yaw << std::endl;
//		std::cout << "pitch: " << hand_pitch << " "
//				<< INIT_HAND_PITCH + MYO_PITCH
//						- MYO_STANDARD_SWIPE_RIGHT_PITCH_THRESHOLD << std::endl;
//		std::cout << "roll: " << hand_roll << " "
//				<< INIT_HAND_ROLL - MYO_ROLL
//						- MYO_STANDARD_SWIPE_RIGHT_ROLL_THRESHOLD << std::endl;
//		std::cout << "roll: " << hand_roll << " "
//				<< INIT_HAND_ROLL - MYO_ROLL
//						+ MYO_STANDARD_SWIPE_RIGHT_ROLL_THRESHOLD << std::endl;
//		std::cout << "yaw: " << hand_yaw << " "
//				<< INIT_HAND_YAW - MYO_STANDARD_SWIPE_RIGHT_YAW_THRESHOLD
//				<< std::endl;
//		std::cout << "yaw: " << hand_yaw << " "
//				<< INIT_HAND_YAW + MYO_STANDARD_SWIPE_RIGHT_YAW_THRESHOLD
//				<< std::endl;

		if (!fist
				&& hand_pitch
						< INIT_HAND_PITCH + MYO_PITCH
								- MYO_STANDARD_SWIPE_RIGHT_PITCH_THRESHOLD
				&& hand_roll
						> INIT_HAND_ROLL - MYO_ROLL
								- MYO_STANDARD_SWIPE_RIGHT_ROLL_THRESHOLD
				&& hand_roll
						< INIT_HAND_ROLL - MYO_ROLL
								+ 2.0*MYO_STANDARD_SWIPE_RIGHT_ROLL_THRESHOLD
				&& hand_yaw
						< INIT_HAND_YAW - MYO_STANDARD_SWIPE_RIGHT_YAW_THRESHOLD
//				&& hand_yaw
//						< INIT_HAND_YAW + MYO_STANDARD_SWIPE_RIGHT_YAW_THRESHOLD) {
						){
			if (!swipe_right) {
//				std::cout << "swipe_right" << std::endl;
				gesture_msg.data = 3;
				pub_hand_gestures.publish(gesture_msg);
				swipe_right = true;
				myo_initial = false;
				nothing_detected = false;
				flat_hand = false;
				flat_hand_swipe_right = false;
			}
		} else if (!fist
				&& hand_roll
						> INIT_HAND_ROLL - MYO_ROLL
								- MYO_STANDARD_INITIAL_ROLL_THRESHOLD
				&& hand_roll
						< INIT_HAND_ROLL - MYO_ROLL
								+ MYO_STANDARD_INITIAL_ROLL_THRESHOLD
				&& hand_pitch
						> INIT_HAND_PITCH + MYO_PITCH
								- MYO_STANDARD_INITIAL_PITCH_THRESHOLD
				&& hand_pitch
						< INIT_HAND_PITCH + MYO_PITCH
								+ MYO_STANDARD_INITIAL_PITCH_THRESHOLD
				&& hand_yaw > INIT_HAND_YAW - MYO_STANDARD_INITIAL_YAW_THRESHOLD
				&& hand_yaw < INIT_HAND_YAW + MYO_STANDARD_INITIAL_YAW_THRESHOLD) {
			if (!myo_initial) {
//				std::cout << "initial pose" << std::endl;
				gesture_msg.data = 0;
				pub_hand_gestures.publish(gesture_msg);
				myo_initial = true;
				swipe_right = false;
				nothing_detected = false;
				flat_hand = false;
				flat_hand_swipe_right = false;
			}
		} else if (!fist
				&& hand_roll > INIT_HAND_ROLL - FLAT_HAND_ROLL_THRESHOLD
				&& hand_roll < INIT_HAND_ROLL + FLAT_HAND_ROLL_THRESHOLD
				&& hand_pitch > INIT_HAND_PITCH - FLAT_HAND_PITCH_THRESHOLD
				&& hand_pitch < INIT_HAND_PITCH + FLAT_HAND_PITCH_THRESHOLD
				&& hand_yaw > INIT_HAND_YAW - FLAT_HAND_YAW_THRESHOLD
				&& hand_yaw < INIT_HAND_YAW + FLAT_HAND_YAW_THRESHOLD) {
			if (!flat_hand) {
//				std::cout << "flat hand" << std::endl;
				gesture_msg.data = 10;
				pub_hand_gestures.publish(gesture_msg);
				flat_hand = true;
				myo_initial = false;
				swipe_right = false;
				nothing_detected = false;
				flat_hand_swipe_right = false;
			}
		} else if (!fist
				&& hand_roll
						> INIT_HAND_ROLL - FLAT_HAND_SWIPE_RIGTH_ROLL_THRESHOLD
				&& hand_roll
						< INIT_HAND_ROLL + FLAT_HAND_SWIPE_RIGTH_ROLL_THRESHOLD
				&& hand_pitch
						< INIT_HAND_PITCH
								- FLAT_HAND_SWIPE_RIGTH_PITCH_THRESHOLD
				&& hand_yaw
						> INIT_HAND_YAW - FLAT_HAND_SWIPE_RIGTH_YAW_THRESHOLD
				&& hand_yaw
						< INIT_HAND_YAW + FLAT_HAND_SWIPE_RIGTH_YAW_THRESHOLD) {
			if (!flat_hand_swipe_right) {
//				std::cout << "flat hand swipe right" << std::endl;
				gesture_msg.data = 13;
				pub_hand_gestures.publish(gesture_msg);
				flat_hand_swipe_right = true;
				myo_initial = false;
				swipe_right = false;
				nothing_detected = false;
				flat_hand = false;
			}
		} else {
			if (!nothing_detected && !swipe_right && !myo_initial && !fist
					&& !flat_hand && !flat_hand_swipe_right) {
//				std::cout << "Nothing detected" << std::endl;
				gesture_msg.data = -1;
				pub_hand_gestures.publish(gesture_msg);
				nothing_detected = true;
			}
			swipe_right = false;
			myo_initial = false;
			flat_hand = false;
			flat_hand_swipe_right = false;
		}
		ros::spinOnce();
		rate.sleep();
	}

//	std::cout << "Initialized values." << std::endl;
//	std::cout << "init: roll: " << INIT_HAND_ROLL << " pitch: "
//			<< INIT_HAND_PITCH << " yaw: " << INIT_HAND_YAW << std::endl;

}

bool PerceptionGestures::srv_init_callback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	ros::Time start = ros::Time::now();
	ros::Duration duration(5.0);

	double avg_hand_roll = 0;
	double avg_hand_pitch = 0;
	double avg_hand_yaw = 0;
	double avg_head_roll = 0;
	double avg_head_pitch = 0;
	double avg_head_yaw = 0;

	int count = 0;

	while ((ros::Time::now() - start).toSec() < duration.toSec()) {

		avg_hand_roll += hand_roll;
		avg_hand_pitch += hand_pitch;
		avg_hand_yaw += hand_yaw;
		avg_head_roll += head_roll;
		avg_head_pitch += head_pitch;
		avg_head_yaw += head_yaw;
		count++;
	}

	INIT_HAND_ROLL = avg_hand_roll / count;
	INIT_HAND_PITCH = avg_hand_pitch / count;
	INIT_HAND_YAW = avg_hand_yaw / count;
	INIT_HEAD_ROLL = avg_head_roll / count;
	INIT_HEAD_PITCH = avg_head_pitch / count;
	INIT_HEAD_YAW = avg_head_yaw / count;

	std::cout << "Initialized values." << std::endl;
	std::cout << "init hand: roll: " << INIT_HAND_ROLL << " pitch: "
			<< INIT_HAND_PITCH << " yaw: " << INIT_HAND_YAW << std::endl;
	std::cout << "init head: roll: " << INIT_HEAD_ROLL << " pitch: "
			<< INIT_HEAD_PITCH << " yaw: " << INIT_HEAD_YAW << std::endl;
	return true;
}

bool PerceptionGestures::detecteNod() {
	bool nod = false;

	std::vector<int> range = getRange(200, 400);
	float basePos = 0.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		basePos += head_data[i].roll;
	}
	basePos /= (range[1] - range[0] + 1);

	range = getRange(10, 200);
	float rollMax = -10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (head_data[i].roll > rollMax) {
			rollMax = head_data[i].roll;
		}
	}

	float current = head_data.front().roll;

	if (rollMax - basePos > NOD_THRESHOLD
			&& fabs(current - basePos) < BASE_POSITION_THRESHOLD) {
		nod = true;
		start_time_wait = time_ms();
	}
	return nod;
}

bool PerceptionGestures::detecteHeadShake() {
	bool headShake = false;

	std::vector<int> range = getRange(200, 400);
	float basePos = 0.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		basePos += head_data[i].pitch;
	}
	basePos /= (range[1] - range[0] + 1);

	range = getRange(10, 200);
	float pitchMax = -10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (head_data[i].pitch > pitchMax) {
			pitchMax = head_data[i].pitch;
		}
	}

	float pitchMin = 10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (head_data[i].pitch < pitchMin) {
			pitchMin = head_data[i].pitch;
		}
	}

	float current = head_data.front().pitch;

	if ((pitchMax - basePos > SHAKE_THRESHOLD
			|| basePos - pitchMin > SHAKE_THRESHOLD)
			&& fabs(current - basePos) < BASE_POSITION_THRESHOLD) {
		headShake = true;
		start_time_wait = time_ms();
	}
	return headShake;
}

std::vector<int> PerceptionGestures::getRange(unsigned long long start_time,
		unsigned long long end_time) const {
	int begin = 0;
	int end = 0;
	for (int i = 0; i < head_data.size(); ++i) {
		if (head_data[i].timestamp < (time_ms() - start_time) && begin == 0) {
			begin = i;
		} else if (head_data[i].timestamp >= (time_ms() - end_time)) {
			end = i;
		}
	}
	return std::vector<int>( { begin, end });
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "perception_gesture_node");
	ros::NodeHandle node;

	PerceptionGestures perception_gestures(node);

	ros::NodeHandle local_node("~");
	double frequency(10.0);
	local_node.getParam("perception/frequency", frequency);
	ros::Rate rate(frequency);

	perception_gestures.detectGestures(rate);

}
