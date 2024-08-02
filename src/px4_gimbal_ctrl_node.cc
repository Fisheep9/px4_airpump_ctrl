
/* This is a example of controlling gimbal using mavros mount control */
#include <ros/ros.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

mavros_msgs::State current_state;

bool last_pos_updated = false;

ros::Time last_switch_request;
bool exec_state_active = false;
bool trigger = false;
bool timer = false;

double roll_position{0.0}; //roll=气泵 pitch=阀门
double pitch_position{0.0};

double freq{0.0};
double offset{0.0};

double loop_rate{50.0};
double rate{50.0};


void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	    current_state = *msg;
}

void gimbal_pos_cb(const geometry_msgs::Quaternion::ConstPtr& msg) {
		if (!last_pos_updated) {
			tf2::Quaternion tf_quat;
    		tf2::convert(*msg, tf_quat);
    		double roll, pitch, yaw;
    		tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
			// update 
			double roll_last_position = std::ceil(roll*180/M_PI); //add
			double pitch_last_position = std::ceil(pitch*180/M_PI);
			roll_position = roll_last_position;
			pitch_position = pitch_last_position;
			last_pos_updated = true;
		}
}

void position_planner(double &target, double &current_pos) {

	if (target != NAN && last_pos_updated) { 
		if (current_pos > target) {
		current_pos -= rate / loop_rate; 
	}
	else if (current_pos < target) {
		current_pos += rate / loop_rate;
	}
	} 
}

void gimbal_trigger_cb(const std_msgs::Bool::ConstPtr& msg){
	trigger = msg->data; // true代表打开指令，false代表关闭指令
	exec_state_active = true;
	last_switch_request = ros::Time::now(); 
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "px4_gimbal_ctrl");
	ros::NodeHandle nh("");
	
	/* Subscribers */
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    				("mavros/state", 10, state_cb);
				   
	ros::Subscriber gimbal_pos_sub = nh.subscribe<geometry_msgs::Quaternion>
					("/mavros/mount_control/orientation", 10, gimbal_pos_cb);

	ros::Subscriber gimbal_trigger_sub = nh.subscribe<std_msgs::Bool>
					("/gimbal_trigger", 1, gimbal_trigger_cb);
	
	/* Publishers */
	ros::Publisher dataPublisher = nh.advertise<mavros_msgs::MountControl>
		 ("mavros/mount_control/command", 10);

	/* Servicecs */
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	            ("mavros/set_mode");

	/* handle params*/
	ros::param::get("gimbal_ctrl/freq", freq);

	// Create a rate
	ros::Rate rate(loop_rate);

	// wait for FCU connection
	// while (ros::ok() && !current_state.connected) {
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }
	
	/* set offboard mode */
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	ros::Time last_request = ros::Time::now();
	last_switch_request = ros::Time::now();

	while (ros::ok()) {

		/* handle current mode */
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
		                ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}	    

		mavros_msgs::MountControl msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "body";
		msg.mode = 2;
		msg.roll = 0;
		msg.pitch = 0;

		if((ros::Time::now() - last_switch_request > ros::Duration(2.0)) && trigger == false){ 
			exec_state_active = false;
		}

		if(exec_state_active){
			if (trigger){
				msg.roll = 90; 
			}else{ 
				msg.pitch = 90;
			}
		}
        	
		if (last_pos_updated) {
			dataPublisher.publish(msg);
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
