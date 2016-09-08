/**
* @file offb_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include "breadcrumb_node.h"
#include "pid.h"


//================================//
// Callback Functions             //
//================================//

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	currentState = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	double dt = (msg->header.stamp - currentPose.header.stamp).toSec();
	currentVelocity.linear.x = (msg->pose.position.x - currentPose.pose.position.x) / dt;
	currentVelocity.linear.y = (msg->pose.position.y - currentPose.pose.position.y) / dt;
	currentVelocity.linear.z = (msg->pose.position.z - currentPose.pose.position.z) / dt;

	currentPose = *msg;
}

void waypoint_cb(const geometry_msgs::PoseArray::ConstPtr& msg) {
	ROS_INFO("Received new waypoint list...");

	waypointList = msg->poses;
	waypointCounter = -1;

	ROS_INFO("Loaded in %i waypoints.", waypointList.size());
}

geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q) {
    geometry_msgs::Vector3 e;

    double q2sqr = q.y * q.y;
    double t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0;
    double t1 = +2.0 * (q.x * q.y + q.w * q.z);
    double t2 = -2.0 * (q.x * q.z - q.w * q.y);
    double t3 = +2.0 * (q.y * q.z + q.w * q.x);
    double t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    e.x = atan2(t3, t4);
    e.y = asin(t2);
    e.z = atan2(t1, t0);

    return e;
}

geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

	return q;
}

bool comparePose(geometry_msgs::Pose goal, geometry_msgs::Pose current) {
	bool reached = false;

	if ( fabs(current.position.x - goal.position.x ) <= waypointRadius )
		if ( fabs(current.position.y - goal.position.y ) <= waypointRadius )
			if ( fabs(current.position.z - goal.position.z ) <= waypointRadius )
				if ( fabs( toEuler( current.orientation ).z - toEuler( goal.orientation ).z ) <= headingAccuracy )
					reached = true;

	return reached;
}

//================================//
// Main Function                  //
//================================//
int main(int argc, char **argv)
{
	//================================//
	// Initialize node                //
	//================================//
	ros::init(argc, argv, "breadcrumb" );
	ros::NodeHandle nh(ros::this_node::getName() );


	//================================//
	// System Variables               //
	//================================//

	//TODO: Prepare a better list of mavros modes
	//TODO: Set a fallback mode for HALT

	//==== Topics ====//
	std::string positionInput = "reference/pose";
	std::string waypointInput = "reference/wapoints";

	std::string positionOutputTopic = "goal/position_target";
	std::string velocityOutputTopic = "goal/velocity_target";
	std::string accelerationOutputTopic = "goal/acceleration_target";

	std::string waypointConfirm = "goal/waypoint_complete";
	std::string missionConfirm = "goal/mission_complete";

	bool sendVelocity = true;
	bool sendAcceleration = true;

	//==== Prepare Topics ====//
	//outputTwist.header.frame_id = "fcu";

	/*
	outputPose.header.frame_id = "fcu";
	outputTarget.header.frame_id = "fcu";
	outputTarget.coordinate_frame = outputTarget.FRAME_BODY_OFFSET_NED;
	outputTarget.type_mask += outputTarget.IGNORE_PX;
	outputTarget.type_mask += outputTarget.IGNORE_PY;
	outputTarget.type_mask += outputTarget.IGNORE_PZ;
	outputTarget.type_mask += outputTarget.IGNORE_AFX;
	outputTarget.type_mask += outputTarget.IGNORE_AFY;
	outputTarget.type_mask += outputTarget.IGNORE_AFZ;
	outputTarget.type_mask += outputTarget.IGNORE_YAW;
	*/

	//==== MavROS Interface ====//

	std::string triggerMode = "POSCTL";
	std::string passbackMode = "AUTO.LOITER";

	mavros_msgs::SetMode setModeOffB;
	setModeOffB.request.custom_mode = "OFFBOARD";

	mavros_msgs::SetMode setModeLand;
	setModeLand.request.custom_mode = "AUTO.LAND";
	//TODO: Put this into a parameter

	mavros_msgs::SetMode setModeLoiter;
	setModeLoiter.request.custom_mode = passbackMode;

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	mavros_msgs::CommandBool disarm_cmd;
	disarm_cmd.request.value = false;

	ros::Time lastRequest = ros::Time(0);

	const std_msgs::Empty outputConfirm;

	//==== Logic Operators ====//
	//bool changedNavMode = true;
	navCurrentMode = NAV_MODE_PRECONNECT;
	bool systemOperational = false;	//Status to see if breadcrumb should be in control
	bool startSystem = true;	//Set by a service to start breadcrumb
	bool sendMovement = false;	//Should be set to false when the UAV should not be moving
	bool terminate = false;
	bool changedMode = false;	//Should be set on each mode change to allow gracefull passover

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(NAV_RATE);

	//==== PID Controllers ====//
	pid pos_x_pid(1.0/NAV_RATE);
	pid pos_y_pid(1.0/NAV_RATE);
	pid pos_z_pid(1.0/NAV_RATE);

	pid vel_x_pid(1.0/NAV_RATE);
	pid vel_y_pid(1.0/NAV_RATE);
	pid vel_z_pid(1.0/NAV_RATE);

	pid acc_x_pid(1.0/NAV_RATE);
	pid acc_y_pid(1.0/NAV_RATE);
	pid acc_z_pid(1.0/NAV_RATE);
	//pid vel_h_pid(1.0/NAV_RATE);

	//================================//
	// Load Parameters                //
	//================================//

	//Input/Output //================================================================
	if( !nh.getParam( "position_input", positionInput ) ) {
		ROS_WARN( "No parameter set for \"position_input\", using: %s", positionInput.c_str() );
	}
	ROS_INFO( "Listening for position input: %s", positionInput.c_str() );

	if( !nh.getParam( "waypoint_input", waypointInput ) ) {
		ROS_WARN( "No parameter set for \"waypoint_input\", using: %s", waypointInput.c_str() );
	}
	ROS_INFO( "Listening for waypoint input: %s", waypointInput.c_str() );

	if( !nh.getParam( "reached_goal", waypointConfirm ) ) {
		ROS_WARN( "No parameter set for \"reached_goal\", using: %s", waypointConfirm.c_str() );
	}
	ROS_INFO( "Broadcasting waypoint incremental status: %s", waypointConfirm.c_str() );

	if( !nh.getParam( "mission_goal", missionConfirm ) ) {
		ROS_WARN( "No parameter set for \"mission_goal\", using: %s", missionConfirm.c_str() );
	}
	ROS_INFO( "Broadcasting mission status: %s", missionConfirm.c_str() );

	/*
	if( !nh.getParam( "velocity_command", velocityOutput) ) {
		ROS_WARN( "No parameter set for \"velocity_command\", using: %s", velocityOutput.c_str() );
	}
	ROS_INFO( "Sending commands to: %s", velocityOutput.c_str() );
	*/

	if( !nh.getParam( "position_goal", positionOutputTopic ) ) {	//We don't disable this so there we always have the "goal waypoint"
		ROS_WARN( "No parameter set for \"position_goal\", using: %s", positionOutputTopic.c_str() );
	} else {
		ROS_INFO( "Sending position commands to: %s", positionOutputTopic.c_str() );
	}

	if( !nh.getParam( "velocity_goal", velocityOutputTopic ) ) {
		ROS_WARN( "No parameter set for \"velocity_goal\", disabling velocity output" );
		sendVelocity = false;
	} else {
		ROS_INFO( "Sending velocity commands to: %s", velocityOutputTopic.c_str() );
	}

	if( !nh.getParam( "acceleration_goal", accelerationOutputTopic) ) {
		ROS_WARN( "No parameter set for \"acceleration_goal\", disabling acceleration output" );
		sendAcceleration = false;
	} else {
		ROS_INFO( "Sending acceleration commands to: %s", accelerationOutputTopic.c_str() );
	}

	if( sendVelocity || sendAcceleration ) {
		//POS XY PID //================================================================
		if( !nh.getParam( "pos_xy_pid/p", pos_x_pid.Kp) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/p\", using: %0.2f", pos_x_pid.Kp);
		}

		if( !nh.getParam( "pos_xy_pid/i", pos_x_pid.Ki) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/i\", using: %0.2f", pos_x_pid.Ki);
		}

		if( !nh.getParam( "pos_xy_pid/d", pos_x_pid.Kd) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/d\", using: %0.2f", pos_x_pid.Kd);
		}

		if( !nh.getParam( "pos_xy_pid/min", pos_x_pid.min_output) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/min\", using: %0.2f", pos_x_pid.min_output);
		}

		if( !nh.getParam( "pos_xy_pid/max", pos_x_pid.max_output) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/max\", using: %0.2f", pos_x_pid.max_output);
		}

		pos_y_pid.Kp = pos_x_pid.Kp;
		pos_y_pid.Ki = pos_x_pid.Ki;
		pos_y_pid.Kd = pos_x_pid.Kd;
		pos_y_pid.min_output = pos_x_pid.min_output;
		pos_y_pid.max_output = pos_x_pid.max_output;

		ROS_INFO( "Setting pos_xy_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", pos_x_pid.Kp, pos_x_pid.Ki, pos_x_pid.Kd, pos_x_pid.min_output, pos_x_pid.max_output);

		//POS Z PID //================================================================
		if( !nh.getParam( "pos_z_pid/p", pos_z_pid.Kp) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/p\", using: %0.2f", pos_z_pid.Kp);
		}

		if( !nh.getParam( "pos_z_pid/i", pos_z_pid.Ki) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/i\", using: %0.2f", pos_z_pid.Ki);
		}

		if( !nh.getParam( "pos_z_pid/d", pos_z_pid.Kd) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/d\", using: %0.2f", pos_z_pid.Kd);
		}

		if( !nh.getParam( "pos_z_pid/min", pos_z_pid.min_output) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/min\", using: %0.2f", pos_z_pid.min_output);
		}

		if( !nh.getParam( "pos_z_pid/max", pos_z_pid.max_output) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/max\", using: %0.2f", pos_z_pid.max_output);
		}

		ROS_INFO( "Setting pos_z_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", pos_z_pid.Kp, pos_z_pid.Ki, pos_z_pid.Kd, pos_z_pid.min_output, pos_z_pid.max_output);
	}

	if( sendAcceleration ) {
		//VEL XY PID //================================================================
		if( !nh.getParam( "vel_xy_pid/p", vel_x_pid.Kp) ) {
			ROS_WARN( "No parameter set for \"vel_xy_pid/p\", using: %0.2f", vel_x_pid.Kp);
		}

		if( !nh.getParam( "vel_xy_pid/i", vel_x_pid.Ki) ) {
			ROS_WARN( "No parameter set for \"vel_xy_pid/i\", using: %0.2f", vel_x_pid.Ki);
		}

		if( !nh.getParam( "vel_xy_pid/d", vel_x_pid.Kd) ) {
			ROS_WARN( "No parameter set for \"vel_xy_pid/d\", using: %0.2f", vel_x_pid.Kd);
		}

		if( !nh.getParam( "vel_xy_pid/min", vel_x_pid.min_output) ) {
			ROS_WARN( "No parameter set for \"vel_xy_pid/min\", using: %0.2f", vel_x_pid.min_output);
		}

		if( !nh.getParam( "vel_xy_pid/max", vel_x_pid.max_output) ) {
			ROS_WARN( "No parameter set for \"vel_xy_pid/max\", using: %0.2f", vel_x_pid.max_output);
		}

		vel_y_pid.Kp = vel_x_pid.Kp;
		vel_y_pid.Ki = vel_x_pid.Ki;
		vel_y_pid.Kd = vel_x_pid.Kd;
		vel_y_pid.min_output = vel_x_pid.min_output;
		vel_y_pid.max_output = vel_x_pid.max_output;

		ROS_INFO( "Setting vel_xy_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_x_pid.Kp, vel_x_pid.Ki, vel_x_pid.Kd, vel_x_pid.min_output, vel_x_pid.max_output);

		//POS Z PID //================================================================
		if( !nh.getParam( "vel_z_pid/p", vel_z_pid.Kp) ) {
			ROS_WARN( "No parameter set for \"vel_z_pid/p\", using: %0.2f", vel_z_pid.Kp);
		}

		if( !nh.getParam( "vel_z_pid/i", vel_z_pid.Ki) ) {
			ROS_WARN( "No parameter set for \"vel_z_pid/i\", using: %0.2f", vel_z_pid.Ki);
		}

		if( !nh.getParam( "vel_z_pid/d", vel_z_pid.Kd) ) {
			ROS_WARN( "No parameter set for \"vel_z_pid/d\", using: %0.2f", vel_z_pid.Kd);
		}

		if( !nh.getParam( "vel_z_pid/min", vel_z_pid.min_output) ) {
			ROS_WARN( "No parameter set for \"vel_z_pid/min\", using: %0.2f", vel_z_pid.min_output);
		}

		if( !nh.getParam( "vel_z_pid/max", vel_z_pid.max_output) ) {
			ROS_WARN( "No parameter set for \"vel_z_pid/max\", using: %0.2f", vel_z_pid.max_output);
		}

		ROS_INFO( "Setting vel_z_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_z_pid.Kp, vel_z_pid.Ki, vel_z_pid.Kd, vel_z_pid.min_output, vel_z_pid.max_output);
	}

	/*
	//H PID //================================================================
	if( !nh.getParam( "h_pid/p", vel_h_pid.Kp) ) {
		ROS_WARN( "No parameter set for \"h_pid/p\", using: %0.2f", vel_h_pid.Kp);
	}

	if( !nh.getParam( "h_pid/i", vel_h_pid.Ki) ) {
		ROS_WARN( "No parameter set for \"h_pid/i\", using: %0.2f", vel_h_pid.Ki);
	}

	if( !nh.getParam( "h_pid/d", vel_h_pid.Kd) ) {
		ROS_WARN( "No parameter set for \"h_pid/d\", using: %0.2f", vel_h_pid.Kd);
	}

	if( !nh.getParam( "h_pid/min", vel_h_pid.min_output) ) {
		ROS_WARN( "No parameter set for \"h_pid/min\", using: %0.2f", vel_h_pid.min_output);
	}

	if( !nh.getParam( "h_pid/max", vel_h_pid.max_output) ) {
		ROS_WARN( "No parameter set for \"h_pid/max\", using: %0.2f", vel_h_pid.max_output);
	}

	ROS_INFO( "Setting h_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_h_pid.Kp, vel_h_pid.Ki, vel_h_pid.Kd, vel_h_pid.min_output, vel_h_pid.max_output);
	*/

	//Subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		( "/mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(positionInput, 10, local_pos_cb);
	ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PoseArray>
		(waypointInput, 10, waypoint_cb);

	//Publishers
	//ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
	//	(velocityOutput, 10);
	ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>
		(accelerationOutputTopic, 10);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
		(velocityOutputTopic, 10);
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		(positionOutputTopic, 10);

	ros::Publisher wp_confirm_pub = nh.advertise<std_msgs::Empty>
		(waypointConfirm, 1);	//Only really want this to be sent or read once at a time.
	ros::Publisher mission_confirm_pub = nh.advertise<std_msgs::Empty>
		(missionConfirm, 1);

	//Services
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		( "/mavros/cmd/arming" );
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		( "/mavros/set_mode" );


	//================================//
	// Load Waypoints                 //
	//================================//

	//TODO: Load specified waypoint file if one is set

	geometry_msgs::Vector3 wp_heading;
	wp_heading.z = 1.57;

	geometry_msgs::Pose waypointFiller;
	waypointFiller.position.x = 1;
	waypointFiller.position.z = 1;
	waypointFiller.orientation = toQuaternion(wp_heading);
	waypointList.push_back(waypointFiller);
	waypointFiller.position.y = 1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.x = -1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.y = -1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.x = 1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.y = 0;
	waypointList.push_back(waypointFiller);


	//================================//
	// Wait for Connection            //
	//================================//

	// Wait for FCU connection
    while(ros::ok() && ( !currentState.connected || ( currentPose.header.seq == 0) ) ) {
		ROS_INFO_THROTTLE( MSG_FREQ, "Breadcrumb is waiting for connection to mav..." );
		//ROS_INFO( "SEQ: %i", currentPose.header.seq );

		ros::spinOnce();
        rate.sleep();
    }


	lastRequest = ros::Time::now();

	//TODO: This should be implemented into the main loop to watch for a lost connection to the mav
	//TODO: Watch the Sequence on the pose stamped

	//TODO: Should have a state message running at the same time

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() && !terminate ) {
		//Flight Control State Machine //================================================================
		//Allow remote starting of the system
		//TODO: Anything?
		if( currentState.mode == triggerMode )	//Set triggerMode to "NULL" to disable this feature
			startSystem = true;


		//TODO: Maybe check to see if the state message is still fresh
		// If the system was running, but the mav has switched modes or disarmed
		if( ( systemOperational ) && ( ( currentState.mode != "OFFBOARD" ) || ( !currentState.armed ) ) ) {
			startSystem = false;
			systemOperational = false;

			navCurrentMode = NAV_MODE_PRECONNECT;
			currentGoal = navGoalHome;

			waypointCounter == -1;

			ROS_ERROR( "The mav has changed to a non-controllable state [MODE: %s, ARMED: %d]", currentState.mode.c_str(), currentState.armed );
			ROS_WARN( "Resetting goal to [HOME] coordinates: [%0.2f, %0.2f, %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z );
			ROS_WARN( "Disconnecting breadcrumb, breadcrumb can be reconnected when the system has returned to a usable state" );
		}	//TODO: Get the user to reset if this is the case

		switch( navCurrentMode ) {
			case NAV_MODE_PRECONNECT:
				if( startSystem ) {
					//New request every 5 seconds
					if( ( ros::Time::now() - lastRequest ) > ros::Duration( 5.0 ) ) {
						if( currentState.mode != "OFFBOARD" ) {	//If the mav is currently not in OFFBOARD mode
							ROS_INFO( "Requesting \"OFFBOARD\" mode [%s]", currentState.mode.c_str() );

							if( set_mode_client.call(setModeOffB) && setModeOffB.response.success ) {
								ROS_INFO( "Offboard control enabled" );
							}
						}

						if( ( currentState.mode == "OFFBOARD" ) && ( !currentState.armed ) ) {
							ROS_INFO( "Attempting to arm mav" );

							if( arming_client.call(arm_cmd) && arm_cmd.response.success ) {
								ROS_INFO( "Mav sucessfully armed" );
							} else {
								ROS_INFO( "Failed to arm mav" );
							}
						}

						lastRequest = ros::Time::now();
					}

					//If the mav is in the right mode, and is armed
					if( ( currentState.mode == "OFFBOARD" ) && ( currentState.armed ) ) {
						//Set the current goal to hold position until the handover is complete (in case this is mid flight)
						currentGoal = currentPose.pose;
						floorHeight = currentPose.pose.position.z;

						geometry_msgs::Vector3 rot = toEuler( currentPose.pose.orientation );
						//ROS_ERROR( "ROTATIONS: [%0.2f,%0.2f,%0.2f,%0.2f]", rot.z,currentPose.pose.orientation.w,currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z);
						rot.x = 0;
						rot.y = 0;
						currentGoal.orientation = toQuaternion( rot );

						//TODO: If no home coords are manually set, then home should be above the copter.

						navGoalHome = currentGoal;
						navGoalHome.position.z += navGoalTakeoffHeight;

						ROS_WARN( "Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( currentGoal.orientation ).z );


						ROS_INFO( "Mav is armed and listening" );
						navCurrentMode = NAV_MODE_SLEEP;
						systemOperational = true;
						startSystem = false;
						ROS_INFO( "Breadcrumb is now active" );

						ROS_WARN( "DEBUG MODE SWITCH [TAKEOFF]" );
						navCurrentMode = NAV_MODE_TAKEOFF; //TODO: Should just not be here
						changedMode = true;
					}
				} else
					ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is in standby, awaiting activation..." );

				break;
			case NAV_MODE_SLEEP:
				ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is conected, awaiting commands..." );
				break;
			case NAV_MODE_MISSION_START:
				//TODO: Should check to see if the mav is flying, if not, add takeoff goal and pass over to mission
				break;
			case NAV_MODE_TAKEOFF:
				if( changedMode ) {
					ROS_INFO("Attempting to takeoff...");

					if( !sendMovement ) { //If the mav is disabled, enable outputs
						ROS_INFO( "Activating goal output" );
						sendMovement = true;

						navGoalTakeoff = currentPose.pose;
						navGoalTakeoff.position.z = navGoalTakeoffHeight;

						currentGoal = navGoalTakeoff;

						ROS_INFO( "Commanding the mav to head to [TAKEOFF]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalTakeoff.position.x, navGoalTakeoff.position.y, navGoalTakeoff.position.z, toEuler( navGoalTakeoff.orientation ).z );
					} else { //Motor commands are active, but the mav might be on the ground?
						ROS_WARN("The mav is already being controlled (and it seems like it should be flying!");
						ROS_WARN("Commanding to hold position!");
						currentGoal = currentPose.pose;
					}

					changedMode = false;
				}

				if( comparePose( currentGoal, currentPose.pose ) ) {
					ROS_INFO( "Reached takeoff goal!" );
					navCurrentMode = NAV_MODE_MISSION;	//TODO: Should be NAV_MODE_SLEEP
					changedMode = true;
				}

				break;
			case NAV_MODE_MISSION:
				if( changedMode ) {	//Reset the waypoint counter if switching to mission mode
					waypointCounter = -1;
					ROS_INFO("Begining waypoint mission...");
					changedMode = false;
				}

				if(waypointCounter == -1) {
					waypointCounter = 0;
					currentGoal = waypointList.at(waypointCounter);
					ROS_INFO( "Heading to next waypoint: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );
				}

				if( comparePose( currentGoal, currentPose.pose ) ) {
					waypointCounter++;

					if( waypointCounter < waypointList.size() ) {
						currentGoal = waypointList.at(waypointCounter);

						wp_confirm_pub.publish( outputConfirm );

						ROS_INFO( "Heading to next waypoint: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );
					} else {
						waypointCounter = -1;

						mission_confirm_pub.publish( outputConfirm );

						navCurrentMode = NAV_MODE_SLEEP;
						changedMode = true;

						ROS_INFO( "All waypoints have been reached..." );
					}
				}

				// TODO: Should have a message to say when a goal was reached
			case NAV_MODE_PAUSE:
				// TODO: Need to have a remote mode change implemented
				break;
			case NAV_MODE_HOME:
				if( changedMode ) {
					changedMode = false;
					ROS_INFO( "Commanding the mav to head to [HOME]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( navGoalHome.orientation ).z );
				}

				currentGoal = navGoalHome;

				if( comparePose( currentGoal, currentPose.pose ) ) {
					changedMode = true;
					navCurrentMode = NAV_MODE_LAND;
				}

				break;
			case NAV_MODE_LAND:
				//TODO: Should check the current position to know when to cut velocity, or to disarm when on the ground.
				currentGoal = currentPose.pose;
				currentGoal.position.z = floorHeight;

				if( changedMode ) {
					changedMode = false;
					ROS_INFO( "Commanding the mav to land: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );
				}

				if( comparePose( currentGoal, currentPose.pose ) ) {
					ROS_INFO( "Reached landing goal!" );

					ROS_WARN( "Attempting to disarm mav" );
					if( arming_client.call(disarm_cmd) && disarm_cmd.response.success ) {
						ROS_INFO( "Mav disarmed" );
					}

					sendMovement = false;
					changedMode = true;
					navCurrentMode = NAV_MODE_SLEEP;
				}
				/*
				if( set_mode_client.call(setModeLand) && setModeLand.response.success) {
					ROS_WARN( "Breadcrumb: Halting operation, landing imidiately" );
				} else {
					ROS_ERROR( "Cannot enter landing mode, mav may be of control!" );
				} */

				//terminate = true;

				break;
			case NAV_MODE_MISSION_END:
				//TODO: Set a goal of waypoints the same as home, then switch to landing mode
				break;
			default:
				ROS_ERROR( "Breadcrumb: Mode was set to an invalid value [%d]", navCurrentMode);
				terminate = true;

				break;
		}


		//Generate Commands //================================================================
		//outputTwist.header.stamp = ros::Time::now();
		//outputTarget.header.stamp = ros::Time::now();
		ros::Time timestamp = ros::Time::now();

		outputPosition.header.stamp = timestamp;
		outputVelocity.header.stamp = timestamp;
		outputAcceleration.header.stamp = timestamp;

		//currentGoal.position.x = 0.0;
		//currentGoal.position.y = 0.0;
		//currentGoal.position.z = 1.0;

		if( sendMovement ) {
			/*
			//Linear Controller
			outputTwist.twist.linear.x = pos_x_pid.step(currentGoal.position.x, currentPose.pose.position.x);
			outputTwist.twist.linear.y = pos_x_pid.step(currentGoal.position.y, currentPose.pose.position.y);
			outputTwist.twist.linear.z = pos_x_pid.step(currentGoal.position.z, currentPose.pose.position.z);

			//Angular Controller
			outputTwist.twist.angular.z = vel_h_pid.step( toEuler( currentGoal.orientation ).z, toEuler( currentPose.pose.orientation ).z );
			*/

			//Position
			outputPosition.pose = currentGoal;

			//Velocity
			if( sendVelocity || sendAcceleration ) {	// Omly bother to calculate if the requested
				outputVelocity.twist.linear.x = pos_x_pid.step(currentGoal.position.x, currentPose.pose.position.x);
				outputVelocity.twist.linear.y = pos_y_pid.step(currentGoal.position.y, currentPose.pose.position.y);
				outputVelocity.twist.linear.z = pos_z_pid.step(currentGoal.position.z, currentPose.pose.position.z);
			}

			//TODO: Heading?

			//Acceleration
			if( sendAcceleration ) {	// Omly bother to calculate if the requested
				outputAcceleration.vector.x = vel_x_pid.step(outputVelocity.twist.linear.x, currentVelocity.linear.x);
				outputAcceleration.vector.y = vel_y_pid.step(outputVelocity.twist.linear.y, currentVelocity.linear.y);
				outputAcceleration.vector.z = vel_z_pid.step(outputVelocity.twist.linear.z, currentVelocity.linear.z);
			}
		} else {
			ROS_WARN_THROTTLE(MSG_FREQ, "Commanding the mav to stay still..." );
			/*
			outputTwist.twist.linear.x = 0;
			outputTwist.twist.linear.y = 0;
			outputTwist.twist.linear.z = 0;
			outputTwist.twist.angular.x = 0;
			outputTwist.twist.angular.y = 0;
			outputTwist.twist.angular.z = 0;
			*/
			/*
			outputTarget.velocity.x = 0.0;
			outputTarget.velocity.y = 0.0;
			outputTarget.velocity.z = 0.0;
			outputTarget.yaw_rate = 0.0;
			*/
			//Set zero for everything, just to be safe

			//Zero out position
			outputPosition.pose = currentPose.pose;

			//Zero out velocity
			outputVelocity.twist.linear.x = 0;
			outputVelocity.twist.linear.y = 0;
			outputVelocity.twist.linear.z = 0;
			outputVelocity.twist.angular.z = 0;

			//Zero out acceleration
			outputAcceleration.vector.x = 0.0;
			outputAcceleration.vector.y = 0.0;
			outputAcceleration.vector.z = 0.0;
		}

		//Publish Velocity Setpoint
		//vel_pub.publish( outputTwist );


		//Publish Data //================================================================
		pos_pub.publish( outputPosition );

		if( sendVelocity )
			vel_pub.publish( outputVelocity );

		if( sendAcceleration )
			acc_pub.publish( outputAcceleration );

		//Sleep //================================================================
		rate.sleep();
		ros::spinOnce();
	}


	//================================//
	// Shutdown                       //
	//================================//

	ROS_INFO( "Breadcrumb is shutting down" );

	if( currentState.mode == "OFFBOARD" )
		ROS_WARN( "Mav is still in OFFBOARD mode!" );

	if( currentState.armed )
		ROS_WARN( "Mav is still armed!" );


	return 0;
}

