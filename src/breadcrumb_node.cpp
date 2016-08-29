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

void local_pos_cb(const geometry_msgs::PoseStamped msg) {
	currentPose = msg.pose;
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
	ros::init(argc, argv, "breadcrumb");
	ros::NodeHandle nh(ros::this_node::getName());


	//================================//
	// System Variables               //
	//================================//

	//TODO: Prepare a better list of mavros modes
	//TODO: Set a fallback mode for HALT

	//==== Topics ====//
	std::string positionInput = "position_reference";
	std::string velocityOutput = "vel_cmd";

	//==== Prepare Topics ====//
	outputTwist.header.frame_id = "fcu";
	outputPose.header.frame_id = "fcu";

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

	//==== Logic Operators ====//
	//bool changedNavMode = true;
	navCurrentMode = NAV_MODE_PRECONNECT;
	bool systemOperational = false;	//Status to see if breadcrumb should be in control
	bool startSystem = true;	//Set by a service to start breadcrumb
	bool sendTrueVel = false;	//Should be set to false when the UAV should not be moving
	bool terminate = false;

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(NAV_RATE);

	//==== PID Controllers ====//
	pid vel_x_pid(1.0/NAV_RATE);
	pid vel_y_pid(1.0/NAV_RATE);
	pid vel_z_pid(1.0/NAV_RATE);
	pid vel_h_pid(1.0/NAV_RATE);

	//================================//
	// Load Parameters                //
	//================================//

	//Input/Output
	if(!nh.getParam("position_input", positionInput)){
		ROS_WARN("No parameter set for \"position_input\", using: %s", positionInput.c_str());
	}
	ROS_INFO("Listening to: %s", positionInput.c_str());

	if(!nh.getParam("velocity_command", velocityOutput)){
		ROS_WARN("No parameter set for \"velocity_command\", using: %s", velocityOutput.c_str());
	}
	ROS_INFO("Sending commands to: %s", velocityOutput.c_str());

	//XY PID
	if(!nh.getParam("xy_pid/p", vel_x_pid.Kp)){
		ROS_WARN("No parameter set for \"xy_pid/p\", using: %0.2f", vel_x_pid.Kp);
	}

	if(!nh.getParam("xy_pid/i", vel_x_pid.Ki)){
		ROS_WARN("No parameter set for \"xy_pid/i\", using: %0.2f", vel_x_pid.Ki);
	}

	if(!nh.getParam("xy_pid/d", vel_x_pid.Kd)){
		ROS_WARN("No parameter set for \"xy_pid/d\", using: %0.2f", vel_x_pid.Kd);
	}

	if(!nh.getParam("xy_pid/min", vel_x_pid.min_output)){
		ROS_WARN("No parameter set for \"xy_pid/min\", using: %0.2f", vel_x_pid.min_output);
	}

	if(!nh.getParam("xy_pid/max", vel_x_pid.max_output)){
		ROS_WARN("No parameter set for \"xy_pid/max\", using: %0.2f", vel_x_pid.max_output);
	}

	vel_y_pid.Kp = vel_x_pid.Kp;
	vel_y_pid.Ki = vel_x_pid.Ki;
	vel_y_pid.Kd = vel_x_pid.Kd;
	vel_y_pid.min_output = vel_x_pid.min_output;
	vel_y_pid.max_output = vel_x_pid.max_output;

	ROS_INFO("Setting xy_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_x_pid.Kp, vel_x_pid.Ki, vel_x_pid.Kd, vel_x_pid.min_output, vel_x_pid.max_output);

	//Z PID
	if(!nh.getParam("z_pid/p", vel_z_pid.Kp)){
		ROS_WARN("No parameter set for \"z_pid/p\", using: %0.2f", vel_z_pid.Kp);
	}

	if(!nh.getParam("z_pid/i", vel_z_pid.Ki)){
		ROS_WARN("No parameter set for \"z_pid/i\", using: %0.2f", vel_z_pid.Ki);
	}

	if(!nh.getParam("z_pid/d", vel_z_pid.Kd)){
		ROS_WARN("No parameter set for \"z_pid/d\", using: %0.2f", vel_z_pid.Kd);
	}

	if(!nh.getParam("z_pid/min", vel_z_pid.min_output)){
		ROS_WARN("No parameter set for \"z_pid/min\", using: %0.2f", vel_z_pid.min_output);
	}

	if(!nh.getParam("z_pid/max", vel_z_pid.max_output)){
		ROS_WARN("No parameter set for \"z_pid/max\", using: %0.2f", vel_z_pid.max_output);
	}

	ROS_INFO("Setting z_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_z_pid.Kp, vel_z_pid.Ki, vel_z_pid.Kd, vel_z_pid.min_output, vel_z_pid.max_output);

	//H PID
	if(!nh.getParam("h_pid/p", vel_h_pid.Kp)){
		ROS_WARN("No parameter set for \"h_pid/p\", using: %0.2f", vel_h_pid.Kp);
	}

	if(!nh.getParam("h_pid/i", vel_h_pid.Ki)){
		ROS_WARN("No parameter set for \"h_pid/i\", using: %0.2f", vel_h_pid.Ki);
	}

	if(!nh.getParam("h_pid/d", vel_h_pid.Kd)){
		ROS_WARN("No parameter set for \"h_pid/d\", using: %0.2f", vel_h_pid.Kd);
	}

	if(!nh.getParam("h_pid/min", vel_h_pid.min_output)){
		ROS_WARN("No parameter set for \"h_pid/min\", using: %0.2f", vel_h_pid.min_output);
	}

	if(!nh.getParam("h_pid/max", vel_h_pid.max_output)){
		ROS_WARN("No parameter set for \"h_pid/max\", using: %0.2f", vel_h_pid.max_output);
	}

	ROS_INFO("Setting h_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", vel_h_pid.Kp, vel_h_pid.Ki, vel_h_pid.Kd, vel_h_pid.min_output, vel_h_pid.max_output);

	//Subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(positionInput, 10, local_pos_cb);

	//Publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
		(velocityOutput, 10);
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("/breadcrumb/goal/pose", 10);

	//Services
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("/mavros/set_mode");


	//================================//
	// Load Waypoints                 //
	//================================//

	//TODO: Load specified waypoint file if one is set

	geometry_msgs::Pose waypointFiller;
	waypointFiller.position.x = 1;
	waypointFiller.position.z = 1;
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

	int waypointCounter = -1;


	//================================//
	// Wait for Connection            //
	//================================//

	// Wait for FCU connection
    while(ros::ok() && !currentState.connected){
		ROS_INFO_THROTTLE( MSG_FREQ, "Breadcrumb is waiting for connection to mav..." );

		ros::spinOnce();
        rate.sleep();
    }

	ROS_INFO( "Connection to mav achieved!" );

	//TODO: This should be implemented into the main loop to watch for a lost connection to the mav


	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() && !terminate ){


		//================================//
		// Callbacks & Common Tasks       //
		//================================//
		ros::spinOnce();


		//================================//
		// Common Mode Tasks              //
		//================================//
		//TODO: Anything?


		//================================//
		// Mode State Machine Tasks       //
		//================================//

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

								//Set the current goal to hold position until the handover is complete (in case this is mid flight)
								currentGoal = currentPose;
								floorHeight = currentPose.position.z;

								geometry_msgs::Vector3 rot = toEuler( currentGoal.orientation );
								rot.x = 0;
								rot.y = 0;
								currentGoal.orientation = toQuaternion( rot );

								//TODO: If no home coords are manually set, then home should be above the copter.

								navGoalHome = currentGoal;
								navGoalHome.position.z += navGoalTakeoffHeight;

								ROS_WARN( "Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( currentGoal.orientation ).z );
							}
						} else {	//Else the mav is in offboard mode, so make sure it is armed
							if( ( currentState.mode == "OFFBOARD" ) && ( !currentState.armed ) ) {
								ROS_INFO( "Attempting to arm mav" );

								if( arming_client.call(arm_cmd) && arm_cmd.response.success ) {
									ROS_INFO("Mav armed");
								}
							}
						}

						lastRequest = ros::Time::now();

						//If the mav is in the right mode, and is armed
						if( ( currentState.mode == "OFFBOARD" ) && ( currentState.armed ) ) {
							ROS_INFO( "Mav is armed and listening" );
							navCurrentMode = NAV_MODE_SLEEP;
							systemOperational = true;
							startSystem = false;
							ROS_INFO( "Breadcrumb is now active" );

							ROS_WARN("DEBUG MODE SWITCH");
							navCurrentMode = NAV_MODE_TAKEOFF;
						}
					}
				} else
					ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is in standby, awaiting activation...");

				break;
			case NAV_MODE_SLEEP:
				ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is conected, awaiting commands...");
				break;
			case NAV_MODE_MISSION_START:
				// TODO:
				break;
			case NAV_MODE_TAKEOFF:
				if( !sendTrueVel ) {
					ROS_INFO("Activating velocity goal output");
					sendTrueVel = true;

					navGoalTakeoff = currentPose;
					navGoalTakeoff.position.z = navGoalTakeoffHeight;

					currentGoal = navGoalTakeoff;

					ROS_INFO("Commanding the mav to head to [TAKEOFF]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalTakeoff.position.x, navGoalTakeoff.position.y, navGoalTakeoff.position.z, toEuler( navGoalTakeoff.orientation ).z );
				}

				if( comparePose( currentGoal, currentPose ) )
					navCurrentMode = NAV_MODE_MISSION;

				break;
			case NAV_MODE_MISSION:
				if(waypointCounter == -1) {
					waypointCounter = 0;
					currentGoal = waypointList.at(waypointCounter);
					ROS_INFO("Beginning waypoint mission: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );
				}

				if( comparePose( currentGoal, currentPose ) ) {
					waypointCounter++;

					if( waypointCounter < waypointList.size() ) {
						currentGoal = waypointList.at(waypointCounter);
						ROS_INFO("Heading to next waypoint: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );
					} else {
						ROS_INFO("All waypoints have been reached...");
						waypointCounter = -1;
						navCurrentMode = NAV_MODE_HOME;
					}
				}

				// TODO: Should have a message to say when a goal was reached
			case NAV_MODE_PAUSE:
				// TODO:
				break;
			case NAV_MODE_EXTERNAL:
				// TODO:
				break;
			case NAV_MODE_HOME:
				ROS_INFO("Commanding the mav to head to [HOME]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( navGoalHome.orientation ).z );
				currentGoal = navGoalHome;

				if( comparePose( currentGoal, currentPose ) )
					navCurrentMode = NAV_MODE_LAND;

				break;
			case NAV_MODE_LAND:
				//TODO: Should check the current position to know when to cut velocity, or to disarm when on the ground.

				currentGoal = currentPose;
				currentGoal.position.z = floorHeight;
				ROS_INFO("Commanding the mav to land: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );

				if( comparePose( currentGoal, currentPose ) ) {
					ROS_INFO( "Reached landing goal!" );
					ROS_WARN( "Attempting to disarm mav" );

					//if( arming_client.call(disarm_cmd) && disarm_cmd.response.success ) {
					//	ROS_INFO("Mav disarmed");
					//}

					navCurrentMode = NAV_MODE_SLEEP;
				}
				/*
				if( set_mode_client.call(setModeLand) && setModeLand.response.success) {
					ROS_WARN( "Breadcrumb: Halting operation, landing imidiately");
				} else {
					ROS_ERROR( "Cannot enter landing mode, mav may be of control!" );
				} */

				//terminate = true;

				break;
			case NAV_MODE_RETURN:
				// TODO: RTL
				break;
			case NAV_MODE_MISSION_END:
				break;
			default:
				ROS_ERROR( "Breadcrumb: Mode was set to an invalid value [%d]", navCurrentMode);
				terminate = true;

				break;
		}

		//================================//
		// Send setpoints                 //
		//================================//
		outputTwist.header.stamp = ros::Time::now();
		outputPose.header.stamp = ros::Time::now();
		outputPose.pose = currentGoal;

		//currentGoal.position.x = 0.0;
		//currentGoal.position.y = 0.0;
		//currentGoal.position.z = 1.0;

		if( sendTrueVel ) {
			//Linear Controller
			outputTwist.twist.linear.x = vel_x_pid.step(currentGoal.position.x, currentPose.position.x);
			outputTwist.twist.linear.y = vel_x_pid.step(currentGoal.position.y, currentPose.position.y);
			outputTwist.twist.linear.z = vel_x_pid.step(currentGoal.position.z, currentPose.position.z);

			//Angular Controller
			outputTwist.twist.angular.z = vel_h_pid.step( toEuler( currentGoal.orientation ).z, toEuler( currentPose.orientation ).z );
		} else {
			ROS_WARN_THROTTLE(MSG_FREQ, "Commanding mav to hold 0 for all velocities");
			outputTwist.twist.linear.x = 0;
			outputTwist.twist.linear.y = 0;
			outputTwist.twist.linear.z = 0;
			outputTwist.twist.angular.x = 0;
			outputTwist.twist.angular.y = 0;
			outputTwist.twist.angular.z = 0;
		}

		//Publish Velocity Setpoint
		vel_pub.publish( outputTwist );
		pos_pub.publish( outputPose );


		//================================//
		// Sleep                          //
		//================================//
		rate.sleep();
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

