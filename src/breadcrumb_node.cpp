/**
* @file offb_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include "breadcrumb_node.h"
#include "breadcrumb/SetMode.h"
#include "breadcrumb/Activate.h"
#include "pid.h"


//================================//
// Service Functions              //
//================================//

bool set_mode_srv(breadcrumb::SetMode::Request &req, breadcrumb::SetMode::Response &res) {
	res.success = false;

	ROS_INFO("Mode switch was requested: %s", modeNames.at( req.mode_req ).c_str() );

	//TODO: Finish off this service!
	//TODO: Only allow switch to mission mode if there are waypoints
	switch( navCurrentMode ) {
		//Don't allow switch from PRECONNECT
		case NAV_MODE_PRECONNECT:
			if( req.mode_req == NAV_MODE_SLEEP ) {
				ROS_WARN( "[NAV] Halting breadcrumb: %s", modeNames.at( navCurrentMode ).c_str() );
			} else {
				ROS_WARN( "[NAV] Refusing to switch out of mode: %s", modeNames.at( navCurrentMode ).c_str() );
			}

			break;
		//Allow switch to any mode (other than preconnect)
		case NAV_MODE_SLEEP:
			navCurrentMode = req.mode_req;
			changedMode = true;
			res.success = true;

			ROS_INFO( "[NAV] Activating navigation, switching to: %s", modeNames.at( navCurrentMode ).c_str());

			break;
		//Allow switch to SLEEP, LAND
		case NAV_MODE_TAKEOFF:
			if( ( req.mode_req == NAV_MODE_SLEEP ) || ( req.mode_req == NAV_MODE_LAND ) ) {
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;
				ROS_WARN( "[NAV] Halting takeoff, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else {
				ROS_ERROR( "[NAV] Takeoff in progress, refusing mode switch to: %s", modeNames.at( req.mode_req ).c_str());
			}

			break;
		//Allow switch to SLEEP, PAUSE, HOME, LAND
		case NAV_MODE_MISSION:
			if( ( req.mode_req == NAV_MODE_SLEEP ) || ( req.mode_req == NAV_MODE_LAND ) ) {
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;
				waypointCounter = -1;
				ROS_WARN( "[NAV] Halting mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else if( req.mode_req == NAV_MODE_PAUSE ) {
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;
				ROS_WARN( "[NAV] Pausing mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else {
				ROS_ERROR( "[NAV] Mission in progress, refusing mode switch to: %s", modeNames.at( req.mode_req ).c_str());
			} //TODO: Should always be able to quit mission

			break;
		//Allow swtich to SLEEP, MISSION, HOME, LAND
		case NAV_MODE_PAUSE:
			if( ( req.mode_req == NAV_MODE_SLEEP ) || ( req.mode_req == NAV_MODE_LAND ) ) {
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;
				waypointCounter = -1;
				ROS_WARN( "[NAV] Halting mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else if( req.mode_req == NAV_MODE_MISSION ) {
				navCurrentMode = req.mode_req;
				res.success = true;
				currentGoal = waypointList.at(waypointCounter);
				ROS_WARN( "[NAV] Resuming mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else {
				ROS_ERROR( "[NAV] Mission in progress, refusing mode switch to: %s", modeNames.at( req.mode_req ).c_str());
			}
			
			break;
		//Allow switch to SLEEP, MISSION, or LAND
		case NAV_MODE_EXTERNAL:
			navCurrentMode = req.mode_req;
			changedMode = true;
			res.success = true;

			ROS_INFO( "[NAV] Cancelling external tracking, switching to: %s", modeNames.at( navCurrentMode ).c_str());

			break;
		//Allow switch to SLEEP, MISSION, or LAND
		case NAV_MODE_HOME:
			navCurrentMode = req.mode_req;
			changedMode = true;
			res.success = true;

			ROS_INFO( "[NAV] Cancelling return to home, switching to: %s", modeNames.at( navCurrentMode ).c_str());

			break;
		//Only allow switch to SLEEP
		case NAV_MODE_LAND:
			if( req.mode_req == NAV_MODE_SLEEP ) {
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;
				ROS_WARN( "[NAV] Halting landing, switching to: %s", modeNames.at( navCurrentMode ).c_str());
			} else {
				ROS_ERROR( "[NAV] Landing in progress, refusing mode switch to: %s", modeNames.at( req.mode_req ).c_str());
			}

			break;
		//Don't allow switch from HALT
		case NAV_MODE_HALT:
			ROS_ERROR( "Breadcrumb should be exiting, can't change mode!" );

			break;
		default:
			ROS_ERROR( "[CMD] Mode has no listed value [%d]", navCurrentMode);	//Will output what mode it was checking against

			break;
	}

	return true;
}

bool activate_srv(breadcrumb::Activate::Request &req, breadcrumb::Activate::Response &res) {
	startSystem = false;
	res.success = req.ACT_ACCEPTED;

	//Only bother checing if there hasn't been a previous error
	if( ( res.success > req.ACT_DENIED_GENERIC ) && !currentState.connected ) {
		ROS_ERROR( "Refusing activation, no connection to mav has been made" );
		res.success = req.ACT_DENIED_NO_CONNECT;
	}

	if( ( res.success > req.ACT_DENIED_GENERIC ) && !inputStreamState ) {
		ROS_ERROR( "Refusing activation, waiting for state stream" );
		res.success = req.ACT_DENIED_STATE;
	}

	if( ( res.success > req.ACT_DENIED_GENERIC ) && !inputStreamPosition ) {
		ROS_ERROR( "Refusing activation, waiting for position stream" );
		res.success = req.ACT_DENIED_POSITION;
	}

	if( ( res.success > req.ACT_DENIED_GENERIC ) && ( lastRequest.nsec == 0 ) && ( lastRequest.sec == 0 ) ) {
		ROS_ERROR( "Refusing activation, system hasn't finished starting" );
		res.success = req.ACT_DENIED_GENERIC;
	}

	//If nothing is preventing activation
	if( res.success == req.ACT_ACCEPTED ) {
		startSystem = true;
		ROS_WARN( "Activating breadcrumb!" );
	}

	return true;
}


//================================//
// Callback Functions             //
//================================//

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	currentState = *msg;
}

void local_pos_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	currentPose.header = msg->header;
	currentPose.header.frame_id = msg->child_frame_id;
	
	currentPose.pose.position.x = msg->transform.translation.x;
	currentPose.pose.position.y = msg->transform.translation.y;
	currentPose.pose.position.z = msg->transform.translation.z;
	
	currentPose.pose.orientation.w = msg->transform.rotation.w;
	currentPose.pose.orientation.x = msg->transform.rotation.x;
	currentPose.pose.orientation.y = msg->transform.rotation.y;
	currentPose.pose.orientation.z = msg->transform.rotation.z;

	/*
	//Create a transform for the "base link" of the mav (One that has no roll or pitch)
	tf::Transform transformRef;
	tf::Transform transformBase;
	tf::Quaternion qBase;
	tf::Quaternion qRef;

	double heading = toEuler( currentPose.pose.orientation ).z;
	qRef.setRPY(0.0, 0.0, 0.0 );
	transformRef.setRotation( qRef );
	qBase.setRPY(0.0, 0.0, heading );
	transformBase.setRotation( qBase );

	transformBase.setOrigin( tf::Vector3( currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z ) );
	transformRef.setOrigin( tf::Vector3( currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z ) );

	tfbr->sendTransform( tf::StampedTransform(transformRef, ros::Time::now(), "world", "fcu/ref") );
	tfbr->sendTransform( tf::StampedTransform(transformBase, ros::Time::now(), "world", "fcu/base") );
	*/
}

void waypoint_cb(const geometry_msgs::PoseArray::ConstPtr& msg) {
	ROS_INFO("Received new waypoint list...");

	waypointList = msg->poses;
	waypointCounter = -1;

	ROS_INFO("Loaded in %i waypoints.", waypointList.size());
}

void ext_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	externalPose = *msg;
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
	std::string externalInput = "reference/external";

	std::string positionOutputTopic = "goal/position_target";
	std::string velocityOutputTopic = "goal/velocity_target";

	std::string waypointConfirm = "goal/waypoint_complete";
	std::string missionConfirm = "goal/mission_complete";

	bool sendVelocity = true;

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

	//==== System Settings ====//
	double navRate = 25.0;
	double stateTimeout = 5.0;
	double posTimeout = 1.0;
	long stateCounter = 0;;
	long posCounter = 0;
	std::string velFrameStr = "world";
	int velocityFrame = -1;

	//==== MavROS Interface ====//

	std::string triggerMode = "POSCTL";
	std::string passbackMode = "AUTO.LOITER";

	mavros_msgs::SetMode setModeOffB;
	//setModeOffB.request.custom_mode = "OFFBOARD";
	setModeOffB.request.custom_mode = "STABILIZED";

	mavros_msgs::SetMode setModeLand;
	setModeLand.request.custom_mode = "AUTO.LAND";
	//TODO: Put this into a parameter
	//TODO: Should autoland even be used?

	mavros_msgs::SetMode setModeLoiter;
	setModeLoiter.request.custom_mode = passbackMode;

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	mavros_msgs::CommandBool disarm_cmd;
	disarm_cmd.request.value = false;

	const std_msgs::Empty outputConfirm;

	double navGoalTakeoffHeight = 1.0;

	outputPosition.header.frame_id = "/world";

	//==== Logic Operators ====//
	//bool changedNavMode = true;
	navCurrentMode = NAV_MODE_PRECONNECT;

	ROS_INFO("[System Parameters]");

	if( !nh.getParam( "system/state_timeout", stateTimeout ) ) {
		ROS_WARN( "No parameter set for \"system/state_timeout\", using: %0.2f", stateTimeout );
	}
	ROS_INFO( "Setting state message timeout to: %0.2f", stateTimeout );

	if( !nh.getParam( "system/pos_timeout", posTimeout ) ) {
		ROS_WARN( "No parameter set for \"system/pos_timeout\", using: %0.2f", posTimeout );
	}
	ROS_INFO( "Setting position input timeout to: %0.2f", posTimeout );

	if( !nh.getParam( "system/nav_rate", navRate ) ) {
		ROS_WARN( "No parameter set for \"system/nav_rate\", using: %0.2f", navRate );
	}
	ROS_INFO( "Setting control loop rate to: %0.2f", navRate );

	if( !nh.getParam( "system/vel_frame", velFrameStr ) ) {
		ROS_WARN( "No parameter set for \"system/vel_frame\", using: %s", velFrameStr );
	}

	if(velFrameStr == "world")
		velocityFrame = VEL_FRAME_WORLD;

	if(velFrameStr == "body")
		velocityFrame = VEL_FRAME_BODY;

	if(velocityFrame == -1) {
		ROS_ERROR( "Invalid velocity output frame: %s", velFrameStr.c_str() );
		ROS_ERROR( "Using default frame!" );
		velFrameStr = "body";
		velocityFrame = VEL_FRAME_BODY;
	}

	ROS_INFO( "Setting velocity output frame to: %i (%s)", velocityFrame, velFrameStr.c_str() );

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(navRate);

	//==== PID Controllers ====//
	pid pos_x_pid(1.0/navRate);
	pid pos_y_pid(1.0/navRate);
	pid pos_z_pid(1.0/navRate);

	pid ang_h_pid(1.0/navRate);

	//================================//
	// Load Parameters                //
	//================================//
	//TODO: TF topic names
	//Input/Output //================================================================
	ROS_INFO("[Input & Output Topics]");

	if( !nh.getParam( "position_input", positionInput ) ) {
		ROS_WARN( "No parameter set for \"position_input\", using: %s", positionInput.c_str() );
	}
	ROS_INFO( "Listening for position input: %s", positionInput.c_str() );

	if( !nh.getParam( "waypoint_input", waypointInput ) ) {
		ROS_WARN( "No parameter set for \"waypoint_input\", using: %s", waypointInput.c_str() );
	}
	ROS_INFO( "Listening for waypoint input: %s", waypointInput.c_str() );
	
	if( !nh.getParam( "external_input", externalInput ) ) {
		ROS_WARN( "No parameter set for \"external_input\", using: %s", externalInput.c_str() );
	}
	ROS_INFO( "Listening for external guidance: %s", externalInput.c_str() );

	if( !nh.getParam( "reached_goal", waypointConfirm ) ) {
		ROS_WARN( "No parameter set for \"reached_goal\", using: %s", waypointConfirm.c_str() );
	}
	ROS_INFO( "Broadcasting waypoint incremental status: %s", waypointConfirm.c_str() );

	if( !nh.getParam( "mission_goal", missionConfirm ) ) {
		ROS_WARN( "No parameter set for \"mission_goal\", using: %s", missionConfirm.c_str() );
	}
	ROS_INFO( "Broadcasting mission status: %s", missionConfirm.c_str() );

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

	if( sendVelocity ) {
		ROS_INFO("[Position Controller]");

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

		//H PID //================================================================
		if( !nh.getParam( "h_pid/p", ang_h_pid.Kp) ) {
			ROS_WARN( "No parameter set for \"h_pid/p\", using: %0.2f", ang_h_pid.Kp);
		}

		if( !nh.getParam( "h_pid/i", ang_h_pid.Ki) ) {
			ROS_WARN( "No parameter set for \"h_pid/i\", using: %0.2f", ang_h_pid.Ki);
		}

		if( !nh.getParam( "h_pid/d", ang_h_pid.Kd) ) {
			ROS_WARN( "No parameter set for \"h_pid/d\", using: %0.2f", ang_h_pid.Kd);
		}

		if( !nh.getParam( "h_pid/min", ang_h_pid.min_output) ) {
			ROS_WARN( "No parameter set for \"h_pid/min\", using: %0.2f", ang_h_pid.min_output);
		}

		if( !nh.getParam( "h_pid/max", ang_h_pid.max_output) ) {
			ROS_WARN( "No parameter set for \"h_pid/max\", using: %0.2f", ang_h_pid.max_output);
		}

		ROS_INFO( "Setting h_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", ang_h_pid.Kp, ang_h_pid.Ki, ang_h_pid.Kd, ang_h_pid.min_output, ang_h_pid.max_output);
	}

	ROS_INFO("[Flight Parameters]");

	//TODO:
		//double waypointRadius = 0.1;
		//double headingAccuracy = 0.1;
		//double floorHeight = 0.0;

	bool homeSetX = false;
	bool homeSetY = false;
	bool homeSetZ = false;
	bool homeSetH = false;
	geometry_msgs::Vector3 tempHome;
	double tempHomeHdg = 0.0;

	homeSetX = nh.getParam( "guidance/home_x", tempHome.x );
	homeSetY = nh.getParam( "guidance/home_y", tempHome.y );
	homeSetZ = nh.getParam( "guidance/home_z", tempHome.z );

	if( homeSetX && homeSetY && homeSetZ ) {
		homeSet = true;

		navGoalHome.position.x = tempHome.x;
		navGoalHome.position.y = tempHome.y;
		navGoalHome.position.z = tempHome.z;

		geometry_msgs::Vector3 rot;
		nh.getParam( "guidance/home_h", rot.z );
		navGoalHome.orientation = toQuaternion( rot );

		ROS_INFO( "Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( navGoalHome.orientation ).z );
	} else {
		ROS_WARN( "No [HOME] set, will by dynamically set when mav is connected" );
	}

	if( !nh.getParam( "guidance/takeoff_height", navGoalTakeoffHeight ) ) {
		ROS_WARN( "No parameter set for \"guidance/takeoff_height\", using: %0.2f", navGoalTakeoffHeight );
	}
	ROS_INFO( "Setting takeoff height goal to: %0.2f", navGoalTakeoffHeight );


	ROS_INFO("[Finished Loading Parameters]");

	//Subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		( "/mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::TransformStamped>
		(positionInput, 10, local_pos_cb);
	ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PoseArray>
		(waypointInput, 10, waypoint_cb);
	ros::Subscriber ext_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(externalInput, 10, ext_pos_cb);

	//Publishers
	//ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
	//	(velocityOutput, 10);
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
	ros::ServiceServer set_mode_service = nh.advertiseService
		("set_mode", set_mode_srv);
	ros::ServiceServer activate_service = nh.advertiseService
		("activate", activate_srv);

	//TF
    tfbr = new tf::TransformBroadcaster();
    tfln = new tf::TransformListener();

	//================================//
	// Load Waypoints                 //
	//================================//

	//TODO: Remove this and test out the callback method

	geometry_msgs::Vector3 wp_heading;
	wp_heading.z = 0.0;

	geometry_msgs::Pose waypointFiller;
	waypointFiller.position.z = 1;
	wp_heading.z = -3*M_PI/4;
	waypointFiller.orientation = toQuaternion(wp_heading);
	waypointList.push_back(waypointFiller);
	wp_heading.z = 0.0;
	waypointFiller.orientation = toQuaternion(wp_heading);
	waypointList.push_back(waypointFiller);
	waypointFiller.position.x = 1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.y = 1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.x = -1;
	waypointList.push_back(waypointFiller);
	waypointFiller.position.x = -1;
	wp_heading.z = 1.57;
	waypointFiller.orientation = toQuaternion(wp_heading);
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
    while(ros::ok() && ( !currentState.connected || !inputStreamState || !inputStreamPosition ) ) {
		ROS_INFO_THROTTLE( MSG_FREQ, "[CMD] Breadcrumb is waiting for connection to mav..." );
		//ROS_INFO( "SEQ: %i", currentPose.header.seq );

		if( posTimeout < ( ros::Time::now() - currentPose.header.stamp ).toSec() ) {
			inputStreamPosition = false;
			posCounter = 0;
		} else {
			posCounter++;
			if( ( posCounter >= MSG_STREAM_STATE ) && !inputStreamPosition ) {
				inputStreamPosition = true;
				ROS_INFO("[CMD] Pose stream acheived!");
			}
		}

		if( stateTimeout < ( ros::Time::now() - currentState.header.stamp ).toSec() ) {
			inputStreamState = false;
			stateCounter = 0;
		} else {
			stateCounter++;
			if( (stateCounter >= MSG_STREAM_STATE) && !inputStreamState ) {
				inputStreamState = true;
				ROS_INFO("[CMD] State stream acheived!");
			}
		}

		ros::spinOnce();
        rate.sleep();
    }


	lastRequest = ros::Time::now();
	currentGoal = currentPose.pose;

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
		if( currentState.mode == triggerMode )	//TODO: Set triggerMode to "NULL" to disable this feature
			startSystem = true;

		//Check for recent messages to make sure there is a constant stream of data
		if( posTimeout < ( ros::Time::now() - currentPose.header.stamp ).toSec() ) {
			inputStreamPosition = false;
			posCounter = 0;
			ROS_ERROR_THROTTLE(MSG_FREQ, "[CMD] Timeout! No fresh positional data is avaliable!");
		} else {
			posCounter++;
			if( ( posCounter >= MSG_STREAM_STATE ) && !inputStreamPosition )
				inputStreamPosition = true;
		}

		if( stateTimeout < ( ros::Time::now() - currentState.header.stamp ).toSec() ) {
			inputStreamState = false;
			stateCounter = 0;
			ROS_ERROR_THROTTLE(MSG_FREQ, "[CMD] Timeout! No fresh system state is avaliable!");
		} else {
			stateCounter++;
			if( ( stateCounter >= MSG_STREAM_STATE ) && !inputStreamState )
				inputStreamState = true;
		}
		
		// If the system was running, but the mav has switched modes or disarmed
		//if( ( systemOperational ) && ( ( currentState.mode != "OFFBOARD" ) || ( !currentState.armed ) || !inputStreamState || !inputStreamPosition ) ) {
		if( ( systemOperational ) && ( ( currentState.mode != "STABILIZED" ) || ( !currentState.armed ) || !inputStreamState || !inputStreamPosition ) ) {
			startSystem = false;
			systemOperational = false;
			sendMovement = false;
			currentGoal = currentPose.pose;
			navCurrentMode = NAV_MODE_PRECONNECT;

			waypointCounter == -1;

			ROS_ERROR( "[CMD] The mav has changed to a non-controllable state [MODE: %s, ARMED: %d]", currentState.mode.c_str(), currentState.armed );
			ROS_WARN( "[CMD] Resetting goal to [HOME] coordinates: [%0.2f, %0.2f, %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z );
			ROS_WARN( "[CMD] Disconnecting breadcrumb, breadcrumb can be reconnected when the system has returned to a usable state" );
		}	//TODO: Get the user to reset if this is the case

		switch( navCurrentMode ) {
			case NAV_MODE_PRECONNECT:
				if( startSystem ) {
					//New request every 5 seconds
					if( ( ros::Time::now() - lastRequest ) > ros::Duration( 5.0 ) ) {
						//if( currentState.mode != "OFFBOARD" ) {	//If the mav is currently not in OFFBOARD mode
						if( currentState.mode != "STABILIZED" ) {	//If the mav is currently not in OFFBOARD mode
							//ROS_INFO( "Requesting \"OFFBOARD\" mode [%s]", currentState.mode.c_str() );
							ROS_INFO( "Requesting \"STABILIZED\" mode [%s]", currentState.mode.c_str() );

							if( set_mode_client.call(setModeOffB) && setModeOffB.response.success ) {
								ROS_INFO( "[CMD] Offboard control enabled" );
							}
						}

						//if( ( currentState.mode == "OFFBOARD" ) && ( !currentState.armed ) ) {
						if( ( currentState.mode == "STABILIZED" ) && ( !currentState.armed ) ) {
							/*
							ROS_INFO( "[CMD] Attempting to arm mav" );

							if( arming_client.call(arm_cmd) && arm_cmd.response.success ) {
								ROS_INFO( "[CMD] Mav sucessfully armed" );
							} else {
								ROS_INFO( "[CMD] Failed to arm mav" );
							}*/

							ROS_INFO_THROTTLE( 1.0, "Waiting for mav to be armed" );
						}

						lastRequest = ros::Time::now();
					}

					//If the mav is in the right mode, and is armed
					//if( ( currentState.mode == "OFFBOARD" ) && ( currentState.armed ) ) {
					if( ( currentState.mode == "STABILIZED" ) && ( currentState.armed ) ) {
						//Set the current goal to hold position until the handover is complete (in case this is mid flight)
						currentGoal = currentPose.pose;
						floorHeight = currentPose.pose.position.z;

						if( !homeSet ) {
							geometry_msgs::Vector3 rot = toEuler( currentPose.pose.orientation );

							rot.x = 0;
							rot.y = 0;
							currentGoal.orientation = toQuaternion( rot );

							navGoalHome = currentGoal;
							navGoalHome.position.z += navGoalTakeoffHeight;

							ROS_WARN( "[NAV] Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( currentGoal.orientation ).z );
						}

						ROS_INFO( "[CMD] Mav is armed and listening" );
						navCurrentMode = NAV_MODE_SLEEP;
						systemOperational = true;
						startSystem = false;
						ROS_INFO( "[CMD] Breadcrumb is now active" );

						ROS_WARN( "DEMO MODE SWITCH [TAKEOFF]" );
						navCurrentMode = NAV_MODE_TAKEOFF; //TODO: Should just not be here
						changedMode = true;
					}
				} else {
					ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is in standby, awaiting activation..." );
				}

				break;
			case NAV_MODE_SLEEP:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;
					currentGoal = currentPose.pose;
					ROS_INFO( "[NAV] Commanding to hold position..." );
				}

				ROS_INFO_THROTTLE(MSG_FREQ, "Breadcrumb is conected, awaiting commands..." );

				break;
			case NAV_MODE_TAKEOFF:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					ROS_INFO("Attempting to takeoff...");

					if( !sendMovement ) { //If the mav is disabled, enable outputs
						ROS_INFO( "Activating goal output" );
						sendMovement = true;

						navGoalTakeoff = currentPose.pose;
						navGoalTakeoff.position.z = navGoalTakeoffHeight;

						currentGoal = navGoalTakeoff;

						ROS_INFO( "[NAV] Commanding the mav to head to [TAKEOFF]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalTakeoff.position.x, navGoalTakeoff.position.y, navGoalTakeoff.position.z, toEuler( navGoalTakeoff.orientation ).z );
					} else { //Motor commands are active, but the mav might be on the ground?
						ROS_WARN("The mav is already being controlled (and it seems like it should be flying!");
						ROS_WARN("Commanding to hold position!");
						currentGoal = currentPose.pose;
					}

					changedMode = false;
				}

				if( comparePose( currentGoal, currentPose.pose ) ) {
					ROS_INFO( "Reached takeoff goal!" );
					navCurrentMode = NAV_MODE_MISSION;	//TODO: Should be NAV_MODE_SLEEP, or maybe Home?
					changedMode = true;
				}

				break;
			case NAV_MODE_MISSION:
				if( changedMode ) {	//Reset the waypoint counter if switching to mission mode
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
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

						navCurrentMode = NAV_MODE_HOME;	//TODO: Should be sleep
						changedMode = true;

						ROS_INFO( "All waypoints have been reached..." );
					}
				}

				// TODO: Should use actionlib to do waypoints?
				break;
			case NAV_MODE_PAUSE:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;
					currentGoal = currentPose.pose;
					ROS_INFO( "[NAV] Commanding to hold position..." );
				}

				ROS_INFO_THROTTLE(MSG_FREQ, "Mission paused, waiting for resume..." );
				
				break;
			case NAV_MODE_EXTERNAL:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;
					ROS_INFO( "[NAV] Commanding to follow external pose goal..." );
				}
				
				if( 2.0 < ( ros::Time::now() - externalPose.header.stamp ).toSec() ) {
					ROS_INFO( "[NAV] Rejecting external mode, no fresh external goal" );
					changedMode = true;
					navCurrentMode = NAV_MODE_SLEEP;
				} else {
					currentGoal = externalPose.pose;
				}
				
				break;
			case NAV_MODE_HOME:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					ROS_INFO( "Commanding the mav to head to [HOME]: [%0.2f, %0.2f, %0.2f; %0.2f]", navGoalHome.position.x, navGoalHome.position.y, navGoalHome.position.z, toEuler( navGoalHome.orientation ).z );
					changedMode = false;
				}

				currentGoal = navGoalHome;

				if( comparePose( currentGoal, currentPose.pose ) ) {
					changedMode = true;
					navCurrentMode = NAV_MODE_LAND;	//TODO: Should be sleep
				}

				break;
			case NAV_MODE_LAND:
				//TODO: Should check the current position to know when to cut velocity, or to disarm when on the ground.
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );

					currentGoal = currentPose.pose;
					currentGoal.position.z = floorHeight;

					ROS_INFO( "Commanding the mav to land: [%0.2f, %0.2f, %0.2f; %0.2f]", currentGoal.position.x, currentGoal.position.y, currentGoal.position.z, toEuler( currentGoal.orientation ).z );

					changedMode = false;
				}

				if( comparePose( currentGoal, currentPose.pose ) ) {
					ROS_INFO_THROTTLE( 2.0, "Reached landing goal!" );
					
					changedMode = true;
					navCurrentMode = NAV_MODE_HALT;
					
					/* TODO: Could be useful, but out of scope
					ROS_WARN( "[CMD] Attempting to disarm mav" );
					if( arming_client.call(disarm_cmd) && disarm_cmd.response.success ) {
						ROS_INFO( "[CMD] Mav disarmed" );
					} else
						ROS_ERROR( "[CMD] Something went wrong, mav won't disarm" );
					*/
					//sendMovement = false;
					//currentGoal = currentPose.pose;
					//changedMode = true;
					//navCurrentMode = NAV_MODE_SLEEP;
				}
				/*
				if( set_mode_client.call(setModeLand) && setModeLand.response.success) {
					ROS_WARN( "Breadcrumb: Halting operation, landing imidiately" );
				} else {
					ROS_ERROR( "Cannot enter landing mode, mav may be of control!" );
				} */

				//terminate = true;

				break;
			case NAV_MODE_HALT:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;
				}

				terminate = true;

				break;
			default:
				ROS_ERROR( "[NAV] Mode was set to an invalid value [%d]", navCurrentMode);
				terminate = true;

				break;
		}


		//Generate Commands //================================================================
		ros::Time timestamp = ros::Time::now();

		outputPosition.header.stamp = timestamp;
		outputVelocity.header.stamp = timestamp;

		tf::Transform transformGoal;
		tf::poseMsgToTF(currentGoal, transformGoal);
		tfbr->sendTransform( tf::StampedTransform(transformGoal, ros::Time::now(), "world", "breadcrumb/goal") );

		tf::StampedTransform transformGoalBody;

		try {
			//Get the latest pose of the fcu in the world
			tfln->lookupTransform("/uav/hdg_link", "/breadcrumb/goal", ros::Time(0), transformGoalBody);
		}

		catch (tf::TransformException ex) {
			ROS_ERROR( "%s",ex.what() );
			ros::Duration( 1.0 ).sleep();
			sendMovement = false;
		}

		geometry_msgs::Pose currentGoalBody;
		tf::poseTFToMsg(transformGoalBody, currentGoalBody);

		if( sendMovement ) {

			//Position
			outputPosition.pose = currentGoal;

			//Velocity
			if( sendVelocity ) {	// Only bother to calculate if the requested
				//WorldFrame
				//outputVelocity.twist.linear.x = pos_x_pid.step(currentGoal.position.x, currentPose.pose.position.x);
				//outputVelocity.twist.linear.y = pos_y_pid.step(currentGoal.position.y, currentPose.pose.position.y);
				//outputVelocity.twist.linear.z = pos_z_pid.step(currentGoal.position.z, currentPose.pose.position.z);

				//Body Frame
				outputVelocity.twist.linear.x = pos_x_pid.step(currentGoalBody.position.x, 0.0);
				outputVelocity.twist.linear.y = pos_y_pid.step(currentGoalBody.position.y, 0.0);
				outputVelocity.twist.linear.z = pos_z_pid.step(currentGoalBody.position.z, 0.0);

				double goalHeading = toEuler( currentGoal.orientation ).z;
				double currentHeading = toEuler( currentPose.pose.orientation ).z;

				if( ( goalHeading - currentHeading ) < -M_PI )
					goalHeading += 2 * M_PI;

				if( ( goalHeading - currentHeading ) > M_PI )
					goalHeading -= 2 * M_PI;

				outputVelocity.twist.angular.z = ang_h_pid.step( goalHeading, currentHeading);
			}
		} else {
			ROS_WARN_THROTTLE(MSG_FREQ, "[NAV] Commanding the mav to stay still..." );
			//Zero out position
			outputPosition.pose = currentGoal;

			//Zero out velocity
			outputVelocity.twist.linear.x = 0.0;
			outputVelocity.twist.linear.y = 0.0;
			outputVelocity.twist.linear.z = 0.0;
			outputVelocity.twist.angular.z = 0.0;
		}

		//Publish Data //================================================================
		pos_pub.publish( outputPosition );

		if( sendVelocity ) {
			//TODO: global velocity

			vel_pub.publish( outputVelocity );
		}
		//Sleep //================================================================
		rate.sleep();
		ros::spinOnce();
	}


	//================================//
	// Shutdown                       //
	//================================//

	ROS_INFO( "[CMD] Breadcrumb is shutting down" );

	if( currentState.mode == "OFFBOARD" )
		ROS_WARN( "[CMD] Mav is still in OFFBOARD mode!" );

	if( currentState.armed )
		ROS_WARN( "[CMD] Mav is still armed!" );


	return 0;
}

