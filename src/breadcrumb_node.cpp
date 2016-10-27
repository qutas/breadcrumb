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

	//If the user is requesting PRECONNECT, deny it
	if( req.mode_req == NAV_MODE_PRECONNECT ) {	//If PRECONNECT is requested
		ROS_ERROR( "[NAV] Refusing to switch to: PRECONNECT" );
	} else if ( req.mode_req == NAV_MODE_HALT ) {	//If HALT is requested
		navCurrentMode = req.mode_req;
		changedMode = true;
		res.success = true;

		ROS_INFO( "[NAV] Halting imidiately!" );
	} else if ( req.mode_req == NAV_MODE_FAILSAFE ) {	//If FAILSAFE is requested
		ROS_ERROR( "[NAV] Refusing to switch to: FAILSAFE" );
	} else if ( ( req.mode_req == NAV_MODE_PAUSE ) && ( navCurrentMode != NAV_MODE_MISSION ) ) {
		ROS_ERROR( "[NAV] Refusing to switch to: PAUSE" );
	} else {	//Check what mode the UAV is in for specifics on how to handle the request
		switch( navCurrentMode ) {
			//Don't allow switch from PRECONNECT
			case NAV_MODE_PRECONNECT:
				ROS_ERROR( "[NAV] Refusing to switch out of mode: %s", modeNames.at( navCurrentMode ).c_str() );

				break;
			//Allow switch to any mode (other than preconnect and failsafe)
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
			//Allow switch to cancel the mission to any mode, or PAUSE
			case NAV_MODE_MISSION:
				if( req.mode_req == NAV_MODE_PAUSE ) {
					navCurrentMode = req.mode_req;
					changedMode = true;
					res.success = true;
					ROS_WARN( "[NAV] Pausing mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
				} else {
					navCurrentMode = req.mode_req;
					changedMode = true;
					res.success = true;
					waypointCounter = -1;
					ROS_WARN( "[NAV] Halting mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
				}

				break;
			//Allow swtich to cancel the mission to any mode, or resume MISSION
			case NAV_MODE_PAUSE:
				if( req.mode_req == NAV_MODE_MISSION ) {
					res.success = true;
					missionResume = true;
					ROS_WARN( "[NAV] Resuming mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
				} else {
					navCurrentMode = req.mode_req;
					changedMode = true;
					res.success = true;
					waypointCounter = -1;
					ROS_WARN( "[NAV] Halting mission, switching to: %s", modeNames.at( navCurrentMode ).c_str());
				}

				break;
			//Allow switch to any mode (other than preconnect)
			case NAV_MODE_EXTERNAL:
				navCurrentMode = req.mode_req;
				changedMode = true;
				res.success = true;

				ROS_INFO( "[NAV] Cancelling external tracking, switching to: %s", modeNames.at( navCurrentMode ).c_str());

				break;
			//Allow switch to any mode (other than preconnect)
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
			case NAV_MODE_FAILSAFE:
				ROS_ERROR( "Cannot mode switch from failsafe, try activating!" );

				break;
			//Don't allow switch from HALT
			case NAV_MODE_HALT:
				ROS_ERROR( "Breadcrumb should be exiting, can't change mode!" );

				break;
			default:
				ROS_ERROR( "[CMD] Mode has no listed value [%d]", navCurrentMode);	//Will output what mode it was checking against

				break;
		}
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

void state_cb( const mavros_msgs::State::ConstPtr& msg ) {
	currentState = *msg;
}

void waypoint_cb( const geometry_msgs::PoseArray::ConstPtr& msg ) {
	ROS_INFO( "Received new waypoint list..." );

	//TODO: Might have an array of transforms instead
	waypointList = msg->poses;
	waypointCounter = -1;

	ROS_INFO( "Loaded in %lu waypoints.", ( unsigned long )waypointList.size() );
}

void ext_pos_cb( const geometry_msgs::PoseStamped::ConstPtr& msg ) {
	bool use_ext = false;

	//If the quaternion is normalized
	if( ( ros::Time::now() - msg->header.stamp ).toSec() < 2.0 ) {
		double q_len = std::sqrt( ( msg->pose.orientation.w * msg->pose.orientation.w ) + ( msg->pose.orientation.x * msg->pose.orientation.x ) + ( msg->pose.orientation.y * msg->pose.orientation.y ) + ( msg->pose.orientation.z * msg->pose.orientation.z ) );

		//If the quaternion is normalized
		if( q_len > 0.98 ) {
			use_ext = true;
		} else {
			ROS_WARN_THROTTLE(2.0, "[NAV] Ignoring external guidance: Quaternion not normalized");
		}
	} else {
		ROS_WARN_THROTTLE(2.0, "[NAV] Ignoring external guidance: Timestamp is too old");
	}

	if( use_ext ) {
		externalPose = *msg;
	}
}

bool comparePositionHeading( const tf::Transform& goal, const tf::Transform& current ) {
	bool reached = false;

	//Check the distance from the current position to the goal position
	if( fabs( current.getOrigin().distance( goal.getOrigin() ) ) < waypointRadius ) {
		//Get RPY from current and goal
		geometry_msgs::Vector3 rotGoal;
		tf::Matrix3x3(goal.getRotation()).getRPY(rotGoal.x, rotGoal.y, rotGoal.z);

		geometry_msgs::Vector3 rotCurrent;
		tf::Matrix3x3(current.getRotation()).getRPY(rotCurrent.x, rotCurrent.y, rotCurrent.z);

		//Compare the current rotation to the goal rotation
		if( fabs( rotCurrent.z - rotGoal.z ) <= headingAccuracy ) {
			reached = true;
		}
	}

	return reached;
}

void setPositionHeading( tf::Transform& goal, const tf::Transform& current ) {
	goal.setOrigin( current.getOrigin() );

	tf::Quaternion rot;
	double roll, pitch, yaw;
	tf::Matrix3x3( current.getRotation() ).getRPY( roll, pitch, yaw );

	rot.setRPY( 0.0, 0.0, yaw);
	goal.setRotation( rot );
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
	//TODO: Set a passback mode for HALT

	//==== Topics ====//
	std::string param_input_waypoint = "reference/wapoints";
	std::string param_input_external = "reference/external";

	std::string param_tf_world = "world";
	std::string param_tf_body = "uav/hdg_link";
	std::string param_tf_goal = "breadcrumb_goal";

	std::string param_output_goal_velocity = "goal/velocity_target";

	std::string param_output_confirm_waypont = "goal/waypoint_complete";
	std::string param_output_confirm_mission = "goal/mission_complete";

	bool sendVelocity = true;

	//==== System Settings ====//
	double param_system_nav_rate = 25.0;
	double param_system_timeout_state = 5.0;
	double param_system_timeout_external = 1.0;
	long stateCounter = 0;;
	long externalPositionCounter = 0;
	std::string param_system_frame_type_vel = "world";
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


	//==== Navigation Settings ====//
	double navGoalTakeoffHeight = 1.0;

	tf::Transform goalTransform;
	tf::Transform homeTransform;

	//================================//
	// Load Parameters                //
	//================================//

	//==== Logic Operators ====//
	navCurrentMode = NAV_MODE_PRECONNECT;

	ROS_INFO("[System Parameters]");

	if( !nh.getParam( "system/state_timeout", param_system_timeout_state ) ) {
		ROS_WARN( "No parameter set for \"system/state_timeout\", using: %0.2f", param_system_timeout_state );
	}
	ROS_INFO( "Setting state message timeout to: %0.2f", param_system_timeout_state );

	if( !nh.getParam( "system/ext_timeout", param_system_timeout_external ) ) {
		ROS_WARN( "No parameter set for \"system/ext_timeout\", using: %0.2f", param_system_timeout_external );
	}
	ROS_INFO( "Setting position input timeout to: %0.2f", param_system_timeout_external );

	if( !nh.getParam( "system/nav_rate", param_system_nav_rate ) ) {
		ROS_WARN( "No parameter set for \"system/nav_rate\", using: %0.2f", param_system_nav_rate );
	}
	ROS_INFO( "Setting control loop rate to: %0.2f", param_system_nav_rate );

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(param_system_nav_rate);

	//==== PID Controllers ====//
	pid param_pid_pos_x(1.0/param_system_nav_rate);
	pid param_pid_pos_y(1.0/param_system_nav_rate);
	pid param_pid_pos_z(1.0/param_system_nav_rate);

	pid param_pid_hdg(1.0/param_system_nav_rate);

	//Input/Output //================================================================
	ROS_INFO("[Input & Output Topics]");

	if( !nh.getParam( "tf/world_frame", param_tf_world ) ) {
		ROS_WARN( "No parameter set for \"tf/world_frame\", using: %s", param_tf_world.c_str() );
	}
	ROS_INFO( "Setting TF world frame: %s", param_tf_world.c_str() );

	if( !nh.getParam( "tf/body_frame", param_tf_body ) ) {
		ROS_WARN( "No parameter set for \"tf/body_frame\", using: %s", param_tf_body.c_str() );
	}
	ROS_INFO( "Setting TF body frame: %s", param_tf_body.c_str() );

	if( !nh.getParam( "tf/goal_frame", param_tf_goal ) ) {
		ROS_WARN( "No parameter set for \"tf/goal_frame\", using: %s", param_tf_goal.c_str() );
	}
	ROS_INFO( "Setting TF goal frame: %s", param_tf_goal.c_str() );

	//TODO: Allow goal waypoints and external input to be given in different frames

	if( !nh.getParam( "waypoint_input", param_input_waypoint ) ) {
		ROS_WARN( "No parameter set for \"waypoint_input\", using: %s", param_input_waypoint.c_str() );
	}
	ROS_INFO( "Listening for waypoint input: %s", param_input_waypoint.c_str() );

	if( !nh.getParam( "external_input", param_input_external ) ) {
		ROS_WARN( "No parameter set for \"external_input\", using: %s", param_input_external.c_str() );
	}
	ROS_INFO( "Listening for external guidance: %s", param_input_external.c_str() );

	if( !nh.getParam( "reached_goal", param_output_confirm_waypont ) ) {
		ROS_WARN( "No parameter set for \"reached_goal\", using: %s", param_output_confirm_waypont.c_str() );
	}
	ROS_INFO( "Broadcasting waypoint incremental status: %s", param_output_confirm_waypont.c_str() );

	if( !nh.getParam( "mission_goal", param_output_confirm_mission ) ) {
		ROS_WARN( "No parameter set for \"mission_goal\", using: %s", param_output_confirm_mission.c_str() );
	}
	ROS_INFO( "Broadcasting mission status: %s", param_output_confirm_mission.c_str() );

	if( !nh.getParam( "velocity_goal", param_output_goal_velocity ) ) {
		ROS_WARN( "No parameter set for \"velocity_goal\", disabling velocity output" );
		sendVelocity = false;
	} else {
		ROS_INFO( "Sending velocity commands to: %s", param_output_goal_velocity.c_str() );
	}

	if( sendVelocity ) {
		if( !nh.getParam( "system/vel_frame", param_system_frame_type_vel ) ) {
		ROS_WARN( "No parameter set for \"system/vel_frame\", using: %s", param_system_frame_type_vel.c_str() );
		}

		if(param_system_frame_type_vel == "world")
			velocityFrame = VEL_FRAME_WORLD;

		if(param_system_frame_type_vel == "body")
			velocityFrame = VEL_FRAME_BODY;

		if(velocityFrame == -1) {
			ROS_ERROR( "Invalid velocity output frame: %s", param_system_frame_type_vel.c_str() );
			ROS_ERROR( "Using default frame!" );
			param_system_frame_type_vel = "body";
			velocityFrame = VEL_FRAME_BODY;
		}

		ROS_INFO( "Setting velocity output frame to: %i (%s)", velocityFrame, param_system_frame_type_vel.c_str() );


		ROS_INFO("[Position Controller]");

		//POS XY PID //================================================================
		if( !nh.getParam( "pos_xy_pid/p", param_pid_pos_x.Kp) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/p\", using: %0.2f", param_pid_pos_x.Kp);
		}

		if( !nh.getParam( "pos_xy_pid/i", param_pid_pos_x.Ki) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/i\", using: %0.2f", param_pid_pos_x.Ki);
		}

		if( !nh.getParam( "pos_xy_pid/d", param_pid_pos_x.Kd) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/d\", using: %0.2f", param_pid_pos_x.Kd);
		}

		if( !nh.getParam( "pos_xy_pid/min", param_pid_pos_x.min_output) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/min\", using: %0.2f", param_pid_pos_x.min_output);
		}

		if( !nh.getParam( "pos_xy_pid/max", param_pid_pos_x.max_output) ) {
			ROS_WARN( "No parameter set for \"pos_xy_pid/max\", using: %0.2f", param_pid_pos_x.max_output);
		}

		param_pid_pos_y.Kp = param_pid_pos_x.Kp;
		param_pid_pos_y.Ki = param_pid_pos_x.Ki;
		param_pid_pos_y.Kd = param_pid_pos_x.Kd;
		param_pid_pos_y.min_output = param_pid_pos_x.min_output;
		param_pid_pos_y.max_output = param_pid_pos_x.max_output;

		ROS_INFO( "Setting pos_xy_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", param_pid_pos_x.Kp, param_pid_pos_x.Ki, param_pid_pos_x.Kd, param_pid_pos_x.min_output, param_pid_pos_x.max_output);

		//POS Z PID //================================================================
		if( !nh.getParam( "pos_z_pid/p", param_pid_pos_z.Kp) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/p\", using: %0.2f", param_pid_pos_z.Kp);
		}

		if( !nh.getParam( "pos_z_pid/i", param_pid_pos_z.Ki) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/i\", using: %0.2f", param_pid_pos_z.Ki);
		}

		if( !nh.getParam( "pos_z_pid/d", param_pid_pos_z.Kd) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/d\", using: %0.2f", param_pid_pos_z.Kd);
		}

		if( !nh.getParam( "pos_z_pid/min", param_pid_pos_z.min_output) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/min\", using: %0.2f", param_pid_pos_z.min_output);
		}

		if( !nh.getParam( "pos_z_pid/max", param_pid_pos_z.max_output) ) {
			ROS_WARN( "No parameter set for \"pos_z_pid/max\", using: %0.2f", param_pid_pos_z.max_output);
		}

		ROS_INFO( "Setting pos_z_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", param_pid_pos_z.Kp, param_pid_pos_z.Ki, param_pid_pos_z.Kd, param_pid_pos_z.min_output, param_pid_pos_z.max_output);

		//H PID //================================================================
		if( !nh.getParam( "h_pid/p", param_pid_hdg.Kp) ) {
			ROS_WARN( "No parameter set for \"h_pid/p\", using: %0.2f", param_pid_hdg.Kp);
		}

		if( !nh.getParam( "h_pid/i", param_pid_hdg.Ki) ) {
			ROS_WARN( "No parameter set for \"h_pid/i\", using: %0.2f", param_pid_hdg.Ki);
		}

		if( !nh.getParam( "h_pid/d", param_pid_hdg.Kd) ) {
			ROS_WARN( "No parameter set for \"h_pid/d\", using: %0.2f", param_pid_hdg.Kd);
		}

		if( !nh.getParam( "h_pid/min", param_pid_hdg.min_output) ) {
			ROS_WARN( "No parameter set for \"h_pid/min\", using: %0.2f", param_pid_hdg.min_output);
		}

		if( !nh.getParam( "h_pid/max", param_pid_hdg.max_output) ) {
			ROS_WARN( "No parameter set for \"h_pid/max\", using: %0.2f", param_pid_hdg.max_output);
		}

		ROS_INFO( "Setting h_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", param_pid_hdg.Kp, param_pid_hdg.Ki, param_pid_hdg.Kd, param_pid_hdg.min_output, param_pid_hdg.max_output);
	}

	ROS_INFO("[Flight Parameters]");

	bool homeSetX = false;
	bool homeSetY = false;
	bool homeSetZ = false;
	bool homeSetH = false;
	double tempHomeX = 0.0;
	double tempHomeY = 0.0;
	double tempHomeZ = 0.0;
	double tempHomeHdg = 0.0;

	homeSetX = nh.getParam( "guidance/home_x", tempHomeX );
	homeSetY = nh.getParam( "guidance/home_y", tempHomeY );
	homeSetZ = nh.getParam( "guidance/home_z", tempHomeZ );
	homeSetH = nh.getParam( "guidance/home_h", tempHomeHdg );

	if( homeSetX && homeSetY && homeSetZ && homeSetH ) {
		homeSet = true;

		homeTransform.setOrigin( tf::Vector3( tempHomeX, tempHomeY, tempHomeZ ) );
		tf::Quaternion rot;
		rot.setRPY( 0.0, 0.0, tempHomeHdg );
		homeTransform.setRotation( rot );

		ROS_INFO( "Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f; %0.2f]", tempHomeX, tempHomeY, tempHomeZ, tempHomeHdg );
	} else {
		ROS_WARN( "No [HOME] set, will by dynamically set when mav is connected" );
	}

	if( !nh.getParam( "guidance/takeoff_height", navGoalTakeoffHeight ) ) {
		ROS_WARN( "No parameter set for \"guidance/takeoff_height\", using: %0.2f", navGoalTakeoffHeight );
	}
	ROS_INFO( "Setting takeoff height goal to: %0.2f", navGoalTakeoffHeight );

	if( !nh.getParam( "guidance/wp_radius", waypointRadius ) ) {
		ROS_WARN( "No parameter set for \"guidance/wp_radius\", using: %0.2f", waypointRadius );
	}
	ROS_INFO( "Setting waypoint radius to: %0.2f", waypointRadius );

	if( !nh.getParam( "guidance/hdg_acc", headingAccuracy ) ) {
		ROS_WARN( "No parameter set for \"guidance/hdg_acc\", using: %0.2f", headingAccuracy );
	}
	ROS_INFO( "Setting heading accuracy to: %0.2f", headingAccuracy );

	ROS_INFO("[Finished Loading Parameters]");

	//Subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		( "/mavros/state", 10, state_cb);
	ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PoseArray>
		(param_input_waypoint, 10, waypoint_cb);
	ros::Subscriber ext_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(param_input_external, 10, ext_pos_cb);

	//Publishers
	//ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
	//	(velocityOutput, 10);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
		(param_output_goal_velocity, 10);

	ros::Publisher wp_confirm_pub = nh.advertise<std_msgs::Empty>
		(param_output_confirm_waypont, 1);	//Only really want this to be sent or read once at a time.
	ros::Publisher mission_confirm_pub = nh.advertise<std_msgs::Empty>
		(param_output_confirm_mission, 1);

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
    tf::TransformBroadcaster tfbr;
	tf::TransformListener tfln;


	//================================//
	// Wait for Connection            //
	//================================//

	// Wait for FCU connection
	ROS_INFO( "[CMD] Breadcrumb is waiting for connection to mav..." );

    while(ros::ok() && ( !currentState.connected || !inputStreamState || !inputStreamPosition ) ) {
		//Wait for the transform to be available
		if( tfln.waitForTransform( param_tf_world, param_tf_body, ros::Time::now(), ros::Duration( 0.5 ) ) ) {
			if( !inputStreamPosition )
				ROS_INFO("[CMD] Position stream acheived!");

			inputStreamPosition = true;
		} else {
			if( inputStreamPosition )
				ROS_INFO("[CMD] Position stream lost!");

			inputStreamPosition = false;
		}

		//Wait for mavros state to be avaiable
		if( ( ros::Time::now() - currentState.header.stamp ) < ros::Duration( param_system_timeout_state ) ) {
			stateCounter++;

			if( (stateCounter >= MSG_STREAM_STATE) && !inputStreamState ) {
				inputStreamState = true;
				ROS_INFO("[CMD] State stream acheived!");
			}
		} else {
			if( inputStreamState )
				ROS_INFO("[CMD] State stream lost!");

			inputStreamState = false;
			stateCounter = 0;
		}

		ros::spinOnce();
        rate.sleep();
    }

	lastRequest = ros::Time::now();

	ROS_INFO( "Breadcrumb is in standby, awaiting activation..." );

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() && !terminate ) {
		tf::Transform currentTransform;
		ros::Time timestamp = ros::Time::now();

		//Flight Control State Machine //================================================================
		//Allow remote starting of the system
		//TODO: Anything?
		//if( currentState.mode == triggerMode )	//TODO: Set triggerMode to "NULL" to disable this feature
		//	startSystem = true;

		//Check for recent messages to make sure there is a constant stream of data

		if( tfln.waitForTransform( param_tf_body, param_tf_world, timestamp, ros::Duration( param_system_nav_rate ) ) ) {
			if( !inputStreamPosition )
				ROS_INFO("[CMD] Position stream acheived!");

			inputStreamPosition = true;

			tf::StampedTransform currentTransformStamped;

			try {
				tfln.lookupTransform(param_tf_body, param_tf_world, timestamp, currentTransformStamped);
				currentTransform.setRotation( currentTransformStamped.getRotation() );
				currentTransform.setOrigin( currentTransformStamped.getOrigin() );

			} catch (tf::TransformException ex ) {
				  ROS_ERROR( "%s", ex.what() );
			}
		} else {
			if( inputStreamPosition )
				ROS_INFO("[CMD] Position stream lost!");

			inputStreamPosition = false;
		}

		//Wait for mavros state to be avaiable
		if( ( ros::Time::now() - currentState.header.stamp ) < ros::Duration( param_system_timeout_state ) ) {
			stateCounter++;

			if( (stateCounter >= MSG_STREAM_STATE) && !inputStreamState ) {
				inputStreamState = true;
				ROS_INFO("[CMD] State stream acheived!");
			}
		} else {
			if( inputStreamState )
				ROS_INFO("[CMD] State stream lost!");

			inputStreamState = false;
			stateCounter = 0;
		}

		if( ( ros::Time::now() - externalPose.header.stamp ) < ros::Duration( param_system_timeout_external ) ) {
			externalPositionCounter++;

			if( ( externalPositionCounter >= MSG_STREAM_EXT ) && !inputStreamExternal ) {
				inputStreamExternal = true;
				ROS_INFO("[CMD] External guidance stream acheived!");
			}
		} else {
			if( inputStreamExternal )
				ROS_INFO("[CMD] External guidance stream lost!");

			inputStreamExternal = false;
			externalPositionCounter = 0;
		}

		// If the system was running, but the mav has switched modes or disarmed
		//if( ( systemOperational ) && ( ( currentState.mode != "OFFBOARD" ) || ( !currentState.armed ) || !inputStreamState || !inputStreamPosition ) ) {
		if( ( systemOperational ) && ( ( currentState.mode != "STABILIZED" ) || ( !currentState.armed ) || !inputStreamState || !inputStreamPosition ) ) {
			startSystem = false;
			systemOperational = false;

			navCurrentMode = NAV_MODE_FAILSAFE;
			changedMode = true;

			waypointCounter == -1;

			ROS_ERROR( "[CMD] The mav has changed to a non-controllable state [MODE: %s, ARMED: %d]", currentState.mode.c_str(), currentState.armed );
		}

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

							ROS_INFO( "Waiting for mav to be armed" );
						}

						lastRequest = ros::Time::now();
					}

					//If the mav is in the right mode, and is armed
					//if( ( currentState.mode == "OFFBOARD" ) && ( currentState.armed ) ) {
					if( ( currentState.mode == "STABILIZED" ) && ( currentState.armed ) ) {
						//Set the current goal to hold position until the handover is complete (in case this is mid flight)
						setPositionHeading( goalTransform, currentTransform );

						if( !homeSet ) {
							//Set HOME to the current location + takeoff height
							homeTransform = goalTransform;
							homeTransform.setOrigin( homeTransform.getOrigin() + tf::Vector3( 0.0, 0.0, navGoalTakeoffHeight ) );

							ROS_WARN( "[NAV] Setting [HOME] coordinates to: [%0.2f, %0.2f, %0.2f]", homeTransform.getOrigin().getX(), homeTransform.getOrigin().getY(), homeTransform.getOrigin().getZ() );
						}

						floorHeight = currentTransform.getOrigin().getZ();

						ROS_INFO( "[CMD] Mav is armed and listening" );
						navCurrentMode = NAV_MODE_SLEEP;
						changedMode = true;

						systemOperational = true;
						startSystem = false;
						ROS_INFO( "[CMD] Breadcrumb is now active" );
					}
				}

				break;
			case NAV_MODE_SLEEP:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;

					setPositionHeading( goalTransform, currentTransform );

					ROS_INFO( "[NAV] Commanding to hold position..." );
				}

				break;
			case NAV_MODE_TAKEOFF:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					ROS_INFO("Attempting to takeoff...");

					if( !sendMovement ) { //If the mav is disabled, enable outputs
						ROS_INFO( "Activating goal output" );
						sendMovement = true;

						setPositionHeading( goalTransform, currentTransform );
						goalTransform.setOrigin( goalTransform.getOrigin() + tf::Vector3( 0.0, 0.0, navGoalTakeoffHeight ) );

						ROS_INFO( "[NAV] Commanding the mav to head to [TAKEOFF]: [%0.2f, %0.2f, %0.2f]", goalTransform.getOrigin().getX(), goalTransform.getOrigin().getY(), goalTransform.getOrigin().getZ() );
					} else { //Motor commands are active, but the mav might be on the ground?
						ROS_WARN("The mav is already being controlled (and it seems like it should be flying)!");
						ROS_WARN("Commanding to hold position!");
						setPositionHeading( goalTransform, currentTransform );
					}

					changedMode = false;
				}

				if( comparePositionHeading( goalTransform, currentTransform ) ) {
					ROS_INFO( "Reached takeoff goal!" );
					navCurrentMode = NAV_MODE_SLEEP;
					changedMode = true;
				}

				break;
			case NAV_MODE_MISSION:
				if( changedMode ) {	//Reset the waypoint counter if switching to mission mode
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );

					waypointCounter = -1;

					if( waypointList.size() > 0 ) {
						ROS_INFO("Begining waypoint mission...");

						waypointCounter = 0;
						tf::poseMsgToTF(waypointList.at(waypointCounter), goalTransform);
						ROS_INFO( "Heading to next waypoint: [%0.2f, %0.2f, %0.2f]", goalTransform.getOrigin().getX(), goalTransform.getOrigin().getY(), goalTransform.getOrigin().getZ() );
					} else {
						ROS_ERROR( "[NAV] No waypoints are loaded, cannot start mission!" );
					}

					changedMode = false;
				}

				if( waypointCounter > -1 ) {
					if( comparePositionHeading( goalTransform, currentTransform ) ) {
						waypointCounter++;

						if( waypointCounter < waypointList.size() ) {
							tf::poseMsgToTF(waypointList.at(waypointCounter), goalTransform);

							wp_confirm_pub.publish( outputConfirm );

							ROS_INFO( "Heading to next waypoint: [%0.2f, %0.2f, %0.2f]", goalTransform.getOrigin().getX(), goalTransform.getOrigin().getY(), goalTransform.getOrigin().getZ() );
						} else {
							waypointCounter = -1;

							mission_confirm_pub.publish( outputConfirm );
							ROS_INFO( "All waypoints have been reached!" );
						}
					}
				}

				if( waypointCounter == -1 ) {
					navCurrentMode = NAV_MODE_SLEEP;
					changedMode = true;
				}

				// TODO: Should use actionlib to do waypoints?

				break;
			case NAV_MODE_PAUSE:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;

					setPositionHeading( goalTransform, currentTransform );
					ROS_INFO( "[NAV] Commanding to hold position..." );


					ROS_INFO( "Mission paused, waiting for resume..." );
				}

				//Pass back to mission mode
				if( missionResume ) {
					tf::poseMsgToTF(waypointList.at(waypointCounter), goalTransform);
					navCurrentMode = NAV_MODE_MISSION;

					missionResume = false;
				}


				break;
			case NAV_MODE_EXTERNAL:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;
					ROS_INFO( "[NAV] Commanding to follow external pose goal..." );
				}

				if( inputStreamExternal ) {
					//TODO: Try to get rid of this?
					tf::poseMsgToTF(externalPose.pose, goalTransform);
				} else {
					ROS_INFO( "[NAV] Rejecting external mode, no fresh external goal" );
					changedMode = true;
					navCurrentMode = NAV_MODE_SLEEP;
				}

				break;
			case NAV_MODE_HOME:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;

					goalTransform = homeTransform;
					ROS_INFO( "Commanding the mav to head to [HOME]" );
				}

				if( comparePositionHeading( goalTransform, currentTransform ) ) {
					changedMode = true;
					navCurrentMode = NAV_MODE_SLEEP;
				}

				break;
			case NAV_MODE_LAND:
				//TODO: Should check the current position to know when to cut velocity, or to disarm when on the ground.
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );

					//Set the current goal to where the UAV is, but at the floor height
					setPositionHeading( goalTransform, currentTransform );

					tf::Vector3 landPoint( goalTransform.getOrigin() );
					landPoint.setZ( floorHeight );
					goalTransform.setOrigin( landPoint );

					ROS_INFO( "Commanding the mav to land at current location..." );

					changedMode = false;
				}

				if( comparePositionHeading( goalTransform, currentTransform ) ) {
					ROS_INFO( "Reached landing goal!" );

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
			case NAV_MODE_FAILSAFE:
				if( changedMode ) {
					ROS_WARN("[NAV] Entered %s mode", modeNames.at( navCurrentMode ).c_str() );
					changedMode = false;

					sendMovement = false;
					ROS_WARN( "[NAV] Disabling navigation!" );
				}


				ROS_INFO_THROTTLE( MSG_FREQ, "[NAV] Waiting for user to reactivate the system!" );

				if( startSystem ) {
					systemOperational = true;
					startSystem = false;

					setPositionHeading( goalTransform, currentTransform );
					ROS_WARN( "[CMD] Setting goal current coordinates: [%0.2f, %0.2f, %0.2f]", currentTransform.getOrigin().getX(), currentTransform.getOrigin().getY(), currentTransform.getOrigin().getZ() );

					sendMovement = true;
					ROS_WARN( "[NAV] Enabling navigation!" );

					navCurrentMode = NAV_MODE_SLEEP;
					changedMode = true;
				}

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
		geometry_msgs::TwistStamped outputVelocity;
		outputVelocity.header.stamp = timestamp;

		if( sendMovement ) {

			tfbr.sendTransform( tf::StampedTransform(goalTransform, timestamp, param_tf_world, param_tf_goal) );

			tf::StampedTransform transformGoalBody;

			try {
				//Get the latest pose of the fcu in the world
				tfln.waitForTransform(param_tf_body, param_tf_goal, timestamp, ros::Duration(0.1));
				tfln.lookupTransform(param_tf_body, param_tf_goal, timestamp, transformGoalBody);
			}

			catch (tf::TransformException ex) {
				ROS_ERROR( "%s",ex.what() );
				ros::Duration( 1.0 ).sleep();
				sendMovement = false;
			}

			geometry_msgs::Pose currentGoalBody;
			tf::poseTFToMsg(transformGoalBody, currentGoalBody);


			//Velocity
			if( sendVelocity ) {	// Only bother to calculate if the requested
			/* TODO: ALL BELOW
				//TODO: WorldFrame
				//outputVelocity.twist.linear.x = param_pid_pos_x.step(currentGoal.position.x, currentPose.pose.position.x);
				//outputVelocity.twist.linear.y = param_pid_pos_y.step(currentGoal.position.y, currentPose.pose.position.y);
				//outputVelocity.twist.linear.z = param_pid_pos_z.step(currentGoal.position.z, currentPose.pose.position.z);

				//TODO:Body Frame
				outputVelocity.twist.linear.x = param_pid_pos_x.step(currentGoalBody.position.x, 0.0);
				outputVelocity.twist.linear.y = param_pid_pos_y.step(currentGoalBody.position.y, 0.0);
				outputVelocity.twist.linear.z = param_pid_pos_z.step(currentGoalBody.position.z, 0.0);

				//TODO: TF HERE
				//geometry_msgs::Vector3 rot;
				//tf::Matrix3x3(currentTransform.getRotation()).getRPY(rot.x, rot.y, rot.z);

				double goalHeading = toEuler( currentGoal.orientation ).z;
				double currentHeading = toEuler( currentPose.pose.orientation ).z;

				if( ( goalHeading - currentHeading ) < -M_PI )
					goalHeading += 2 * M_PI;

				if( ( goalHeading - currentHeading ) > M_PI )
					goalHeading -= 2 * M_PI;

				outputVelocity.twist.angular.z = param_pid_hdg.step( goalHeading, currentHeading);
				*/
			}
		} else {
			ROS_WARN_THROTTLE(MSG_FREQ, "[NAV] Commanding the mav to stay still..." );

			//Zero out velocity
			if( sendVelocity ) {
				outputVelocity.twist.linear.x = 0.0;
				outputVelocity.twist.linear.y = 0.0;
				outputVelocity.twist.linear.z = 0.0;
				outputVelocity.twist.angular.z = 0.0;
			}
		}

		//Publish Data //=========================================================
		if( sendVelocity ) {
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

	//if( currentState.mode == "OFFBOARD" )
	//	ROS_WARN( "[CMD] Mav is still in OFFBOARD mode!" );

	if( currentState.armed )
		ROS_WARN( "[CMD] Mav is still armed!" );

	ros::shutdown();

	return 0;
}

