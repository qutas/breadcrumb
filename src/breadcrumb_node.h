#ifndef BREADCRUMB
#define BREADCRUMB

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string.h>
#include <math.h>
#include <vector>

//================================//
// Global Parameters              //
//================================//

#define MSG_FREQ 15.0
#define NAV_RATE 25.0
#define MSG_FREQ 15.0

enum navModes {
	NAV_MODE_PRECONNECT,	// Waiting for MavROS
	NAV_MODE_SLEEP,			// Connection is running (and system may or may not be broadcasting control signals)
	NAV_MODE_MISSION_START,	// Check to see if the mav is flying, initialize the waypoint procedure
	NAV_MODE_TAKEOFF,		// Set the current goal straight above the mav at takeoff height, then fly to home
	NAV_MODE_MISSION,		// Follow waypoint mission
	NAV_MODE_PAUSE,			// Waypoints are put on hold to stay at current location (could modify waypoints here)
	NAV_MODE_EXTERNAL,		// Pass over to an external navigator (tf input?)
	NAV_MODE_HOME,			// Returns to a defined home location
	NAV_MODE_LAND,			// Land at current position (then rely on the auto disarm)
	NAV_MODE_RETURN,			// Land at current position (then rely on the auto disarm)
	NAV_MODE_MISSION_END,	// Return to the home position and land (then rely on the auto disarm)
	NAV_MODE_HALT			// Stop breadcrumb, but leave the UAV in a pre-set mode (Loiter if not specified)
};



//================================//
// Global Variables               //
//================================//

unsigned int navCurrentMode;

mavros_msgs::State currentState;

geometry_msgs::Pose currentPose;
geometry_msgs::Pose navGoalHome;
geometry_msgs::Pose navGoalTakeoff;
geometry_msgs::Pose currentGoal;

geometry_msgs::TwistStamped outputTwist;	//Should have a default of 0 for all velocities
geometry_msgs::PoseStamped outputPose;	//Should have a default of 0 for all velocities

double navGoalTakeoffHeight = 1;
double waypointRadius = 0.1;
double headingAccuracy = 0.1;
std::vector<geometry_msgs::Pose> waypointList;

/*
Services:
	SET_HOME: Puts the home location where the mav currently is, rather than where it started
	START_MISSION: Starts waypoint mission
	STOP_MISSION: Gracefull passover to loiter
	PAUSE_MISSION: Pause at current point
	HALT_MISSION: Gracefull passover to emergency landing
	RETURN_TO_ORIGIN: Return back to [0,0,Z]
	RETURN_TO_START: Return back to where the mission was started [X,Y,Z]
	LOAD_MISSION: Loads the waypoint file into the planner
	DUMP_MISSION: Prints the current mission to the screen (in correct format)
*/

//================================//
// Function Prototypes            //
//================================//

void state_cb(const mavros_msgs::State::ConstPtr& msg);

void local_pos_cb(const geometry_msgs::PoseStamped msg);

geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q);

geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e);

bool comparePose( geometry_msgs::Pose goal, geometry_msgs::Pose current );

#endif
