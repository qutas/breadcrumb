#ifndef BREADCRUMB
#define BREADCRUMB

#include <ros/ros.h>

//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>

#include <std_msgs/Empty.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//#include <mavros_msgs/PositionTarget.h>

#include <string.h>
#include <math.h>
#include <vector>

//================================//
// Global Parameters              //
//================================//

#define MSG_FREQ 15.0
#define MSG_STREAM_POS 20
#define MSG_STREAM_STATE 5

enum navModes {
	NAV_MODE_PRECONNECT = 0,// Waiting for MavROS
	NAV_MODE_SLEEP,			// Connection is running, command to hold position (and system may or may not be broadcasting control signals)
	NAV_MODE_TAKEOFF,		// Set the current goal straight above the mav at takeoff height, then fly to home
	NAV_MODE_MISSION,		// Follow waypoint mission
	NAV_MODE_PAUSE,			// Waypoints are put on hold to stay at current location (could modify waypoints here)
	NAV_MODE_HOME,			// Returns to a defined home location
	NAV_MODE_LAND,			// Land at current position (then rely on the auto disarm)
	NAV_MODE_HALT			// Stop breadcrumb, but leave the UAV in a pre-set mode (Loiter if not specified)
};

const std::vector<std::string> modeNames = {
	"PRECONNECT",
	"SLEEP",
	"TAKEOFF",
	"MISSION",
	"PAUSE",
	"HOME",
	"LAND",
	"HALT"
};

//These should be in a top-level controller, not here
//	NAV_MODE_MISSION_START,	// Check to see if the mav is flying, initialize the waypoint procedure
//	NAV_MODE_MISSION_END,	// Return to the home position and land (then rely on the auto disarm)


//Mission control
	//somehow receive the mission (probably a service
	//could use actionlib to provide mission feedback
	//proper implamentation would cause a sharp difference between this and external
//Mixed mission
	//missions are passed over rostopic
	//each time the message is received, the missions is refreshed)
	//no need for external as this could be acheived by sending just one waypoint at a time
	//could still have start, stop, and pause control via services
	//could make a relatively simple actionlib interface on top of this as a seperate node
	//use the poseArray message, as that's a relatively standard and controlled format
//Pure external
	//only accept the current goal over rostopic



//================================//
// Global Variables               //
//================================//

unsigned int navCurrentMode;

mavros_msgs::State currentState;
//mavros_msgs::PositionTarget outputTarget;

//geometry_msgs::PoseStamped outputTwist;
geometry_msgs::Vector3Stamped outputAcceleration;
geometry_msgs::TwistStamped outputVelocity;
geometry_msgs::PoseStamped outputPosition;

geometry_msgs::PoseStamped currentPose;
geometry_msgs::Twist currentVelocity;
geometry_msgs::Pose navGoalHome;
geometry_msgs::Pose navGoalTakeoff;
geometry_msgs::Pose currentGoal;

//geometry_msgs::TwistStamped outputTwist;	//Should have a default of 0 for all velocities
geometry_msgs::PoseStamped outputPose;	//Should have a default of 0 for all velocities

double waypointRadius = 0.1;
double headingAccuracy = 0.1;
double floorHeight = 0.0;
std::vector<geometry_msgs::Pose> waypointList;
int waypointCounter = -1;

bool systemOperational = false;	//Status to see if breadcrumb should be in control
bool startSystem = true;	//Set by a service to start breadcrumb
bool homeSet = false;		//Used to track if the user has manually set a home location
bool sendMovement = false;	//Should be set to false when the UAV should not be moving
bool terminate = false;
bool changedMode = false;	//Should be set on each mode change to allow gracefull passover
bool inputStreamPosition = false;	//Set to true when there haven't been any fresh inputs, will cause panic
bool inputStreamState = false;	//Set to true when there haven't been any fresh inputs, will cause panic


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
