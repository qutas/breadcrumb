#include <ros/ros.h>

#include <breadcrumb/breadcrumb.h>
#include <breadcrumb/RequestPath.h>
#include <breadcrumb/SetDiagonals.h>
#include <breadcrumb/SetHeuristic.h>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <math.h>

Breadcrumb::Breadcrumb() :
	nh_("~"),
	topic_input_grid_("grid"),
	topic_output_array_("path"),
	flag_got_grid_(false),
	param_do_viz_(true),
	param_heuristic_("euclidean"),
	param_allow_diag_(true),
	param_obstacle_threshold_(50) {

	nh_.param("topic_reference_grid", topic_input_grid_, topic_input_grid_);
	nh_.param("topic_path_visualization", topic_output_array_, topic_output_array_);
	nh_.param("do_visualization", param_do_viz_, param_do_viz_);
	nh_.param("heuristic", param_heuristic_, param_heuristic_);
	nh_.param("allow_diagonals", param_allow_diag_, param_allow_diag_);
	nh_.param("obstacle_threshold", param_obstacle_threshold_, param_obstacle_threshold_);

	set_defaults();

	sub_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>( topic_input_grid_, 10, &Breadcrumb::callback_grid, this );
	pub_viz_ = nh_.advertise<geometry_msgs::PoseArray>( topic_output_array_, 10 );

	srv_request_path_ = nh_.advertiseService("request_path", &Breadcrumb::request_path, this);
	srv_set_diagonals_ = nh_.advertiseService("set_diagonals", &Breadcrumb::set_diagonals, this);
	srv_set_heuristic_ = nh_.advertiseService("set_heuristic", &Breadcrumb::set_heuristic, this);
}

Breadcrumb::~Breadcrumb() {
}

void Breadcrumb::set_defaults( void ) {
    astar_.setDiagonalMovement(param_allow_diag_);

	if( param_heuristic_ == "manhattan" ) {
		astar_.setHeuristic(AStar::Heuristic::manhattan);
		ROS_INFO("[BC] Using solver: %s", param_heuristic_.c_str());
	} else if( param_heuristic_ == "euclidean" ) {
		astar_.setHeuristic(AStar::Heuristic::euclidean);
		ROS_INFO("[BC] Using solver: %s", param_heuristic_.c_str());
	} else if( param_heuristic_ == "octagonal" ) {
		astar_.setHeuristic(AStar::Heuristic::octagonal);
		ROS_INFO("[BC] Using solver: %s", param_heuristic_.c_str());
	} else {
		ROS_ERROR("[BC] Unsupported heuristic, using: %s", param_heuristic_.c_str());
	}
}

bool Breadcrumb::request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res) {
	if(flag_got_grid_) {
		res.path.header.frame_id = frame_id_;
		res.path.header.stamp = ros::Time::now();

		int start_i = (int)( (req.start.x - map_info_.origin.position.x) / map_info_.resolution);
		int start_j = (int)( (req.start.y - map_info_.origin.position.y) / map_info_.resolution);
		int end_i = (int)( (req.end.x - map_info_.origin.position.x) / map_info_.resolution);
		int end_j = (int)( (req.end.y - map_info_.origin.position.y) / map_info_.resolution);

		ROS_DEBUG("[BC] Start/End: [%i, %i]; [%i, %i]", start_i, start_j, end_i, end_j);

		std::vector<AStar::Vec2i> path;

		if( ( start_i >= 0 ) && ( start_i < map_info_.width ) &&
			( start_j >= 0 ) && ( start_j < map_info_.height ) &&
			( end_i >= 0 ) && ( end_i < map_info_.width ) &&
			( end_j >= 0 ) && ( end_j < map_info_.height ) ) {

			path = astar_.findPath({start_i, start_j}, {end_i, end_j});
		} else {
			ROS_ERROR("[BC] Requested start/end out of bounds");
		}

		if(path.size() > 0) {
			if( ( path[path.size() - 1].x == start_i ) &&
				( path[path.size() - 1].y == start_j ) &&
				( path[0].x == end_i ) &&
				( path[0].y == end_j ) ) {

				ROS_INFO("[BC] Solution found!");

				for(int k=path.size()-1; k>=0; k--) {
					geometry_msgs::Pose step;

					//Calculate position in the parent frame
					step.position.x = (path[k].x * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.x;
					step.position.y = (path[k].y * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.y;
					step.position.z = 0.0;

					//Fill in thet rotation data
					if(k > 0) {
						double yaw = atan2(path[k-1].y - path[k].y, path[k-1].x - path[k].x);

						tf2::Quaternion q;
						q.setEuler(0.0, 0.0, yaw);

						step.orientation.w = q.getW();
						step.orientation.x = q.getX();
						step.orientation.y = q.getY();
						step.orientation.z = q.getZ();

					} else {
						//This is the last value in the set, use the same orientation as the second last value
						step.orientation = res.path.poses.back().orientation;
					}
					res.path.poses.push_back(step);

					ROS_DEBUG("[BC] Path: %d, %d", path[k].x, path[k].y);
				}

				if(param_do_viz_)
					pub_viz_.publish(res.path);
			} else {
				ROS_ERROR("[BC] No possible solution found!");
			}
		} else {
			ROS_ERROR("[BC] Path finding failed to run!");
		}

	} else {
		ROS_ERROR("[BC] Cannot do path planning, no map has been received!");
	}

	return true;
}

bool Breadcrumb::set_diagonals(breadcrumb::SetDiagonals::Request& req, breadcrumb::SetDiagonals::Response& res) {
    astar_.setDiagonalMovement(req.allow_diagonals);

	if(req.allow_diagonals) {
		ROS_INFO("[BC] Diagonals: allowed");
	} else {
		ROS_INFO("[BC] Diagonals: not allowed");
	}

	res.success = true;

	return true;
}

bool Breadcrumb::set_heuristic(breadcrumb::SetHeuristic::Request& req, breadcrumb::SetHeuristic::Response& res) {
	res.success = false;

	switch(req.heuristic) {
		case req.HEURISTIC_MANHATTAN: {
			astar_.setHeuristic(AStar::Heuristic::manhattan);
			res.success = true;

			ROS_INFO("[BC] Now using heuristic: manhattan");

			break;
		}
		case req.HEURISTIC_EUCLIDEAN: {
			astar_.setHeuristic(AStar::Heuristic::euclidean);
			res.success = true;

			ROS_INFO("[BC] Now using heuristic: euclidean");

			break;
		}
		case req.HEURISTIC_OCTAGONAL: {
			astar_.setHeuristic(AStar::Heuristic::octagonal);
			res.success = true;

			ROS_INFO("[BC] Now using heuristic: octagonal");

			break;
		}
		default:
			ROS_ERROR("[BC] Unsupported heuristic requested, please only use those defined in the service");
	}

	return true;
}

void Breadcrumb::callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in) {
	ROS_INFO("Received a new occupancy grid!");
	flag_got_grid_ = true;
	frame_id_ = msg_in->header.frame_id;
	map_info_ = msg_in->info;

    astar_.setWorldSize({(int)msg_in->info.width, (int)msg_in->info.height});

	//Clean up obstacles
	astar_.clearCollisions();

	//Add in the new obstacles
	for(int j=0; j<msg_in->info.height; j++) {
		for(int i=0; i<msg_in->info.width; i++) {
			//If the obstacle is above the acceptable threshold, add it as an obstacle
			if(msg_in->data[i + (j*msg_in->info.width)] > param_obstacle_threshold_)
				astar_.addCollision({i,j});
		}
	}
}
