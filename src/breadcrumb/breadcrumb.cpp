#include <ros/ros.h>

#include <breadcrumb/breadcrumb.h>
#include <breadcrumb/RequestPath.h>
#include <breadcrumb/AStarParamsConfig.h>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <math.h>

Breadcrumb::Breadcrumb() :
	nh_(),
	nhp_("~"),
	flag_got_grid_(false),
	dyncfg_settings_(nhp_) {

	dyncfg_settings_.setCallback(boost::bind(&Breadcrumb::callback_cfg_settings, this, _1, _2));

	sub_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>( "grid", 10, &Breadcrumb::callback_grid, this );

	ROS_INFO("[Breadcrumb] Waiting for path");
}

Breadcrumb::~Breadcrumb() {
}

void Breadcrumb::callback_cfg_settings( breadcrumb::AStarParamsConfig &config, uint32_t level ) {
	param_obstacle_threshold_ = config.obstacle_threshold;

	astar_.setDiagonalMovement(config.allow_diagonals);

	switch(config.search_heuristic) {
		case 0: {
			astar_.setHeuristic(AStar::Heuristic::manhattan);
			break;
		}
		case 1: {
			astar_.setHeuristic(AStar::Heuristic::euclidean);
			break;
		}
		case 2: {
			astar_.setHeuristic(AStar::Heuristic::octagonal);
			break;
		}
		default: {
			ROS_ERROR("[Breadcrumb] Error setting heuristic, reverting to 'Manhattan'");
			astar_.setHeuristic(AStar::Heuristic::manhattan);
		}
	};
}

bool Breadcrumb::request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res) {
	res.path.header.frame_id = frame_id_;
	res.path.header.stamp = ros::Time::now();

	int start_i = (int)( (req.start.x - map_info_.origin.position.x) / map_info_.resolution);
	int start_j = (int)( (req.start.y - map_info_.origin.position.y) / map_info_.resolution);
	int end_i = (int)( (req.end.x - map_info_.origin.position.x) / map_info_.resolution);
	int end_j = (int)( (req.end.y - map_info_.origin.position.y) / map_info_.resolution);

	ROS_DEBUG("[Breadcrumb] Start/End: [%i, %i]; [%i, %i]", start_i, start_j, end_i, end_j);

	std::vector<AStar::Vec2i> path;

	if( ( start_i >= 0 ) && ( start_i < map_info_.width ) &&
		( start_j >= 0 ) && ( start_j < map_info_.height ) &&
		( end_i >= 0 ) && ( end_i < map_info_.width ) &&
		( end_j >= 0 ) && ( end_j < map_info_.height ) ) {

		path = astar_.findPath({start_i, start_j}, {end_i, end_j});
	} else {
		ROS_ERROR("[Breadcrumb] Requested start/end out of bounds");
	}

	if(path.size() > 1) {
		if( ( path[path.size() - 1].x == start_i ) &&
			( path[path.size() - 1].y == start_j ) &&
			( path[0].x == end_i ) &&
			( path[0].y == end_j ) ) {

			ROS_INFO("[Breadcrumb] Solution found!");

			for(int k=path.size()-1; k>=0; k--) {
				geometry_msgs::Pose step;

				//Calculate position in the parent frame
				step.position.x = (path[k].x * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.x;
				step.position.y = (path[k].y * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.y;
				step.position.z = req.start.z;

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

				ROS_DEBUG("[Breadcrumb] Path: %d, %d", path[k].x, path[k].y);
			}
		} else {
			ROS_ERROR("[Breadcrumb] No possible solution found!");
		}
	} else if(path.size() == 1) {
		ROS_WARN("[Breadcrumb] 1-step path detected, no planning required!");
	} else {
		ROS_ERROR("[Breadcrumb] Path finding failed to run!");
	}

	return true;
}

void Breadcrumb::callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in) {
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

	if(!flag_got_grid_) {
		flag_got_grid_ = true;
		srv_request_path_ = nhp_.advertiseService("request_path", &Breadcrumb::request_path, this);
	}

	ROS_INFO("[Breadcrumb] Received a new occupancy grid, path planning service started!");
}
