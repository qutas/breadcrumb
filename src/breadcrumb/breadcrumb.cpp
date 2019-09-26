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
	nhp_("~"),
	flag_got_grid_(false),
	param_calc_sparse_(false),
	dyncfg_settings_(nhp_) {

	dyncfg_settings_.setCallback(boost::bind(&Breadcrumb::callback_cfg_settings, this, _1, _2));

	sub_grid_ = nhp_.subscribe<nav_msgs::OccupancyGrid>( "grid", 10, &Breadcrumb::callback_grid, this );

	ROS_INFO("[Breadcrumb] Waiting for occupancy grid");
}

Breadcrumb::~Breadcrumb() {
}

void Breadcrumb::callback_cfg_settings( breadcrumb::AStarParamsConfig &config, uint32_t level ) {
	param_obstacle_threshold_ = config.obstacle_threshold;
	param_calc_sparse_ = config.calc_sparse_path;

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

	if( param_calc_sparse_ )
		res.path_sparse.header = res.path.header;

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

			int sk_last = path.size()-1;

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

				//Only calculate if we should, and we're testing a final pass
				if( param_calc_sparse_ ) {
					if( (k > 0) && (k < sk_last) ) {
						//Calculate the angle from the last sparse step (sk_last) to k.
						double dx = path[k].x - path[sk_last].x;
						double dy = path[k].y - path[sk_last].y;
						double sang = std::atan2( dy, dx );
						double dxn = path[k-1].x - path[sk_last].x;
						double dyn = path[k-1].y - path[sk_last].y;
						double sangn = std::atan2( dyn, dxn );
						ROS_DEBUG("[Breadcrumb] sparse [a,an]: [%0.2f;%0.2f]", sang, sangn);

						//If the angles aren't the same (give or take a bit)
						if( fabs(sangn - sang) > 0.001 ) {
							//Then point k is the end of the line
							res.path_sparse.poses.push_back( res.path.poses.back() );
							sk_last = k;
							ROS_DEBUG("[Breadcrumb] sparse end");
						}
					} else if(k == (path.size()-1) ) {
						//Then this is the initial point, so create the start
						res.path_sparse.poses.push_back( res.path.poses.back() );
					} else if(k == 0) {
						//Then this is the final point, so create the end
						res.path_sparse.poses.push_back( res.path.poses.back() );
					}
				}
			}

			if( param_calc_sparse_ ) {
				ROS_INFO("[Breadcrumb] Sparse solution reduced %d points to %d", (int)res.path.poses.size(), (int)res.path_sparse.poses.size() );
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
