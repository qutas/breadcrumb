#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <breadcrumb/planner.hpp>
#include <breadcrumb_interfaces/srv/request_path.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <math.h>
#include <stdint.h>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

namespace Breadcrumb {

Planner::Planner() :
	rclcpp::Node("breadcrumb_planner"),
	param_calc_sparse_(true),
	param_obstacle_threshold_(50)
{
	//Configure parameter event setup so that our node gets configured as parameters are set
	//XXX: Anything that utilizes "Planner::callback_parameters()" should be constructed in advance
	params_ = this->add_on_set_parameters_callback(std::bind(&Planner::callback_parameters, this, _1));
	// //TODO: Add descriptions
	this->declare_parameter("search_heuristic",  "euclidean");
	this->declare_parameter("allow_diagonals", true);
	this->declare_parameter("any_angle", true);
	this->declare_parameter("obstacle_threshold", param_obstacle_threshold_);
	this->declare_parameter("calc_sparse_path", param_calc_sparse_);

	sub_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>( "grid", 10, std::bind(&Planner::callback_grid, this, _1) );

	RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for occupancy grid");
}

rcl_interfaces::msg::SetParametersResult Planner::callback_parameters(const std::vector<rclcpp::Parameter> &parameters) {
	auto result = rcl_interfaces::msg::SetParametersResult();
	result.successful = true;

	for(const auto& p : parameters) {
		// RCLCPP_INFO(
		// 	this->get_logger(), "cb: Received an update to parameter \"%s\" of type: %s",
		// 	p.get_name().c_str(),
		// 	p.get_type_name().c_str()
		// );

		if (p.get_name() == "search_heuristic") {
			if(!astar_.setHeuristicByName(p.as_string())) {
				result.successful = false;
				result.reason = "Could not set heuristic method to '" + p.as_string() + "'";
				break;
			}
		} else if (p.get_name() == "any_angle") {
			astar_.setThetaStar(p.as_bool());
		} else if (p.get_name() == "allow_diagonals") {
			astar_.setDiagonalMovement(p.as_bool());
		} else if (p.get_name() == "obstacle_threshold") {
			param_obstacle_threshold_ = p.get_value<decltype(param_obstacle_threshold_)>();
		} else if (p.get_name() == "calc_sparse_path") {
			param_calc_sparse_ = p.get_value<decltype(param_calc_sparse_)>();
		} else {
			RCLCPP_WARN_STREAM(this->get_logger(), "Parameter '" << p.get_name() << "' does not support live updates");
		}
	}

	return result;
}

bool Planner::request_path(const breadcrumb_interfaces::srv::RequestPath::Request::ConstSharedPtr req, breadcrumb_interfaces::srv::RequestPath::Response::SharedPtr res) {
	res->path.header.frame_id = frame_id_;
	res->path.header.stamp = this->get_clock()->now();

	if( param_calc_sparse_ )
		res->path_sparse.header = res->path.header;

	const int64_t start_i = (int64_t)( (req->start.x - map_info_.origin.position.x) / map_info_.resolution);
	const int64_t start_j = (int64_t)( (req->start.y - map_info_.origin.position.y) / map_info_.resolution);
	const int64_t end_i = (int64_t)( (req->end.x - map_info_.origin.position.x) / map_info_.resolution);
	const int64_t end_j = (int64_t)( (req->end.y - map_info_.origin.position.y) / map_info_.resolution);

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Start/End: [" << start_i << ", " << start_j << "]; [ " << end_i << ", " << end_j << "]");

	if( ( start_i < 0 ) || ( start_i >= map_info_.width ) ||
		( start_j < 0 ) || ( start_j >= map_info_.height ) ||
		( end_i < 0 ) || ( end_i >= map_info_.width ) ||
		( end_j < 0 ) || ( end_j >= map_info_.height ) ) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Requested start/end out of bounds");

		return true;
	}

	std::vector<AStar::Vec2i> path;

	if(astar_.detectCollision({end_i, end_j})) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Requested end is within a obstacle");
		return true;
	}

	path = astar_.findPath({start_i, start_j}, {end_i, end_j});

	if(!path.size()) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Path finding failed to run!");
		return true;
	}

	if(path.size() == 1) {
		RCLCPP_DEBUG_STREAM(this->get_logger(), "1-step path detected, no planning required!");
		geometry_msgs::msg::Pose start;
		geometry_msgs::msg::Pose finish;
		start.position = req->start;
		start.orientation.w = 1;
		finish.position = req->end;
		finish.orientation.w = 1;

		res->path.poses.push_back(start);
		res->path.poses.push_back(finish);
		res->path_sparse.poses.push_back(start);
		res->path_sparse.poses.push_back(finish);

		return true;
	}

	//XXX: path.size() > 1)

	if( ( path[path.size() - 1].x != start_i ) ||
		( path[path.size() - 1].y != start_j ) ||
		( path[0].x != end_i ) ||
		( path[0].y != end_j ) ) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "No possible solution found!");
		return true;
	}

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Solution found!");

	size_t sk_last = path.size()-1;
	size_t k= path.size() - 1;

	//XXX: Loop k times
	while(true) {
		geometry_msgs::msg::Pose step;

		//Calculate position in the parent frame
		step.position.x = (path[k].x * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.x;
		step.position.y = (path[k].y * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.y;
		step.position.z = req->start.z;

		//Fill in the rotation data
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
			step.orientation = res->path.poses.back().orientation;
		}

		res->path.poses.push_back(step);

		RCLCPP_DEBUG_STREAM(this->get_logger(), "Path: " << path[k].x << ", " << path[k].y);

		//Only calculate if we should, and we're testing a final pass
		if( param_calc_sparse_ && !astar_.usingThetaStar() ) {
			if( (k > 0) && (k < sk_last) ) {
				//Calculate the angle from the last sparse step (sk_last) to k.
				double dx = path[k].x - path[sk_last].x;
				double dy = path[k].y - path[sk_last].y;
				double sang = std::atan2( dy, dx );
				double dxn = path[k-1].x - path[sk_last].x;
				double dyn = path[k-1].y - path[sk_last].y;
				double sangn = std::atan2( dyn, dxn );
				RCLCPP_DEBUG(this->get_logger(), "sparse [a,an]: [%0.2f;%0.2f]", sang, sangn);

				//If the angles aren't the same (give or take a bit)
				if( fabs(sangn - sang) > 0.001 ) {
					//Then point k is the end of the line
					res->path_sparse.poses.push_back( res->path.poses.back() );
					sk_last = k;
					RCLCPP_DEBUG_STREAM(this->get_logger(), "sparse end");
				}
			} else if(k == (path.size()-1) ) {
				//Then this is the initial point, so create the start
				res->path_sparse.poses.push_back( res->path.poses.back() );
			} else if(k == 0) {
				//Then this is the final point, so create the end
				res->path_sparse.poses.push_back( res->path.poses.back() );
			}
		}

		if(k == 0) {
			break;
		}

		k--;
	}

	if( param_calc_sparse_ ) {
		RCLCPP_DEBUG_STREAM(this->get_logger(), "Sparse solution reduced " << res->path.poses.size() << " points to " << res->path_sparse.poses.size() );
	}

	return true;
}

void Planner::callback_grid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg_in) {
	frame_id_ = msg_in->header.frame_id;
	map_info_ = msg_in->info;

    astar_.setWorldSize({msg_in->info.width, msg_in->info.height});

	//Clean up obstacles
	astar_.clearCollisions();

	//Add in the new obstacles
	for(int64_t j=0; j<msg_in->info.height; j++) {
		for(int64_t i=0; i<msg_in->info.width; i++) {
			//If the obstacle is above the acceptable threshold, add it as an obstacle
			if(msg_in->data[i + (j*msg_in->info.width)] > param_obstacle_threshold_)
				astar_.addCollision({i,j});
		}
	}

	if(!srv_request_path_) {
		srv_request_path_ = this->create_service<breadcrumb_interfaces::srv::RequestPath>("request_path", std::bind(&Planner::request_path, this, _1, _2));
		RCLCPP_INFO_STREAM(this->get_logger(), "Received first occupancy grid, path planning service started!");
	}
}

}