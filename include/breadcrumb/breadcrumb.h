#pragma once

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <breadcrumb/RequestPath.h>
#include <breadcrumb/SetDiagonals.h>
#include <breadcrumb/SetHeuristic.h>

#include <AStar/AStar.h>

#include <string>

class Breadcrumb {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_grid_;
		ros::Publisher pub_viz_;
		ros::ServiceServer srv_request_path_;
		ros::ServiceServer srv_set_diagonals_;
		ros::ServiceServer srv_set_heuristic_;

		std::string topic_input_grid_;
		std::string topic_output_array_;

		nav_msgs::MapMetaData map_info_;
		std::string frame_id_;
		AStar::Generator astar_;

		bool flag_got_grid_;
		bool param_do_viz_;
		std::string param_heuristic_;
		bool param_allow_diag_;
		int param_obstacle_threshold_;

	public:
		Breadcrumb( void );

		~Breadcrumb( void );

		void set_defaults( void );

		void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in);

		bool request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res);
		bool set_diagonals(breadcrumb::SetDiagonals::Request& req, breadcrumb::SetDiagonals::Response& res);
		bool set_heuristic(breadcrumb::SetHeuristic::Request& req, breadcrumb::SetHeuristic::Response& res);
};
