#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <breadcrumb/RequestPath.h>
#include <breadcrumb/AStarParamsConfig.h>

#include <breadcrumb/AStar.h>

#include <string>

class Breadcrumb {
	private:
		ros::NodeHandle nhp_;

		ros::Subscriber sub_grid_;
		ros::ServiceServer srv_request_path_;

		dynamic_reconfigure::Server<breadcrumb::AStarParamsConfig> dyncfg_settings_;

		std::string topic_input_grid_;
		std::string topic_output_array_;

		nav_msgs::MapMetaData map_info_;
		std::string frame_id_;
		AStar::Generator astar_;

		bool flag_got_grid_;
		int param_obstacle_threshold_;
		bool param_calc_sparse_;

	public:
		Breadcrumb( void );

		~Breadcrumb( void );

	private:
		void callback_cfg_settings( breadcrumb::AStarParamsConfig &config, uint32_t level );

		void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in);

		bool request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res);
};
