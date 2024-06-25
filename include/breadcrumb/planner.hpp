#pragma once
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <breadcrumb_interfaces/srv/request_path.hpp>
#include <breadcrumb/AStar.hpp>

#include <string>
#include <string_view>

namespace Breadcrumb {

class Planner : public rclcpp::Node {
	private:
	    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
		rclcpp::Service<breadcrumb_interfaces::srv::RequestPath>::SharedPtr srv_request_path_;
		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_;
		std::string topic_input_grid_;
		std::string topic_output_array_;

		nav_msgs::msg::MapMetaData map_info_;
		std::string frame_id_;
		AStar::Generator astar_;

		bool param_calc_sparse_;
		int64_t param_obstacle_threshold_;
		

	public:
		Planner();

	private:
		void callback_grid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg_in);

		bool request_path(const breadcrumb_interfaces::srv::RequestPath::Request::ConstSharedPtr req, breadcrumb_interfaces::srv::RequestPath::Response::SharedPtr res);

		rcl_interfaces::msg::SetParametersResult callback_parameters(const std::vector<rclcpp::Parameter> &parameters);
};

}