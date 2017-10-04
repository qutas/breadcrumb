#include <ros/ros.h>
#include <breadcrumb/breadcrumb.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "breadcrumb");
	Breadcrumb bc;

	ros::spin();

	return 0;
}
