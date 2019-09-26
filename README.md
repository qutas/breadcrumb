# breadcrumb
An A* path planner for grid-based navigation

## Download & Compile
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/breadcrumb
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

#### Updating
```sh
roscd breadcrumb
git pull
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage
This node listens for an OccupancyGrid and will advertise a service which can be called to do path planning within this grid space.

#### Running Breadcrumb
First, copy the launch file and open it to edit:
```sh
mkdir -p ~/catkin_ws/launch
cd ~/catkin_ws/launch
roscp breadcrumb breadcrumb.launch ./
nano breadcrumb.launch
```
At this point, you will have to uncomment the grid remap command to tell breadcrumb which occupancy grid to connect to:
```xml
<remap from="~grid" to="/grid" />
```

You can now run breadcrumb:
```sh
roslaunch ~/catkin_ws/breadcrumb.launch
```
If breadcrumb stops with the message `[Breadcrumb] Waiting for occupancy grid`, then it is waiting to recieve the grid. You will recieve the message `[Breadcrumb] Received a new occupancy grid, path planning service started!` when breadcrumb has successfully received the occupancy grid and is ready for operation.

#### Provided Services
- Request Path
  - Topic: `~request_path`
  - Type: `breadcrumb/RequestPath`
  - Description: This is a service server that allows a service client to query breadcrumb for a path from point A to point B. This interface only becomes available after breadcrumb has received a valid occupancy grid. The response message may return an empty path (size of 0), which represents that no solution could be found. 

#### Subscribed Topics
- Occupancy Grid
  - Topic: `grid`
  - Type: `nav_msgs/OccupancyGrid`
  - Description: This input sets the grid that is used by breadcrumb to perform the path planning steps. This should be connected to a pre-determined occupancy grid.

#### Published Topics
The breadcrumb path planner does not publish to any topics.


## Credit
This package utilizes source code developed by [github user daancode](https://github.com/daancode/a-star).
