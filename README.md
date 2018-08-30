# breadcrumb
An A* pathplanner for grid-based navigation

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
This node listens for an OccupancyGrid and will advertise a service which can be called to do path planning within this grid space

## Credit
This package utilizes source code developed by [github user daancode](https://github.com/daancode/a-star).
