# Light-Chaser-FrontierExploration

To run the project, three separate terminals are needed. For each of them, run the following commands: 

#### Terminal 1: 
This command must be run so that ROS nodes can communicate.

```
roscore
```


#### Terminal 2:

First SSH into the Raspberry Pi on the TurtleBot3 Burger. 
```
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
```
Next, bring up packages that are needed to start TurtleBot3 applications.
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```


#### Terminal 3:
```
roslaunch light_chaser light_chaser.launch
```
---

## Abstract

Light Chaser is a ROS package to allow [TurtleBot](https://www.turtlebot.com/turtlebot3/) style robots to navigate in unknown, unmapped cluttered environments autonomously. 

The package uses Lidar to get information about the environment. A modified version of the A* search algorithm powers the autonomous navigation anchored on _Frontier Points_. _SLAM_ is also going on in the background while the robot navigates.


![Frontiers and SLAM](/assets/img1.png)

The image above shows different points of navigation taking place. Red marks are the frontiers that act as anchor points for navigation. The green line is Lidar's current vision. Black lines are the environment borders. You can also see the tiny TurtleBot moving around.
---

Part of the getfrontier() function was adapted from 
```
H. Umari and S. Mukhopadhyay, "Autonomous robotic exploration based on multiple rapidly-exploring randomized trees," 
2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017, pp. 1396-1402, doi: 10.1109/IROS.2017.8202319.
```
Their Github repository can be found [here](https://github.com/hasauino/rrt_exploration)
