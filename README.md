# Light-Chaser-FrontierExploration

To run the project, three separate terminals are needed. For each of them, run the following commands: 

##### Terminal 1: 
This command must be run so that ROS nodes can communicate.

```
roscore
```


##### Terminal 2:

First SSH into the Raspberry Pi on the TurtleBot3 Burger. 
```
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
```
Next, bring up packages that are needed to start TurtleBot3 applications.
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```


##### Terminal 3:
```
roslaunch light_chaser light_chaser.launch
```


Part of the getfrontier() function was adapted from 
```
H. Umari and S. Mukhopadhyay, "Autonomous robotic exploration based on multiple rapidly-exploring randomized trees," 
2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017, pp. 1396-1402, doi: 10.1109/IROS.2017.8202319.
```
Their Github repository can be found [here](https://github.com/hasauino/rrt_exploration)
