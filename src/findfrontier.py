"""Frontier based navigation"""
#!/usr/bin/env python

import subprocess
import time
import rospy
from actionlib_msgs.msg import GoalStatusArray
from rosgraph_msgs.msg import Log
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from getfrontier import get_frontier

# constants
DEST_X = 2  # relative X coordinate of target
DEST_Y = 0  # relative Y coordinate of target

# variables
map_data = OccupancyGrid()
move_flag = 1  # indicates if turtlebot can move
goal_visible = 0  # indicates if goal is visible


def callback_result(msg: any) -> None:
    """Displays current status of robot"""
    global status, move_flag

    # Statues are robot actions : navigation, scanning, etc
    print("########## MOVE_BASE STATUS ##########", move_flag)
    if msg.status_list:
        status = msg.status_list[0].status
        print(status)

        # Indicates robot is ready to move
        if status == 3 or status == 4:
            move_flag = 1
            print("FLAG", move_flag)


def callback_log(msg: any) -> None:
    """Displays navigation goal visibility"""
    global goal_visible
    print(msg.msg)


def callback_location(msg: any) -> None:
    """Updates current turtlebot location"""
    global tb_x, tb_y  # turtlebot x , y
    tb_x = msg.pose.pose.position.x
    tb_y = msg.pose.pose.position.y


def callback_map(data: any) -> None:
    """Updates current map data"""
    global map_data
    map_data = data


def callback_resolution(res: any) -> None:
    """Checks for goal in visible map"""
    global resolution, origin_x, origin_y, DEST_X, DEST_Y, goal_visible

    start_time = time.time()
    resolution = res.info.resolution  # changes as more area is explored
    origin_x = res.info.origin.position.x  # Relative X origin
    origin_y = res.info.origin.position.y  # Relative Y origin
    grid_x = round((DEST_X - origin_x) / resolution)
    grid_y = round((DEST_Y - origin_y) / resolution)
    index = int(grid_y * res.info.height - (res.info.width - grid_x))

    # Navigation goal present in scanned region
    if res.data[index] == 0:
        goal_visible = 1

    end_time = time.time()
    print(
        grid_x,
        grid_y,
        res.data[index],
        goal_visible,
        (start_time - end_time),
        "########## CHECKING IF GOAL IN MAP ###########",
    )


def result() -> None:
    """main logic"""
    global move_flag, goal_visible, DEST_X, DEST_Y

    # Subscribing to relevant topics
    rospy.init_node("resulty", anonymous=True)
    rospy.Subscriber("/move_base/status", GoalStatusArray, callback_result)
    rospy.Subscriber("/rosout_agg", Log, callback_log)
    map_topic = rospy.get_param("~map_topic", "/map")
    rospy.Subscriber(map_topic, OccupancyGrid, callback_map)
    rospy.Subscriber("/map", OccupancyGrid, callback_resolution)
    rospy.Subscriber("/odom", Odometry, callback_location)
    time.sleep(10)

    # Main Logic
    while True:
        # Goal is visible and permission to navigate
        # Publishes goal to navigation stack
        if goal_visible and move_flag:
            str_a = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "
            str_b = '{header: {stamp: now, frame_id: "map"}, '
            str_c = (
                "pose: {position: {x: "
                + str(DEST_X)
                + ", y: "
                + str(DEST_Y)
                + ", z: 0.0}, orientation: {w: 1.0}}}"
            )

            subprocess.Popen(str_a + "'" + str_b + str_c + "'", shell=True)
            print(
                "***************************GOAL CAN BE SEEN*********************************"
            )
            break

        # Permission to move
        # Publishes intermediate goal to navigation stack
        if move_flag:
            start_time = time.time()
            frontier = get_frontier(map_data, tb_x, tb_y, DEST_X, DEST_Y)
            print("GOING TO FRONTIER")
            end_time = time.time()
            print(
                end_time - start_time, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
            )
            str_a = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "
            str_b = '{header: {stamp: now, frame_id: "map"}, '
            str_c = (
                "pose: {position: {x: "
                + str(frontier[0][0])  # Intermediate goal X coordinate
                + ", y: "
                + str(frontier[0][1])  # Intermediate goal Y coordinate
                + ", z: 0.0}, orientation: {w: 1.0}}}"
            )

            subprocess.Popen(str_a + "'" + str_b + str_c + "'", shell=True)
            move_flag = 0


if __name__ == "__main__":
    result()
