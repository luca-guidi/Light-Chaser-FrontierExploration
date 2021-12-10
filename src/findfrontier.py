#!/usr/bin/env python

import subprocess
import rospy
import time
from actionlib_msgs.msg import GoalStatusArray
from rosgraph_msgs.msg import Log
from getfrontier import getfrontier
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

mapData=OccupancyGrid()
flag = 1
goal = 0


dest_x=2
dest_y=0
def callbackresult(msg):
	global status, flag
	print('########## MOVE_BASE STATUS ##########', flag)
	if msg.status_list:
		status = msg.status_list[0].status
		print(status)
		if status==3 or status==4:

			flag = 1
			print('FLAG', flag)



	
def callbacklog(msg):
	global goal	
	print(msg.msg)

def callbacklocation(msg):
	global tb_x, tb_y
	tb_x = msg.pose.pose.position.x
	tb_y = msg.pose.pose.position.y

def mapCallBack(data):
    	global mapData
    	mapData=data

def resolutionCallBack(res):
	global resol,ox,oy,dest_x,dest_y, goal
	start=time.time()	
	resol=res.info.resolution
	ox=res.info.origin.position.x
	oy=res.info.origin.position.y
	gridx=round((dest_x-ox)/resol)
	gridy=round((dest_y-oy)/resol)
	index=int(gridy*res.info.height-(res.info.width-gridx))
	if res.data[index]==0:
		goal=1

	end=time.time()
	print(gridx,gridy,res.data[index],goal,(start-end),"########## CHECKING IF GOAL IN MAP ###########")


def result():
	global flag,goal,dest_x,dest_y,goal
	rospy.init_node('resulty', anonymous=True)
	rospy.Subscriber('/move_base/status', GoalStatusArray, callbackresult)
	rospy.Subscriber('/rosout_agg', Log, callbacklog)
	map_topic= rospy.get_param('~map_topic','/map')
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber('/map', OccupancyGrid, resolutionCallBack) 
	rospy.Subscriber('/odom', Odometry, callbacklocation)
	time.sleep(10)

	while 1:
		if goal and flag:
			a='rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ' 
			b='{header: {stamp: now, frame_id: "map"}, '
			c= 'pose: {position: {x: '+str(dest_x)+', y: '+str(dest_y)+', z: 0.0}, orientation: {w: 1.0}}}'
			

			subprocess.Popen(a+"\'"+b+c+"\'",shell=True)
			print("***************************GOAL CAN BE SEEN*********************************")
			break
		if flag:
			
			start=time.time()
			frontier = getfrontier(mapData,tb_x,tb_y, dest_x, dest_y)
			print("GOING TO FRONTIER")
			end=time.time()
			print(end-start,"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
			a='rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ' 
			b='{header: {stamp: now, frame_id: "map"}, '
			c= 'pose: {position: {x: '+str(frontier[0][0])+', y: '+str(frontier[0][1])+', z: 0.0}, orientation: {w: 1.0}}}'


			subprocess.Popen(a+"\'"+b+c+"\'",shell=True)
			flag = 0
	

if __name__=='__main__':
	result()




