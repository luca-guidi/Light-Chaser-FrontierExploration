#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

#-----------------------------------------------------

def getfrontier(mapData,tb_x,tb_y, dest_x, dest_y):

	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	 
	img = np.zeros((h, w, 1), np.uint8)
	
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]==100:
				img[i,j]=0
			elif data[i*w+j]==0:
				img[i,j]=255
			elif data[i*w+j]==-1:
				img[i,j]=205
	
	
       	o=cv2.inRange(img,0,1)
	edges = cv2.Canny(img,0,255)
	im2, contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 5)
	o=cv2.bitwise_not(o) 
	res = cv2.bitwise_and(o,edges)
	#------------------------------

	frontier=copy(res)
	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	all_pts=[]
	if len(contours)>0:
		upto=len(contours)-1
		i=0
		maxx=0
		maxind=0
		
		for i in range(0,len(contours)):
				cnt = contours[i]
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				xr=cx*resolution+Xstartx
				yr=cy*resolution+Xstarty
				pt=[np.array([xr,yr])]
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
							
					all_pts=pt


	frontier_cost = np.empty([len(all_pts),1])
	for i,frontier in enumerate(all_pts):
		#frontier_tb_dist = ((frontier[0]-tb_x)**2 + (frontier[1]-tb_y)**2)**(1/2)
		frontier_tb_dist = np.linalg.norm(frontier-[tb_x,tb_y])

		#frontier_dest_dist = ((frontier[0]-dest_x)**2 + (frontier[1]-dest_y)**2)**(1/2)
		frontier_dest_dist = np.linalg.norm(frontier-[dest_x,dest_y])

		frontier_cost[i]=(frontier_tb_dist+frontier_dest_dist)
	
	
	frontiers = sorted(np.hstack((all_pts,frontier_cost)),key=lambda x:x[2])
	#frontiers = sorted(all_pts, lambda x:x[2])
	return frontiers


