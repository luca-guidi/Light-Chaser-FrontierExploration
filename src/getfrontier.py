#!/usr/bin/env python
"""Returns frontier list"""


from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

# -----------------------------------------------------
# Adapted from source mentioned in README


def get_frontier(map_data, tb_x, tb_y, dest_x, dest_y) -> list:
    """Finds frontiers of current map"""
    data = map_data.data
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x  # Relative X origin
    origin_y = map_data.info.origin.position.y  # Relative Y origin

    # Init map display
    img = np.zeros((height, width, 1), np.uint8)

    # Sketches the map information
    for i in range(height):
        for j in range(width):
            # Location known and boundry
            if data[i * width + j] == 100:
                img[i, j] = 0
            # Location known and empty
            elif data[i * width + j] == 0:
                img[i, j] = 255
            # Location unknown
            elif data[i * width + j] == -1:
                img[i, j] = 205

    output_img = cv2.inRange(img, 0, 1)
    edges = cv2.Canny(img, 0, 255)
    im2, contours, hierarchy = cv2.findContours(
        output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(output_img, contours, -1, (255, 255, 255), 5)
    output_img = cv2.bitwise_not(output_img)
    res = cv2.bitwise_and(output_img, edges)
    # ------------------------------

    frontier = copy(res)
    im2, contours, hierarchy = cv2.findContours(
        frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

    im2, contours, hierarchy = cv2.findContours(
        frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    # Unclear on logic below
    # Gives a list of fromtiers
    all_pts = []
    if len(contours) > 0:
        upto = len(contours) - 1
        i = 0
        maxx = 0
        maxind = 0

        for i, _ in enumerate(contours):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            xr = cx * resolution + origin_x
            yr = cy * resolution + origin_y
            pt = [np.array([xr, yr])]
            if len(all_pts) > 0:
                all_pts = np.vstack([all_pts, pt])
            else:
                all_pts = pt

    # Frontier control for optimal navigation
    # Modified A* approach
    frontier_cost = np.empty([len(all_pts), 1])
    for i, frontier in enumerate(all_pts):
        # frontier_tb_dist = ((frontier[0]-tb_x)**2 + (frontier[1]-tb_y)**2)**(1/2)
        frontier_tb_dist = np.linalg.norm(frontier - [tb_x, tb_y])

        # frontier_dest_dist = ((frontier[0]-dest_x)**2 + (frontier[1]-dest_y)**2)**(1/2)
        frontier_dest_dist = np.linalg.norm(frontier - [dest_x, dest_y])

        # prevents oscillation of navigation goal
        frontier_cost[i] = frontier_tb_dist + frontier_dest_dist

    frontiers = sorted(np.hstack((all_pts, frontier_cost)), key=lambda x: x[2])
    # frontiers = sorted(all_pts, lambda x:x[2])
    return frontiers
