#! /usr/bin/env python

"""
ROS Rapidly-Exploring Random Trees (RRT) path planning algorithm exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: December 2020
Usage: roslaunch unit4 unit4_exercise.launch
"""

import rospy

from bresenham import bresenham
from math import atan2, cos, sin
from random import randrange as rand


# Node class
class Node:
    def __init__(self, coordinates, parent=None):
        # coordinates: list with [x,y] values of grid cell coordinates
        self.coordinates = coordinates
        # parent: Node object
        self.parent = parent


def calculate_distance(p1, p2):
    """
    Calculates distance between two [x,y] coordinates.
    p1: the point (as list containing x and y values) from which to measure
    p2: the point (as list containing x and y values) to which to measure
    returns: the distance
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = (dx ** 2 + dy ** 2) ** 0.5
    return distance


def calculate_angle(p1, p2):
    """
    Calculates the angle of a straight line between two [x,y] coordinates.
    p1: the point (as list containing x and y values) from which to measure
    p2: the point (as list containing x and y values) to which to measure
    returns: the angle in radians
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    theta = atan2(dy, dx)
    return theta


def collision_detected(p1, p2, map, map_width):
    """
    Test if two nodes are separated by an obstacle by tracing a line between them
    p1: the point (as list containing x and y values) from which the check starts
    p2: the point (as list containing x and y values) at which the check ends
    map: the map containing free space and obstacles
    map_width: the map's width
    returns: True if a collision is detected, False if no collision is found
    """
    # Compute cells covered by the line p1-p2 using the Bresenham ray tracing algorithm
    covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
    # Check if any of the cells is an obstacle cell
    for cell in covered_cells:
        # Access an element in a 1D-array (map) providing index = x + map_width*y
        if map[cell[0] + map_width * cell[1]]:
            # Detects a collision if map has a 1
            return True
    # No collision
    return False


def find_closest_node(random_pt, node_list):
    """
    Finds the closest node in the tree
    random_pt: a [x,y] point (as a list)
    node_list: list that keeps all nodes in the tree
    returns: Node instance that is the closest node in the tree
    """
    nearest_distance = float('inf')
    for n in node_list:
        current_distance = calculate_distance(random_pt, n.coordinates)
        if current_distance < nearest_distance:
            nearest_node = n
            nearest_distance = current_distance
    return nearest_node



def create_new_branch_point(p1, p2, max_distance):
    """
    Creates a new point at the max_distance towards a second point
    p1: the point to go from (as a list containing x and y values)
    p2: the point to go to (as a list containing x and y values)
    max_distance: the expand distance (in grid cells)
    returns: new point as a list containing x and y values
    """
    updated_node = list(p2)
    distance = calculate_distance(p1, p2)
    if distance>max_distance:
        angle = calculate_angle(p1, p2)
        updated_node[0] = p1[0] + int(max_distance * cos(angle))
        updated_node[1] = p1[1] + int(max_distance * sin(angle))
    else:
        updated_node = p2
    return updated_node

def test_goal(p1, p_goal, tolerance):
    """
    Test if goal has been reached considering a tolerance distance
    p1: a [x,y] point (as a list) from where to test
    p_goal: a [x,y] point (as a list) corresponding to the goal
    tolerance: distance margin (in grid cells) allowed around goal
    returns: True goal is within tolerance, False if goal is not within tolerance
    """
    goal_reached = False
    dist2goal = calculate_distance(p1, p_goal)
    if dist2goal<tolerance:
        goal_reached = True
    return goal_reached


def rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, tree_viz):
    '''
    Performs Rapidly-Exploring Random Trees (RRT) algorithm on a costmap with a given start and goal node
    '''
    ## Add your code ##
    root_node = Node(initial_position)
    nodes = []
    nodes.append(root_node)

    iterations = 0
    #the maximum number of iterations
    max_iterations = 10000
    #the maximum length of each branch
    max_branch_lenght = 20
    #tolerance for reaching a position goal
    goal_tolerance = 20

    path = []

    while True:
        iterations += 1
        if iterations > max_iterations:
            rospy.logwarn("RRT: Max iterations exceeded")
            return path

        new_node = [rand(width), rand(height)]
        closest_node = find_closest_node(new_node, nodes)
        updated_node = create_new_branch_point(closest_node.coordinates, new_node, max_branch_lenght)
        if not collision_detected(closest_node.coordinates, updated_node, map, width):
            latest_node = Node(updated_node,closest_node)
            nodes.append(latest_node)
            tree_viz.append(latest_node)
            if test_goal(latest_node.coordinates, target_position, goal_tolerance):
                rospy.loginfo('RRT: Goal reached')
                break

    path.append(target_position)
    node = latest_node
    while node.parent:
        path.append(node.coordinates)
        node = node.parent
    path.append(node.coordinates)

    path.reverse()
    rospy.loginfo('RRT: Done reconstructing path')
    return path
