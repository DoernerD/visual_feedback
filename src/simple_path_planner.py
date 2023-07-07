#!/usr/bin/python3
"""
Simple, bezier based path planner for underwater docking.

Bezier implementation from
https://towardsdatascience.com/b%C3%A9zier-curve-bfffdadea212

Copyright 2023 David Doerner (ddorner@kth.se)
"""
from __future__ import division, print_function

import os

import numpy as np
import matplotlib.pyplot as plt


class SimplePathPlanner(object):
    """
    Bezier curve-based path planner. Returns the current waypoint
    The computation happens in the agent's base frame to simplify the trigonometry.
    All interfacing happens with values in the map frame. 
    Pay Attention!
    """
    def __init__(self):
        """
        Init function
        TODO: Might be good to externalise the int values?
        """
        # map frame
        self.start_map = np.array([0., 0., np.deg2rad(0)])
        self.goal_map = np.array([0., 0., np.deg2rad(90)])
        self.target_map = np.array([0., 0., 0.])

        # sam base_link frame
        self.goal_base = np.array([0., 0., 0.])
        self.target_base = np.array([0., 0., 0.])

        self.received_goal = False

        self.eps = 1.

        self.control_points_count = 3

        self.control_points_base = {}
        self.control_points_base["x"] = np.zeros(self.control_points_count)
        self.control_points_base["y"] = np.zeros(self.control_points_count)
        self.control_points_base["theta"] = np.zeros(self.control_points_count)

        self.control_points_map = {}
        self.control_points_map["x"] = np.zeros(self.control_points_count)
        self.control_points_map["y"] = np.zeros(self.control_points_count)
        self.control_points_map["theta"] = np.zeros(self.control_points_count)

        self.path_base = {}
        self.path_base["x"] = []
        self.path_base["y"] = []

        self.path_map = {}
        self.path_map["x"] = np.zeros(self.control_points_count)
        self.path_map["y"] = np.zeros(self.control_points_count)

        self.path_calculated = False
        self.current_index = 0

        self.current_waypoint_map = [0., 0.]


    #region functions
    def check_require_plan(self):
        """
        Check if we need a motion plan:
        Yes:  - no plan
              - Docking Station Estimate changed, e.g. new external waypoint. 
                  -> give some good lee-way, otherwise there might be a lot of jumping...
              - TODO: Too far from current waypoint (How to measure?)
        """
        if not self.path_calculated:
            print("No plan yet")
            return True

        x_diff = self.target_map[0] - self.path_map["x"][-1]
        y_diff = self.target_map[1] - self.path_map["y"][-1]
        distance_current_goal_to_ds_estimate = np.linalg.norm([x_diff, y_diff])

        if distance_current_goal_to_ds_estimate > 0.5:
            print("Goal moved")
            return True

        theta_diff = self.goal_map[2] - self.control_points_map["theta"][-1]
        if abs(theta_diff) > 0.175/2: # 5deg change
            print("Goal rotated")
            return True

        return False


    def check_in_feasible_region(self):
        """
        Check if the agent is in a feasible region to do a docking approach
        """
        return True
        if self.goal_base[0] < 0:
            rospy.logwarn("Not facing docking station, xDS: {}".format(self.goal_base[0]))
            return False

        if abs(self.goal_base[2]) < np.pi/2:
            rospy.logwarn("Wrong angle, phiDS: {}".format(abs(self.goal_base[2])))
            return False

        return True


    def calculate_path_segments(self):
        """
        Calculate the control points for the bezier curve.
        """
        # Calculate Bezier Control Point (second array entry)
        phi_prime = self.goal_base[2] - np.pi/2   # 90deg offset for the rotation. sign doesn't matter here, bc. tan is periodic with pi.
        x_prime = self.target_base[1] * np.tan(phi_prime)
        self.control_points_base["x"]= np.array([0, self.target_base[0] + x_prime, self.target_base[0]])
        self.control_points_base["y"] = np.array([0, 0,  self.target_base[1]])
        self.control_points_base["theta"] = np.array([0., 0., self.goal_base[2]])


    def calculate_bezier_curve(self):
        """
        Calculate a bezier curve based on a number of control points.
        """
        # Definition Bezier curve
        P = lambda t: (1 - t)**2 * np.array([self.control_points_base["x"][0],self.control_points_base["y"][0]]) \
                    + 2 * t * (1 - t) * np.array([self.control_points_base["x"][1],self.control_points_base["y"][1]]) \
                    + t**2 * np.array([self.control_points_base["x"][2],self.control_points_base["y"][2]])

        # Calculation Bezier curve in sam/base_link
        points = np.array([P(t) for t in np.linspace(0, 1, 3)])
        self.path_base["x"], self.path_base["y"] = points[:,0], points[:,1]

        # Reset the index for the current waypoint to 0 after replanning
        self.current_index = 0


    def get_current_waypoint(self, current_position_map):
        """
        Here we have the logic on how to populate self.currentWaypoint.
        Get the current waypoint in the map frame based on SAM's position
        in the map frame.
        """
        distance_to_waypoint = self.calculate_distance_to_waypoint(current_position_map)
        
        if distance_to_waypoint < self.eps:
            if self.current_index < len(self.path_map["x"])-1:
                self.current_index += 1
        self.current_waypoint_map = [self.path_map["x"][self.current_index],
                                 self.path_map["y"][self.current_index]]


    def calculate_distance_to_waypoint(self, current_position_map):
        """
        Returns the distance to the current waypoint in the x,y-plane
        Everything in the map frame
        """
        x_diff = self.current_waypoint_map[0]- current_position_map[0]
        y_diff = self.current_waypoint_map[1]- current_position_map[1]

        distance = np.linalg.norm([x_diff, y_diff])

        return distance


    #region Plotting
    def plot_path(self):
        """
        Plot Path in map frame
        """
        _, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path Planning
        ax.plot(self.goal_map[0], self.goal_map[1], 'o', color='red', label='DS')
        ax.plot(self.control_points_map["x"], self.control_points_map["y"], label='path')
        ax.plot(self.control_points_map["x"][0] ,self.control_points_map["y"][0],
                's', color='C2', markersize=10, label='start')
        ax.plot(self.control_points_map["x"][1] ,self.control_points_map["y"][1],
                'o', color='C2', markersize=10)
        ax.plot(self.control_points_map["x"][-1] ,self.control_points_map["y"][-1],
                '*', color='C2', markersize=10, label='end')
        ax.plot(self.path_map["x"], self.path_map["y"], 'x')


        # Coordinate visualization
        origin_ds = [self.goal_map[0], self.goal_map[1]]
        x_axis_ds, y_axis_ds = self.calculate_orientation_axes(self.goal_map[2], 0.5)

        origin_sam = [self.start_map[0], self.start_map[1]]
        x_axis_sam, y_axis_sam = self.calculate_orientation_axes(self.start_map[2], 0.5)

        # DS Axes
        plt.arrow(*origin_ds, *x_axis_ds, width=0.01, color='r')
        plt.arrow(*origin_ds, *y_axis_ds, width=0.01, color='g')
        plt.arrow(*self.target_map, *x_axis_ds, width=0.01, color='r')
        plt.arrow(*self.target_map, *y_axis_ds, width=0.01, color='g')

        # # SAM Coordinate axes
        plt.arrow(*origin_sam, *x_axis_sam, width=0.01, color='r')
        plt.arrow(*origin_sam, *y_axis_sam, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('map')
        ax.legend()

        home_dir = os.path.expanduser('~')
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/bezierPathMap.png', dpi=300)


    def plot_position(self):
        """
        Plot position of SAM and docking station
        """
        _, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path Planning
        ax.plot(self.goal_map[0], self.goal_map[1], 'o', color='red', label='DS')
        ax.plot(self.start_map[0], self.start_map[1], 'o', color='b', label='SAM')

        # Coordinate visualization
        origin_ds = [self.goal_map[0], self.goal_map[1]]
        x_axis_ds, y_axis_ds = self.calculate_orientation_axes(self.goal_map[2], 0.5)

        origin_sam = [self.start_map[0], self.start_map[1]]
        x_axis_sam, y_axis_sam = self.calculate_orientation_axes(self.start_map[2], 0.5)

        plt.arrow(*origin_ds, *x_axis_ds, width=0.01, color='r')
        plt.arrow(*origin_ds, *y_axis_ds, width=0.01, color='g')
        plt.arrow(*origin_sam, *x_axis_sam, width=0.01, color='r')
        plt.arrow(*origin_sam, *y_axis_sam, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('simple path')
        ax.legend()

        home_dir = os.path.expanduser('~')
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/position.png', dpi=300)


    def plot_tf_sam_base(self):
        """
        Plot everythin in SAM base_link
        """
        _, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path segments
        ax.plot(self.control_points_base["x"], self.control_points_base["y"], label='path')
        ax.plot(self.control_points_base["x"][0] ,self.control_points_base["y"][0],
                's', color='C2', markersize=10, label='start')
        ax.plot(self.control_points_base["x"][1] ,self.control_points_base["y"][1],
                'o', color='C2', markersize=10)
        ax.plot(self.control_points_base["x"][-1] ,self.control_points_base["y"][-1],
                '*', color='C2', markersize=10, label='end')
        ax.plot(self.path_base["x"], self.path_base["y"], 'x')

        # Targets
        ax.plot(self.goal_base[0], self.goal_base[1], 'o', color='red', label='DS')
        ax.plot(self.target_base[0], self.target_base[1], 'o', color='green', label='target')
        ax.plot(0, 0, 'o', color='b', label='SAM')

        # Coordinate visualization
        origin_ds = [self.goal_base[0], self.goal_base[1]]
        x_axis_ds, y_axis_ds = self.calculate_orientation_axes(self.goal_base[2], 0.5)

        origin_target_base = [self.target_base[0], self.target_base[1]]

        origin_sam = [0, 0]
        x_axis_sam, y_axis_sam = self.calculate_orientation_axes(0, 0.5)

        plt.arrow(*origin_ds, *x_axis_ds, width=0.01, color='r')
        plt.arrow(*origin_ds, *y_axis_ds, width=0.01, color='g')
        plt.arrow(*origin_target_base, *x_axis_ds, width=0.01, color='r')
        plt.arrow(*origin_target_base, *y_axis_ds, width=0.01, color='g')

        plt.arrow(*origin_target_base, *x_axis_sam, width=0.01, color='r', alpha=0.5)
        plt.arrow(*origin_target_base, *y_axis_sam, width=0.01, color='g', alpha=0.5)
        plt.arrow(*origin_sam, *x_axis_sam, width=0.01, color='r')
        plt.arrow(*origin_sam, *y_axis_sam, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('sam/base_link')
        ax.legend()

        home_dir = os.path.expanduser('~')
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/bezierPathSAMFrame.png', dpi=300)


    def calculate_orientation_axes(self, theta, length_axis):
        """
        Rotate the x,y-axis of a point for plotting
        """
        x_axis = [length_axis, 0]
        y_axis = [0, length_axis]
        rotation = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
            ])

        x_axis_prime = rotation.dot(x_axis)
        y_axis_prime = rotation.dot(y_axis)

        return x_axis_prime, y_axis_prime
    #endregion

    def print_states(self):
        """
        Verbose output
        """
        print("SAM Pose (map): x = {}, y = {}, phiSAM = {}".format(self.start_map[0], self.start_map[1], self.start_map[2]))
        print("DS Pose (map): x = {}, y = {}, phiDS = {}".format(self.goal_map[0], self.goal_map[1], self.goal_map[2]))

    #endregion

    #region Outward-facing stuff
    # The callbacks set the corresponding variables here
    # Then you have the corresponding functions that return whatever and the ROS node can convert
    # it into the proper frame etc.

    #endregion