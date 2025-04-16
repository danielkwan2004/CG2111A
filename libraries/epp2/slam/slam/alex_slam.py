"""
This module provides functions and classes for setting up and using SLAM (Simultaneous Localization and Mapping) with LIDAR sensors. The module wraps the BreezySLAM library and provides a simplified interface for setting up and using SLAM with LIDAR sensors. The module also provides a placeholder function for displaying SLAM results.

Refer to alex_example_lidar_basic.py for an example of how to use this module.

Classes:
    RMHC_SLAM: Class for performing SLAM using the RMHC algorithm.
    Laser: Class representing a LIDAR sensor.
    PyRPlidar: Class for interfacing with RPLIDAR devices.
Functions:
    setupLidarModel(scan_size=360, scan_rate=5.5, detection_angle_deg=360, distance_no_detection=3072, detection_margin=0, offset_mm=0):
        Sets up and returns a LIDAR model with the specified parameters.
    resample_lidar_scan(distance, angles, target_measurements_per_scan=360, offset_degrees=0, merge_strategy=np.mean, fill_value=0):
        Resamples LIDAR scan data to a specified number of measurements per scan, applying an optional offset and merge strategy.
    setupSlam(laser, map_size_pixels, map_size_meters, map_quality=None, hole_width_mm=None, random_seed=None, sigma_xy_mm=None, sigma_theta_degrees=None, max_search_iter=None):
        Sets up and returns an RMHC_SLAM instance with the specified parameters.
    slam_display():
        Placeholder function for displaying SLAM results.
"""

import breezyslam
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from pyrplidar import PyRPlidar
import numpy as np
import queue

# Constants for the A1M8 LIDAR
SLAM = RMHC_SLAM
LIDAR_MODEL = lambda: Laser(360, 5.5, 360, 1200, 0, 75)

def getGridAlignedSlamPose(x,y,theta,map_size_millimeters):
    """
    Returns the grid-aligned pose of the robot based on the given pose.
    By default, BreezySLAM uses a coordinate system where Positive X is to the back and Positive Y is left.
    This function converts the pose to a coordinate system where Positive X is to the Right/East and Positive Y is Forward/North.
    The origin is set to the bottom-left corner of the map.

    Args:
        x (float): The X-coordinate of the robot.
        y (float): The Y-coordinate of the robot.
        theta (float): The orientation of the robot in degrees.
        map_size_millimeters (int): The size of the map in millimeters.

    Returns:
        tuple: The grid-aligned pose

    """
    return -y+map_size_millimeters, -x+map_size_millimeters, theta

def mapBytesToGrid(mapbytes, width, height):
    """
    Converts a byte array representing a map to a 2D grid. 
    The positve Y-axis is assumed to be north (i.e., front)
    The positive X-axis is assumed to be east (i.e., right)

    Args:
        map_bytes (bytes): The byte array representing the map.
        width (int): The width of the map.
        height (int): The height of the map.

    Returns:
        list: A 2D grid representing the map.
    """
    mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (width, height), order="C")
    mapimg = mapimg[::-1, ::-1].T
    return mapimg

def gridToMapBytes(grid):
    """
    Converts a 2D grid representing a map to a byte array.

    Args:
        grid (list): A 2D grid representing the map.

    Returns:
        bytes: A byte array representing the map.
    """
    # Currently, the map is stored as a 2D grid.
    # if transformed to a 1D byte array, it can be stored efficiently.
    # Assuming the origin at the bottom-left corner of the map.
    mapbytes = bytearray(np.array(grid).T[::-1, ::-1].flatten())
    return mapbytes
