# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

from typing import List
from unicodedata import decimal
import numpy as np
import threading

"""
    The Classes in this package mimic the ones of the ROS implementation to make the handling of the Coppeliasim version and the ROS version as similar as possible.
"""

Vector = List[float]

class Position():
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]

    def as_list(self):
        return [self.x,self.y, self.z]

class Orientation():
    def __init__(self, orientation):
        self.x = orientation[0]
        self.y = orientation[1]
        self.z = orientation[2]
        self.w = orientation[3]
        

class NicolPose():
    
    def __init__(self, position: Vector = None, orientation: Vector = None):
        """
            Subtype of geometry_msgs/Pose.

            Args:
                position (Vector): (x, y, z) coordinates as list of float-values (length: 3)
                orientation (Vector, optional): Quaternion of form (x, y, z, w) as list of float-values (length: 4)
        """
        if position is not None:
            self.position = Position(position)
        else:
            position = None
        if orientation is not None:
            self.orientation = Orientation(orientation)
        else:
            orientation = None

        #if position is not None:
        #    self.position.x = position[0]
        #    self.position.y = position[1]
        #    self.position.z = position[2]
        #if orientation is not None:
        #    self.orientation.x = orientation[0]
        #    self.orientation.y = orientation[1]
        #    self.orientation.z = orientation[2]
        #    self.orientation.w = orientation[3]

class NicolPose2():
    def __init__(self, position: Vector = None, orientation: Vector = None):
        """
            Compatibility Wrapper for Coppeliasim to make data types consistent with the ROS implementation

            Args:
                position (Vector): (x, y, z) coordinates as list of float-values (length: 3)
                orientation (Vector, optional): Quaternion of form (x, y, z, w) as list of float-values (length: 4)
        """
        self.position = None
        self.orientation = [0,0,0,1]

        if position is not None:
            self.position = position

        if orientation is not None:
            self.orientation = orientation

    def __str__(self):
        return "(%s , %s)"% (self.position, self.orientation)

class NicolKinematicsPose():

    def __init__(self, pose: NicolPose):
        """
            Compatibility Wrapper for Coppeliasim to make data types consistent with the ROS implementation


            Args:
                pose (NicolPose): Pose to be wrap in NicolKinematicsPose object
        """
        assert pose is not None
        self.pose = pose
        self.tolerance = 0.2
        self.max_accelerations_scaling_factor = 1.0
        self.max_velocity_scaling_factor = 1.0


class NicolJointPosition():

    def __init__(self, joint_positions: [list, dict] = None):
        """
            Compatibility Wrapper for Coppeliasim to make data types consistent with the ROS implementation

            Args:
                joint_positions (Vector): List of float-value joint positions, length depends on which size is expected
                                          by the function in which this object is inputted to.
        """
        assert joint_positions is not None
        input_type = type(joint_positions)
        if input_type == list or input_type == np.ndarray:
            names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8',  'jointT0', 'jointT1', 'jointI1', 'jointM1',  'jointLR1']
            self.joint_name = names[:len(joint_positions)]
            self.position = joint_positions
            self.max_accelerations_scaling_factor = 1.0
            self.max_velocity_scaling_factor = 1.0
        elif input_type == dict:
            if "joint_name" in joint_positions.keys():
                self.joint_name = joint_positions['joint_name']
                self.position = joint_positions['joint_position']
            else:
                self.joint_name = list(joint_positions.keys())
                self.position =   list(joint_positions.values())
            self.max_accelerations_scaling_factor = 1.0
            self.max_velocity_scaling_factor = 1.0
        else:
            print("Pleas enter Joint position as list, ndarray or dict in form {jointname : [...], joint position : [...]} or {joint_x:value_x,joint_y:value_y ...}")
            
    def __str__(self):
        round_pos = np.round(self.position,decimals = 4)
        return str(list(zip(self.joint_name,round_pos)))
