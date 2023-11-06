# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

import numpy as np
from cycleik_pytorch import CycleIK
from nicol_base import NicolPose
from nicol_api import NicolSide, NICOL
from nicol_base import *
import torch

class NicolSideCycleIK(NicolSide):
    """
    CycleIK based NicolSide class for coppeliaSim. Overrides the set_pose_target method to use CycleIk instead of gyaikpy
    """
    
    def __init__(self,client, start_left_arm=False):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        if start_left_arm:
            self.cycleik = CycleIK(robot="nicol", chain="left_arm" ,cuda_device='0', verbose=True)
        else:
            self.cycleik = CycleIK(robot="nicol", chain="right_arm", cuda_device='0', verbose=True)
        super().__init__(client, start_left_arm=start_left_arm, gaikpy_on=False)


    @torch.no_grad()        
    def set_pose_target(self, pose: NicolPose, only_position: bool = False,block=True) -> bool:
        pose_array = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],dtype=np.float32)
        #pose_array = np.array([*pose.position, *pose.orientation],dtype=np.float32)
        pose_array = pose_array.reshape((1, 7))
        js_array, error, timeout_ik, timeout_fk = self.cycleik.inverse_kinematics(pose_array, calculate_error=True)
        self.set_joint_position(js_array[0],block=block)
    
    @torch.no_grad()        
    def calc_pose_batch(self, trajectory: list, only_position: bool = False) -> bool:
        pose_array = np.zeros(shape=(len(trajectory), 7), dtype=np.float32)
        for index, pose in enumerate(trajectory):
            pose_array[index] = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],dtype=np.float32)
        
        js_result, error, elapsed_time_ik, elapsed_time_fk = self.cycleik.inverse_kinematics(pose_array)
        return js_result
    

    def calc_fk(self, js:NicolJointPosition, link: str):
        target = np.array([js.position])
        return self.cycleik.forward_kinematics(target)

class NicolCycleIK(NICOL):

    def __init__(self, scene = "nicol_ik.ttt", headless = False, start_scene = True, frame_res = [1920,1080], left_eye_res = [1024,768], right_eye_res = [1024,768],talker=True):
        super().__init__(scene, headless, start_scene, frame_res, left_eye_res, right_eye_res,talker,gaikpy_on=False)
        self.left_  = NicolSideCycleIK(self.client,start_left_arm=True,)
        self.right_ = NicolSideCycleIK(self.client)

