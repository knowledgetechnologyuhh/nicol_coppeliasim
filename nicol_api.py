# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

from coppeliasim_zmqremoteapi_client import *
import numpy as np
import os
import subprocess
import time
from PIL import Image
from nicol_side import NicolSide
from nicol_head import NicolHead
from nicol_base import NicolPose


class NICOL:
    """
    A NICOL objects combines the NicolHead and NicolSide objects into one easy to use class
    """
    def __init__(self, scene = "nicol_ik.ttt", headless = False, start_scene = True, frame_res = [1920,1080], left_eye_res = [1024,768], right_eye_res = [1024,768],talker=True,gaikpy_on=True,output_path=None):
        """
        Constructor for the NICOL object

        :param scene: The name of the coppeliasim scene file (CoppeliaSim only)
        :param headless: Wether the simulation should be run headless or not (CoppeliaSim only)
        :param frame_res: The resolution of the frame camera (CoppeliaSim only)
        :param left_eye_res: The resolution of the left eye camera (CoppeliaSim only)
        :param right_eye_res: The resolution of the right eye camera (CoppeliaSim only)
        """



        coppelia_path = os.getenv("COPPELIASIM_ROOT")

        if output_path:
            self.output_path = output_path
        else:
            self.output_path = os.getcwd()

        if start_scene:
            if headless:
                self.sim_process = subprocess.Popen([
                    f"{coppelia_path}/coppeliaSim",
                    "-h",
                    os.path.abspath(scene)]) 
            else:
                self.sim_process = subprocess.Popen([
                    f"{coppelia_path}/coppeliaSim",
                    os.path.abspath(scene)]) 

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        #create robot parts
        self.left_ = NicolSide(self.client,True,gaikpy_on=gaikpy_on)
        self.right_ = NicolSide(self.client,False,gaikpy_on=gaikpy_on)
        self.head_ = NicolHead(self.client, left_eye_res,right_eye_res,self.output_path,talker)


        #get Frame camera and set resolution
        self.__frame_camera = self.sim.getObject("/frame_camera")
        self.sim.setObjectInt32Param(self.__frame_camera, self.sim.visionintparam_resolution_x,frame_res[0])
        self.sim.setObjectInt32Param(self.__frame_camera, self.sim.visionintparam_resolution_y,frame_res[1])
        self.start_simulation()
        self.client.setStepping(True)





        # some things don't work (speaker ids have no parameter??)
        # for now the model is copied to working directory but this is dirty
        # model_path="dependencies/nicol_talker/tts_models--en--vctk--vits/model_file.pth", config_path="dependencies/nicol_talker/tts_models--en--vctk--vits/config.json",use_cuda=False



    def left(self) -> NicolSide:
        """
        Getter for the left arm
        """
        return self.left_


    def right(self) -> NicolSide:
        """
        Getter for the right arm
        """
        return self.right_

    def head(self) -> NicolHead:
        """
        Getter for the right head
        """
        return self.head_



    def get_frame_camera_img(self, persistent:bool = True, prefix:str ="") -> np.ndarray:
        """
        Take an image with the frame camera

        :param persistent: Wether the image should be saved directly
        :param save_path: Relative path to the directory
        :return: The rgb image as an ndarray
        """
        save_path = self.output_path + "/frame_rgb"

        self.sim.handleVisionSensor(self.__frame_camera)
        image,res = self.sim.getVisionSensorImg(self.__frame_camera)
        i = Image.frombytes("RGB", res, image)
        i = i.transpose(Image.FLIP_TOP_BOTTOM)

        if persistent:
            filename = "frame_rgb" + prefix + time.strftime("%d-%m-%y-%H-%M-%S") + ".png"
            if not os.path.isdir(save_path):
                os.makedirs(save_path)
            filename = os.path.join(save_path,filename)
            self.sim.saveImage(image,res, 0, filename,-1)
        return np.array(i)



    def get_frame_camera_depth(self, persistent:bool = False, prefix:str = "") -> np.ndarray:
        """
        Gets the depth image from the frame camera

        :param persistent: save the image to a file
        :param save_path: path to the save directory if None this will be cwd/frame_depth/

        :return i: A float array with values between 0 and 1 indicating the distance for that pixel with 0 beeing the close clipping plane and 1 beeing the far clipping plane. Please note, that images will be saved as grey scale images with values between 0 and 255
        """

        save_path = self.output_path + "/frame_depth"
        self.sim.handleVisionSensor(self.__frame_camera)
        image,res = self.sim.getVisionSensorDepth(self.__frame_camera)
        i = Image.frombytes("F", res, image)
        i = i.transpose(Image.FLIP_TOP_BOTTOM)
        i = np.array(i)
        int_imgae = (Image.fromarray(i*256)).convert("L")
        if persistent:
            filename = "frame_depth" + time.strftime("%d-%m-%y-%H-%M-%S") + ".png"
            if not os.path.isdir(save_path):
                os.makedirs(save_path)
            filename = os.path.join(save_path,filename)

            int_imgae.save(filename)
            #self.sim.saveImage(image,res, 0, filename,-1)
        return np.array(i)



    def stop_coppelia(self):
        """
        Function to stop the running instance of Coppeliasim.
        """
        self.sim_process.terminate()

    def step_simulation(self,steps):
        """
        Steps the simulation (CoppeliaSim only)

        :param steps: The number of simulation steps that should be performed
        """
        for i in range(steps):
            self.client.step()

    def set_stepping(self, stepping=True):
        """
        Sets if the simulation steps by its own or only by calling the step_simulation function

        :param stepping=True: True: only step by calling the step_simulation function, False: step the simulation by its own
        """
        self.client.setStepping(stepping)

    def start_simulation(self):
        """
        starts the simulation. Gets called by NICOL constructor
        """
        self.sim.startSimulation()
    
    def stop_simulation(self):
        """
        stops the simulation
        """
        self.sim.stopSimulation()

    def get_output_path(self):
        return self.output_path

    def pause_simulation(self):
        """
        Pauses the simulation
        """
        self.sim.pauseSimulation()

    def set_joint_position_for_arms_and_head(self,
        left_joint_position,
        right_joint_position,
        head_joint_position,
        uniform_input = False):
        """
        Sets the joint positions for both arms and the head, generally needed for paralization of the ROS version
        
        :param left_joint_position: The joint position for the left arm as a NicolJointPosition
        :param right_joint_position: The joint position for the right arm as a NicolJointPosition
        :head_joint_position: The joint position for the neck joints as a NicolJointPosition 
        """
        self.left_.set_joint_position(left_joint_position)
        self.right_.set_joint_position(right_joint_position)
        self.head_.set_joint_position(head_joint_position)

    def set_joint_position_for_both_arms(self,
        left_joint_position,
        right_joint_position,
        uniform_input = False):
        """
        Sets the joint positions for both arms, generally needed for paralization of the ROS version
        
        :param left_joint_position: The joint position for the left arm as a NicolJointPosition
        :param right_joint_position: The joint position for the right arm as a NicolJointPosition
        """
        self.left_.set_joint_position(left_joint_position)
        self.right_.set_joint_position(right_joint_position)

    def set_pose_target_for_both_arms(self,
        left_pose: NicolPose,
        right_pose: NicolPose,
        only_position = False):
        """
        Sets the ik pose for both arms, generally needed for paralization of the ROS version

        :param left_pose: The pose for the left arm as a NicolPose
        :param right_pose: The pose for the right arm as a NicolPose
        """
        self.left_.set_pose_target(left_pose)
        self.right_.set_pose_target(right_pose)

    def set_camera_resolution(self, camera: str, resolution):
        """
        Setter for the camera resolution.

        :param camera: string that defines which camera should be set {"left", "right", "frame"}
        """
        if camera == "left":
            self.sim.setObjectInt32Param(self.head_.left_camera(), self.sim.visionintparam_resolution_x,resolution[0])
            self.sim.setObjectInt32Param(self.head_.left_camera(), self.sim.visionintparam_resolution_y,resolution[1])
        elif camera == "right":
            self.sim.setObjectInt32Param(self.head_.right_camera(), self.sim.visionintparam_resolution_x,resolution[0])
            self.sim.setObjectInt32Param(self.head_.right_camera(), self.sim.visionintparam_resolution_y,resolution[1])
        elif camera == "frame":
            self.sim.setObjectInt32Param(self.__frame_camera, self.sim.visionintparam_resolution_x,resolution[0])
            self.sim.setObjectInt32Param(self.__frame_camera, self.sim.visionintparam_resolution_y,resolution[1])
        else:
            print("error unknown camera name")


    def set_camera_fov(self, camera, fov):
        """
        Setter for the camera fov. The scene is preset with the original perspective angles for all cameras.

        :param camera: string that defines which camera should be set {"left", "right", "frame"}
        """
        if camera == "left":
            self.sim.setObjectFloatParam(self.head_.left_camera() ,self.sim.visionfloatparam_perspective_angle, fov)
        elif camera == "right":
            self.sim.setObjectFloatParam(self.head_.right_camera() ,self.sim.visionfloatparam_perspective_angle, fov)
        elif camera == "frame":
            self.sim.setObjectFloatParam(self.__frame_camera ,self.sim.visionfloatparam_perspective_angle, fov)
        else:
            print("error unknown camera name")

    def get_platform(self):
        """
        Always returns false since we are in simulation
        """
        return False

    def get_robot_commander(self):
        """
        Compatability function for the ROS api. No robot commander in Coppelia so this is None.
        """
        return None

    def get_planning_scene(self):
        """
        Compatability function for the ROS api and Moveit. Since this is not available in Coppelia returns None.
        """
        return None

    def get_workspace_path(self):
        """
        Compatability function for the ROS api.
        Should return ROS workspace, but since this does not exist returns current working directory instead.
        """
        return 