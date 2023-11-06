# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

from coppeliasim_zmqremoteapi_client import *
from nicol_base import *
import numpy as np
from PIL import Image
import os
import time
import math
import pytorch_kinematics as pk
import torch

class NicolHead:
    """
    A NicolHead Object represents the head of the Nicol robot. It contains the two neck joints and the eye cameras
    """
    def __init__(self, client, left_eye_res, right_eye_res, output_path, talker):
        """
        Constructor for the head.

        :param left_eye_res: The resolution of the left eye camera
        :param right_eye_res: The resolution of the right eye camera
        """

        self.client = client
        self.sim = self.client.getObject('sim')

        self.__joints ={"head_y":self.sim.getObject("/joint_head_y")
                      ,"head_z":self.sim.getObject("/joint_head_z")
                        }
        #self.left_eye_camera_image = None
        #self.right_eye_camera_image = None
        self.__face = self.sim.getObject("/face_link_visual")
        self.left_eye = self.sim.getObject("/left_eye_sensor")
        self.right_eye = self.sim.getObject("/right_eye_sensor")
        self.sim.setObjectInt32Param(self.left_eye, self.sim.visionintparam_resolution_x,left_eye_res[0])
        self.sim.setObjectInt32Param(self.left_eye, self.sim.visionintparam_resolution_y, left_eye_res[1])
        self.sim.setObjectInt32Param(self.right_eye, self.sim.visionintparam_resolution_x,right_eye_res[0])
        self.sim.setObjectInt32Param(self.right_eye, self.sim.visionintparam_resolution_y,right_eye_res[1])
        self.face_expressions = ["anger", "disgust", "fear", "happiness", "neutral", "sadness", "surprise"]
        
        self.output_path = output_path

        self.talker = talker
        if talker:
            from dependencies.nicol_talker.nicol_speech import NICOL_TALKER
            self.__talker = NICOL_TALKER(model_path='dependencies/nicol_talker/tts_models--en--vctk--vits/model_file.pth', config_path='dependencies/nicol_talker/tts_models--en--vctk--vits/config.json')
        
        #self.cycleik = CycleIK(robot="nicol", chain="right_arm", cuda_device='0', verbose=True)
        urdf_path = "./resources/NICOL.urdf"
        head_0_chain = pk.build_serial_chain_from_urdf(open(urdf_path).read(), "head_tool0_link", "world")
        self.head_0_chain = head_0_chain.to(dtype=torch.float32, device="cuda:0")

        head_1_chain = pk.build_serial_chain_from_urdf(open(urdf_path).read(), "head_tool1_link", "world")
        self.head_1_chain = head_1_chain.to(dtype=torch.float32, device="cuda:0")



    def left_camera(self):
        """
        Getter for the left_camera handle
        :return: the handle of the left camera
        """
        return self.left_eye

    def right_camera(self):
        """
        Getter for the right_camera handle
        :return: the handle of the right camera
        """
        return self.right_eye

    def set_joint_position(self, joint_position, block = True):
        """
        Set the joint positions of the robots neck
        :param joint_position: the target positions for the joints, can be list [head_y,head_z] or dict with name:position
        """
        if type(joint_position) != NicolJointPosition:
            if type(joint_position) == list or type(joint_position) ==  np.ndarray:
                joint_position = {"joint_name":["head_y","head_z"],"joint_position":joint_position}
            nicol_position = NicolJointPosition(joint_position)
        else:
            nicol_position = joint_position
            nicol_position.joint_name = ["head_y", "head_z"]

        for idx, joint in enumerate(nicol_position.joint_name):
            handle = self.__joints[joint]
            self.sim.setJointTargetPosition(handle, nicol_position.position[idx])

        if block:
            self.wait_for_execution_js(nicol_position)



    def get_joint_position(self, joints=None):
        """
        Getter for the neck joint positions
        :param joints: A list of joint names for which you want to get the positions. If none get all joint positions
        :return positions: A dict with joint name, position pairs
        """
        positions = {}
        if joints == None:
            joints = self.__joints.keys()
        for joint in joints:
            handle = self.__joints[joint]
            positions[joint] = self.sim.getJointPosition(handle)
        return NicolJointPosition(positions)
    

    def set_face_expression(self, expression):
        """
        Sets the face of NICOL to the texture at given path

        :param texture_path: The path to the texture of the face expression
        """
        texture_path = f"{os.getcwd()}/expressions/{expression}.png"
        _, textureId,_ = self.sim.createTexture(texture_path,0)        
        self.sim.setShapeTexture(self.__face, textureId, self.sim.texturemap_plane,0 ,(0.5,0.6), (0.0,0,0.02),(0,-np.pi/2,np.pi/2))


    def get_right_eye_camera_img(self, persistent = False, prefix:str =""):
        """
        Takes a picture from the right eye camera

        :param persistent: Wether the image should be saved directly
        :return: The rgb image as an nparray
        :param save_path: Relative path to the directory
        """

        save_path = self.output_path + "/right_eye"

        self.sim.handleVisionSensor(self.right_eye)
        image,res = self.sim.getVisionSensorImg(self.right_eye)
        i = Image.frombytes("RGB", res, image)
        i = i.transpose(Image.FLIP_TOP_BOTTOM)
        if persistent:
            filename = prefix + "right" +  time.strftime("%d-%m-%y-%H-%M-%S") + ".png"
            if not os.path.isdir(save_path):
                os.makedirs(save_path)
            filename = os.path.join(save_path,filename)
            self.sim.saveImage(image,res, 0, filename,-1)
        return np.array(i)

    def get_left_eye_camera_img(self, persistent = False, save_path = None):
        """
        Takes a picture from the left eye camera

        :param persistent: Wether the image should be saved directly
        :return: The rgb image as an nparray
        :param save_path: Relative path to the directory
        """

        save_path = self.output_path + "/left_eye"
        self.sim.handleVisionSensor(self.left_eye)
        image,res = self.sim.getVisionSensorImg(self.left_eye)
        i = Image.frombytes("RGB", res, image)
        i = i.transpose(Image.FLIP_TOP_BOTTOM)

        if persistent:
            filename = prefix + "left" + time.strftime("%d-%m-%y-%H-%M-%S") + ".png"
            if not os.path.isdir(save_path):
                os.makedirs(save_path)
            filename = os.path.join(save_path,filename)
            self.sim.saveImage(image,res, 0, filename,-1)
        return np.array(i)


    def check_joint_diff(self, from_js: NicolJointPosition, to_js: NicolJointPosition, error=0.001):
        if from_js is None:
            return True
        too_far = False
        #point to the current to_joint
        to_idx = 0
        #from_js has all the joints
        for from_idx, _ in enumerate(from_js.position):
            #is this name in to_js?
            if from_js.joint_name[from_idx] == to_js.joint_name[to_idx]:
                #is it a arm joint, how large is the error
                if from_idx < 6 and abs(to_js.position[to_idx] - from_js.position[from_idx]) > error:
                    too_far = True
                #is it a hand joint? then accept larger error
                elif from_idx >= 6 and abs(to_js.position[to_idx] - from_js.position[from_idx]) > error * 100:
                    too_far = True
                #increase to_idx to look at the next to_joint
                to_idx += 1
                #if we checked all joints in to_js we break
                if to_idx >= len(to_js.position):
                    break
        return too_far

    def wait_for_execution_js(self, target_joint_position: NicolJointPosition = None, error=0.001):
        assert target_joint_position is not None
        target_js = target_joint_position

        timeout_counter = 0
        current_js = None
        while self.check_joint_diff(current_js, target_js, error=error):
            #print("Waiting to execute js")
            current_js = self.get_joint_position()
            t_out = 20
            if timeout_counter >= 20 * t_out:
                break

            if timeout_counter >= t_out * 5 :
                print("Head takes longer than usual")

            #step simulation
            self.client.step()
            timeout_counter += 1
            #print("sleep counter: " + str(timeout_counter))
        return
    

    def say(self, text, block=False):
        """
        Use the TTS with preset voice parameters to say a given sentance

        :param text: The text the robot should say
        :param block: Wheter the TTS blocks the process or runs in the background default False
        """
        if self.talker:
            self.__talker.synth(text,"./temp.wav")
            self.__talker.play("./temp.wav",block=block)


    def set_pose_target(self, pose, block: bool = True) -> bool:
        target_point = [pose.position.x, pose.position.y, pose.position.z]
        current_js = self.get_joint_position()
        
        temp_js = np.array([[current_js.position[1], -current_js.position[0]]])
        tool0_pos = self.head_0_chain.forward_kinematics(temp_js)
        tool1_pos = self.head_1_chain.forward_kinematics(temp_js)
        tool0_pos = self.slice_fk_pose(tool0_pos, batch_size=1)
        tool1_pos = self.slice_fk_pose(tool1_pos, batch_size=1)
        tool0_pos = list(tool0_pos[0].cpu().numpy())
        tool1_pos = list(tool1_pos[0].cpu().numpy())
        #calculate angle difference and apply to motors
        angle = self.look_at(target_point, tool0_pos, tool1_pos)
        if abs(angle[0]) > 0.01 or abs(angle[1]) > 0.01:
            y_angle=(current_js.position[0] + angle[0])
            z_angle=(current_js.position[1] + angle[1])
            #lower="-0.583" upper="1.282"
            if y_angle > 0.583:
                y_angle = 0.583
            if y_angle < -1.282:
                y_angle = -1.282
            if z_angle < -np.pi:
                z_angle = -np.pi
            if z_angle > np.pi:
                z_angle = np.pi
            self.set_joint_position([y_angle, z_angle])
            #time.sleep(0.25)
            for i in range(50):
                self.client.step()
            return True
        return False
    

    def look_at(self, target_pos, tool0_pos, tool1_pos):
        #calculate
        eef_slope_yaw = (target_pos[1] - tool0_pos[1]) / (target_pos[0] - tool0_pos[0]+ 0.0000001)
        tool1_slope_yaw = (tool1_pos[1] - tool0_pos[1]) / (tool1_pos[0] - tool0_pos[0]+ 0.0000001)
        eef_slope_pitch = (target_pos[0] - tool0_pos[0]) / (target_pos[2] - tool0_pos[2]+ 0.0000001)
        tool1_slope_pitch = (tool1_pos[0] - tool0_pos[0]) / (tool1_pos[2] - tool0_pos[2] + 0.0000001)
        yaw_coeff = (tool1_slope_yaw - eef_slope_yaw ) / (1 + tool1_slope_yaw * eef_slope_yaw)
        pitch_coeff = (tool1_slope_pitch - eef_slope_pitch) / (1 + tool1_slope_pitch * eef_slope_pitch)
        

        z_correction_coeff = 0.0
        if abs(target_pos[0]) + abs(target_pos[1]) + abs(target_pos[2] - 1.4) <= 2.2:
            correction_rad = ((1 - ((abs(target_pos[0]) + abs(target_pos[1]) + abs(target_pos[2] - 1.4)) / 2.2)) * z_correction_coeff)
            return [math.atan(pitch_coeff) + correction_rad, -math.atan(yaw_coeff)]
        else:
            print("Greater 1.5")
            return [math.atan(pitch_coeff), -math.atan(yaw_coeff)]
        
    def slice_fk_pose(self, pose, batch_size, rotation='quaternion'):
        pos = torch.reshape(pose.get_matrix()[:, :3, 3:], shape=(batch_size, 3))
        rot = None
        if rotation == 'quaternion':
            rot = pk.matrix_to_quaternion(pose.get_matrix()[:, :3, :3])
            rot = torch.concat((rot[:, 1:], rot[:, :1]), dim=1)
        elif rotation == 'angle':
            rot = pk.matrix_to_euler_angles(pose.get_matrix()[:, :3, :3], convention='ZYX')
        else:
            raise NotImplementedError
        return torch.concat((pos, rot), dim=1)
