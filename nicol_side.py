# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

import scipy.spatial.transform as transform
from nicol_base import *
from coppeliasim_zmqremoteapi_client import *
import numpy as np
import functools

def external(func):
    @functools.wraps(func)
    def inner(self, *args, **kwargs):
        return func(self, *args, **kwargs)

    return inner


def internal(func):
    @functools.wraps(func)
    def inner(self, *args, **kwargs):
        return func(self, *args, **kwargs)

    return inner


class NicolSide:
    
    """
    NicolSide represents one side of the nicol robot (arm and hand)
    """
    def __init__(self, client, start_left_arm=False, gaikpy_on=True):
        """
        Construct a NicolSide object

        :param start_left_arm: wether left arm or not
        """
        if gaikpy_on:
            from gaikpy import robot_definitions

        #load joints from the Scene
        self.client = client
        self.sim = self.client.getObject('sim')

        if start_left_arm:
            prefix = "/l"
        else:
            prefix = "/r"
            
        #Dict linking joint names to the corresponding simulation handles
        self.__joints = {   'joint1':[self.sim.getObject(prefix+"_joint1")],
                            'joint2':[self.sim.getObject(prefix+"_joint2")],
                            'joint3':[self.sim.getObject(prefix+"_joint3")],
                            'joint4':[self.sim.getObject(prefix+"_joint4")],
                            'joint5':[self.sim.getObject(prefix+"_joint5")],
                            'joint6':[self.sim.getObject(prefix+"_joint6")],
                            'joint7':[self.sim.getObject(prefix+"_joint7")],
                            'joint8':[self.sim.getObject(prefix+"_joint8")],
                            'jointT0':[self.sim.getObject(prefix+"_jointT0")],
                            'jointT1':[self.sim.getObject(prefix+"_jointT1"),self.sim.getObject(prefix+"_jointT2"),self.sim.getObject(prefix+"_jointT3")],
                            'jointI1':[self.sim.getObject(prefix+"_jointI1"),self.sim.getObject(prefix+"_jointI2"),self.sim.getObject(prefix+"_jointI3")],
                            'jointM1':[self.sim.getObject(prefix+"_jointM1"),self.sim.getObject(prefix+"_jointM2"),self.sim.getObject(prefix+"_jointM3")],
                            'jointLR1':[self.sim.getObject(prefix+"_jointR1"),self.sim.getObject(prefix+"_jointR2"),self.sim.getObject(prefix+"_jointR3"),self.sim.getObject(prefix+"_jointL1"),self.sim.getObject(prefix+"_jointL2"),self.sim.getObject(prefix+"_jointL3")]
                        }
                    
        self.fingers = {1:"jointT1" ,2:"jointI1" ,3:"jointM1" ,4:"jointR1", 5: "jointL1"}
        
        #The arms endeffector
        self.__eef = self.sim.getObject(prefix+"_tool0")
        #is this a left arm?
        self.left=start_left_arm
        #the base of the robot
        self.__base = self.sim.getObject("/joint_base")
        #rotation of the arms in Coppelia relative to the urdf chains
        self.__laser = self.sim.getObject(prefix+"_laser")

        if gaikpy_on:
            #ik object for the gaikpy ik
            self.ik = robot_definitions.NICOL_robot(visualisation=False)
        #target for the coppelia ik
        self.coppelia_ik_target = self.sim.getObject(prefix+"_target")
        #the ik script for the coppelia ik

        #self.ik = robot_definitions.NICOL_robot(visualisation=False)
    

    def set_joint_position(self, joint_position, uniform_input: bool = False, block = True):
        """
        Sets the joint positions for the arm. Joint angles in rad.

        :param joint_position: Can be either a list or nparray with 13 entries (8 arm joints and 5 fingers), a dict of the form {joint_name:[names],joint_position:[positions]} or a NicolJointPosition object
        :param uniform_input: Wether the input angles are normalized to the intervall [0,1]
        """

        #turn of the ik script of the arm


        nicol_position = NicolJointPosition(joint_position)

        for idx, joint in enumerate(nicol_position.joint_name):

            if joint in ["jointT1", "jointI1", "jointM1", "jointLR1"]:
                nicol_position.position[idx] += np.pi
                nicol_position.position[idx] /= 2
            if joint == "jointT0":
                #print(f"ffff {nicol_position.position[idx]}")
                nicol_position.position[idx] += np.pi
                nicol_position.position[idx] = nicol_position.position[idx] / 4
                if not self.left:
                    nicol_position.position[idx] = -nicol_position.position[idx]

            if joint == "jointLR1":
                scaler = 0.5
            else:
                scaler = 1
            handle = self.__joints[joint]
            for i in handle:
                self.sim.setJointTargetPosition(i, nicol_position.position[idx]/(len(handle) * scaler))

#        if type(joint_position) is dict:
#            for joint in joint_position.keys():
#                handle = self.__joints[joint]
#                #some joint names are aliases for groups of joints so loop over all of them and distribute angle equally
#                for i in handle:
#                    self.sim.setJointTargetPosition(i, joint_position[joint]/len(handle))
#        else:
#            keys = list(self.__joints.keys())
#            for i, position in enumerate(joint_position):
#                handle=self.__joints[keys[i]]
#                for j in handle:
#                    self.sim.setJointTargetPosition(j, joint_position[i]/len(handle))

        if block:
            self.wait_for_execution_js(NicolJointPosition(joint_position))
                      


    def set_pose_target(self, pose, only_position: bool = False, non_reachable_mode = "error",block=True) -> bool:
        """
        Solves the ik for a given pose and sets the joint target positions accordingly

        :param pose: NicolPose object containing the cartesian position and orientation in quarternions (Qx,Qy,Qz,Qw)
        :only_position: Wether the ik calculation should only take the position into account default = False
        """

        #get the orientation and turn it into an Rotation object
        orientation = transform.Rotation.from_quat(pose.orientation)

        #noticed orientation was of by 90° so this is a dirty fix
        r = transform.Rotation.from_rotvec(np.pi/2 * np.array([0, -1, 0]))
        orientation = orientation.__mul__(r)
        #transform it into a 4x4 transforamtion matrix
        matrix = np.pad(transform.Rotation.as_matrix(orientation),1)[1:,1:]
        position = pose.position
        matrix[:,3] = np.append(position,1)

        #try:
        if self.left:
            joints = self.ik.get_ik(matrix,robot_chain="left_arm",not_reachable_mode=non_reachable_mode, dist_acc=0.01,or_acc=10,include_orientation=not only_position,multiproc=True,num_generations=1000,initial_position=self.get_joint_position_list()[:8])
        else:
            joints = self.ik.get_ik(matrix,robot_chain="right_arm",not_reachable_mode=non_reachable_mode, dist_acc=0.01,or_acc=10,include_orientation=not only_position,
                    multiproc=True,num_generations=1000,initial_position=self.get_joint_position_list()[:8])
        self.set_joint_position(joints)
        #except:
        #        print("The given pose is not reachable, set non_reachable_mode to \"distance\", to find a close possible solution or set a different target pose that is in reach of the robot.")
        #    return True


    def get_joint_position(self, joints=None):
        """
        Getter for the joint positions
        :param joints: A list of joint names for which you want to get the positions. If none get all joint positions
        :return positions: A dict with joint name, position pairs
        """
        positions = {}
        if joints == None:
            joints = self.__joints.keys()
        for joint in joints:
            handle = self.__joints[joint]
            position = 0
            #sum up over all "sub joint" (for tenand based fingers)
            for h in handle:
                temp_pos = self.sim.getJointPosition(h)
                #print(f"handle: {handle}, position: {temp_pos}")
                position += temp_pos
            #since ring and littel finger are merged we have to devide by 2 again 
            if joint == "jointLR1":
                position /= 2
            #print(f"get position: joint name: {joint}, state: {position}")

            if joint in ["jointT1", "jointI1", "jointM1", "jointLR1"]:
                position *= 2
                position -= np.pi
            if joint == "jointT0":
                #print(f"ffff {nicol_position.position[idx]}")
                if not self.left:
                    position = -position
                position = position * 4
                position -= np.pi

            positions[joint] = position
        return NicolJointPosition(positions)


    def get_joint_position_list(self):
        """
        Gets a list of joint positions for all joints
        :return positions: A list of all joint angles of the arm
        """
        positions = []
        joints = self.__joints.keys()
        for joint in joints:
            handle = self.__joints[joint]
            positions.append(self.sim.getJointPosition(handle[0]))
        return positions


    def get_eef_pose(self):
            """
            getter for the end effector pose (tool0)

            :return: NicolPose object containing the eulerian coordinate and the orientation in quartenions (Qx,Qy,Qz,Qw)
            """
            #Pyrep Pose has the Form (X,Y,Z,Qx,Qy,Qz,Qw)
            pose = self.sim.getObjectPose(self.__eef, self.sim.handle_world)
            position = pose[:3]
            orientation = pose[3:]
            return NicolPose(position,orientation)

    def get_laser_distance(self):
        return self.sim.readProximitySensor(self.__laser)[1]

    def close_hand(self,amount,block = True):
        #if self.left:
        #    self.set_joint_position_for_hand([np.pi, np.pi* 2*amount] + [amount * np.pi * 2]*4,block = block)
        #else:
        #    self.set_joint_position_for_hand([-np.pi, np.pi/2*amount] + [amount * np.pi]*3, block=block)
        self.set_joint_position_for_hand([np.pi] + [((amount * (2 * np.pi)) - np.pi)] * 4, block=block)

    def set_joint_position_for_hand(self, joint_position, block = True) -> bool:
        """
        Sets the joint positions for the Hand. Since fingers are tenant based only one value per finger is used. The angle will be split uniformly on all three finger joints.

        :param joint_position: Can be either a list or ndarray with 6 entries (thumb position and 5 fingers), 
                                a dict of the form {joint_name:[names],joint_position:[positions]} or a NicolJointPosition object
        """

        joints = ['jointT0','jointT1','jointI1', 'jointM1', 'jointLR1']
        if type(joint_position) == list:
            joint_position = dict(zip(joints, joint_position))
        elif type(joint_position) == dict:
            joint_position = NicolJointPosition(joint_position)
        self.set_joint_position(joint_position,block=block)


    @internal
    def check_joint_diff_2(self, from_js: NicolJointPosition, to_js: NicolJointPosition, error=0.001, hand=False):
        if from_js is None:
            return True
        too_far = False
        #point to the current to_joint
        to_idx = 0
        #from_js has all the joints
        for from_idx, _ in enumerate(from_js.position):
            #is this name in to_js?
            if from_js.joint_name[from_idx] == to_js.joint_name[to_idx]:
                #print(f"current: name: {from_js.joint_name[from_idx]} joint state: {from_js.position[from_idx]}")
                #print(f"target: name: {to_js.joint_name[to_idx]} joint state: {to_js.position[to_idx]}")
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

    @internal
    def wait_for_execution_js(self, target_joint_position: NicolJointPosition = None, hand=False, error=0.001):
        assert target_joint_position is not None
        target_js = target_joint_position

        timeout_counter = 0
        current_js = None
        while self.check_joint_diff_2(current_js, target_js, error=error, hand=hand):
            #print("Waiting to execute js")
            current_js = self.get_joint_position()


            t_out = 20
            if timeout_counter >= 20 * t_out:
                break

            if timeout_counter >= t_out * 5 :
                print("Arm takes longer than usual")

            #step simulation
            self.client.step()
            timeout_counter += 1
            #print("sleep counter: " + str(timeout_counter))
        return




    @internal
    def wait_for_execution_js_2(self, target_joint_position: NicolJointPosition = None, hand=False, error=0.001):
        assert target_joint_position is not None
        target_js = target_joint_position.position

        timeout_counter = 0
        current_js = None
        while self.check_joint_diff(current_js, target_js, error=error, hand=hand):
            #print("Waiting to execute js")
            current_js = self.get_joint_position()
            if current_js is not None:
                current_js = current_js.position

            t_out = 60
            if timeout_counter >= 20 * t_out:
                break

            if timeout_counter >= 60 * 5 :
                print("Arm takes longer than usual")

            #step simulation
            self.client.step()
            timeout_counter += 1
            #print("sleep counter: " + str(timeout_counter))
        return


    @internal
    def check_joint_diff(self, from_js, to_js, error=0.001, hand=False):
        if from_js is None:
            return True
        too_far = False
        vector_len = len(to_js)
        
        for i in range(vector_len):
            if i < 6 and abs(to_js[i] - from_js[i]) > error:
                too_far = True
            elif i >= 6 and abs(to_js[i] - from_js[i]) > error * 100:
                too_far = True
        return too_far
