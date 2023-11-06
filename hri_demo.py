# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

#!/usr/bin/env python

start_coppelia = True
start_gazebo = False
use_platform = False

############################ Import #################################
if start_coppelia:
    import nicol_api
    import matplotlib.pyplot as plt
    from nicol_base import NicolJointPosition, NicolPose
    from nicol_cycleik import NicolCycleIK
    try:
        from dependencies.nicol_tts.nicol_speech import NICOL_TALKER

        class Participant:
            def __init__(self, speaker):
                self.talker = NICOL_TALKER()
                self.speaker = speaker

            def say(self, text):
                self.talker.synth(text,"./temp.wav",speaker_idx=self.speaker)
                self.talker.play("./temp.wav",block = True)
    except:
        pass


else:
    from nias_action.core.nicol_cycleik import NicolCycleIK
    from nias_action.core.nicol_base import NicolJointPosition, NicolPose
    
import random
import numpy as np
import random
import time
import pathlib

import cv2 as cv

############################ local functions #################################

#sample random pose from reachable space
def get_random_pose(left=False):
    if left:
        x = random.uniform(0.35, 0.75)
        y = random.uniform(0.2, 0.7)
        z = random.uniform(0.9, 1.2)
    else:
        x = random.uniform(0.35, 0.75)
        y = random.uniform(-0.2, -0.7)
        z = random.uniform(0.9, 1.2)

    if left:
        return NicolPose(position=[x, y, z], orientation=[np.pi/4, -np.pi/4, 0., 0.])
    else:
        return NicolPose(position=[x, y, z], orientation=[0., 0., np.pi/4, np.pi/4])

#execute a trajectory in joint space
def execute_js_trajectory(js_array, trajectory):
    for k in range(len(js_array)):
        head.set_pose_target(right.get_eef_pose())
        js = js_array[k]
        if not start_coppelia:
            fk_pose = right.calc_fk(NicolJointPosition(js))
            print("\n\nHere\n\n")
        js_reachable = right.set_joint_position(js)


def calculate_trajectory(origin, target, points_per_trajectory):
    delta_x = (target.position.x - origin.position.x) / (points_per_trajectory - 1)
    delta_y = (target.position.y - origin.position.y) / (points_per_trajectory - 1)
    delta_z = (target.position.z - origin.position.z) / (points_per_trajectory - 1)
    trajectory = []
    current_point = [0., 0., 0.]
    for i in range(points_per_trajectory):
        current_point[0] = origin.position.x + (i * delta_x)
        current_point[1] = origin.position.y + (i * delta_y)
        current_point[2] = origin.position.z + (i * delta_z)
        #print(np.array(current_point))
        #print(np.array(orientation))
        trajectory.append(NicolPose(position=current_point, orientation=[origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w]))
    return trajectory

def point_at(point):
    point = np.array(point)
    eef_point = point.copy()
    eef_point -= np.array([0.165, 0.05,0])
    eef_point[2] = 0.95

    orientation = [0.21738 , 0.24031, 0.62923 ,0.70645]
    target_pose = NicolPose(eef_point, orientation)
    return target_pose

############################ Initialize #################################
nicol = None
if start_coppelia:
    nicol = NicolCycleIK(scene="./nicol_hri.ttt",start_scene=True,talker=True)
    participant = Participant("p225")
else:
    if use_platform:
        nicol = NicolCycleIK(launch_nicol=False, sleep_timeout=0.003)
    else:
        nicol = NicolCycleIK(launch_nicol=False, service_timeout=40)


head  = nicol.head()    # 3D-Printed head structure
left  = nicol.left()    # Left  OpenManipulator + RH8D
right = nicol.right()   # Right OpenManipulator + RH8D

############################ Set base Pose ###########################

head.set_joint_position([0,0],block=True)
right.set_joint_position([1.57] + [0.] * 7,block=True)
left.set_joint_position([-1.57] + [0.] * 7,block=True)
if not start_coppelia:
    right.set_joint_position_for_hand([-np.pi] * 5,block=True)

#right.close_hand(0.85)
#print(f"joint_position: {right.get_joint_position()}")

#right.set_joint_position_for_hand([np.pi] * 5,block=True)

#print(f"joint_position: {right.get_joint_position()}")

#right.close_hand(0.85)

#time.sleep(5)

############################ Face Expressions ###########################

head.set_face_expression("neutral")
head.say("Hello I am NICOL the Neural Inspired Collaborator, what can I do for you?", block= True)

if start_coppelia:
    participant.say("Hello NICOL, could you please describe your functionalities for me?")
else:
    input("\n Participant Speak \n")

head.set_face_expression("happiness")
head.say("Sure. I can talk via text to speech, see with the two cameras in my head and freely move my two articulated arms to gesture and manipulate objects. Additionally I can communicate emotions with my face expressions.", block= True)

if start_coppelia:
    participant.say("This sounds great. Let's start with the face expressions. Please show me what you are capable of.")
else:
    input("\n Participant Speak \n")

head.say("Okay, lets make a little game out of this. I show you my face expressions and you guess the associated emotion", block=True)

if start_coppelia:
    participant.say("Great, this sounds like fun!")
else:
    input("\n Participant Speak \n")
 
for expression in ["happiness", "sadness", "anger", "fear"]:
    head.set_face_expression(expression)
    head.say("Can you guess this expression?", block=True)
    answer = input("\n Participant Speak \n")
    if start_coppelia:
        participant.say(f"I think it's {answer}.")
    if answer  == expression:
        head.say("Great you got it right.", block=True)
    else:
        head.say(f"No sorry I was trying to look {expression} there.", block=True)


############################ Inverse Kinematics #########################

if start_coppelia:
    participant.say("Hey NICOL, I heared you got a new Inverse Kinematics system. Can you please show me some of your poses?")
else:
    input("\n Participant Speak \n")
head.say("Yes I show you some solutions for random points",block = True)

for i in range(5):
    pose = get_random_pose()
    right.set_pose_target(pose)

for i in range(5):
    pose = get_random_pose(True)
    left.set_pose_target(pose)
############################ Balance Demo #########################
'''
points_per_trajectory = 100
if start_coppelia:
    participant.say("Now show me some balancing, please get ready to balance the ball on your hand along the given trajectories")
else:
    input("\n Participant Speak \n")

orientation = [0.5, -0.5, 0.5, 0.5]

right.set_joint_position([1.57, 0., 0., 0., 0., 0., 0., 0.])
head.set_pose_target(right.get_eef_pose())
current_pose = NicolPose(position=[0.712, -0.513, 1.172], orientation=orientation)
right.set_pose_target(current_pose)
head.say("Hey, I am ready to balance the baseball for you!",block=True)
head.say("Please, press enter when the device is placed in my hand",block=True)
input("Press enter when device is placed in hand")
if start_coppelia:
    sim = nicol.sim
    target_handle=sim.getObject("/Ball")
    position = right.get_eef_pose().position.as_list()
    position[2]+= 0.06
    sim.setObjectPosition(target_handle,sim.handle_world,position)
    nicol.step_simulation(5)

head.say("Thank you!")


target_pose = NicolPose(position=[0.5, -0.7, 1.25], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose, points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
print(js_array.shape)
execute_js_trajectory(js_array, trajectory)

head.say("Finished the first trajectory")

current_pose = target_pose
target_pose = NicolPose(position=[0.5, 0.0, 1.25], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose, points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
execute_js_trajectory(js_array, trajectory)

head.say("Finished the second trajectory")

current_pose = target_pose
target_pose = NicolPose(position=[0.5, 0.0, 0.95], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose, points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
execute_js_trajectory(js_array, trajectory)

head.say("Finished the third trajectory")

current_pose = target_pose
target_pose = NicolPose(position=[0.5, -0.65, 0.95], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose, points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
execute_js_trajectory(js_array, trajectory)

head.say("Finished the fourth trajectory")

current_pose = target_pose
target_pose = NicolPose(position=[0.5, -0.65, 1.25], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose, points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
execute_js_trajectory(js_array, trajectory)

head.say("Finished the fifth trajectory")

current_pose = target_pose
target_pose = NicolPose(position=[0.712, -0.513, 1.172], orientation=orientation)
trajectory = calculate_trajectory(current_pose, target_pose,points_per_trajectory)
js_array = right.calc_pose_batch(trajectory)
execute_js_trajectory(js_array, trajectory)

head.say("Please remove device from hand and press enter")

input("Please remove device from hand and press enter")
if start_coppelia:
    sim.setObjectPosition(target_handle,sim.handle_world,[0,0,0])
else:
    input("\n Participant Speak \n")

right.set_joint_position([1.57] + [0.] * 7,block=True)

'''
############################ HRI Stuff #######################################

block_positions = {
    "red": [0.6,-0.52,0.86],
    "green": [0.6, -0.375,0.825],
    "yellow": [0.6, -0.225,0.825],
    "blue": [0.6, -0.075,0.825],
    "orange":[0.6, 0.075,0.825]
    }

if start_coppelia:
    for name in block_positions.keys():
        sim = nicol.sim
        target_handle=sim.getObject("/"+name)
        position = block_positions[name]
        if name == "red":
            position[2] = 0.91
        sim.setObjectPosition(target_handle,sim.handle_world,position)
        nicol.step_simulation(1)
else:
    input("please place the objects on the table")

if start_coppelia:  
    participant.say("Can you please show me where the green block is?")
else:
    input("\n Participant Speak \n")

head.set_pose_target(NicolPose(block_positions["green"],[0,0,0,0]))

# do the pointing
right.set_joint_position_for_hand([0.0,0.0, -3, np.pi, np.pi])
right.set_pose_target(NicolPose([0.5,-0.65,1],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([0.4,-0.5,1],[0,0,np.pi/4, np.pi/4]))

if start_coppelia:
    nicol.step_simulation(50)

right.set_pose_target(point_at(block_positions["green"]))


head.say("I think it's here", block=True)
if start_coppelia:
    participant.say("Good job")
else:
    input("\n Participant Speak \n")

head.set_face_expression("happiness")

if start_coppelia:
    participant.say("Now please show me where the orange block is")
else:
    input("\n Participant Speak \n")

head.set_face_expression("neutral")
head.set_pose_target(NicolPose(block_positions["orange"],[0,0,0,0]))

#do the pointing
right.set_pose_target(point_at(block_positions["orange"]))
head.say("I think it's here", block=True)


if start_coppelia:
    participant.say("Oh sorry I ment the yellow block")
else:
    input("\n Participant Speak \n")

head.set_face_expression("surprise")
head.say("Okay let me try again",block=True)
head.set_face_expression("neutral")
head.set_pose_target(NicolPose(block_positions["yellow"],[0,0,0,0]))

#do the pointing
right.set_pose_target(point_at(block_positions["yellow"]))

if start_coppelia:
    participant.say("Great you got it")
else:
    input("\n Participant Speak \n")

head.set_face_expression("happiness")
if start_coppelia:
    participant.say("Now please show me where the red block is")
else:
    input("\n Participant Speak \n")
head.set_face_expression("neutral")
head.set_pose_target(NicolPose(block_positions["blue"],[0,0,0,0]))

#do the pointing
right.set_pose_target(point_at(block_positions["blue"]))
head.say("I think it's here", block=True)

if start_coppelia:
    participant.say("No that's the blue block")
else:
    input("\n Participant Speak \n")

head.set_face_expression("sadness")
head.say("I'm sorry, let me try again",block=True)
head.set_face_expression("neutral")
head.set_pose_target(NicolPose(block_positions["red"],[0,0,0,0]))

#do the pointing
right.set_pose_target(point_at(block_positions["red"]))


if start_coppelia:
    participant.say("Good job now you got it right")
else:
    input("\n Participant Speak \n")

head.set_face_expression("happiness")

if start_coppelia:
    participant.say("Now please lift the red block and place it behind the blue block")
else:
    input("\n Participant Speak \n")

head.set_face_expression("neutral")

if start_coppelia:
    sim = nicol.sim
    target_handle=sim.createPrimitiveShape(sim.primitiveshape_spheroid,[0.1]*3)

right.set_joint_position([np.pi/2]+[0]*7,block=True)
right.close_hand(0)

x = block_positions["red"][0]
y = block_positions["red"][1]
z = block_positions["red"][2]

if start_coppelia:
    sim = nicol.sim
    cube = sim.getObject("/red")
    sim.setObjectPose(cube, sim.handle_world, [x,y,z ,4.0691401181642666e-07, 2.0342476147886475e-07, -3.54541316480672e-06, 0.9999999999936116])

head.set_pose_target(NicolPose(block_positions["red"],[0,0,0,0]))

right.set_pose_target(NicolPose([x-0.15,y-0.15,z+0.3],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x-0.1,y-0.1,z+0.05],[0,0,np.pi/4, np.pi/4]))
if not start_coppelia:
    right.set_pose_target(NicolPose([x-0.08,y-0.08,z+0.03],[0,0,np.pi/4, np.pi/4]))
    right.set_pose_target(NicolPose([x-0.02,y-0.05,z+0.03],[0,0,np.pi/4, np.pi/4]))
else:
    right.set_pose_target(NicolPose([x-0.08,y-0.08,z],[0,0,np.pi/4, np.pi/4]))
    right.set_pose_target(NicolPose([x-0.02,y-0.05,z],[0,0,np.pi/4, np.pi/4]))   
right.close_hand(0.5)

if start_coppelia:
    nicol.step_simulation(100)

right.set_joint_position([np.pi/2,0,0,0,0,0,0,0])

x = block_positions["blue"][0] - 0.15
y = block_positions["blue"][1] - 0.05
z = 0.90

head.set_pose_target(NicolPose(block_positions["blue"],[0,0,0,0]))

right.set_pose_target(NicolPose([x,y,z+0.2],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x,y,z+0.1],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x,y,z],[0,0,np.pi/4, np.pi/4]))

right.close_hand(0)
if start_coppelia:
    nicol.step_simulation(100)

right.set_pose_target(NicolPose([x - 0.0,y-0.02,z+0.01],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.0,y-0.05,z+0.01],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.05,y-0.1,z+0.025],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.05,y-0.2,z+0.1],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x + 0.1,y-0.3,z+0.2],[0,0,np.pi/4, np.pi/4]))

right.set_joint_position([np.pi/2])
right.set_joint_position([np.pi/2]+[0]*7)

head.set_joint_position(NicolJointPosition([0.0, 0.0]))
right.set_joint_position([1.57, 0., 0., 0., 0., 0., 0., 0.])

#reset right arm in safe position
right.set_joint_position([0.31, 0.2, -0.2, 1.5, 1.28, 0.0, 0.0, 0.0])


if start_coppelia:
    nicol.stop_simulation()
    nicol.stop_coppelia()