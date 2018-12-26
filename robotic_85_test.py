import os
import time
import pdb
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
serverMode = p.GUI # GUI/DIRECT
robotUrdfPath = "./urdf/robotiq_85_gripper_simple.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# p.setPhysicsEngineParameter(enableFileCaching=0)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0, 0, -10) # NOTE
planeID = p.loadURDF("plane.urdf")
#######################################
###    define and setup robot       ###
#######################################
controlJoints = ["robotiq_85_left_knuckle_joint",
                 "robotiq_85_right_knuckle_joint",
                 "robotiq_85_left_inner_knuckle_joint",
                 "robotiq_85_right_inner_knuckle_joint",
                 "robotiq_85_left_finger_tip_joint",
                 "robotiq_85_right_finger_tip_joint"]

robotStartPos = [0, 0, 0.5]
robotStartOrn = p.getQuaternionFromEuler([0, 0, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn,
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

# load Object
ObjectID = p.loadURDF("./urdf/object_demo.urdf", [0, 0, 0.10], globalScaling=0.0045)

jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo",
                       ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity"])

joints = AttrDict()
dummy_center_indicator_link_index = 0

for i in range(numJoints):
    info = p.getJointInfo(robotID, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = jointTypeList[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity)
    joints[singleInfo.name] = singleInfo
    # register index of dummy center link
    if jointName == "gripper_roll":
        dummy_center_indicator_link_index = i


gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint"
mimic_joint_name = ["robotiq_85_right_knuckle_joint",
                    "robotiq_85_left_inner_knuckle_joint",
                    "robotiq_85_right_inner_knuckle_joint",
                    "robotiq_85_left_finger_tip_joint",
                    "robotiq_85_right_finger_tip_joint"]
mimic_multiplier = [1, 1, 1, -1, -1]

# id of gripper control user debug parameter
gripper_opening_angle_control = p.addUserDebugParameter("gripper_opening_angle",
                                                joints[gripper_main_control_joint_name].lowerLimit,
                                                joints[gripper_main_control_joint_name].upperLimit,
                                                0)
# position control
position_control_joint_name = ["center_x",
                               "center_y",
                               "center_z",
                               "gripper_roll",
                               "gripper_pitch",
                               "gripper_yaw"]

position_control_group = dict()
for jointName in position_control_joint_name:
    initial_value = 0
    if jointName == "gripper_pitch":
        initial_value = 1.57
    # ID list of position control user debug parameter
    position_control_group[jointName] = p.addUserDebugParameter(jointName,
                                                joints[jointName].lowerLimit,
                                                joints[jointName].upperLimit,
                                                initial_value)

# try:
while (1):
    # position control
    for jointName in position_control_joint_name:
        joint = joints[jointName]
        parameter = p.readUserDebugParameter(position_control_group[jointName])
        p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                targetPosition=parameter,
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)

    # gripper control
    gripper_opening_angle = p.readUserDebugParameter(gripper_opening_angle_control)


    p.setJointMotorControl2(robotID,
                            joints[gripper_main_control_joint_name].id,
                            p.POSITION_CONTROL,
                            targetPosition=gripper_opening_angle,
                            force=joints[gripper_main_control_joint_name].maxForce,
                            maxVelocity=joints[gripper_main_control_joint_name].maxVelocity)
    for i in range(len(mimic_joint_name)):
        joint = joints[mimic_joint_name[i]]
        p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle * mimic_multiplier[i],
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)

    # key control
    keys = p.getKeyboardEvents()
    # print(keys)
    # key "R" in down: reset the position control state to initial
    if 114 in keys:
        pass
        # CAN NOT WORK PROPERLY
        # p.removeAllUserDebugItems()
        # gripper_opening_angle_control = p.addUserDebugParameter("gripper_opening_angle",
        #                                                         joints[gripper_main_control_joint_name].lowerLimit,
        #                                                         joints[gripper_main_control_joint_name].upperLimit,
        #                                                         0)
        # for jointName in position_control_joint_name:
        #     initial_value = 0
        #     if jointName == "gripper_pitch":
        #         initial_value = 1.57
        #     # ID list of position control user debug parameter
        #     position_control_group[jointName] = p.addUserDebugParameter(jointName,
        #                                                                 joints[jointName].lowerLimit,
        #                                                                 joints[jointName].upperLimit,
        #                                                                 initial_value)

    # key "S" is down: print the state of dummy_center_link (for the grasp center and gripper pose)
    if (115 in keys) and (keys[115] == 3):
        dummy_center_indicator_link_info = p.getLinkState(robotID, dummy_center_indicator_link_index)
        center_position = dummy_center_indicator_link_info[0]
        center_orientation = dummy_center_indicator_link_info[1]
        print(center_position, '----', center_orientation)

    p.stepSimulation()
# except:
#     p.disconnect()