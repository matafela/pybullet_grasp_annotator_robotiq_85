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

robotStartPos = [0, 0, 0.1]
robotStartOrn = p.getQuaternionFromEuler([0, 0, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn,
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo",
                       ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity"])

joints = AttrDict()

for i in range(numJoints):
    info = p.getJointInfo(robotID, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = jointTypeList[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    controllable = True if jointName in controlJoints else False
    singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity)
    joints[singleInfo.name] = singleInfo
    # if info.type=="REVOLUTE": # set revolute joint to static
    #     p.setJointMotorControl2(robotID, info.id, p.POSITION_CONTROL, targetVelocity=0, force=0)


###############################################
## set up mimic joints in robotiq_c2 gripper ##
###############################################
# mimicParentName = "robotiq_85_left_knuckle_joint"
# mimicChildName = ["robotiq_85_right_knuckle_joint",
#                   "robotiq_85_left_inner_knuckle_joint",
#                   "robotiq_85_right_inner_knuckle_joint",
#                   "robotiq_85_left_finger_tip_joint",
#                   "robotiq_85_right_finger_tip_joint"]
#
# mimicMul = [-1, -1, -1, 1, 1]
# mimicChildList = []
# parent = joints[mimicParentName]
# constraints = dict()
# for i, name in enumerate(mimicChildName):
#     child = joints[name]
#     c = p.createConstraint(robotID, parent.id,
#                            robotID, child.id,
#                            jointType=p.JOINT_GEAR,
#                            jointAxis=[0,0,1],
#                            parentFramePosition=[0,0,0],
#                            childFramePosition=[0,0,0])
#     p.changeConstraint(c, gearRatio=mimicMul[i], maxForce=child.maxForce)
#     constraints[name] = c

# start simulation

gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint"
mimic_joint_name = ["robotiq_85_right_knuckle_joint",
                    "robotiq_85_left_inner_knuckle_joint",
                    "robotiq_85_right_inner_knuckle_joint",
                    "robotiq_85_left_finger_tip_joint",
                    "robotiq_85_right_finger_tip_joint"]
mimic_multiplier = [1, 1, 1, -1, -1]

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
    position_control_group[jointName] = p.addUserDebugParameter(jointName,
                                                joints[jointName].lowerLimit,
                                                joints[jointName].upperLimit,
                                                0)

try:
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

        p.stepSimulation()
except:
    p.disconnect()

