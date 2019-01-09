import sys
import copy
import rospy
# import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
import numpy as np
import ipdb
from tf import transformations

def pose_msg_to_matrix(pose_msg):
    """
    :param pose_msg: a geometry_msgs.msg.Pose instance
    """
    qx = pose_msg.orientation.x
    qy = pose_msg.orientation.y
    qz = pose_msg.orientation.z
    qw = pose_msg.orientation.w

    pose_matrix = transformations.quaternion_matrix(np.array([qx, qy, qz, qw]))
    pose_matrix[0, 3] = pose_msg.position.x
    pose_matrix[1, 3] = pose_msg.position.y
    pose_matrix[2, 3] = pose_msg.position.z

    return pose_matrix

def matrix_to_pose_msg(matrix_4x4):
    """
    :param matrix_4x4: a numpy matrix
    """
    tmp_rotation = np.eye(4)
    tmp_rotation[:3, :3] = matrix_4x4[:3, :3]
    qx, qy, qz, qw = transformations.quaternion_from_matrix(tmp_rotation)

    pose_msg = geometry_msgs.msg.Pose()
    pose_msg.orientation.x = qx
    pose_msg.orientation.y = qy
    pose_msg.orientation.z = qz
    pose_msg.orientation.w = qw

    pose_msg.position.x = matrix_4x4[0, 3]
    pose_msg.position.y = matrix_4x4[1, 3]
    pose_msg.position.z = matrix_4x4[2, 3]
    return pose_msg

if __name__ == '__main__':
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    # group_name = "manipulator"
    # group = moveit_commander.MoveGroupCommander(group_name)
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                            moveit_msgs.msg.DisplayTrajectory,
    #                                            queue_size=20)
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/4
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = pi/3
    # group.go(joint_goal, wait=True)
    # group.stop()

    pose_matrix = np.array([[-0.2178185,  0.2840907, 0.9337277,  0],
                            [0.9337277,  -0.2178185, 0.2840907,  0],
                            [0.2840907,   0.9337277, -0.2178185, 0],
                            [0 ,          0,          0,         1]])
    pose_matrix[0, 3] = 0.4
    pose_matrix[1, 3] = 0.1
    pose_matrix[2, 3] = 0.4

    # given a pose matrix  
    pose_msg_goal = matrix_to_pose_msg(pose_matrix)
    # group.set_pose_target(pose_msg_goal)
    # plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()
    ipdb.set_trace()
    # given a pose quaternion
    pose_msg_goal = geometry_msgs.msg.Pose()
    pose_msg_goal.orientation.w = 0.2943401
    pose_msg_goal.position.x = 0.5517741
    pose_msg_goal.position.y = 0.5517741
    pose_msg_goal.position.z = 0.5517741
    # group.set_pose_target(pose_msg_goal)
    # plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()
    
    pose_matrix = np.array([[-0.2178185,  0.2840907, 0.9337277,  0],
                            [0.9337277,  -0.2178185, 0.2840907,  0],
                            [0.2840907,   0.9337277, -0.2178185, 0],
                            [0 ,          0,          0,         1]])
    pose_matrix[0, 3] = 0.6
    pose_matrix[1, 3] = 0.2
    pose_matrix[2, 3] = 0.6

    # given a pose matrix  
    pose_msg_goal = matrix_to_pose_msg(pose_matrix)
    ipdb.set_trace()

    # group.set_pose_target(pose_msg_goal)
    # plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # waypoints = []
    # scale = 25
    # wpose = group.get_current_pose().pose
    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))


    # (plan, fraction) = group.compute_cartesian_path(
    #                                 waypoints,   # waypoints to follow
    #                                 0.01,        # eef_step
    #                                 0.0)         # jump_threshold
    # group.execute(plan)
