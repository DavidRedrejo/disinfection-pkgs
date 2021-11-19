#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_data_node', anonymous=True, log_level=rospy.INFO)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

group.set_end_effector_link("rbkairos_spray_tool_end_link")
#rospy.logerr(group.set_pose_reference_frame("rbkairos_odom"))

#group.compute_cartesian_path()

""" pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.46
pose_target.position.y = 0.0
pose_target.position.z = 1.1
pose_target.orientation.w = 1

group.set_pose_target(pose_target)

plan = group.plan() """

rospy.loginfo("Reference frame: %s" % group.get_planning_frame())
rospy.logwarn("End effector: %s" % group.get_end_effector_link())
rospy.loginfo("Robot Groups:")
rospy.loginfo(robot.get_group_names())
rospy.loginfo("Current Joint Values:")
rospy.loginfo(group.get_current_joint_values())
rospy.loginfo("Current Pose:")
rospy.logerr(group.get_current_pose())
rospy.loginfo("Robot State:")
rospy.loginfo(robot.get_current_state())

#rospy.sleep(5)

moveit_commander.roscpp_shutdown()
