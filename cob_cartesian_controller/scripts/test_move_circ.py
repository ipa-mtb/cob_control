#! /usr/bin/env python
import math
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

if __name__ == '__main__':
    rospy.init_node('test_move_circ')
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    client.wait_for_server()
    rospy.logwarn("...done")

    # Fill in the goal here
    goal = CartesianControllerGoal()

    goal.move_type = CartesianControllerGoal.CIRC
    goal.move_circ.pose_center.position.x = 0.0
    goal.move_circ.pose_center.position.y = 0.7
    goal.move_circ.pose_center.position.z = 1.0
    goal.move_circ.pose_center.orientation.x = 0.0
    goal.move_circ.pose_center.orientation.y = 0.0
    goal.move_circ.pose_center.orientation.z = 0.0
    goal.move_circ.pose_center.orientation.w = 1.0
    goal.move_circ.frame_id = 'world'

    goal.move_circ.start_angle = 0 * math.pi / 180.0
    goal.move_circ.end_angle = 90 * math.pi / 180.0
    goal.move_circ.radius = 0.3

    goal.profile.vel = 0.1
    goal.profile.accl = 0.2
    goal.profile.profile_type = Profile.SINOID

    #print goal

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result()
