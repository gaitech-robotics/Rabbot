#
# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#!/usr/bin/env python
import sys
import rospy
import actionlib
import math
import time
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState


def gripper_move(gripper_client, width, speed, force, timeout=10):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = force
    gripper_client.send_goal(goal)


def is_motor_on():
    rospy.wait_for_service(
        '/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy(
            '/get_motor_state', GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

rospy.init_node("gripper_test")

gripper_client = actionlib.SimpleActionClient(
    '/gripper_move', GripperMoveAction)

while True:
    if is_motor_on() is not True:
        print >> sys.stderr, "  Please motor on."
        continue
    gripper_move(gripper_client, 0.03, 80, 10, 10)
    print("Gripper open")
    time.sleep(2)
    gripper_move(gripper_client, 0.00, 80, 10, 10)
    print("Gripper close")
    time.sleep(2)
