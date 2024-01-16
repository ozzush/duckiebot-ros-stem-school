#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
import keyboard

# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.5       # 50% throttle
DIRECTION_RIGHT = -1       # backward


class KeyboardControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(KeyboardControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            vel_left = 0
            vel_right = 0
            lforward_key = 'w'
            lbackwards_key = 's'
            rforward_key = 'o'
            rbackwards_key = 'l'
            if keyboard.is_pressed(lforward_key) and not keyboard.is_pressed(lbackwards_key):
                vel_left = 1 * THROTTLE_LEFT
            elif keyboard.is_pressed(lbackwards_key) and not keyboard.is_pressed(lforward_key):
                vel_left = -1 * THROTTLE_LEFT
            if keyboard.is_pressed(rforward_key) and not keyboard.is_pressed(rbackwards_key):
                vel_right = 1 * THROTTLE_RIGHT
            elif keyboard.is_pressed(rbackwards_key) and not keyboard.is_pressed(rforward_key):
                vel_right = -1 * THROTTLE_RIGHT
            message = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = KeyboardControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()