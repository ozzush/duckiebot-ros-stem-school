#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

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
        self._vel_left = 0
        self._vel_right = 0
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        while not rospy.is_shutdown():
            command = input("input command [a, w, s, d]: ")
            if command == 'w':
                self._vel_left = THROTTLE_LEFT
                self._vel_right = THROTTLE_RIGHT
            elif command == 's':
                self._vel_left = - THROTTLE_LEFT
                self._vel_right = - THROTTLE_RIGHT
            elif command == 'a':
                self._vel_left = - THROTTLE_LEFT
                self._vel_right = THROTTLE_RIGHT
            elif command == 'd':
                self._vel_left = THROTTLE_LEFT
                self._vel_right = - THROTTLE_RIGHT
            else:
                self._vel_left = 0
                self._vel_right = 0
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
            self._publisher.publish(message)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    # create the node
    node = KeyboardControlNode(node_name='keyboard_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()