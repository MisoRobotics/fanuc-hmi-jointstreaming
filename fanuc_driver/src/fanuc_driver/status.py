import numpy as np
import socket

import rospy
from sensor_msgs.msg import JointState

from fanuc_msgs.msg import FanucStatus


class FanucStatusMonitor(object):
    """Monitors the state of the FANUC controller.

    Can be used to check if a program executed and finished.
    """
    DEBOUNCE_COUNT = 5

    def __init__(self):
        self.__checking = False
        self.__running = False
        self.__counter = 0
        self.__zone_counter = 0
        self.__zone = 0
        self.__joints = None
        self.__started = False
        self.__status_sub = rospy.Subscriber(
            '/fanuc_status', FanucStatus, self.__stat_callback, queue_size=1)
        self.__joint_sub = rospy.Subscriber(
            '/joint_states', JointState, self.__joint_callback, queue_size=1)

    @property
    def zone(self):
        """Returns the number of the highest priority activated zone.

        The number has the following designation:
            0 - no zones active
            1 - slowdown zone 1 active
            2 - slowdown zone 2 active
            3 - danger zone active
        """
        return self.__zone

    @property
    def joints(self):
        """Waits until joints are received, and then returns them
        Blocks if joint state has not been received yet.
        """
        while not rospy.is_shutdown() and self.__joints is None:
            rospy.sleep(0.001)  # Sleep for 1 ms while waiting for joints
        return self.__joints

    def __stat_callback(self, msg):
        """This callback just watches zone status
        """
        self.__zone = len(np.where([msg.slowdown_zone_1_active,
                                    msg.slowdown_zone_2_active,
                                    msg.danger_zone_active])[0])
        if self.__zone == 0:
            self.__zone_counter += 1
        else:
            self.__zone_counter = 0

    def __joint_callback(self, msg):
        """Handles joint position updates
        """
        self.__joints = msg.position
