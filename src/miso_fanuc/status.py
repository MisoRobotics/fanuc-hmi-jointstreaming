import numpy as np
import socket

from miso_fanuc import Registers
from miso_msgs.msg import FanucStatus
import rospy


class FanucStatusMonitor(object):
    """Monitors the state of the FANUC controller.

    Can be used to check if a program executed and finished.
    """
    DEBOUNCE_COUNT = 5

    def __init__(self, server_address):
        self.__checking = False
        self.__running = False
        self.__counter = 0
        self.__zone_counter = 0
        self.__zone = 0
        self.__started = False
        self.__status_sub = rospy.Subscriber(
            '/fanuc_status', FanucStatus, self.__callback, queue_size=1)

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

    def __callback(self, msg):
        """This callback just watches zone status
        """
        self.__zone = len(np.where([msg.slowdown_zone_1_active,
                                    msg.slowdown_zone_2_active,
                                    msg.danger_zone_active])[0])
        if self.__zone == 0:
            self.__zone_counter += 1
        else:
            self.__zone_counter = 0
