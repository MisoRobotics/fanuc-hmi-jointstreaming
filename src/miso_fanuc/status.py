import numpy as np
import socket

from miso_fanuc import Registers
from miso_msgs.msg import FanucStatus
from miso_msgs.srv import ResetFanucAck
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
        self.__ack_target = None
        svc = '/reset_fanuc_ack'
        rospy.wait_for_service(svc)
        self.__reset_ack_svc = rospy.ServiceProxy(svc, ResetFanucAck)

    @property
    def running(self):
        """Returns True if the robot is currently executing a trajectory
        program.
        """
        return self.__running

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
    def finished(self):
        """Returns True if a program has executed and finished.
        """

        return (self.__zone == 0
                and self.__zone_counter >= FanucStatusMonitor.DEBOUNCE_COUNT
                and self.__tp_ack)

    @property
    def __tp_ack(self):
        """Checks for ACK register at target and signal register cleared.
        """
        return (self.__sig_reg == 0
                and self.__ack_reg == self.__ack_target)

    def reset_for_new_action(self, ack_target_value):
        """This function resets the program status watch and should be
        called before sending an execution packet to the robot.
        """
        assert isinstance(ack_target_value, int)
        assert ack_target_value != 0, 'Target reg value cannot be zero'
        res = self.__reset_ack_svc()
        if not res.success:
            rospy.logerr('Resetting ACK register failed!')
        # Wait for the fanuc status messages to reflect the ack register setting
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and (self.__ack_reg != 0 or self.__sig_reg != 0):
             rate.sleep()
        self.__ack_reg = 0
        self.__sig_reg = 0
        self.__zone_counter = 0
        self.__checking = True
        self.__ack_target = ack_target_value

    def __callback(self, msg):
        """This callback just watches zone status
        """
        self.__sig_reg = msg.sig_reg
        self.__ack_reg = msg.ack_reg
        self.__zone = len(np.where([msg.slowdown_zone_1_active,
                                    msg.slowdown_zone_2_active,
                                    msg.danger_zone_active])[0])
        if self.__zone == 0:
            self.__zone_counter += 1
        else:
            self.__zone_counter = 0
