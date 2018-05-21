'''
Converts trajectories to TP .ls format.
'''

from __future__ import print_function

import ftplib
from io import BytesIO
import math
import numpy as np

from miso_fanuc import Registers
import rospy


class Point(object):
    """Maps canonical joint angles to Fanuc robot joint angles
    """

    def __init__(self, register_no, joints, timestamp_sec):
        assert isinstance(register_no, int)
        assert register_no > 0
        assert register_no < 10000
        assert len(joints) == 6
        assert isinstance(joints, np.ndarray)

        self.i = register_no
        self.timestamp_sec = timestamp_sec
        self.joints = [j * 180 / math.pi for j in Point.to_robot(joints)]

    # Some axes are reversed and there is a J2-J3 coupling
    TO_ROBOT = np.array(
        [[1,  0,  0,  0,  0,  0],
         [0,  1,  0,  0,  0,  0],
         [0, -1,  1,  0,  0,  0],
         [0,  0,  0, -1,  0,  0],
         [0,  0,  0,  0,  1,  0],
         [0,  0,  0,  0,  0, -1]])

    TO_CANONICAL = np.linalg.inv(TO_ROBOT)

    @staticmethod
    def to_robot(joints):
        return np.dot(Point.TO_ROBOT, joints)

    @staticmethod
    def to_canonical(joints):
        return np.dot(Point.TO_CANONICAL, joints)

    def delta_time(self, last_point):
        return self.timestamp_sec - last_point.timestamp_sec
