from threading import RLock

import numpy as np

from fanuc_driver import DigitalIO
from fanuc_driver.hmi_engine import AlarmInterface, \
                                    DataRegisterInterface, \
                                    IOInterface, \
                                    JointAngleInterface, \
                                    JointTorqueInterface, \
                                    SnpxManager, \
                                    SystemStatusInterface, \
                                    REGISTER_COUNT
from fanuc_driver.tp import Point
from fanuc_msgs.msg import FanucStatus
from fanuc_msgs.srv import SetJointSetpoint, \
                           SetJointSetpointResponse
from sensor_msgs.msg import JointState
import rospy


NUM_ALARMS = 10
NUM_PROGRAMS = 20
START_REGISTER = 1

STARTUP_SLEEP_TIME = .01


class HmiDriver(object):
    """Uses the hmi_engine package to provide the features needed for the
    driver stack. This includes resetting alarms, auto-starting the
    controller, configuring the controller HMI settings via bootstrap
    registers, reading digital IO, and reading signaling registers.
    """

    def __init__(self, server):
        # Used to lock across robot transactions that need to be atomic
        self.__config_lock = RLock()

        self.__snpx_manager = SnpxManager(server)
        self.__alarm_interface = AlarmInterface(num_alarms=NUM_ALARMS)
        self.__system_interface = SystemStatusInterface(
            num_programs=NUM_PROGRAMS)
        self.__io_interface = IOInterface()
        self.__register_interface = DataRegisterInterface(
            start_register=1, register_count=REGISTER_COUNT)
        self.__joint_angle_interface = JointAngleInterface()
        self.__joint_setpoint_interface = JointAngleInterface(
            var_name_prefix='PR[1]')
        self.__snpx_manager.add_interfaces([self.__alarm_interface,
                                            self.__system_interface,
                                            self.__io_interface,
                                            self.__register_interface,
                                            self.__joint_angle_interface,
                                            self.__joint_setpoint_interface])
        self.__alrm_whitelist = [
            (11, 1),    # SRVO-001 Operator Panel E-stop
            (11, 2),    # SRVO-002 Teach Pendant E-stop
            (11, 7),    # SRVO-007 External Emergency stops
            (11, 403),  # SRVO-403 DCS Cart. speed limit
            (24, 11),   # SYST-011 Failed to run task
            (24, 34),   # SYST-034 HOLD signal from SOP/UOP is lost
            (12, 106),  # INTP-106 (MISOMAIN, 29) Continue request failed
            (12, 222),  # INTP-222 (RUNMISO, 3) Call program failed
            (12, 267),  # INTP-267 (RUNMISO, 1) RUN stmt failed
            ]
        self.__last_alarms = []
        self.__spub = rospy.Publisher(
            '/fanuc_status', FanucStatus, queue_size=1)
        self.__jpub = rospy.Publisher(
            '/joint_states', JointState, queue_size=1)
        self.__set_joint_srv = rospy.Service(
            '/send_setpoint', SetJointSetpoint, self.__set_joint_setpoint)

    def system_startup(self):
        """Resets any alarms that are present, then kills all running fanuc
        controller user programs, then sends start signal to controller
        """
        while not rospy.is_shutdown() and not self.__check_alarms():
            rospy.sleep(STARTUP_SLEEP_TIME)

        if rospy.is_shutdown():
            rospy.logwarn('System stopped during HMI Driver startup')

        rospy.loginfo('Flippy controller is in startup!')

        with self.__config_lock:
            while (not rospy.is_shutdown() and
                   len([prog for prog in
                        self.__system_interface.program_statuses
                        if not prog.is_aborted]) > 0):
                self.__io_interface.abort()

            rospy.loginfo('Killed all previously running fanuc user '
                          'programs, restarting system')
            self.__joint_setpoint_interface.joint_angles = (
                self.__joint_angle_interface.joint_angles)
            self.__io_interface.start()

        rospy.loginfo('Flippy controller startup complete!')

    def spin(self):
        """Checks for alarms, and then publishes the fanuc status message
        """
        self.__check_alarms()
        # Read out and publish fanuc_status
        self.__spub.publish(self.__fanuc_status_message)
        self.__jpub.publish(self.__joint_state_message)

    def __check_alarms(self):
        """Check alarms, if there is an alarm, check that all alarms are
        in whitelist, and reset if so.

        Returns:
            True if no alarms found, False if alarms were found (regardless
                of whether reset was successful)
        """
        alarms = self.__alarm_interface.alarms
        unrecognized_alarms = [
            alm for alm in alarms
            if alm.alarm_identifier not in self.__alrm_whitelist]

        if len(alarms) >= NUM_ALARMS:
            rospy.logwarn('There are the max number of %d alarms on the '
                          'robot. Potentially missing some alarms',
                          NUM_ALARMS)

        if len(alarms) > 0:
            rospy.logdebug('Alarms on Robot: %s', alarms)

            if len(unrecognized_alarms) == 0:
                rospy.loginfo('Alarm in whitelist, resetting')

                with self.__config_lock:
                    self.__io_interface.reset()
            else:
                rospy.logerr_throttle(
                    5, 'At least one alarm is not in whitelist, check teach '
                    'pendant. All alarms: %s' % (alarms,))

        return len(alarms) == 0

    @property
    def __fanuc_status_message(self):
        """Gets the fanuc status message
        """
        with self.__config_lock:
            read_input = self.__io_interface.read_digital_input
            pneumatic_pressure_low = read_input(DigitalIO.PNEUMGOOD) == 0
            slowdown_zone_1_active = read_input(DigitalIO.WARN1) == 0
            slowdown_zone_2_active = read_input(DigitalIO.WARN2) == 0
            danger_zone_active = read_input(DigitalIO.SIR1) == 0
        msg = FanucStatus(
            slowdown_zone_1_active=slowdown_zone_1_active,
            slowdown_zone_2_active=slowdown_zone_2_active,
            danger_zone_active=danger_zone_active,
            pneumatic_pressure_low=pneumatic_pressure_low)
        msg.header.stamp = rospy.Time.now()
        return msg

    @property
    def __joint_state_message(self):
        NO_JOINTS = 6
        with self.__config_lock:
            joints = self.__joint_angle_interface.joint_angles
        joints = Point.to_canonical([np.deg2rad(joint) for joint in joints])
        assert len(joints) == NO_JOINTS, (
            'Only %d joints are supported' % NO_JOINTS)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['joint_%d' % (i + 1) for i in range(NO_JOINTS)]
        msg.position = joints
        return msg

    def __set_joint_setpoint(self, req):
        """
        """
        deg_joints = Point.to_robot([np.rad2deg(j) for j in req.joints])
        rospy.logdebug("setting joints to %s", str(deg_joints))
        res = SetJointSetpointResponse()
        assert len(req.joints) == 6
        res.success = False
        with self.__config_lock:
            self.__joint_setpoint_interface.joint_angles = deg_joints
        res.success = True
        return res
