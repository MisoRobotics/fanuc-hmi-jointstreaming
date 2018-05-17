'''
Converts trajectories to TP .ls format.
'''

from __future__ import print_function

import ftplib
from io import BytesIO
import math
import numpy as np

from miso_fanuc import Registers
from misopy.console import green
import rospy

CNT0_SETTLING_TIME = 0.2
MIN_WAIT_TIME = 0.02


class Point(object):

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

    def __str__(self):
        line = '''\
P[{:d}]{{
   GP1:
    UF : 0, UT : 1, 
    J1={:10.3f} deg,    J2={:10.3f} deg,    J3={:10.3f} deg,
    J4={:10.3f} deg,    J5={:10.3f} deg,    J6={:10.3f} deg
}};\
'''.format(*([self.i] + list(self.joints)))
        return line

    def instruction(self, line_no, cnt=100):
        return '{:4d}:J P[{:d}] 60% CNT{:d}    ;'.format(
            line_no, self.i, cnt)


def make_trajectory_program(prog_name, goal):
    def incrementer(start):
        line_number = start
        while True:
            yield line_number
            line_number += 1

    line_number = incrementer(start=1)

    assert np.array(goal.points[0].positions).shape[0] == 6

    no_points = len(goal.points)
    points = [Point(register_no=i + 1,
                    joints=np.array(goal.points[i].positions),
                    timestamp_sec=goal.points[i].time_from_start.to_sec())
                    for i in range(no_points)]
    
    instruction_list = []
    instruction_list.append(points[0].instruction(next(line_number), cnt=0))
    for i in range(no_points):
        delta_time = (0. if i == 0
                      else points[i].delta_time(points[i-1]))
        assert delta_time >= 0.0, 'Delta time is less than zero {}'.format(delta_time)
        if delta_time > 0.09:
            rospy.loginfo('Time delay greater than 0.09 seconds, inserting Fanuc WAIT')
            # Let program go to final position of previous point
            instruction_list.append(points[i-1].instruction(next(line_number), cnt=0))
            # Settling time from above instruction takes ~0.2 seconds
            if delta_time > CNT0_SETTLING_TIME + MIN_WAIT_TIME:
                delta_time -= CNT0_SETTLING_TIME 
                instruction_list.append('{:4d}:  WAIT {}(sec) ;'.format(next(line_number),
                                                                        delta_time))
        instruction_list.append(points[i].instruction(next(line_number), cnt=100))

    instruction_list.append(points[-1].instruction(next(line_number), cnt=0))
    instructions = '\n'.join(instruction_list)
    positions = '\n'.join([str(p) for p in points])

    line = '''\
/PROG  %s
/ATTR
OWNER       = MNEDITOR;
COMMENT     = "";
PROG_SIZE   = 571;
CREATE      = DATE 17-05-10  TIME 05:00:06;
MODIFIED    = DATE 17-05-11  TIME 08:53:30;
FILE_NAME   = ;
VERSION     = 0;
LINE_COUNT  = 0;
MEMORY_SIZE = 1055;
PROTECT     = READ_WRITE;
TCD:  STACK_SIZE    = 0,
      TASK_PRIORITY = 50,
      TIME_SLICE    = 0,
      BUSY_LAMP_OFF = 0,
      ABORT_REQUEST = 0,
      PAUSE_REQUEST = 0;
DEFAULT_GROUP   = 1,*,*,*,*;
CONTROL_CODE    = 00000000 00000000;
/APPL
/APPL

AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : TRUE;
/MN
%s
/POS
%s
/END
''' % (prog_name, instructions, positions)
    return line

class TPProgram(object):
    program_template = '''\
/PROG  {prog_name:s}
/ATTR
OWNER       = MNEDITOR;
COMMENT     = "";
PROG_SIZE   = 246;
CREATE      = DATE 17-05-18  TIME 17:34:40;
MODIFIED    = DATE 17-05-18  TIME 17:48:56;
FILE_NAME   = ;
VERSION     = 0;
LINE_COUNT  = {line_count:d};
MEMORY_SIZE = 730;
PROTECT     = READ_WRITE;
TCD:  STACK_SIZE    = 0,
      TASK_PRIORITY = 50,
      TIME_SLICE    = 0,
      BUSY_LAMP_OFF = 0,
      ABORT_REQUEST = 0,
      PAUSE_REQUEST = 0;
DEFAULT_GROUP   = 1,*,*,*,*;
CONTROL_CODE    = 00000000 00000000;
/MN
{prog_code:s}
/POS
/END
'''

    def __init__(self, prog_name):
        self.__prog_name = prog_name
        self.__program = None

    def format(self, prog_code):
        prog_args = {
            'prog_name': self.__prog_name,
            'prog_code': TPCode(prog_code),
            'line_count': len(prog_code)
        }
        self.__program = self.program_template.format(**prog_args)

    def send_file_by_ftp(self, server):
        data = BytesIO(self.__program)
        session = ftplib.FTP(server)
        session.login(user='anonymous')
        session.storbinary('STOR %s' % self.__prog_name, data)
        session.quit()
        print('%s> STOR %s' % (server, self.__prog_name))


    def upload(self, server, verbosity=0):
        '''
        Uploads the driver to the specified server via FTP.
        :param server: the hostname of the server to which to send
        the driver
        '''

        fmt_args = tuple((green(x) for x in (self.__prog_name, server)))

        print('Uploading file %s to %s...' % fmt_args)
        if verbosity > 0:
            print('''\
==============================================================================
%s
==============================================================================
''' % self.__program)

        self.send_file_by_ftp(server)

        print('''\
The driver %s has been uploaded to %s. You must start driver on the robot to \
use it.''' % fmt_args)

    def __str__(self):
        return self.__program


class RobotTPDriver(TPProgram):

    pneum_actions = {'PNEUM_1_CLOSE': -1,
                     'PNEUM_1_OPEN': -2,
                     'PNEUM_2_CLOSE': -3,
                     'PNEUM_2_OPEN': -4}

    def __init__(self, prog_name='misomain.ls', buffer_size=5,
                 loop_rate_hz=20, signaling_register=6):
        '''
        :param prog_name: name of the program on the TP
        :param buffer_size: number of trajectories to store on the robot
        :param loop_period_hz: frequency of spinning main loop
        :param signaling_register: index of the register to use for
         signaling between the communication driver and the main driver
        (this)
        '''
        assert signaling_register == Registers.SIG, 'Only one signal reg supported'
        super(RobotTPDriver, self).__init__(prog_name)

        self.buffer_size = buffer_size
        self.loop_rate_hz = loop_rate_hz
        self.synchronization_delay = 0.05 #Delay 0.05 seconds to synchronize r_signal
        self.signaling_register = signaling_register
        self.labels = {}

        #Add labels
        self.add_label('main_loop')
        self.add_label('main_loop_end')
        self.add_label('sig_reg_process')
        self.add_label('sig_reg_process_end')

        prog_code = self.loop_begin_code() + \
                    self.sig_reg_check_code() + \
                    self.sig_reg_process_code() + \
                    self.loop_end_code()
        self.format(prog_code)

    def loop_begin_code(self):
        return [
            'R[%d]=0' % Registers.SIG,
            'LBL[%d]' % self.labels['main_loop']
        ]

    def loop_end_code(self):
        return [
            'WAIT %3.2f(sec)' % (1. / self.loop_rate_hz),
            'JMP LBL[%d]' % self.labels['main_loop'],
            'LBL[%d]' % self.labels['main_loop_end']
            ]

    def sig_reg_check_code(self):
        return ['IF R[%d]=0, JMP LBL[%d]' % (Registers.SIG,
                                             self.labels['sig_reg_process_end'])]

    def sig_reg_process_code(self):
        code = ['LBL[%d]' % self.labels['sig_reg_process']]
        code += self.buffer_code()
        code += self.pneumatics_code()
        code += ['WAIT %3.2f(sec)' % self.synchronization_delay]
        code += ['R[%d]=R[%d]' % (Registers.ACK, Registers.SIG)]
        code += ['R[%d]=0' % Registers.SIG]
        code += ['LBL[%d]' % self.labels['sig_reg_process_end']]
        return code

    def buffer_code(self):

        line = 'IF R[%d]=%d,CALL FOOBAR%d'
        return [line % (Registers.SIG, i + 1, i)
                for i in range(self.buffer_size)]

    def pneumatics_code(self):
        for act in self.pneum_actions.keys():
            self.add_label(act+'_end')

        return (self.get_pneum_action_block(channel=1, action='OPEN')
                + self.get_pneum_action_block(channel=1, action='CLOSE')
                + self.get_pneum_action_block(channel=2, action='OPEN')
                + self.get_pneum_action_block(channel=2, action='CLOSE')) 

    def add_label(self, name):
        assert isinstance(name, str)
        assert name not in self.labels  #Make sure label doesn't
                                        #already exist

        self.labels[name] = len(self.labels) + 1 #get next label index
        print ('LBL[%d] = %s ' % (self.labels[name], name))
        return self.labels[name]

    def get_pneum_action_block(self, channel, action='OPEN'):
        assert isinstance(channel, int)
        assert channel in [1, 2]
        assert isinstance(action, str)
        assert action in ['OPEN', 'CLOSE']

        #define robot outputs in a dictionary with channel number as key
        #syntax:
        #  channel: [open_robot_output_on, close_robot_output_on]
        robot_output_sets = {1: [1, 2],
                             2: [3, 4]}
        robot_outputs = robot_output_sets[channel]
        if action != 'OPEN':
            robot_outputs.reverse()

        pneum_action_key = 'PNEUM_{}_{}'.format(channel, action)
        return ['IF R[%d]<>%d, JMP LBL[%d]' % (Registers.SIG,
                                               self.pneum_actions[pneum_action_key],
                                               self.labels[pneum_action_key + '_end']),
                'RO[%d]=ON' % robot_outputs[0],
                'RO[%d]=OFF' % robot_outputs[1],
                'LBL[%d]' % self.labels[pneum_action_key + '_end']]

class TPCode(object):
    def __init__(self, lines):
        self.lines = lines

    def __str__(self):
        return '\n'.join(['%4d:  %s ;' % (i + 1, self.lines[i])
                          for i in range(len(self.lines))])
