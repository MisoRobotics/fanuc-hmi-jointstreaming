from abc import (ABCMeta,
                 abstractproperty)
import struct
import time

from fanuc_driver import DigitalIO
from fanuc_driver.modbus_interface import ModbusInterface
from fanuc_driver.alarm import FanucAlarm

import rospy


NUM_JOINTS = 6

REGISTER_COUNT = 200


class HMIEngine(object):
    """Handles low-level interfacing with the SNPX registers for HMI.

    The structure for how this is implemented is summarized below.
    The FANUC dedicates SPNX registers 76-79 to configure the ADDRESS,
    SIZE, VAR_NAME, and MULTIPLY properties of SPNX register 80, respectively.
    These values are stored starting at SNPX_80_START, and then the ADDRESS is
    configured to point to BOOTSTRAP_START, where the information that will
    be written to desired VARIABLES is stored during the bootstrapping
    process.
    """

    BOOTSTRAP_SNPX_START = 76
    BOOTSTRAP_START = 16000
    SNPX_80_START = 16300

    SNPX_ADDRESS_SIZE = 2
    SNPX_SIZE_SIZE = 2
    SNPX_VAR_NAME_SIZE = 40
    SNPX_MULTIPLY_SIZE = 2
    SNPX_SIZE = (SNPX_ADDRESS_SIZE + SNPX_SIZE_SIZE +
                 SNPX_VAR_NAME_SIZE + SNPX_MULTIPLY_SIZE)

    def __init__(self, server_ip, port):
        self.__modbus_interface = ModbusInterface(server_ip, port)

    @property
    def modbus_interface(self):
        return self.__modbus_interface

    def read_bootstrap_snpx(self):
        bootstrap_snpx = SnpxReg()
        bootstrap_snpx.deserialize(
            self.__modbus_interface.read_holding_registers(
                HMIEngine.SNPX_80_START, HMIEngine.SNPX_SIZE))
        return bootstrap_snpx

    def __write_bootstrap_snpx(self, snpx):
        assert snpx.address + snpx.size < HMIEngine.SNPX_80_START, \
            'Make sure not to cover anything in bootstrap snpx'
        data = snpx.serialize()
        self.__modbus_interface.write_holding_registers(
            HMIEngine.SNPX_80_START, data)

    def read_snpx(self, index):
        snpx = SnpxReg()
        var_name = '$SNPX_ASG[' + str(index) + '].${field}'
        bootstrap_snpx = SnpxReg()
        bootstrap_snpx.address = HMIEngine.BOOTSTRAP_START

        def read_field(name, size, multiply=1.):
            bootstrap_snpx.size = size
            bootstrap_snpx.var_name = var_name.format(field=name)
            bootstrap_snpx.multiply = multiply
            self.__write_bootstrap_snpx(bootstrap_snpx)
            return self.__modbus_interface.read_holding_registers(
                bootstrap_snpx.address, bootstrap_snpx.size)
        data = []
        data.extend(read_field('ADDRESS', HMIEngine.SNPX_ADDRESS_SIZE))
        data.extend(read_field('SIZE', HMIEngine.SNPX_SIZE_SIZE))
        data.extend(read_field('VAR_NAME',
                               HMIEngine.SNPX_VAR_NAME_SIZE, multiply=-1.))
        data.extend(read_field('MULTIPLY',
                               HMIEngine.SNPX_MULTIPLY_SIZE, multiply=0.))
        snpx.deserialize(data)
        return snpx

    def write_snpx(self, snpx, index):
        if snpx.address + snpx.size > HMIEngine.BOOTSTRAP_START:
            raise RuntimeError('HMI Engine is overwriting bootstrap space. '
                               'Going into address: {}'.format(
                                   snpx.address + snpx.size))
        if not (0 < index < HMIEngine.BOOTSTRAP_SNPX_START):
            raise RuntimeError('HMI Engine is accessing invalid SNPX '
                               'Register: [{}]. Valid registers are '
                               '0 < SNPX_ASG < {}'.format(
                                   index, HMIEngine.BOOTSTRAP_SNPX_START))

        var_name = '$SNPX_ASG[' + str(index) + '].${field}'
        bootstrap_snpx = SnpxReg()
        bootstrap_snpx.address = HMIEngine.BOOTSTRAP_START

        def write_field(name, data, multiply=1.):
            bootstrap_snpx.size = len(data)
            bootstrap_snpx.var_name = var_name.format(field=name)
            bootstrap_snpx.multiply = multiply
            self.__write_bootstrap_snpx(bootstrap_snpx)
            return self.__modbus_interface.write_holding_registers(
                bootstrap_snpx.address, data)

        data = snpx.serialize()
        count = 0

        write_field('ADDRESS',
                    data[count:count + HMIEngine.SNPX_ADDRESS_SIZE])
        count += HMIEngine.SNPX_ADDRESS_SIZE
        write_field('SIZE', data[count:count + HMIEngine.SNPX_SIZE_SIZE])
        count += HMIEngine.SNPX_SIZE_SIZE
        write_field('VAR_NAME',
                    data[count:count + HMIEngine.SNPX_VAR_NAME_SIZE],
                    multiply=-1.)
        count += HMIEngine.SNPX_VAR_NAME_SIZE
        write_field('MULTIPLY',
                    data[count:count + HMIEngine.SNPX_MULTIPLY_SIZE],
                    multiply=0.)

        snpx.deserialize(data)
        return snpx


class SnpxManager(object):
    MODBUS_PORT = 502

    def __init__(self, server_ip):
        self.__hmi_engine = HMIEngine(server_ip, SnpxManager.MODBUS_PORT)
        self.__snpx_interfaces = []

    def add_interface(self, interface):
        """Adds an snpx interface to the manager
        """
        self.add_interfaces([interface])

    def add_interfaces(self, interfaces):
        for interface in interfaces:
            if self.__snpx_interfaces:
                interface.previous_data_interface = self.__snpx_interfaces[-1]
            interface.modbus_interface = self.__hmi_engine.modbus_interface
            self.__snpx_interfaces.append(interface)
        self.write_config()

    def check_config(self):
        """Checks the snpx configuration status
        and returns True if it matches perfectly,
        else False
        """
        for i, snpx in enumerate(self.__padded_snpx):
            snpx_robot = self.__hmi_engine.read_snpx(i+1)
            if snpx_robot != snpx:
                rospy.logerr('snpx_robot is not equal to snpx.\nsnpx_robot: '
                             '%s\nsnpx: %s', snpx_robot, snpx)
                return False
        return True

    def write_config(self):
        """Writes the snpx configuration
        """
        for i, snpx in enumerate(self.__padded_snpx):
            self.__hmi_engine.write_snpx(snpx, i+1)

    @property
    def __padded_snpx(self):
        for i in range(HMIEngine.BOOTSTRAP_SNPX_START - 1):
            if i < len(self.__snpx_interfaces):
                yield self.__snpx_interfaces[i].snpx
            else:
                yield SnpxReg(address=0,
                              size=0,
                              var_name='',
                              multiply=1.)


class SnpxDataInterface(object):
    """Provides data using an SNPX register
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        self.__prev_data_interface = None
        self._modbus_interface = None
        self.__snpx_size = None

    @property
    def snpx(self):
        return SnpxReg(address=self.snpx_start_address,
                       size=self.snpx_size,
                       var_name=self.snpx_var_name,
                       multiply=self.snpx_multiply)

    @property
    def snpx_size(self):
        if self.__snpx_size is None:
            raise ValueError('snpx_size has not been assigned')
        return self.__snpx_size

    @snpx_size.setter
    def snpx_size(self, value):
        if self.__snpx_size is not None:
            raise ValueError('snpx_size has been assigned already, and is '
                             'immutable after initialization in the current '
                             'implementation.')
        self.__snpx_size = value

    @abstractproperty
    def snpx_var_name(self):
        raise NotImplementedError()

    @abstractproperty
    def snpx_multiply(self):
        raise NotImplementedError()

    @property
    def previous_data_interface(self):
        return self.__prev_data_interface

    @previous_data_interface.setter
    def previous_data_interface(self, value):
        assert isinstance(value, SnpxDataInterface)
        self.__prev_data_interface = value

    @property
    def modbus_interface(self):
        return self._modbus_interface

    @modbus_interface.setter
    def modbus_interface(self, value):
        assert isinstance(value, ModbusInterface)
        self._modbus_interface = value

    @property
    def snpx_start_address(self):
        if self.__prev_data_interface is not None:
            prev_start = self.__prev_data_interface.snpx_start_address
            prev_size = self.__prev_data_interface.snpx_size
        else:
            prev_start = 1
            prev_size = 0
        return prev_start + prev_size


class JointAngleInterface(SnpxDataInterface):
    """Provides the joint angles of the robot
    """
    def __init__(self, var_name_prefix='POS[0]'):
        super(JointAngleInterface, self).__init__()
        self.snpx_size = 19
        self.__var_name = var_name_prefix + '@27.%d'

    @property
    def joint_angles(self):
        start_address = self.snpx_start_address
        size = self.snpx_size
        joint_data = self._modbus_interface.read_holding_registers(
            start_address, size)
        joints = []
        for j in range(NUM_JOINTS):
            joint_angle = _bytes_to_float(joint_data[2*j:2*j + 2])
            joints.append(joint_angle)
        if joint_data[-1] != 1:
            raise ValueError('Joint data is not valid. Must be 1. Value is %s',
                             joint_data[-1])
        return joints

    @joint_angles.setter
    def joint_angles(self, value):
        start_address = self.snpx_start_address
        if len(value) != NUM_JOINTS:
            raise ValueError('Joint angles must be a list of length %d'
                             % NUM_JOINTS)
        joint_bytes = []
        for j in value:
            joint_bytes.extend(_float_to_bytes(j))
        for j in range(3):
            # Add J7-J9 to data
            joint_bytes.extend(_float_to_bytes(0.))
        joint_bytes.append(1)
        self._modbus_interface.write_holding_registers(
            start_address, joint_bytes)

    @property
    def snpx_var_name(self):
        return self.__var_name % self.snpx_size

    @property
    def snpx_multiply(self):
        return 0.


class JointTorqueInterface(SnpxDataInterface):
    """Provides the joint torques of the robot
    """

    def __init__(self):
        super(JointTorqueInterface, self).__init__()
        self.snpx_size = 12

    @property
    def joint_torques(self):
        start_address = self.snpx_start_address
        size = self.snpx_size
        joint_data = self._modbus_interface.read_holding_registers(
            start_address, size)
        joints = []
        for j in range(NUM_JOINTS):
            joint_angle = _bytes_to_float(joint_data[2*j:2*j + 2])
            joints.append(joint_angle)
        return joints

    @property
    def snpx_multiply(self):
        return 0.

    @property
    def snpx_var_name(self):
        return '$MOR_GRP.$CUR_DIS_TRQ'


class AlarmInterface(SnpxDataInterface):
    """Provides the alarms on the robot
    """

    def __init__(self,
                 num_alarms):
        super(AlarmInterface, self).__init__()
        self.__num_alarms = num_alarms
        self.__alarm_size = 100
        self.snpx_size = self.__num_alarms * self.__alarm_size

    @property
    def snpx_var_name(self):
        return 'ALM[1]'

    @property
    def snpx_multiply(self):
        return -1.

    @property
    def alarms(self):
        base_index = self.snpx_start_address
        alarms = []
        for i in range(self.__num_alarms):
            offset_index = base_index + i * self.__alarm_size
            data = self._modbus_interface.read_holding_registers(
                offset_index, self.__alarm_size)

            alarm = FanucAlarm(data)
            if alarm.is_alarm:
                alarms.append(alarm)
            else:
                break

        return alarms


class DataRegisterInterface(SnpxDataInterface):
    """Provides an interface to write Fanuc registers
    """

    def __init__(self,
                 start_register,
                 register_count):
        super(DataRegisterInterface, self).__init__()

        if not (1 <= start_register <= REGISTER_COUNT):
            raise ValueError('Start register [%d] is outside of Fanuc '
                             'register count (1 <= register <= %d)' %
                             (start_register, REGISTER_COUNT))
        if not (1 <= start_register + register_count - 1 <= REGISTER_COUNT):
            raise ValueError('End register [%d] is outside of Fanuc '
                             'register count (1 <= register <= %d)' %
                             (start_register + register_count - 1,
                              REGISTER_COUNT))

        self.__start_register = start_register
        self.__register_count = register_count
        self.snpx_size = self.__register_count * 2

    @property
    def snpx_var_name(self):
        return 'R[%d]' % self.__start_register

    @property
    def snpx_multiply(self):
        return 1.

    def read_register(self, index):
        """Reads the register at the given index. The index must be in the
        range of registers the provider was initialize with.
        """
        reg_index = self.__calc_register_index(index)
        data = self._modbus_interface.read_holding_registers(
            reg_index, quantity=2)
        return _bytes_to_int(data)

    def write_register(self, index, value):
        reg_index = self.__calc_register_index(index)
        data = _int_to_bytes(value)
        self._modbus_interface.write_holding_registers(reg_index, values=data)

    def __calc_register_index(self, index):
        offset_index = index - self.__start_register
        if not 0 <= offset_index < self.__register_count:
            raise ValueError('Register is outside of access range '
                             'initialized at provider creation')
        return offset_index * 2 + self.snpx_start_address


class IOInterface(SnpxDataInterface):
    """Provides an interface to the digital io
    """
    HIGH = 1
    LOW = 0

    def __init__(self):
        super(IOInterface, self).__init__()
        self.snpx_size = 0

    @property
    def snpx_var_name(self):
        return ''

    @property
    def snpx_multiply(self):
        return 1.

    def __pulse(self, output, mid_transition=HIGH, pulse_width=0.2):
        start_end_level = (IOInterface.LOW
                           if mid_transition == IOInterface.HIGH
                           else IOInterface.HIGH)
        self._modbus_interface.write_coil(output, start_end_level)
        time.sleep(pulse_width)
        self._modbus_interface.write_coil(output, mid_transition)
        time.sleep(pulse_width)
        self._modbus_interface.write_coil(output, start_end_level)
        time.sleep(pulse_width)

    def __quick_pulse(self, output):
        """Pulse for 0.2 seconds to meet fanuc timing requirements."""
        self.__pulse(output,
                     mid_transition=IOInterface.HIGH,
                     pulse_width=0.2)

    def abort(self):
        self.__quick_pulse(DigitalIO.CYCLESTOP)

    def start(self):
        self.__quick_pulse(DigitalIO.CYCLESTOP)

    def reset(self):
        self.__quick_pulse(DigitalIO.RESET)

    def read_digital_input(self, address):
        return self._modbus_interface.read_discrete_inputs(address, 1)[0]


class SystemStatusInterface(SnpxDataInterface):
    """Provides an interface to the Fanuc controller to stop and start
    the software
    """

    def __init__(self,
                 num_programs):
        super(SystemStatusInterface, self).__init__()
        self.__num_programs = num_programs
        self.__struct_length = 18
        self.snpx_size = self.__struct_length * num_programs

    @property
    def snpx_var_name(self):
        return 'PRG[1]'

    @property
    def snpx_multiply(self):
        return -1.

    @property
    def program_statuses(self):
        base_index = self.snpx_start_address
        programs = []
        for i in range(self.__num_programs):
            offset_index = i * self.__struct_length
            data = self._modbus_interface.read_holding_registers(
                base_index+offset_index, quantity=self.__struct_length)
            programs.append(ProgramStatus(data))
        return programs


class ProgramStatus(object):
    ProgramStatusCodes = {0: 'ABORTED',
                          1: 'PAUSED',
                          2: 'RUNNING'}

    def __init__(self, data):
        assert len(data) == 18, 'Data structure must be 18 bytes in length'
        self.__name = _parse_str(data[0:8])
        self.__line_number = data[8]
        self.__execution_status = data[9]
        self.__parent_name = _parse_str(data[10:18])

    @property
    def name(self):
        return self.__name

    @property
    def execution_status(self):
        return ProgramStatus.ProgramStatusCodes[self.__execution_status]

    @property
    def is_aborted(self):
        return self.execution_status == 'ABORTED'

    @property
    def line_number(self):
        return self.__line_number

    @property
    def parent_name(self):
        return self.__parent_name

    def __repr__(self):
        return """\
Name: {name}
Line number: {line}
Exec status: {status}
Parent name: {parent_name}
""".format(name=self.__name,
           line=self.__line_number,
           status=self.execution_status,
           parent_name=self.__parent_name)

    __str__ = __repr__


class SnpxReg(object):

    def __init__(self, address=None, size=None, var_name=None, multiply=None):
        self.address = address
        self.size = size
        self.var_name = var_name
        self.multiply = multiply

    def deserialize(self, data):
        self.address = _bytes_to_int(data[0:2])
        self.size = _bytes_to_int(data[2:4])
        self.var_name = _parse_str(data[4:44])
        self.multiply = _bytes_to_float(data[44:46])

    def serialize(self):
        reg_data = []
        reg_data.extend(_int_to_bytes(self.address))
        reg_data.extend(_int_to_bytes(self.size))
        reg_data.extend(_serialize_str(self.var_name))
        reg_data.extend(_float_to_bytes(self.multiply))
        return reg_data

    def __eq__(self, other):
        if isinstance(other, SnpxReg):
            return (self.address == other.address and
                    self.size == other.size and
                    self.var_name.strip('\0') == other.var_name.strip('\0') and
                    self.multiply == other.multiply)
        return NotImplemented

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return ("""\
address: {address}
size: {size}
var_name: {var_name}
multiply: {multiply}

""".format(address=self.address,
           size=self.size,
           var_name=self.var_name,
           multiply=self.multiply))


# Utility functions for hmi engine
# TODO(WHW): Change FanucAlarm class to use these utility methods
def _parse_str(data):
    return ''.join([chr(char // 256) + chr(char & 0xFF) for char in data])


def _serialize_str(data, pad_to_length=80):
    if len(data) > pad_to_length:
        raise ValueError('string is too long')
    res = []
    for i, datum in enumerate(data):
        if i % 2 == 0:
            res.append(ord(datum) << 8)
        else:
            res[-1] += ord(datum)
    # Pad the result to be padding_length elements long
    while len(res) < pad_to_length / 2:
        res.append(0)
    assert len(res) == pad_to_length/2
    return res


def _bytes_to_int(data):
    return struct.unpack('<i', struct.pack('<hh', data[0], data[1]))[0]


def _int_to_bytes(data):
    return struct.unpack('<hh', struct.pack('<i', data))


def _bytes_to_float(data):
    return struct.unpack('<f', struct.pack('<hh', data[0], data[1]))[0]


def _float_to_bytes(data):
    return struct.unpack('<hh', struct.pack('<f', data))
