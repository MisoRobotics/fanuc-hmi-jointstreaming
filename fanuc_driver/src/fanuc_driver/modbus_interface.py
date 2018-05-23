import errno
import socket
import time
import threading
from umodbus import conf
from umodbus.client import tcp

import rospy


class ModbusInterface(object):
    """Provides a persistent connection to a modbus server on Fanuc
    """

    def __init__(self, address, port, slave_id=1):
        # Used to enforce single accessing thread to modbus socket
        self.__lock = threading.RLock()

        self.__address = address
        self.__port = port
        self.__retry_time = 3
        self.__socket_timeout = 10
        conf.SIGNED_VALUES = True
        self.__modbus_socket = None
        self.connect()

    def connect(self):
        with self.__lock:
            rospy.loginfo_throttle(1, 'Modbus connecting to %s:%d' %
                                   (self.__address, self.__port))
            while not rospy.is_shutdown():
                try:
                    self.__modbus_socket = socket.socket(socket.AF_INET,
                                                         socket.SOCK_STREAM)
                    self.__modbus_socket.settimeout(self.__socket_timeout)
                    self.__modbus_socket.connect((self.__address, self.__port))
                    rospy.loginfo('Successfully connected to modbus')
                    break
                except socket.timeout:
                    rospy.logwarn('Timeout on modbus connect. Reconnecting.')
                except socket.gaierror as ex:
                    rospy.logwarn('''\
    getaddrinfo() failed for server %s. Reconnecting.\
    ''', self.__address)

                except socket.error as ex:
                    if ex.errno in {errno.ECONNABORTED,
                                    errno.ECONNREFUSED,
                                    errno.EADDRINUSE,
                                    errno.EHOSTUNREACH}:
                        rospy.logwarn(
                            'Modbus connect failed (%s). Reconnecting.', ex)
                    else:
                        raise
                time.sleep(self.__retry_time)

    def read_holding_register(self, address):
        """Returns the holding register at given address
        """
        with self.__lock:
            return self.read_holding_registers(address,
                                               quantity=1)[0]

    def read_holding_registers(self, starting_address, quantity):
        """Returns the sequence of holding registers at given addresses
        """
        with self.__lock:
            message = tcp.read_holding_registers(
                slave_id=1, starting_address=starting_address-1,
                quantity=quantity)
            return self.__send_modbus_message(message)

    def write_holding_register(self, address, value):
        with self.__lock:
            self.write_holding_registers(address, [value])

    def write_holding_registers(self, starting_address, values):
        with self.__lock:
            message = tcp.write_multiple_registers(
                slave_id=1, starting_address=starting_address-1, values=values)
            self.__send_modbus_message(message)

    def write_coil(self, address, value):
        with self.__lock:
            message = tcp.write_single_coil(slave_id=1,
                                            address=address-1,
                                            value=value)
            self.__send_modbus_message(message)

    def read_coil(self, address):
        with self.__lock:
            return self.read_coils(starting_address=address,
                                   quantity=1)[0]

    def read_coils(self, address, quantity):
        with self.__lock:
            message = tcp.read_coils(slave_id=1,
                                     starting_address=address-1,
                                     quantity=quantity)
            return self.__send_modbus_message(message)

    def read_discrete_inputs(self, starting_address, quantity):
        with self.__lock:
            message = tcp.read_discrete_inputs(
                slave_id=1, starting_address=starting_address-1,
                quantity=quantity)
            return self.__send_modbus_message(message)

    def __send_modbus_message(self, adu):
        with self.__lock:
            while True:
                try:
                    result = tcp.send_message(adu, self.__modbus_socket)
                    return result
                except socket.timeout as ex:
                    rospy.logwarn(
                        'Modbus socket timed out. Reconnecting.')
                    self.connect()
                except socket.error as ex:
                    if ex.errno in {errno.EPIPE, errno.ETIMEDOUT}:
                        rospy.logwarn_throttle(
                            1, 'Receive modbus data failed (Error: %s). '
                            'Reconnecting.' % (ex,))
                        self.connect()
                    else:
                        raise
