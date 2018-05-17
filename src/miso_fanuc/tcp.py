import errno
import socket
from struct import calcsize, Struct
import time

import numpy as np

from miso_fanuc.tp import Point
from miso_msgs.msg import FanucStatus
import rospy
from sensor_msgs.msg import JointState


class MRDPacket(object):
    seq_nr = 1

    MSG_TYPE_CALL = 2
    MSG_TYPE_JPOS = 4

    COMM_TYPE_SERVICE_REQUEST = 2
    COMM_TYPE_SERVICE_REPLY = 3

    REPLY_TYPE_INVALID = 0
    REPLY_TYPE_SUCCESS = 1
    REPLY_TYPE_FAILURE = 2

    LITTLE_ENDIAN_FMT = '<'
    BIG_ENDIAN_FMT = '>'

    LENGTH_FMT = 'i'
    HEADER_FMT = 'iiiiiiii'
    JPOS_FMT = '' + 'f' * 10

    LENGTH_SZ = calcsize(LENGTH_FMT)
    HEADER_SZ = calcsize(HEADER_FMT)
    JPOS_SZ = calcsize(JPOS_FMT)

    def __init__(self,
                 sim,
                 msg_type,
                 comm_type,
                 reply_type,
                 sequence_number,
                 r_signal,
                 slowdown_zone_1_status,
                 slowdown_zone_2_status,
                 danger_zone_status,
                 jpos=None):
        assert isinstance(sim, bool)
        assert isinstance(msg_type, int)
        assert isinstance(comm_type, int)
        assert isinstance(reply_type, int)
        assert isinstance(sequence_number, (int, type(None)))
        assert isinstance(r_signal, int)
        assert isinstance(slowdown_zone_1_status, int)
        assert isinstance(slowdown_zone_2_status, int)
        assert isinstance(danger_zone_status, int)

        if jpos:
            assert isinstance(jpos, list) or isinstance(jpos, tuple)
            assert len(jpos) <= 10

        if sequence_number is None:
            self.seq_nr = MRDPacket.seq_nr
            MRDPacket.seq_nr += 1 # not thread-safe
        else:
            self.seq_nr = sequence_number

        self.msg_type = msg_type
        self.comm_type = comm_type
        self.reply_type = reply_type
        self.r_signal = r_signal
        self.slowdown_zone_1_status = slowdown_zone_1_status
        self.slowdown_zone_2_status = slowdown_zone_2_status
        self.danger_zone_status = danger_zone_status
        self.jpos = jpos

        self.length = MRDPacket.HEADER_SZ
        if self.jpos:
            self.length += MRDPacket.JPOS_SZ

        self.sim = sim
        self.serialize()

    @property
    def total_bytes(self):
        return self.length + MRDPacket.LENGTH_SZ

    def __str__(self):
        fmt_args = tuple((ord(d) for d in self.data))
        return ('%02x ' * self.total_bytes) % fmt_args

    def serialize(self):
        packet = (self.msg_type,               # int 32
                  self.comm_type,              # int 32
                  self.reply_type,             # int 32
                  self.seq_nr,                 # int 32
                  self.r_signal,               # int 32
                  self.slowdown_zone_1_status, # int 32
                  self.slowdown_zone_2_status, # int 32
                  self.danger_zone_status)     # int 32
        packet = (self.length,) + packet
        fmt = MRDPacket.LENGTH_FMT + MRDPacket.HEADER_FMT

        if self.jpos:
            self.jpos = list(self.jpos)
            packet += tuple(self.jpos)
            fmt += MRDPacket.JPOS_FMT

        self.data = self.packme(fmt, packet)

    @staticmethod
    def deserialize(data, sim):
        assert data is not None
        assert len(data) >= MRDPacket.HEADER_SZ

        length, = MRDPacket.unpack(sim, MRDPacket.LENGTH_FMT, data)
        assert length == len(data) - MRDPacket.LENGTH_SZ

        header_fmt = MRDPacket.HEADER_FMT + MRDPacket.LENGTH_FMT
        header = MRDPacket.unpack(sim, header_fmt, data)

        header_args = header[1:]
        packet = MRDPacket(sim, *header_args)

        if packet.msg_type == MRDPacket.MSG_TYPE_JPOS:
            packet.jpos = list(MRDPacket.unpack(
                sim,
                MRDPacket.JPOS_FMT,
                data,
                MRDPacket.LENGTH_SZ + MRDPacket.HEADER_SZ))
            packet.length += MRDPacket.JPOS_SZ
            packet.serialize()

        return packet

    def packme(self, fmt, data):
        return MRDPacket.pack(self.sim, fmt, data)

    @staticmethod
    def pack_pfx(sim):
        return MRDPacket.LITTLE_ENDIAN_FMT if sim \
            else MRDPacket.BIG_ENDIAN_FMT

    @staticmethod
    def pack(sim, fmt, data):
        assert isinstance(fmt, str)
        data_struct = Struct(MRDPacket.pack_pfx(sim) + fmt)
        return  data_struct.pack(*data)

    @staticmethod
    def unpack(sim, fmt, data, offset=0):
        assert isinstance(sim, bool)
        assert isinstance(fmt, str)
        data_struct = Struct(MRDPacket.pack_pfx(sim) + fmt)
        return data_struct.unpack_from(data, offset)


class CallPacket(MRDPacket):
    def __init__(self, sim, r_signal):
        super(CallPacket, self).__init__(
            sim,
            MRDPacket.MSG_TYPE_CALL,
            MRDPacket.COMM_TYPE_SERVICE_REQUEST,
            MRDPacket.REPLY_TYPE_INVALID,
            None,
            r_signal,
            0,
            0,
            0)


class TcpTalker(object):
    buffer_size_ = 1024

    BEHAVIOR_CALL_PORT = 12000
    JOINT_LISTEN_PORT = 12002

    def __init__(self, address, port, buffer_size=1024):
        assert isinstance(address, str)
        assert isinstance(port, int)

        self.__address = address
        self.__port = port
        self.__socket = None
        self.__retry_time = 3
        self.__buffer_size = buffer_size
        self.__data = None

    @property
    def data(self):
        return self.__data

    def connect(self):
        rospy.loginfo('Connecting to %s:%d', self.__address, self.__port)
        while not rospy.is_shutdown():
            try:
                self.__socket = socket.socket(socket.AF_INET,
                                              socket.SOCK_STREAM)
                self.__socket.settimeout(10)
                self.__socket.connect((self.__address, self.__port))
                break
            except socket.timeout:
                rospy.logwarn('Timeout on connect. Reconnecting')
            except socket.gaierror as ex:
                rospy.logwarn('''\
getaddrinfo() failed for server %s. Waiting %.3f seconds to retry.\
''', self.__address, self.__retry_time)
            except socket.error as ex:
                if ex.errno in {errno.ECONNABORTED,
                                errno.ECONNREFUSED,
                                errno.EADDRINUSE,
                                errno.EHOSTUNREACH}:
                    rospy.logwarn('Connect failed (%s). Reconnecting.', ex)
                else:
                    raise
            time.sleep(self.__retry_time)

    def send(self, data):
        while True:
            try:
                self.__socket.send(data)
                break
            except socket.timeout:
                rospy.logwarn(
                    'Socket timed out. Reconnecting.')
                self.connect()
            except socket.error as ex:
                if ex.errno in {errno.EPIPE, errno.ETIMEDOUT, errno.ECONNRESET}:
                    rospy.logwarn(
                        'Send data failed (Error: %s). Reconnecting.', ex)
                    self.connect()
                else:
                    raise

    def __read(self):
        while True:
            try:
                data = self.__socket.recv(self.__buffer_size)
                # Sometimes, when the network connection to the fanuc
                # disconnects, and then the connection comes back,
                # recv returns '' repeatedly and never starts receiving
                # data again. Re-connecting fixes this.
                if not data:
                    rospy.logwarn('Received no data, reconnecting')
                    self.connect()
                else:
                    return data
            except socket.timeout:
                rospy.logwarn(
                    'Socket timed out. Reconnecting.')
                self.connect()
            except socket.error as ex:
                if ex.errno in {errno.EPIPE, errno.ETIMEDOUT}:
                    rospy.logwarn(
                        'Receive data failed (Error: %s). Reconnecting.', ex)
                    self.connect()
                else:
                    raise

    def spin_once(self):
        ''' Blocking read of joint data. '''
        self.__data = self.__read()

    def disconnect(self):
        self.__socket.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
