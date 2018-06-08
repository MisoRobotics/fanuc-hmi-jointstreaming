"""Handles the Service Request Transfer Protocol,
a GE Fanuc protocol for Human Machine Interface
communication with PLC's and robot controllers.
This implementation is based off of SNP and this paper:
https://www.sciencedirect.com/science/article/pii/S1742287617301925
"""
import datetime

ALARM_SIZE = 100


class FanucAlarm(object):
    """Parses an alarm retrieved via HMI.
    """

    def __init__(self, data):
        assert len(data) == ALARM_SIZE, 'Alarm is not correct size'
        self.alarm_id = data[0]
        self.alarm_num = data[1]
        self.alarm_id_cause = data[2]
        self.alarm_num_cause = data[3]
        self.alarm_severity = data[4]
        self.year = data[5]
        self.month = data[6]
        self.day = data[7]
        self.hour = data[8]
        self.minute = data[9]
        self.second = data[10]
        self.alarm_msg = self.__parse_str(data[11:51])
        self.alarm_cause_msg = self.__parse_str(data[51:91])
        self.alarm_severity_word = self.__parse_str(data[91:100])
        try:
            self.time = datetime.datetime(self.year,
                                          self.month,
                                          self.day,
                                          self.hour,
                                          self.minute,
                                          self.second)
        except ValueError:
            # For some of the HMI alarm var_name's,
            # a lack of an alarm leads to a time field
            # that is all 0's which is not parsed
            # by datetime.datetime().
            self.time = None

    @property
    def is_alarm(self):
        """Returns False if this is a null alarm, else True
        Returns:
            False if the alarm class signifies no alarm.
            Because alarms are read in a polling fashion,
            it is necessary to distinguish between real
            alarms and null alarms, where id and num are 0.
        """
        return not (self.alarm_id == 0 and self.alarm_num == 0)

    @property
    def alarm_identifier(self):
        """Returns a tuple of elements to use as a shorthand
        identifier of the alarm type
        Returns:
            (alarm_id, alarm_num)
        """
        return (self.alarm_id, self.alarm_num)

    def __parse_str(self, data):
        """Parses the string representation the SRTP protocol
        returns. The bytes in each short are out of order,
        but the shorts are in order.
        """
        # TODO(WHW): Use the utility functions defined at the bottom
        # of hmi_engine.py
        alm_msg = ''
        for chars in data:
            char1 = chars & 0xFF
            char2 = (chars & 0xFF00) >> 8
            alm_msg += chr(char2) + chr(char1)
        return alm_msg

    def __eq__(self, other):
        if isinstance(other, FanucAlarm):
            return (other.alarm_id == self.alarm_id and
                    other.alarm_num == self.alarm_num and
                    other.alarm_id_cause == self.alarm_id_cause and
                    other.alarm_num_cause == self.alarm_num_cause and
                    other.alarm_severity == self.alarm_severity and
                    other.alarm_msg == self.alarm_msg and
                    other.alarm_cause_msg == self.alarm_cause_msg and
                    other.alarm_severity_word == self.alarm_severity_word and
                    other.time == self.time)
        return NotImplemented

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return """\n\
Alarm id: {}
Alarm number: {}
Alarm cause id: {}
Alarm cause number: {}
Alarm severity: {}
Alarm time: {}
Alarm msg: {}
Alarm cause msg: {}
Alarm severity word: {}
""".format(self.alarm_id,
           self.alarm_num,
           self.alarm_id_cause,
           self.alarm_num_cause,
           self.alarm_severity,
           self.time,
           self.alarm_msg,
           self.alarm_cause_msg,
           self.alarm_severity_word)

    __str__ = __repr__
