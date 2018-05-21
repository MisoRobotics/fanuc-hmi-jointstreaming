class Registers(object):
    """Contains the indexes of the fanuc registers
    used in the fanuc driver stack
    """
    SIG = 6
    ACK = 7

class DigitalIO(object):
    """Contains the indexes of the fanuc digital IO
    used in the fanuc driver stack
    """
    CYCLESTOP = 6
    CYCLESTART = 3
    RESET = 5
    WARN1 = 120
    WARN2 = 119
    PNEUMGOOD = 118
    SIR1 = 185