from enum import Enum


class CommandTypes(Enum):
    Move = 0x00
    Jump = 0x0B
    Call = 0x0C
    SingleServo = 0x0F
    MultiServoSingleVelocity = 0x10
    MultiServoMultiVelocities = 0x11
    ServoParam = 0x12
    Version = 0xFD
    AckCheck = 0xFE
    _None = 0xFF


class ServoParams(Enum):
    Stretch = 0x01
    Speed = 0x02
    CurrentLimit = 0x03
    TemperatureLimit = 0x04


rcb4_dof = 36
