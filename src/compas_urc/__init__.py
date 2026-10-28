"""compas_urc: compas_rrc-style remote control for Universal Robots.

Instructions are streamed over RTDE registers into a buffer in a small
URScript driver on the robot. Motions and IO are executed by the robot in
sequence, so digital outputs switch in sync with blended motions, and paths
can have any number of points.
"""
from .client import DriverError
from .client import FutureResult
from .client import TimeoutException
from .client import UrClient
from .instructions import End
from .instructions import GetFrame
from .instructions import GetJoints
from .instructions import Motion
from .instructions import MoveToFrame
from .instructions import MoveToJoints
from .instructions import ReadDigital
from .instructions import SetAnalog
from .instructions import SetDigital
from .instructions import SetTcp
from .instructions import SetToolDigital
from .instructions import WaitForDigitalInput
from .instructions import WaitForResume
from .instructions import WaitTime

__version__ = '0.1.0'

__all__ = [
    'UrClient',
    'FutureResult',
    'DriverError',
    'TimeoutException',
    'Motion',
    'MoveToFrame',
    'MoveToJoints',
    'SetDigital',
    'SetToolDigital',
    'SetAnalog',
    'WaitTime',
    'SetTcp',
    'GetFrame',
    'GetJoints',
    'ReadDigital',
    'WaitForResume',
    'WaitForDigitalInput',
    'End',
]
