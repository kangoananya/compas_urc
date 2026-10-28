"""Instructions that can be sent to the compas_urc driver.

Instructions are executed by the robot in the order they are sent. Read
instructions (``GetFrame``, ``GetJoints``, ``ReadDigital``) return their value
through the future returned by ``send``.

Units follow the compas_rrc conventions where possible:
frames, speeds, accelerations and blend radii in millimeters, time in seconds.
Joint values are in radians, as in compas robot configurations.
"""
from compas.geometry import Frame

from . import driver

__all__ = [
    'Motion',
    'Instruction',
    'MoveToFrame',
    'MoveToJoints',
    'SetDigital',
    'SetToolDigital',
    'SetAnalog',
    'WaitTime',
    'SetTcp',
    'ReadInstruction',
    'GetFrame',
    'GetJoints',
    'ReadDigital',
    'WaitForResume',
    'WaitForDigitalInput',
    'End',
]


class Motion(object):
    """Motion types for :class:`MoveToFrame`.

    LINEAR: ``movel``, straight lines in tool space.
    PROCESS: ``movep``, straight lines with circular blends at constant tool
    speed. Best for printing and other process paths.
    JOINT: ``movej``, fastest path in joint space, the tool does not move in a straight line.
    """
    LINEAR = 'L'
    PROCESS = 'P'
    JOINT = 'J'


class Instruction(object):
    """Base class: an instruction code and up to ``driver.NUM_PARAMS`` float parameters."""
    code = None

    def __init__(self, params=None):
        self.params = list(params or [])
        if len(self.params) > driver.NUM_PARAMS:
            raise ValueError('Too many parameters for instruction')
        self.params += [0.0] * (driver.NUM_PARAMS - len(self.params))
        self.sequence_id = None

    def __repr__(self):
        return '{}(seq={})'.format(type(self).__name__, self.sequence_id)


def frame_to_pose(frame):
    """Converts a compas frame in millimeters to a UR pose [x, y, z, rx, ry, rz] in meters / radians."""
    if not isinstance(frame, Frame):
        raise TypeError('Expected a compas.geometry.Frame, got {}'.format(type(frame).__name__))
    return [frame.point.x / 1000, frame.point.y / 1000, frame.point.z / 1000] + list(frame.axis_angle_vector)


class MoveToFrame(Instruction):
    """Move the TCP to a frame.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`
        Target frame in millimeters, in the robot base coordinate system.
    speed : float
        TCP speed in mm/s (``Motion.LINEAR``, ``Motion.PROCESS``) or joint speed in rad/s (``Motion.JOINT``).
    accel : float
        TCP acceleration in mm/s^2 (``Motion.LINEAR``, ``Motion.PROCESS``) or joint acceleration in rad/s^2 (``Motion.JOINT``).
    blend : float
        Blend radius in mm. 0 stops exactly at the frame.
    motion_type : str
        ``Motion.LINEAR`` (movel), ``Motion.PROCESS`` (movep) or ``Motion.JOINT`` (movej).
    """

    def __init__(self, frame, speed, accel, blend=0.0, motion_type=Motion.LINEAR):
        if motion_type == Motion.LINEAR:
            self.code = driver.CODE_MOVEL
            speed, accel = speed / 1000, accel / 1000
        elif motion_type == Motion.PROCESS:
            self.code = driver.CODE_MOVEP
            speed, accel = speed / 1000, accel / 1000
        elif motion_type == Motion.JOINT:
            self.code = driver.CODE_MOVEJ_POSE
        else:
            raise ValueError('Unknown motion type: {}'.format(motion_type))
        super(MoveToFrame, self).__init__(frame_to_pose(frame) + [speed, accel, blend / 1000])


class MoveToJoints(Instruction):
    """Move to joint values with movej.

    Parameters
    ----------
    joints : list of float
        Six joint values in radians.
    speed : float
        Joint speed in rad/s.
    accel : float
        Joint acceleration in rad/s^2.
    blend : float
        Blend radius in mm.
    """
    code = driver.CODE_MOVEJ_JOINTS

    def __init__(self, joints, speed, accel, blend=0.0):
        joints = list(joints)
        if len(joints) != 6:
            raise ValueError('MoveToJoints needs 6 joint values')
        super(MoveToJoints, self).__init__(joints + [speed, accel, blend / 1000])


class SetDigital(Instruction):
    """Set a standard digital output, in sequence with the motions around it."""
    code = driver.CODE_SET_DIGITAL

    def __init__(self, io, value):
        super(SetDigital, self).__init__([io, 1.0 if value else 0.0])


class SetToolDigital(Instruction):
    """Set a tool digital output, in sequence with the motions around it."""
    code = driver.CODE_SET_TOOL_DIGITAL

    def __init__(self, io, value):
        super(SetToolDigital, self).__init__([io, 1.0 if value else 0.0])


class SetAnalog(Instruction):
    """Set a standard analog output (0..1, as set_standard_analog_out)."""
    code = driver.CODE_SET_ANALOG

    def __init__(self, io, value):
        super(SetAnalog, self).__init__([io, value])


class WaitTime(Instruction):
    """Wait on the robot. The robot stops, a blend into the next move is not possible."""
    code = driver.CODE_WAIT_TIME

    def __init__(self, seconds):
        super(WaitTime, self).__init__([seconds])


class SetTcp(Instruction):
    """Set the TCP offset as a frame in millimeters, relative to the tool flange."""
    code = driver.CODE_SET_TCP

    def __init__(self, frame):
        super(SetTcp, self).__init__(frame_to_pose(frame))


class ReadInstruction(Instruction):
    """Base class for instructions that return a value.

    By default the value is read by the robot in sequence, after all
    instructions sent before. With ``immediate=True`` the client answers right
    away from the live robot state, without waiting for the queue.
    """

    def __init__(self, params=None, immediate=False):
        super(ReadInstruction, self).__init__(params)
        self.immediate = immediate

    def parse_result(self, values):
        """Converts the six result values written by the driver."""
        raise NotImplementedError

    def read_state(self, state):
        """Reads the value from the live RTDE state, for ``immediate=True``."""
        raise NotImplementedError


def _pose_to_frame(pose):
    return Frame.from_axis_angle_vector(pose[3:6], point=[pose[0] * 1000, pose[1] * 1000, pose[2] * 1000])


class GetFrame(ReadInstruction):
    """Returns the current TCP frame in millimeters, in the robot base coordinate system."""
    code = driver.CODE_GET_FRAME

    def __init__(self, immediate=False):
        super(GetFrame, self).__init__(immediate=immediate)

    def parse_result(self, values):
        return _pose_to_frame(values)

    def read_state(self, state):
        return _pose_to_frame(state['actual_TCP_pose'])


class GetJoints(ReadInstruction):
    """Returns the current joint values in radians."""
    code = driver.CODE_GET_JOINTS

    def __init__(self, immediate=False):
        super(GetJoints, self).__init__(immediate=immediate)

    def parse_result(self, values):
        return list(values)

    def read_state(self, state):
        return list(state['actual_q'])


class ReadDigital(ReadInstruction):
    """Returns the state of a standard digital input (0-7) as a bool."""
    code = driver.CODE_READ_DIGITAL

    def __init__(self, io, immediate=False):
        super(ReadDigital, self).__init__([io], immediate=immediate)
        self.io = io

    def parse_result(self, values):
        return values[0] > 0.5

    def read_state(self, state):
        return bool(state['actual_digital_input_bits'] >> self.io & 1)


class WaitForResume(Instruction):
    """The robot stops here until :meth:`URClient.resume` is called."""
    code = driver.CODE_WAIT_RESUME


class WaitForDigitalInput(Instruction):
    """The robot stops here until a standard digital input (0-7) has the given value."""
    code = driver.CODE_WAIT_DIGITAL_INPUT

    def __init__(self, io, value=True):
        super(WaitForDigitalInput, self).__init__([io, 1.0 if value else 0.0])


class End(Instruction):
    """Ends the driver program on the robot after all previous instructions."""
    code = driver.CODE_END
