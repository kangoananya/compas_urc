"""Client that streams instructions to the compas_urc driver on a UR robot.

Usage follows compas_rrc::

    ur = UrClient('192.168.10.10')
    ur.connect()
    ur.send(SetTcp(tcp_frame))  # compas frames in millimeters
    for frame in frames:
        ur.send(MoveToFrame(frame, speed=50, accel=500, blend=1))
    ur.send_and_wait(SetDigital(1, True))
    ur.close()

``send`` only puts the instruction in a local queue and returns immediately.
A background thread streams the queue into the buffer on the robot, keeping it
as full as possible. The PC does not need to be real-time: it only has to keep
the robot buffer from running empty.
"""
import collections
import random
import socket
import threading
import time

from . import driver
from .instructions import End
from .instructions import ReadInstruction
from .instructions import WaitForResume
from .rtde import RTDE_PORT
from .rtde import RTDEConnection
from .rtde import RTDEError

__all__ = ['UrClient', 'FutureResult', 'DriverError', 'TimeoutException']

SCRIPT_PORT = 30002

# runtime_state values reported over RTDE
RUNTIME_STOPPING = 0
RUNTIME_STOPPED = 1
RUNTIME_PLAYING = 2
RUNTIME_PAUSING = 3
RUNTIME_PAUSED = 4
RUNTIME_RESUMING = 5


class DriverError(Exception):
    """The driver on the robot is not running anymore, or the connection was lost."""
    pass


class TimeoutException(Exception):
    pass


class FutureResult(object):
    """Completes when the robot has executed the instruction.

    :meth:`result` returns the value of a read instruction, ``True`` for all others.
    """

    def __init__(self, instruction):
        self.instruction = instruction
        self.done = False
        self.value = True
        self.error = None
        self._event = threading.Event()

    def result(self, timeout=None):
        if not self._event.wait(timeout):
            raise TimeoutException('Timeout: {} not executed yet'.format(self.instruction))
        if self.error:
            raise self.error
        return self.value

    def _set(self, value=True, error=None):
        self.value = value
        self.error = error
        self.done = True
        self._event.set()


class UrClient(object):
    """Streams instructions to a UR robot through RTDE registers.

    Parameters
    ----------
    ip : str
        IP address of the robot.
    register_offset : int, optional
        First RTDE register used. By default 24, the upper register range,
        leaving 0-23 free for other software such as ur_rtde. Controllers
        without the upper range (CB3 before 3.9, e-Series before 5.3) use 0.
    buffer_size : int
        Number of instructions buffered on the robot.
    frequency : float, optional
        RTDE frequency in Hz. By default the controller frequency: 125 for CB3, 500 for e-Series.
    rtde_port : int
        Port of the RTDE interface.
    script_port : int
        Port used to upload URScript programs.
    """

    def __init__(self, ip, register_offset=None, buffer_size=200, frequency=None,
                 rtde_port=RTDE_PORT, script_port=SCRIPT_PORT):
        self.ip = ip
        self.register_offset = register_offset
        self.buffer_size = buffer_size
        self.frequency = frequency
        self.rtde_port = rtde_port
        self.script_port = script_port
        self.batch_size = None
        self.controller_version = None

        self.rtde = None
        self.session = None
        self.counter = 0  # last sequence id handed out by send()

        self._lock = threading.Condition()
        self._queue = collections.deque()  # instructions not yet sent to the robot
        self._futures = collections.deque()  # futures not yet executed, in sequence order
        self._sent = 0  # last sequence id acknowledged by the robot
        self._results = {}  # sequence id -> values written by the driver
        self._last_result_seq = 0
        self._waits = collections.deque()  # sequence ids of WaitForResume not resumed yet
        self._input_lock = threading.Lock()  # input registers are written from two threads
        self._state = {}
        self._error = None
        self._closing = False
        self._sender = None

    # connection

    def connect(self, timeout=10.0):
        """Connects over RTDE, uploads the driver and waits until it is ready."""
        self.rtde = RTDEConnection(self.ip, self.rtde_port)
        self.controller_version = self.rtde.controller_version
        self._configure_for_controller(self.controller_version[:2])

        o = self.register_offset
        outputs = (['runtime_state', 'actual_TCP_pose', 'actual_q', 'actual_digital_input_bits'] +
                   ['output_int_register_{}'.format(o + i) for i in range(6)] +
                   ['output_double_register_{}'.format(o + i) for i in range(driver.NUM_RESULTS)])
        inputs = (['input_int_register_{}'.format(o + i) for i in range(4 + self.batch_size)] +
                  ['input_double_register_{}'.format(o + i) for i in range(self.batch_size * driver.NUM_PARAMS)])

        self.rtde.setup_outputs(outputs, self.frequency)
        self.rtde.setup_inputs(inputs)
        self._inputs = dict((name, 0) for name in inputs)
        for name in inputs:
            if name.startswith('input_double'):
                self._inputs[name] = 0.0
        self.rtde.on_data = self._on_data
        self.rtde.on_disconnect = self._on_disconnect
        self.rtde.start()

        # registers keep their values between programs, so a random session id
        # tells a fresh driver apart from an old one
        self.session = random.randint(1, 2 ** 31 - 1)
        self._inputs[self._in_int(2)] = self.session
        self.rtde.send_inputs(self._inputs)
        self._wait_for_state(timeout)

        self.upload_script(driver.generate_driver_script(o, self.batch_size, self.buffer_size))
        start = time.time()
        with self._lock:
            while self._state.get(self._out_int(2)) != self.session:
                if self._error:
                    raise self._error
                if time.time() - start > timeout:
                    raise DriverError('Driver did not start. Is the robot in remote control mode and powered on?')
                self._lock.wait(0.1)

        self._sender = threading.Thread(target=self._send_loop, daemon=True)
        self._sender.start()

    def close(self, timeout=None):
        """Waits until all sent instructions are executed, ends the driver and disconnects."""
        if self.rtde is None:
            return
        if not self._error:
            self.send_and_wait(End(), timeout)
        with self._lock:
            self._closing = True
            self._lock.notify_all()
        self.rtde.close()
        self.rtde = None

    def abort(self, digital_outputs=None):
        """Stops the robot immediately by replacing the driver with a stop script.

        Parameters
        ----------
        digital_outputs : dict, optional
            Standard digital outputs to set after stopping, e.g. ``{1: True}``
            to switch an extruder off.
        """
        lines = ['def compas_urc_abort():', '  stopl(1.0)']
        for io, value in (digital_outputs or {}).items():
            lines.append('  set_standard_digital_out({}, {})'.format(io, 'True' if value else 'False'))
        lines += ['  textmsg("compas_urc: aborted")', 'end', 'compas_urc_abort()']
        self.upload_script('\n'.join(lines) + '\n')
        self._fail(DriverError('Aborted'))
        if self.rtde:
            self.rtde.close()
            self.rtde = None

    def upload_script(self, script):
        """Sends a URScript program to the robot, replacing the running program."""
        s = socket.create_connection((self.ip, self.script_port), timeout=5.0)
        try:
            s.sendall(script.encode('utf-8'))
            # The port streams robot state. Closing with unread data sends a TCP
            # reset that makes the controller drop the rest of a long script
            # (anything over ~14 kB), so close gracefully and drain first.
            s.shutdown(socket.SHUT_WR)
            s.settimeout(0.1)
            end = time.time() + 0.5
            while time.time() < end:
                try:
                    if not s.recv(65536):
                        break
                except socket.timeout:
                    pass
        finally:
            s.close()

    # sending

    def send(self, instruction):
        """Queues an instruction and returns a :class:`FutureResult` immediately.

        Read instructions with ``immediate=True`` are not queued: their future
        is completed right away from the live robot state.
        """
        with self._lock:
            if self._error:
                raise self._error
            future = FutureResult(instruction)
            if isinstance(instruction, ReadInstruction) and instruction.immediate:
                if not self._state:
                    raise DriverError('Not connected')
                future._set(instruction.read_state(self._state))
                return future
            self.counter += 1
            instruction.sequence_id = self.counter
            if isinstance(instruction, WaitForResume):
                self._waits.append(instruction.sequence_id)
            self._queue.append(instruction)
            self._futures.append(future)
            self._lock.notify_all()
        return future

    def send_and_wait(self, instruction, timeout=None):
        """Queues an instruction and waits until the robot has executed it.

        Returns the value of a read instruction, ``True`` for all others.
        For a blended move, executed means the robot has started blending into
        the next move, as with movel in URScript.
        """
        return self.send(instruction).result(timeout)

    def resume(self):
        """Lets the robot continue past the next :class:`WaitForResume`.

        Can be called before the robot reaches it, the robot then does not stop.
        """
        with self._lock:
            if not self._waits:
                raise ValueError('No WaitForResume to resume')
            seq = self._waits.popleft()
        with self._input_lock:
            self._inputs[self._in_int(3)] = seq
            self.rtde.send_inputs(self._inputs)

    def wait_all(self, timeout=None):
        """Waits until every instruction sent so far has been executed."""
        with self._lock:
            future = self._futures[-1] if self._futures else None
        if future:
            future.result(timeout)

    # state

    @property
    def executed(self):
        """Sequence id of the last executed instruction."""
        return self._state.get(self._out_int(1), 0)

    @property
    def underruns(self):
        """Number of times the robot buffer ran empty after a blended move."""
        return self._state.get(self._out_int(3), 0)

    @property
    def pending(self):
        """Number of instructions sent but not yet executed."""
        return self.counter - self.executed

    @property
    def waiting(self):
        """True while the robot is stopped at a :class:`WaitForResume`."""
        return self._state.get(self._out_int(5), 0) > 0

    # internals

    def _configure_for_controller(self, version):
        """Picks frequency and registers for the controller version (major, minor).

        Register availability as documented by UR and used by ur_rtde:
        registers 0-23 from CB3 3.4, registers 24-47 from CB3 3.9 and e-Series 5.3.
        """
        major, minor = version
        cb3 = major < 5
        if cb3 and (major, minor) < (3, 4):
            raise DriverError('Controller {}.{} is too old: compas_urc needs CB3 3.4 or newer'.format(major, minor))
        if self.frequency is None:
            self.frequency = 125.0 if cb3 else 500.0
        if self.register_offset is None:
            has_upper_range = (major, minor) >= (3, 9) if cb3 else (major, minor) >= (5, 3)
            self.register_offset = 24 if has_upper_range else 0
        # each batch slot needs NUM_PARAMS double registers, 48 registers in total
        self.batch_size = min(4, (48 - self.register_offset) // driver.NUM_PARAMS)
        if self.batch_size < 1:
            raise ValueError('register_offset too high, not enough registers left')

    def _in_int(self, i):
        return 'input_int_register_{}'.format(self.register_offset + i)

    def _in_double(self, i):
        return 'input_double_register_{}'.format(self.register_offset + i)

    def _out_int(self, i):
        return 'output_int_register_{}'.format(self.register_offset + i)

    def _out_double(self, i):
        return 'output_double_register_{}'.format(self.register_offset + i)

    def _wait_for_state(self, timeout):
        start = time.time()
        with self._lock:
            while not self._state:
                if time.time() - start > timeout:
                    raise DriverError('No data received from the robot over RTDE')
                self._lock.wait(0.1)

    def _on_data(self, state):
        with self._lock:
            self._state = state
            if state[self._out_int(2)] == self.session:
                # the driver holds each result for at least one cycle, so every result is seen here
                result_seq = state[self._out_int(4)]
                if result_seq != self._last_result_seq:
                    self._last_result_seq = result_seq
                    self._results[result_seq] = [state[self._out_double(i)] for i in range(driver.NUM_RESULTS)]
                executed = state[self._out_int(1)]
                while self._futures and self._futures[0].instruction.sequence_id <= executed:
                    future = self._futures.popleft()
                    instruction = future.instruction
                    if isinstance(instruction, ReadInstruction):
                        values = self._results.pop(instruction.sequence_id, None)
                        if values is None:
                            future._set(error=DriverError('Result of {} was not received'.format(instruction)))
                        else:
                            future._set(instruction.parse_result(values))
                    else:
                        future._set()
                if state['runtime_state'] in (RUNTIME_STOPPING, RUNTIME_STOPPED) and not self._closing:
                    self._fail_locked(DriverError('Robot program stopped'))
            self._lock.notify_all()

    def _on_disconnect(self, error):
        self._fail(DriverError('RTDE connection lost: {}'.format(error)))

    def _fail(self, error):
        with self._lock:
            self._fail_locked(error)

    def _fail_locked(self, error):
        if self._error:
            return
        self._error = error
        while self._futures:
            self._futures.popleft()._set(error=error)
        self._queue.clear()
        self._lock.notify_all()

    def _send_loop(self):
        try:
            while True:
                with self._lock:
                    # wait for instructions and free space in the robot buffer
                    while True:
                        if self._error or self._closing:
                            return
                        executed = self._state.get(self._out_int(1), 0)
                        room = self.buffer_size - (self._sent - executed)
                        if self._queue and room > 0:
                            break
                        self._lock.wait(0.1)
                    n = min(self.batch_size, len(self._queue), room)
                    batch = [self._queue.popleft() for _ in range(n)]

                last_seq = batch[-1].sequence_id
                with self._input_lock:
                    for k, instruction in enumerate(batch):
                        self._inputs[self._in_int(4 + k)] = instruction.code
                        for j, value in enumerate(instruction.params):
                            self._inputs[self._in_double(k * driver.NUM_PARAMS + j)] = float(value)
                    self._inputs[self._in_int(1)] = n
                    # data and sequence id go out in the same package, so the robot sees them together
                    self._inputs[self._in_int(0)] = last_seq
                    self.rtde.send_inputs(self._inputs)

                with self._lock:
                    while self._state.get(self._out_int(0)) != last_seq:
                        if self._error or self._closing:
                            return
                        self._lock.wait(0.1)
                    self._sent = last_seq
        except (OSError, RTDEError) as error:
            self._fail(DriverError('Sending failed: {}'.format(error)))
