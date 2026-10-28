import math
import time

import pytest
from compas.geometry import Frame
from compas.geometry import Point

import compas_urc as urc
from compas_urc import driver
from compas_urc.instructions import frame_to_pose

from fake_robot import FakeRobot


def connect(robot, buffer_size=200):
    ur = urc.URClient('127.0.0.1', buffer_size=buffer_size, rtde_port=robot.rtde_port, script_port=robot.script_port)
    ur.connect()
    return ur


def frames(n):
    return [Frame(Point(300 + (i % 100), 200 + i * 0.001, 10), [1, 0, 0], [0, 1, 0]) for i in range(n)]


def check_log(robot, expected):
    log = robot.executed_log
    assert log[-1][0] == driver.CODE_END
    log = log[:-1]
    assert len(log) == len(expected)
    for (code, params), instruction in zip(log, expected):
        assert code == instruction.code
        assert params == pytest.approx(instruction.params, abs=1e-12)


def test_streams_all_instructions_in_order():
    robot = FakeRobot(exec_time=0.0005)
    ur = connect(robot)
    expected = [urc.SetTcp(Frame.from_axis_angle_vector([0, math.pi, 0], point=[-44.29, -59.34, 189.65]))]
    for i, frame in enumerate(frames(5000)):
        expected.append(urc.MoveToFrame(frame, 100, 500, 2))
        if i % 500 == 0:
            expected.append(urc.SetDigital(1, (i // 500) % 2 == 0))
    for instruction in expected:
        ur.send(instruction)
    ur.wait_all(timeout=60)
    assert ur.executed == len(expected)
    ur.close(timeout=10)
    check_log(robot, expected)


def test_buffer_never_overflows_with_slow_robot():
    robot = FakeRobot(exec_time=0.005)
    ur = connect(robot, buffer_size=50)
    expected = [urc.MoveToFrame(frame, 100, 500, 2) for frame in frames(300)]
    for instruction in expected:
        ur.send(instruction)
    ur.close(timeout=30)
    assert robot.max_count == 50
    check_log(robot, expected)


def test_send_and_wait_returns_when_executed():
    robot = FakeRobot()
    ur = connect(robot)
    for frame in frames(20):
        ur.send(urc.MoveToFrame(frame, 100, 500, 1))
    instruction = urc.SetDigital(2, True)
    assert ur.send_and_wait(instruction, timeout=10)
    assert ur.executed >= instruction.sequence_id
    ur.close(timeout=10)


def test_abort_fails_pending_futures():
    robot = FakeRobot(exec_time=0.01)
    ur = connect(robot)
    futures = [ur.send(urc.MoveToFrame(frame, 100, 500, 1)) for frame in frames(500)]
    ur.abort(digital_outputs={1: True})
    with pytest.raises(urc.DriverError):
        futures[-1].result(timeout=5)
    with pytest.raises(urc.DriverError):
        ur.send(urc.WaitTime(1))


def test_frame_to_pose_units():
    frame = Frame(Point(100, 200, 300), [1, 0, 0], [0, 1, 0])
    assert frame_to_pose(frame) == pytest.approx([0.1, 0.2, 0.3, 0, 0, 0])
    with pytest.raises(TypeError):
        frame_to_pose([0.1, 0.2, 0.3, 0, 0, 0])


def test_move_to_frame_units():
    instruction = urc.MoveToFrame(Frame.worldXY(), speed=100, accel=500, blend=2)
    assert instruction.code == driver.CODE_MOVEL
    assert instruction.params[6:9] == pytest.approx([0.1, 0.5, 0.002])


def test_driver_script_register_layout():
    script = driver.generate_driver_script(24, 2, 100)
    assert 'def compas_urc_driver():' in script
    assert script.rstrip().endswith('compas_urc_driver()')
    # highest double register used: 24 + 1 * 9 + 8 = 41, must stay below 48
    assert 'read_input_float_register(24 + k * 9 + 8)' in script
    assert 'bcode[w] = read_input_integer_register(28 + k)' in script
    assert '{' not in script and '}' not in script


def test_get_frame_is_in_sequence():
    robot = FakeRobot()
    ur = connect(robot)
    path = frames(50)
    for i, frame in enumerate(path):
        ur.send(urc.MoveToFrame(frame, 100, 500, 1))
        if i in (10, 30):
            future = ur.send(urc.GetFrame())
            if i == 10:
                first = future
    # each read returns the frame of the move right before it, not the latest one
    assert list(first.result(timeout=10).point) == pytest.approx(list(path[10].point), abs=1e-9)
    assert list(future.result(timeout=10).point) == pytest.approx(list(path[30].point), abs=1e-9)
    ur.close(timeout=10)


def test_get_joints_and_read_digital():
    robot = FakeRobot()
    ur = connect(robot)
    ur.send(urc.MoveToJoints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 1.0, 1.0))
    assert ur.send_and_wait(urc.GetJoints(), timeout=10) == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    robot.outputs['actual_digital_input_bits'] = 0b100
    assert ur.send_and_wait(urc.ReadDigital(2), timeout=10) is True
    assert ur.send_and_wait(urc.ReadDigital(1), timeout=10) is False
    ur.close(timeout=10)


def test_consecutive_reads_are_not_lost():
    robot = FakeRobot()
    ur = connect(robot)
    futures = [ur.send(urc.GetJoints()) for _ in range(30)]
    for future in futures:
        assert future.result(timeout=10) == pytest.approx([0.1, -1.5, 1.5, -1.5, -1.5, 0.0])
    ur.close(timeout=10)


def test_immediate_read_does_not_wait_for_queue():
    robot = FakeRobot(exec_time=0.01)
    ur = connect(robot)
    for frame in frames(300):
        ur.send(urc.MoveToFrame(frame, 100, 500, 1))
    future = ur.send(urc.GetFrame(immediate=True))
    assert future.done
    assert ur.pending > 100
    ur.abort()


def test_wait_for_resume():
    robot = FakeRobot()
    ur = connect(robot)
    ur.send(urc.MoveToFrame(frames(1)[0], 100, 500))
    ur.send(urc.WaitForResume())
    after = ur.send(urc.SetDigital(1, True))
    time.sleep(0.3)
    assert ur.waiting
    assert not after.done
    ur.resume()
    after.result(timeout=5)
    assert not ur.waiting
    # resuming before the robot gets there: it does not stop
    ur.send(urc.WaitForResume())
    ur.resume()
    ur.send_and_wait(urc.SetDigital(1, False), timeout=5)
    with pytest.raises(ValueError):
        ur.resume()
    ur.close(timeout=10)


def test_wait_for_digital_input():
    robot = FakeRobot()
    ur = connect(robot)
    after = ur.send(urc.WaitForDigitalInput(3, True))
    time.sleep(0.3)
    assert not after.done
    robot.outputs['actual_digital_input_bits'] = 0b1000
    after.result(timeout=5)
    ur.close(timeout=10)


def test_movep():
    instruction = urc.MoveToFrame(Frame.worldXY(), speed=100, accel=500, blend=2, motion_type=urc.Motion.PROCESS)
    assert instruction.code == driver.CODE_MOVEP
    assert instruction.params[6:9] == pytest.approx([0.1, 0.5, 0.002])
    robot = FakeRobot()
    ur = connect(robot)
    path = frames(20)
    for frame in path:
        ur.send(urc.MoveToFrame(frame, 100, 500, 1, motion_type=urc.Motion.PROCESS))
    frame = ur.send_and_wait(urc.GetFrame(), timeout=10)
    assert list(frame.point) == pytest.approx(list(path[-1].point), abs=1e-9)
    ur.close(timeout=10)


@pytest.mark.parametrize('version, offset, batch, frequency', [
    ((5, 15), 24, 2, 500.0),  # e-Series with upper registers
    ((5, 2), 0, 4, 500.0),    # early e-Series, lower registers only
    ((3, 15), 24, 2, 125.0),  # CB3 with upper registers
    ((3, 8), 0, 4, 125.0),    # older CB3, lower registers only
])
def test_configures_for_controller_version(version, offset, batch, frequency):
    robot = FakeRobot(version=version)
    ur = connect(robot)
    assert (ur.register_offset, ur.batch_size, ur.frequency) == (offset, batch, frequency)
    assert robot.frequency == frequency
    expected = [urc.MoveToFrame(frame, 100, 500, 1) for frame in frames(200)]
    for instruction in expected:
        ur.send(instruction)
    ur.close(timeout=20)
    check_log(robot, expected)


def test_refuses_too_old_controller():
    robot = FakeRobot(version=(3, 3))
    ur = urc.URClient('127.0.0.1', rtde_port=robot.rtde_port, script_port=robot.script_port)
    with pytest.raises(urc.DriverError):
        ur.connect()


@pytest.mark.parametrize('buffer_size', [1, 200, 1000])
def test_driver_script_buffer_size(buffer_size):
    script = driver.generate_driver_script(24, 2, buffer_size)
    lines = script.splitlines()
    assert next(line for line in lines if line.startswith('  global bcode = ')).count('0') == buffer_size
    assert next(line for line in lines if line.startswith('  global b0 = ')).count('0.0') == buffer_size


def test_small_buffer_keeps_order():
    robot = FakeRobot(exec_time=0.002)
    ur = connect(robot, buffer_size=120)
    expected = [urc.MoveToFrame(frame, 100, 500, 1) for frame in frames(500)]
    for instruction in expected:
        ur.send(instruction)
    ur.close(timeout=30)
    check_log(robot, expected)
