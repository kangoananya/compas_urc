# compas_urc

Remote control for Universal Robots in the style of [compas_rrc](https://github.com/compas-rrc/compas_rrc): send compas frames and IO instructions from Python, and the robot executes them in order.

- **Any path length.** Instructions are streamed to the robot while it moves, so a print path can have 100 000 points or more.
- **IO in sync with motion.** Digital outputs are switched by the robot between the moves, not by your PC, so an extruder turns on and off at the right point, even with blending.
- **No ROS, no Docker.** Pure Python, connects directly to the robot over RTDE.

> **Status: alpha.** The Python side is tested against a simulated controller. Test new setups in the air, at low speed, with a hand on the emergency stop.

## Install

```bash
cd compas_urc
pip install -e .
```

Requires Python 3.9+ and compas 2.

## Quick start

```python
import math
from compas.geometry import Frame
import compas_urc as urc

ur = urc.URClient("192.168.10.10")
ur.connect()  # uploads the driver to the robot and starts it

# TCP offset from the flange, in mm
ur.send(urc.SetTcp(Frame.from_axis_angle_vector([0, math.pi, 0], point=[0, 0, 190])))

for frame in frames:  # compas frames in mm, robot base coordinates
    ur.send(urc.MoveToFrame(frame, speed=50, accel=200, blend=0.5))

ur.send(urc.SetDigital(1, True))
ur.wait_all()
ur.close()
```

`send` returns immediately: all instructions are queued on your PC and streamed to the robot in the background. Your script can queue a whole print in a second.

See [examples/](examples/):

- `00_hello.py`: connect and read where the robot is. Nothing moves. Start here.
- `01_ellipse_spiral.py`: motion test with `movep`, an elliptical spiral starting where you jogged the robot. No tool or measurements needed.
- `02_print_from_json.py`: a print from a Grasshopper JSON export, switching the extruder per point.

## Instructions

All lengths in millimeters, angles in radians, time in seconds.

| Instruction | Robot does |
|---|---|
| `MoveToFrame(frame, speed, accel, blend=0, motion_type=Motion.LINEAR)` | `movel` to the frame (mm/s, mm/s²). `Motion.PROCESS` uses `movep`, `Motion.JOINT` uses `movej` (rad/s, rad/s²) |
| `MoveToJoints(joints, speed, accel, blend=0)` | `movej` to six joint values (rad, rad/s, rad/s²) |
| `SetDigital(io, value)` | set a standard digital output |
| `SetToolDigital(io, value)` | set a tool digital output |
| `SetAnalog(io, value)` | set a standard analog output (0..1) |
| `WaitTime(seconds)` | wait; the robot stops, no blending through it |
| `SetTcp(frame)` | set the TCP offset from the flange |
| `GetFrame(immediate=False)` | returns the TCP frame (mm) |
| `GetJoints(immediate=False)` | returns the joint values (rad) |
| `ReadDigital(io, immediate=False)` | returns a standard digital input (0-7) as a bool |
| `WaitForResume()` | stops until you call `ur.resume()` |
| `WaitForDigitalInput(io, value=True)` | stops until a standard digital input (0-7) has the value |
| `End()` | end the driver program (`close()` sends this for you) |

### Motion types

- `Motion.LINEAR` (`movel`): straight lines, blends shorten the corners. The tool slows down in blends.
- `Motion.PROCESS` (`movep`): straight lines with circular blends at **constant tool speed**. Use this for printing and other process paths.
- `Motion.JOINT` (`movej`): fastest path in joint space, the tool does not move in a straight line. For travel moves in free space.

### Reading values

Read instructions return their value through `send_and_wait` or the future:

```python
frame = ur.send_and_wait(urc.GetFrame())
```

By default they are **in sequence**: the robot reads the value when it gets to the instruction, after everything sent before it. If you queued 1000 moves, `GetFrame()` returns the frame after the 1000th move.

An in-sequence read holds the driver for one controller cycle, so don't put one between two blended moves of a print path: the robot may stop there.

With `immediate=True`, the value comes right away from the live robot state, without waiting for the queue: "where is the robot now?"

```python
now = ur.send_and_wait(urc.GetFrame(immediate=True))
```

### Waiting for a human

`WaitForResume()` stops the robot at that point in the sequence, even if more instructions are already queued. The robot continues when your script calls `ur.resume()`:

```python
ur.send(urc.MoveToFrame(check_frame, 50, 200))
ur.send(urc.WaitForResume())
for frame in rest_of_path:
    ur.send(urc.MoveToFrame(frame, 50, 200, blend=0.5))

while not ur.waiting:  # robot not there yet
    time.sleep(0.1)
input("Remove the support, then press Enter")
ur.resume()
```

`WaitForDigitalInput(io)` does the same with a button wired to a digital input on the robot.

### Sending and waiting

As in compas_rrc:

- `ur.send(instruction)` queues and returns a future immediately.
- `ur.send_and_wait(instruction)` waits until the robot has executed it.
- `future = ur.send(...)` and later `future.result(timeout)` waits when you need it.
- `ur.wait_all()` waits until everything sent so far is executed.

"Executed" means the robot has run the instruction. For a blended move that is the moment the robot starts blending into the next move, as with `movel` in URScript. For a move with `blend=0`, it means the robot has arrived.

Progress while running: `ur.executed` (instructions executed), `ur.counter` (instructions sent), `ur.pending`, `ur.underruns`, `ur.waiting`.

### Stopping

`ur.abort()` stops the robot immediately and ends the driver. Pass outputs to set after stopping, e.g. `ur.abort(digital_outputs={1: True})` to switch an extruder off. Wrap your script in `try / except KeyboardInterrupt` and call `abort()` there, as in the examples.

## How it works

```
your script ──send()──▶ queue on PC ──RTDE registers──▶ buffer on robot ──▶ movel / IO
                                     ◀── executed, ack ──
```

1. `connect()` uploads a small URScript driver to the robot (port 30002).
2. A background thread on your PC copies instructions from the queue into RTDE input registers, two at a time, and waits for the robot to acknowledge them.
3. On the robot, a receiver thread copies acknowledged instructions into a ring buffer (200 instructions by default).
4. The driver's main loop executes instructions from that buffer. Because the next move is already in the robot's memory when a blended move ends, the robot blends through the path, and `set_standard_digital_out` runs exactly between two moves.
5. The robot reports the last executed instruction back over RTDE. This completes the futures and tells the PC how much room is left in the buffer. Read instructions write their value to RTDE output registers, where the client picks it up.

Your PC does not need to be real-time: it only has to keep the buffer from running empty. At 50 mm/s with 2 mm point spacing, 200 buffered points last 8 seconds.

### Underruns

If the buffer runs empty while the robot blends, the robot stops at the last point and `ur.underruns` goes up. This should not happen in normal use. If it does, increase `buffer_size`, e.g. `URClient(ip, buffer_size=500)`.

### Blend radius

Keep `blend` below half the distance between neighbouring points. Larger blends overlap and the robot may skip them.

## Robot setup

Works with CB3 (PolyScope 3.4 or newer) and e-Series. The client reads the controller version when connecting and configures itself:

| Controller | RTDE frequency | Registers used |
|---|---|---|
| e-Series 5.3+ | 500 Hz | 24–47 (0–23 stay free for other software) |
| e-Series before 5.3 | 500 Hz | 0–23 |
| CB3 3.9+ | 125 Hz | 24–47 |
| CB3 3.4–3.8 | 125 Hz | 0–23 |

You can override both: `URClient(ip, register_offset=0, frequency=125)`.

- **e-Series:** the robot must be in **remote control** mode (the driver is uploaded over the network). CB3 has no such mode.
- **CB3 is slower to stream:** at 125 Hz, roughly 100 instructions per second, versus several hundred on e-Series. That is enough for 2 mm points at 100 mm/s, but very dense paths at high speed may need sparser points.
- Other software using the same registers at the same time conflicts.
- Do not keep a ur_rtde `RTDEControlInterface` connected at the same time: it uploads its own program and replaces the driver.

If the driver does not start, check the robot's log for `compas_urc: driver starting` / `driver ready`, or a script error.

## Development

```bash
pip install -e .[dev]
pytest
```

The tests run against a fake controller (`tests/fake_robot.py`) that speaks RTDE and imitates the driver in Python. They test the client, not the URScript driver.
