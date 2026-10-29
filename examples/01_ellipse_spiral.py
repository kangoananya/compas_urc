"""Motion test: an elliptical spiral with movep, no tool and no IO needed.

The spiral is placed at CENTER, a fixed point in the robot base coordinate
system (mm). Check that the area around it is free: about 140 x 100 mm in the
base XY plane and LAYERS mm above, plus APPROACH mm above the start point.
The robot first moves in a straight line to APPROACH mm above the start,
then down to the start, keeping the current tool orientation throughout.
The spiral lies parallel to the robot base and rises upwards.
"""
import math

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Translation

import compas_urc as urc

IP_ADDRESS = "127.0.0.1"

CENTER = Point(230.0, 475.0, 120.0)  # mm, in the robot base coordinate system

SPEED = 10.0  # mm/s
ACCEL = 100.0  # mm/s2
TRAVEL_SPEED = 100.0  # mm/s, for the approach and retract moves
TRAVEL_ACCEL = 500.0  # mm/s2

SEMI_AXIS_X = 100.0  # mm, along base x
SEMI_AXIS_Y = 50.0  # mm, along base y
LAYER_HEIGHT = 1.0  # mm
LAYERS = 30
POINT_SPACING = 2.0  # mm between points along the path
BLEND = 0.5  # mm, keep below half the point spacing

APPROACH = 50.0  # mm above the start point
RETRACT = 30.0  # mm up at the end


def ellipse_spiral(center, xaxis, yaxis, a, b, layer_height, layers, spacing):
    """Frames on a continuous elliptical spiral around ``center``.

    The ellipse lies parallel to the base XY plane, starts on its +x end, and
    rises layer_height per turn. All frames have the orientation given by
    ``xaxis`` and ``yaxis``. A spiral instead of stacked layers avoids a z step
    per layer, so the robot blends through the whole path without stopping.
    """
    # Ramanujan approximation of the ellipse perimeter
    h = ((a - b) / (a + b)) ** 2
    perimeter = math.pi * (a + b) * (1 + 3 * h / (10 + math.sqrt(4 - 3 * h)))
    per_layer = max(8, int(round(perimeter / spacing)))
    frames = []
    for i in range(per_layer * layers + 1):
        t = 2 * math.pi * i / per_layer
        z = layer_height * i / per_layer
        point = Point(center.x + a * math.cos(t), center.y + b * math.sin(t), center.z + z)
        frames.append(Frame(point, xaxis, yaxis))
    return frames


if __name__ == "__main__":
    ur = urc.URClient(IP_ADDRESS)
    ur.connect()
    print("Connected to controller {}.{}".format(*ur.controller_version[:2]))

    try:
        current = ur.send_and_wait(urc.GetFrame())
        print("Current TCP at", current.point)

        frames = ellipse_spiral(CENTER, current.xaxis, current.yaxis,
                                SEMI_AXIS_X, SEMI_AXIS_Y, LAYER_HEIGHT, LAYERS, POINT_SPACING)
        length = POINT_SPACING * (len(frames) - 1)
        print("Spiral around {}, starting at {}".format(CENTER, frames[0].point))
        print("{} frames, {:.0f} mm path, about {:.0f} s".format(len(frames), length, length / SPEED))

        ur.send(urc.MoveToFrame(frames[0].transformed(Translation.from_vector([0, 0, APPROACH])),
                                TRAVEL_SPEED, TRAVEL_ACCEL))
        ur.send(urc.MoveToFrame(frames[0], TRAVEL_SPEED, TRAVEL_ACCEL))
        for frame in frames[1:-1]:
            ur.send(urc.MoveToFrame(frame, SPEED, ACCEL, BLEND, urc.Motion.PROCESS))
        ur.send(urc.MoveToFrame(frames[-1], SPEED, ACCEL, 0.0, urc.Motion.PROCESS))
        ur.send(urc.MoveToFrame(frames[-1].transformed(Translation.from_vector([0, 0, RETRACT])),
                                TRAVEL_SPEED, TRAVEL_ACCEL))

        while True:
            try:
                ur.wait_all(timeout=1.0)
                break
            except urc.TimeoutException:
                print("{:.1f}%  underruns: {}".format(100.0 * ur.executed / ur.counter, ur.underruns))
        print("underruns: {}".format(ur.underruns))
        ur.close()
        print("done")

    except KeyboardInterrupt:
        ur.abort()
        print("aborted")
