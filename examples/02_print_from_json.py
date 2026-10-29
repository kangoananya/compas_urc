"""Print a path exported from Grasshopper as JSON, switching the extruder per point.

Expected JSON: a list of print points, each with
    "point":  [x, y, z] in mm, in the workobject
    "blend":  blend radius in mm
    "toggle": true while extruding on the segment leaving this point
"""
import math
import os
import time

import compas
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas.geometry import Translation

import compas_urc as urc

IP_ADDRESS = "127.0.0.1"
PRINT_FILE = os.path.join(os.path.dirname(__file__), "data", "out_printpoints.json")

SPEED = 100.0  # mm/s
ACCEL = 100.0  # mm/s2
Z_OFFSET = 0.9  # mm, fine tuning of the first layer

WORKOBJECT = Frame.from_points(Point(514.20, 99.0, -8.95), Point(494.6, 273.5, -9.62), Point(364.59, 219.5, -8.70))
TCP = Frame.from_axis_angle_vector([0.0, math.pi, 0.0], point=[-44.29, -59.34, 189.65])


# extruder wiring: DO1 False + DO2 False + DO3 True = extruding, DO1 True = off
def extruder_on(ur):
    ur.send(urc.SetDigital(1, False))
    ur.send(urc.SetDigital(2, False))
    ur.send(urc.SetDigital(3, True))


def extruder_off(ur):
    ur.send(urc.SetDigital(1, True))


if __name__ == "__main__":
    with open(PRINT_FILE, "r") as f:
        data = compas.json_load(f)

    T = Translation.from_vector([0, 0, Z_OFFSET]) * Transformation.from_frame_to_frame(Frame.worldXY(), WORKOBJECT)
    frames = [Frame(item["point"], [1, 0, 0], [0, 1, 0]).transformed(T) for item in data]
    blends = [item["blend"] for item in data]
    toggles = [item["toggle"] for item in data]
    print("{} frames loaded from {}".format(len(frames), PRINT_FILE))

    ur = urc.URClient(IP_ADDRESS)
    ur.connect()
    start_time = time.time()

    try:
        ur.send(urc.SetTcp(TCP))
        extruding = None
        for i, frame in enumerate(frames):
            blend = 0.0 if i == 0 or i == len(frames) - 1 else blends[i]
            ur.send(urc.MoveToFrame(frame, SPEED, ACCEL, blend, urc.Motion.PROCESS))
            # the robot switches the IO when it starts blending around this point
            if toggles[i] != extruding:
                extruding = toggles[i]
                if extruding:
                    extruder_on(ur)
                else:
                    extruder_off(ur)
        extruder_off(ur)

        while True:
            try:
                ur.wait_all(timeout=1.0)
                break
            except urc.TimeoutException:
                print("{:.1f}%  underruns: {}".format(100.0 * ur.executed / ur.counter, ur.underruns))
        ur.close()
        print("done in {:.0f} s".format(time.time() - start_time))

    except KeyboardInterrupt:
        ur.abort(digital_outputs={1: True})
        print("aborted")
