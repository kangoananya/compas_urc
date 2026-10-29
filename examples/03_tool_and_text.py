"""Sets a tool and writes messages to the robot log. Nothing moves.

Check on the teach pendant:
- the Log tab shows "compas_urc: hello 1" ... "hello 3", in order,
- Installation > TCP shows the active TCP set by the script,
- the driver keeps running after the messages (the script reads the frame afterwards).
"""
import math

from compas.geometry import Frame

import compas_urc as urc

IP_ADDRESS = "127.0.0.1"

# the extruder from 02_print_from_json.py; set MASS to the real tool weight in kg,
# or leave it None to keep the payload set on the robot
MASS = None
EXTRUDER = urc.Tool(Frame.from_axis_angle_vector([0.0, math.pi, 0.0], point=[-44.29, -59.34, 189.65]),
                    mass=MASS, name="extruder")

ur = urc.URClient(IP_ADDRESS)
ur.connect()
print("Connected, controller", ur.controller_version)

try:
    flange = ur.send_and_wait(urc.GetFrame())
    ur.send_and_wait(urc.SetTool(EXTRUDER))
    tcp = ur.send_and_wait(urc.GetFrame())
    print("Frame before SetTool (mm):", flange.point)
    print("Frame after SetTool (mm): ", tcp.point)

    for i in range(1, 4):
        ur.send(urc.PrintText("compas_urc: hello {}".format(i)))
    ur.wait_all()

    # if a message had replaced the driver, this read fails with DriverError
    frame = ur.send_and_wait(urc.GetFrame(), timeout=5)
    print("Driver still running after the messages, frame (mm):", frame.point)
    ur.close()
    print("Bye.")
except KeyboardInterrupt:
    ur.abort()
