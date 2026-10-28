"""First connection: starts the driver and reads where the robot is. Nothing moves."""
import compas_urc as urc

IP_ADDRESS = "192.168.10.10"

ur = urc.UrClient(IP_ADDRESS)
ur.connect()
print("Connected, driver running on the robot.")

frame = ur.send_and_wait(urc.GetFrame())
joints = ur.send_and_wait(urc.GetJoints())
print("TCP frame (mm):", frame)
print("Joints (rad):", [round(j, 3) for j in joints])

ur.close()
print("Bye.")
