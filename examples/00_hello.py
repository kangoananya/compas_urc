"""First connection: starts the driver and reads where the robot is."""
import compas_urc as urc

IP_ADDRESS = "127.0.0.1"

ur = urc.URClient(IP_ADDRESS)
ur.connect()
print("Connected, driver running on the robot.")

frame = ur.send_and_wait(urc.GetFrame())
joints = ur.send_and_wait(urc.GetJoints())
print("TCP frame (mm):", frame)
print("Joints (rad):", [round(j, 3) for j in joints])

ur.close()
print("Bye.")
