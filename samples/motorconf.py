import pypot
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [1, 100], start = False)

for m in ctrl.motors:
    ctrl.io.set_angle_limits(m.id, 20, 280)

print('Done')    