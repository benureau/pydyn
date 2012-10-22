import debugenv # only to debug local (not installed) pypot version

import sys, time

import pypot
import pypot.dynamixel as dyn

baudrate = 1000000
if len(sys.argv) == 2:
    baudrate = int(sys.argv[1])

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 20], start = False, baudrate = baudrate)
ctrl.start()

# Choosing a motor

if len(ctrl.motors) == 0:
    exit(0)

print('Choose a motor [{}]: '.format(ctrl.motors[0].id)),
choosen_id = raw_input()
if choosen_id == '':
    choosen_id = ctrl.motors[0].id
    
motor = None
for m in ctrl.motors:
    if m.id == int(choosen_id):
        motor = m
        break

if motor is None:
    print('Choosen motor {} not present. Aborting'.format(choosen_id))
    exit(0)
    

# Printing EEPROM

print('\n')
print(motor.eeprom_desc())

# Reconfiguring

print('in the following, press Enter to skip')

print('new id [{}] : '.format(motor.id)),
new_id = raw_input()
if new_id != '':
    motor.id = int(new_id)
    
print('new return delay time [{} usec] : '.format(motor.return_delay_time)),
new_rdt = raw_input()
if new_rdt != '':
    motor.return_delay_time = int(new_rdt)
    
print('new cw angle limits [{} degree] : '.format(motor.cw_angle_limit)),
new_cwal = raw_input()
if new_cwal != '':
    motor.cw_angle_limit = float(new_cwal)

print('new ccw angle limits [{} degree] : '.format(motor.ccw_angle_limit)),
new_ccwal = raw_input()
if new_ccwal != '':
    motor.ccw_angle_limit = float(new_ccwal)
    
print('new max torque [{} % of max] : '.format(motor.max_torque)),
new_mt = raw_input()
if new_mt != '':
    motor.max_torque = float(new_mt)
    
print('new status return level [{}] : '.format(motor.status_return_level)),
new_srl = raw_input()
if new_srl != '':
    motor.status_return_level = float(new_srl)

print('new baudrate [{}] : '.format(motor.baudrate)),
new_bps = raw_input()
if new_bps != '':
    motor.baudrate = float(new_bps)
    time.sleep(0.2)
    ctrl.close()
    ctrl = dyn.create_controller(motor_range = [motor.id, motor.id], start = False, baudrate = new_bps, verbose = False)
    
time.sleep(0.2)
    
print('done !')
print('\n')
print(ctrl.motors[0].eeprom_desc())
