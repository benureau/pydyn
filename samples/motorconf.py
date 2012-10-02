import pypot
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [1, 100], start = False)

for m in ctrl.motors:
    print m.eeprom_repr()

print 'All motors ({}) eeprom will be configured as so :'.format(', '.join('{}'.format(m.id) for m in  ctrl.motors))
print '  angles limits to 20 and 280 degree.\n'

print 'Are you ok with that [y/n] ? ',
d = raw_input() 
if d == 'y':
    print 'Applying changes...'
    for m in ctrl.motors:
        ctrl.io.set_angle_limits(m.id, 20, 280)
    print 'Done'

    for m in ctrl.motors:
        print m.eeprom_repr()

else:
    print 'Exiting without making changes.'