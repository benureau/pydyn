import pypot
import pypot.dynamixel as dyn

raise DeprecationError

ctrl = dyn.create_controller(verbose = True, motor_range = [1, 100], start = False)

for m in ctrl.motors:
    print m.eeprom_repr()

print 'All motors ({}) will be configured as so :'.format(', '.join('{}'.format(m.id) for m in  ctrl.motors))
print '  - angles limits to 20 and 280 degree (EEPROM).'
print '  - compliance level to level 5/7 (RAM).'
print '  - compliance margins to 0 (RAM).'
print '  - max torque to 30% (RAM).'
print ''

print 'Are you ok with that [y/n] ? ',
d = raw_input() 
if d == 'y':
    print 'Applying changes...'
    for m in ctrl.motors:
        ctrl.io.set_angle_limits(m.id, 20, 280)
        ctrl.io.set_compliance_slopes(m.id, 32, 32)
        ctrl.io.set_compliance_margins(m.id, 0, 0)
        ctrl.io.set_max_torque(m.id, 100)
    print 'Done'

    for m in ctrl.motors:
        print m.eeprom_repr()

else:
    print 'Exiting without making changes.'