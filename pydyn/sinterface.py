import time, traceback
import socket
import threading

available = True
try:
    import sockit
except ImportError:
    available = False

# Protocol
HELLO_TYPE = 0
BYE_TYPE   = 1
STEM_TYPE  = 2
ORDER_TYPE = 3

def encode(ctrl):
    msg = sockit.OutboundMessage(type_msg = STEM_TYPE)
    # number of motors
    msg.appendInt(len(ctrl.motors))

    # position
    for m in ctrl.motors:
        msg.appendDouble(float(m.torque_enable))

    # position
    for m in ctrl.motors:
        msg.appendDouble(m.position)
    # goal positions
    for m in ctrl.motors:
        msg.appendDouble(m.goal_position)
    # position limits
    for m in ctrl.motors:
        cw_i, ccw_i = m.angle_limits
        msg.appendDouble(cw_i)
        msg.appendDouble(ccw_i)

    # load
    for m in ctrl.motors:
        msg.appendDouble(m.load)
    # max_torque
    for m in ctrl.motors:
        msg.appendDouble(m.max_torque)
    # torque_limit
    for m in ctrl.motors:
        msg.appendDouble(m.torque_limit)

    # speed
    for m in ctrl.motors:
        msg.appendDouble(m.speed)
    # max_speed
    for m in ctrl.motors:
        msg.appendDouble(m.moving_speed)

    # speed
    for m in ctrl.motors:
        msg.appendDouble(float(m.temp))
    # max_speed
    for m in ctrl.motors:
        msg.appendDouble(float(m.max_temp))


    return msg

decoder = {

    0 : 'torque_enable',

    1 : 'goal_position',
    2 : 'cw_angle_limit',
    3 : 'ccw_angle_limit',

    4 : 'present_load',
    5 : 'torque_limit',
    6 : 'max_torque',

    7 : 'present_speed',
    8 : 'moving_speed',

    9 : 'present_temp',
    10 : 'max_temp',

}

def decode(msg, ctrl):
    assert msg.type == ORDER_TYPE

    motor_value = decoder[msg.readInt()]
    m_index     = msg.readInt()
    value       = msg.readDouble()

    #print motor_value, m_index, value
    ctrl.motors[m_index].request_write(motor_value, value)


class SInterface(threading.Thread):


    def __init__(self, ctrl, ip = '127.0.0.1', port = 1984, debug = False):
        threading.Thread.__init__(self)

        self.ctrl  = ctrl
        self.ip    = ip
        self.port  = port
        self.debug = debug

        self.daemon = True

    def run(self):

        if not available:
            print("error importing sockit. socket interface disabled.")
            return

        while True:
            c = sockit.Client(debug = False)
            b = c.connect(self.ip, self.port)

            try:
                assert b, "ctrl: error in connection"

                hello = sockit.OutboundMessage()
                hello.type = HELLO_TYPE
                hello.appendString("Hello server !")
                c.send(hello)

                while True:
                    f = encode(self.ctrl)
                    a = c.sendAndReceive(f)
                    assert a is not None
                    if a.type == ORDER_TYPE:
                        decode(a, self.ctrl)
                    else:
                        assert a.type == STEM_TYPE, "ctrl, error in exchange"
                    time.sleep(0.001)

                bye = sockit.OutboundMessage()
                bye.type = BYE_TYPE
                bye.appendString("Bye Server !")
                c.send(bye)
                c.disconnect()

            except KeyboardInterrupt:
                c.disconnect()
                return
            except Exception as inst:
                if self.debug:
                    traceback.print_exc()
                c.disconnect()
