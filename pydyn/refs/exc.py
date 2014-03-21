
class MotorError(Exception):
    """\
    MotorError exception are raised by the communication module,
    processed by the Controller, and set in the _error variable of
    the Motor instance. Motor instance will check that _error does not
    contains any errors before registering a request.
    """
    def __init__(self, mid, alarms):
        self.mid = mid
        self.alarms = alarms

    def __str__(self):
        return 'Motor {} triggered alarm{}: {}'.format(self.mid,
                    's' if len(self.alarms) > 1 else '',
                    self.alarms if len(self.alarms) > 1 else self.alarms[0])

