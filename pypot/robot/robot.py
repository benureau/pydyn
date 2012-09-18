
class Robot(object):
    def __init__(self, name, controllers=()):
        self.name = name
        self.controllers = controllers
        self._motor4name = {}
    
        [c.start_sync() for c in self.controllers]
    
    def __getattr__(self, name):
        print 'try to access', name
        
        # TODO: check that name is a motor_name
        return self._lazy_get_motor(name)
    
    def _lazy_get_motor(self, motor_name):
        if not self._motor4name.has_key(motor_name):
            self._get_motor(motor_name)
        
        return self._motor4name[motor_name]
    
    def _get_motor(self, motor_name):
        for c in self.controllers:
            for m in c.motors:
                if m.name == motor_name:
                    self._motor4name[motor_name] = m
                    return m

