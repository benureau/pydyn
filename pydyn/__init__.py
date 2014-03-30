from . import color
from .msets import MotorSet

from .refs.exc import MotorError
from .dynamixel.hub import connect, disconnect, motors, controller

version_info = (0, 9, 3, 'beta1')
__version__ = '.'.join(str(v) for v in version_info)
