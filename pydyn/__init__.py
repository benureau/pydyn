from . import color
from .msets import MotorSet

from dynamixel.hub import connect, disconnect, motors, controller

version_info = (0, 9, 2, 'dev')
__version__ = '.'.join(str(v) for v in version_info)
