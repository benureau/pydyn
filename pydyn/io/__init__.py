"""
The io module is responsible for abstracting low-level communications, so that
different back-end can be implemented. Currently there is a
    * serial backend
    * vrep backend (with simulated motors)
"""

from . import ftdiserial
