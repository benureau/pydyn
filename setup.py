#!/usr/bin/env python

from distutils.core import setup
from pydyn import __version__ as VERSION

setup(name='pydyn',
      version=VERSION,
      author=', '.join(['Fabien Benureau', 'Olivier Mangin', 'Pierre Rouanet', 'Matthieu Lapeyre', 'Haylee Fogg']),
      author_email='fabien.benureau+pydyn@gmail.com',
      url='https://github.com/humm/pydyn.git',
      description='Python Library for Robot Control',
      requires=['serial', 'pyftdi',
                'sphinx_rtd_theme' # for the doc
               ],
      packages = ['pydyn', 'pydyn.dynamixel',
                           'pydyn.ios',       'pydyn.ios.fakeio',
                                              'pydyn.ios.kinio',
                                              'pydyn.ios.serialio',
                                              'pydyn.ios.vrepio',
                            'pydyn.msets',
                            'pydyn.refs',],
      )
