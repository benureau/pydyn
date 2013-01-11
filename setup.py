#!/usr/bin/env python

from distutils.core import setup

setup(name='pydyn',
      version='0.2.0',
      author=('Haylee Fogg, Olivier Mangin, Matthieu Lapeyre, Pierre Rouanet,'
              'Fabien Benureau'),
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pydyn-fork',
      description='Python Library for Robot Control',
      requires=['serial'],
      packages = ['pydyn', 'pydyn.robot', 'pydyn.dynamixel'],
      )
