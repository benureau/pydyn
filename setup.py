#!/usr/bin/env python

from distutils.core import setup

setup(name='pydyn',
      version='1.0',
      author=('Fabien Benureau, Pierre Rouanet, Olivier Mangin, Matthieu Lapeyre, Haylee Fogg'),
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pydyn-fork',
      description='Python Library for Robot Control',
      requires=['serial'],
      packages = ['pydyn', 'pydyn.robot', 'pydyn.dynamixel'],
      )
