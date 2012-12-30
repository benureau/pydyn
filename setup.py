#!/usr/bin/env python

from distutils.core import setup

setup(name='pypot',
      version='0.3.0',
      author='Haylee Fogg, Matthieu Lapeyre, Pierre Rouanet, Fabien Benureau',
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pypot-fork',
      description='Python Library for Dynamixel Motor Control',
      requires=['serial'],
      packages = ['pypot', 'pypot.robot', 'pypot.dynamixel'],
      )
