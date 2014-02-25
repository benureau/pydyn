#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='pydyn',
      version='0.9.1',
      author=('Haylee Fogg, Olivier Mangin, Matthieu Lapeyre, Pierre Rouanet,'
              'Fabien Benureau'),
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pydyn-fork',
      description='Python Library for Robot Control',
      requires=['serial'],
      packages = find_packages(),
      )
