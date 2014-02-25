#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='pydyn',
      version='0.9.1',
      author=('Fabien Benureau', 'Pierre Rouanet', 'Matthieu Lapeyre',
              'Olivier Mangin', 'Haylee Fogg'
             ),
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pydyn-fork',
      description='Python Library for Robot Control',
      requires=['serial', 'pyftdi'],
      packages = find_packages(),
     )
