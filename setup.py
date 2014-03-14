#!/usr/bin/env python

from distutils.core import setup

setup(name='pydyn',
      version='0.9.1',
      author=', '.join(['Fabien Benureau', 'Pierre Rouanet', 'Matthieu Lapeyre',
              'Olivier Mangin', 'Haylee Fogg']
             ),
      author_email='fabien.benureau+inria@gmail.com',
      url='https://bitbucket.org/humm/pydyn-fork',
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
