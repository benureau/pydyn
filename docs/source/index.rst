
Welcome to the `pydyn` documentation
====================================

`pydyn` is a small python library for Dynamixel servomotors control. It has been designed to allow children to play with the motors while allowing depths of customization of power-users.

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

With this snipet, we moved all connected motors by 10 degrees in the clockwise direction.

From then on, you have three different routes:

* explore the high-level API by incrementally complexifying this example.
* understand what is going on when those line run.
* get to the nitty gritty of the servomotor behavior with the low level API.

.. toctree::
   :maxdepth: 2

   highlevel.rst
   reference.rst

