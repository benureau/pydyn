
Welcome to the `pydyn` documentation
====================================

`pydyn` is a small python library for Dynamixel servomotors control. It has been designed to allow children to play with the motors while allowing depths of customization of power-users.

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

With this snipet, we moved all connected motors by 10 degrees in the clockwise direction.

From then on, you have three different routes:

* explore the :ref:`high-level-api` by incrementally complexifying this example. This recommended as a first contact with `pydyn`.
* understand the magic behind the previous lines by reading :ref:`understand-controller`.
* :ref:`low-level` to dissect the behavior of the servomotors.

.. toctree::
   :maxdepth: 2

   tuts/highlevel.rst
   tuts/understand_controller.rst
   tuts/lowlevel.rst
   api/reference.rst

