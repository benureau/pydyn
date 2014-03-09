.. _high-level-api:

High-level API
==============

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

In the high-level API, there are only motors and group of motor called `motor sets`. Anything done on a motor set is done on all the motors of the motor set. In the preceding example, we created a motorset that detected and loaded all connected motors, and then increased all the motor positions by 10 degrees.


Get a Working Connection
------------------------

If the previous example worked for you out of the box, you can skip this version. If it does not, read on.

There are many diverse ways to connect dynamixel servos to motors.

Accessing ``Motor`` instances
-----------------------------

Given a motor set, we can access the motors directly by ``mset.motors``. We can rewrite the previous script using that:

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    for motor in mset.motors:
        motor.position += 10

To read the status of a motor and send orders to it, we use a simple attribute-based syntax :

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()

    for motor in mset.motors:
        motor.position = 0

    time.sleep(1.0) # we wait for motor to get in position
    for motor in mset.motors:
        print(motor.position)
