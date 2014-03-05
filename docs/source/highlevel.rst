High-level API
==============

.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

In the high-level API, there are only motors and group of motor called `motor sets`. Anything done on a motor set is done on all the motors of the motor set. In the preceding example, we created a motorset that detected and loaded all connected motors, and then increased all the motor positions by 10 degrees.

Given a motor set, we can access the motors directly by ``mset.motors``. We can rewrite the previous script using that:


.. code:: python

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

