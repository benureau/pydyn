# pydyn

`pydyn` is a small python library for Dynamixel servomotors control ([protocol 1.0](http://support.robotis.com/en/product/actuator/dynamixel/dxl_communication.htm) only). It has been designed to allow Python beginners to play with the motors while allowing depths of customization for power users.

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

With this snipet, we moved all connected motors by 10 degrees in the clockwise direction.

The `pydyn` library was designed by the [Flowers team](https://flowers.inria.fr/), at the [Inria research institute](https://www.inria.fr/en/), and is distributed under the [Open Science License](https://fabien.benureau.com/openscience).

Because of a lack of access to new motors, the 2.0 version of the protocol is not implemented.

## Status

The project is on indefinite hiatus at the moment. The project is in usable but beta status. It has been tested on scientific setups and courses. The documentation, test coverage and sample quality is subpar at the moment.

## Documentation

The documentation is available in the ``docs/`` folder. You have samples in the ``samples/`` folders and tests in the ``tests/`` folder.
