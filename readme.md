# pydyn

`pydyn` is a small python library for Dynamixel servomotors control. It has been designed to allow Python beginners to play with the motors while allowing depths of customization for power users.

    import pydyn
    mset = pydyn.MotorSet()
    mset.position += 10

With this snipet, we moved all connected motors by 10 degrees in the clockwise direction.

The `pydyn` library was designed by the [Flowers team](https://flowers.inria.fr/), at the INRIA research institute.

## Status

The project is in usable but beta status. It is being tested on scientific setups and courses. The documentation, test coverage and sample quality is subpar at the moment but should improve quickly.

We welcome issues and pull requests.

## Documentation

The documentation is available in the ``docs/`` folder. You have samples in the ``samples/`` folders and tests in the ``tests/`` folder.