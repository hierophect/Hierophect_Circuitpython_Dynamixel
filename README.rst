Introduction
============

.. image:: https://readthedocs.org/projects/circuitpython-dynamixel/badge/?version=latest
    :target: https://circuitpython-dynamixel.readthedocs.io/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/hierophect/CircuitPython_dynamixel/workflows/Build%20CI/badge.svg
    :target: https://github.com/hierophect/CircuitPython_dynamixel/actions
    :alt: Build Status

Circuitpython driver library for the Dynamixel series of servo motors from Robotis


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Getting Started
===============

For a full description of the Dynamixel's capabilities, please refer to the official materials:

* `Datasheet <https://www.trossenrobotics.com/images/productdownloads/AX-12(English).pdf>`_
* `Web Manual <https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/>`_

Dynamixel servos use a single-wire, half-duplex UART mode of communication. Since most Adafruit boards are full duplex two-wire UART, a simple circuit must be created to use the servos. This can use buffer logic, as per the datasheet, or use a more crude version with a diode and resistor:

.. image:: images/diode.png
   :width: 600

By pulling down the Direction Control pin while TX is in use, the RX pin will not recieve problematic data off of the TX pin. A 10k resistor worked in breadboard testing, but your results may vary.

Note that 7V or higher on the motor power line is required for the internal controller to turn on - motors will not even return temperature data at lower voltages. The communication line, however, is TTL 5V logic. Double check your connections to ensure the motor power line is not applied to the communication pin.

Usage Example
=============

The following sketch will print the temperature of a motor addressed as 0x00 (the default) and move it between the minimum, halfway, and maximum positions. A communication control pin of D12 is used but can be swapped out with any IO pin.

::

    import board
    import busio
    import digitalio
    import time
    import circuitpython_dynamixel

    control = digitalio.DigitalInOut(board.D12)
    control.direction = digitalio.Direction.OUTPUT
    control.value = True
    uart = busio.UART(board.TX, board.RX, baudrate=1000000)
    ax12 = circuitpython_dynamixel.Dynamixel(uart,control)
    ax12.set_speed(0xfe,0x100)
    while True:
        print(ax12.get_temp(0x00))
        ax12.set_position(0x00,0)
        time.sleep(1)
        ax12.set_position(0x00,512)
        time.sleep(1)
        ax12.set_position(0x00,1023)
        time.sleep(1)

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/hierophect/CircuitPython_dynamixel/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

Readthedocs can be found `here <https://circuitpython-dynamixel.readthedocs.io/en/latest/index.html>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
