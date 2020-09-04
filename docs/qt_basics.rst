QT Basics
=========

Computers and sensors
---------------------

Head computer
^^^^^^^^^^^^^

QT's head computer is a Raspberry Pi. Things to note:

* Has access to speakers and a microphone (which is right next to a fan)
* Has access to the screen on QT's head
* Has gesture files
* Hosts the ROS core
* Turns off and on the body computer
* Internet isn't very reliably

Body computer
^^^^^^^^^^^^^

QT's body computer is an Intel NUC. Things to note:

* This is a powerful computer
* Internet is reliable

Turning QT on and off
---------------------

To turn on QT, just plug in power to QT and it will boot.

To turn off QT, there are two options:

    1. Press the button on the backside of QT, near its feet.

    2. Login to the head computer (see below) and do :code:`sudo shutdown`.

If you do :code:`sudo shutdown` on the body computer, you only turn off the body computer---the head computer is still on.

.. note::

    If you unplug QT to restart QT, it may mess up the boot timing of the two computers.  Probably one of them takes longer because it boots in recovery mode.  This screws up how the head and body computer network.  You will not be able to connect to the head computer from the body computer.  If this occurs, simple restart QT by pushing the button on its backside.

Accessing QT's body computer
----------------------------

When you connect a monitor to QT and turn QT on, you will start on QT's body computer.

Accessing QT's head computer
----------------------------

To setup the head, you must Secure-SHell into it (SSH) from QT's body computer.  To do this

0. Turn on QT.

1. Open a terminal.

2. Type the following and hit return::

    ssh qtrobot@192.168.100.1

