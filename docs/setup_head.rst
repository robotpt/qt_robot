Head Computer Setup
-------------------

Turning off the default face
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. If you haven't already, SSH into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Update QT::

    cd ~/robot/packages/deb
    git pull
    sudo dpkg -i ros-kinetic-qt-robot-interface_1.1.8-0xenial_armhf.deb

.. note::

    If the :code:`git pull` step fails, the head computer might be having trouble with it its network.  You can check this with :code:`ping google.com`.  If there's nothing, there is a problem with the network.  To fix this, the best think we've found is to restart QT: :code:`sudo reboot`.

2. Edit a configuration file to turn off QT's default face:

    a. Open the configuration file::

        sudo nano /opt/ros/kinetic/share/qt_robot_interface/config/qtrobot-interface.yaml

    b. Change the line that says :code:`disable_interface: false` to :code:`disable_interface: true`

    c. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

Setting up our code
^^^^^^^^^^^^^^^^^^^

0. Secure-Shell (SSH) into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Install our project's dependencies:

    .. parsed-literal::

        git clone https://github.com/robotpt/qt-robot.git
        bash ~/qt-robot/scripts/pi_setup.bash

2. Increase the swap size, so we're able to build without running out of virtual memory:

    a. Turn off your swap memory::

        sudo /sbin/dphys-swapfile swapoff

    b. Open your swap configuration file::

        sudo nano /etc/dphys-swapfile

    c. Set `CONF_SWAPFACTOR` to 2 by changing the line that says :code:`#CONF_SWAPFACTOR=2` to :code:`CONF_SWAPFACTOR=2`, that is by deleting the :code:`#` character to uncomment the line.

    d. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

    e. Turn the swap file back on::

        sudo /sbin/dphys-swapfile swapon

3. Clone our repositories and build them:

    a. Go to the source code directory in the catkin workspace::

        cd ~/catkin_ws/src

    b. Clone our repositories:

        .. parsed-literal::

            git clone https://github.com/robotpt/cordial
            git clone https://github.com/robotpt/qt-robot

    c. Build our workspace::

        cd ~/catkin_ws
        catkin_make

    .. note::

        It takes around five minutes for this command to finish.  You can setup QT's body computer at the same time as it runs, if you like.

4. Setup our code to run when QT's head computer turns on.

    a. Copy the autostart script into the correct directory::

        roscp qt_robot_pi start_usc.sh /home/qtrobot/robot/autostart/

    b. Enable the autostart script:

        i. Open a webbrowser on QT (e.g., Firefox) and go to `http://192.168.100.1:8080/ <http://192.168.100.1:8080/>`_.

        .. figure:: images/qt_menu.png
            :align: center

            QT's configuration menu.

        ii. Click 'Autostart'.  You'll be prompted for a username and password. Enter :code:`qtrobot` for both.

        iii. Click the 'Active' checkbox next to :code:`start_usc.sh`.

        .. figure:: images/autostart_checked.png
            :align: center

            QT's autostart menu with our script, :code:`start_usc.sh`, checked.

        iv. Click 'Save' and then 'Return' twice.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

    If you'd like, you can confirm that things are running after a reboot by opening a terminal and running the following command.  You should see both :code:`/sound_listener` and :code:`/start_face_server`::

       rosnode list | grep "/\(sound_listener\|start_face_server\)"

    .. figure:: images/head_nodes_running.png
        :align: center

        What you should see if the head nodes are running correctly.
