======
README
======

To setup QT, we'll use a Docker image to setup the main body computer (an Intel NUC) and we'll have to manually setup the head computer (a Raspberry Pi).  Fortunately, setting up the head computer isn't many steps.

**********************
QT Head computer setup
**********************


The first thing is to increase the swap size on QT's head computer.  Otherwise, the code that we're building will be too large for the Raspberry Pi's RAM memory.

0. SSH into QT's head computer from the body computer::

    ssh qtrobot@192.168.100.1

1. Turn off your swap memory::

    sudo /sbin/dphys-swapfile swapoff

2. Open your swap configuration file::

    sudo vi /etc/dphys-swapfile

3. Set `CONF_SWAPFACTOR` to 2::

    CONF_SWAPFACTOR=2

4. Turn the swap file back on::

    sudo /sbin/dphys-swapfile swapon

Then make your workspace::

    cd ~/catkin_ws
    catkin_make

Finally, setup autostart

1. Copy the autostart files to the autostart directory (note that symbolic links didn't work for Audrow)::

    cp ~/catkin_ws/src/cordial-public/autostart_scripts/start_cordial_face.sh ~/robot/autostart/

2. Configure QT to start the script from a webbrowser on QT's chest computer (the NUC), such as Firefox.  In the webbrowser, go to `http://192.168.100.1:8080/` and you should see a menu.  Click 'Autostart'. You will be prompted to enter a username and password. Enter `qtrobot` for both.  Then click the 'Active' checkbox for `start_cordail_face.sh`  Hit 'Save' and then 'Return' (twice).  Then reboot QT by clicking 'Reboot'
