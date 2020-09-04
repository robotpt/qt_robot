Body
----

Getting your Amazon Web Service credentials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For QT to speak, we use Amazon Polly, which requires an Amazon Web Services account. At our current usage, using `Amazon Polly is free up to a certain level <https://aws.amazon.com/polly/pricing/>`_), but you will need a credit card to create an account.

1. `Create an Amazon Web Services account <https://portal.aws.amazon.com/billing/signup#/start>`_.
2. Once you sign in, in the top right of the page, click your account name (mine says "Audrow"), then in the drop-down menu click "My Security Credentials," then click "Create New Access Key."
3. Record your access key and keep it somewhere safe.  You can do this by downloading this or just viewing it and copy-pasting it to somewhere for later reference.

.. note::

    It is best practice to create separate accounts with less access than your root account and use those access keys, see `Amazon's security best practices <https://aws.amazon.com/blogs/security/getting-started-follow-security-best-practices-as-you-configure-your-aws-resources/>`_.


Setting up our interaction in a Docker container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. Change your system timezone to be in your current timezone.  To do this, you can click the time in the upper-right of the desktop on QT and then click 'Time & Date settings...'

1. Open a terminal and clone this repository onto QT's body computer::

    git clone https://github.com/robotpt/qt-robot.git

2. Run a script to allow for updates::

    sudo bash ~/qt-robot/scripts/nuc_setup.bash

.. warning::

    If this step fails, try the following commands before rerunning::

        sudo apt install --reinstall python3-six
        sudo apt install --reinstall python3-chardet

.. note::

    This step takes five minutes or so.

3. Setup Cordial by following `this <https://cordial.readthedocs.io/en/latest//>`_ guide.

Setting up remote access to QT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Get `Dataplicity <https://www.dataplicity.com/devices/>`_ login credentials from Audrow and sign on.  Go to the devices tab and then click "+ Add New Device".  Copy or enter this command into a terminal on QT's body PC and enter QT's password 'qtrobot'.  After that runs, remote access should be setup.  You can confirm this by clicking the added device and confirming that you can explore the file system (e.g., :code:`ls /home/qtrobot`).

.. note::

    When you login on Dataplicity, you will be signed in with the user :code:`dataplicity`. You should then switch to the user :code:`qtrobot` with :code:`su qtrobot` and then enter the password, 'qtrobot'.
