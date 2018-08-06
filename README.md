# MAPLE v2 Control Software

This repository contains the software to control the FlySorter fly manipulation platform,
specifically the hardware version MAPLE version 2.

The software is distributed under the GPL v2.0 license. See the LICENSE file for more details.

Installation (Windows only, for now)
------------

1. Install Python 2.7

https://www.python.org/download/releases/2.7/

Choose the appropriate installer package (x86 or x86-64) for your system. Preferable to install in default location (C:\python27)

Add c:\python27 and c:\python27\scripts to your path.

2. Install pip, numpy, OpenCV, pyserial & "future"

Download https://bootstrap.pypa.io/get-pip.py and then run it.

https://pypi.python.org/pypi/numpy -- you're looking for the installer like numpy-1.13.3-2-cp27-none-win32.whl or numpy-1.13.3-cp27-none-win_amd64.whl
From a command prompt, run pip install c:\path\to\numpy\file\numpy-1.13.3-2-cp27-none-win32.whl (change path and filename as appropriate)
Double check by running python and typing: import numpy

https://pypi.python.org/pypi/opencv-python -- same as numpy...

https://pypi.python.org/pypi/pyserial -- ditto

From a command prompt, type: pip install future
This module makes some code written for Python 3.x compatible with Python 2.7, specifically py-ic-imaging-control

3. Install driver & SKD for The Imaging Source cameras (optionally, viewer)

https://www.theimagingsource.com/support/downloads-for-windows/
* Device Driver for USB Cameras
* IC Imaging Control C Library
* optionally IC Capture - image acquisition

You'll have to add the DLL directory to your path at this point, which should be:

C:\Users\username\Documents\The Imaging Source Europe GmbH\TIS Grabber DLL\bin\x64
  OR
C:\Users\username\Documents\The Imaging Source Europe GmbH\TIS Grabber DLL\bin\win32

depending on which architecture you've got.

4. Install py-ic-imaging-control

https://github.com/morefigs/py-ic-imaging-control - download archive as zip, unpack, then, in command prompt window, run: python setup.py install

5. Download this github repository (https://github.com/FlySorterLLC/MAPLEControlSoftware).

6. Copy Examples\ExampleRobot.cfg to the main directory, rename (most scripts assume the name "MAPLE.cfg"), and edit as necessary.

7. Use the GUI to drive the robot around

If necessary, edit MAPLE-GUI.py to refer to the correct config file, then run it. If everything installed correctly, you should
get a window that shows the camera view and the current position / state of the robot. Press '?' for a list of keyboard commands.

8. Create or edit a workspace file that defines which modules are being used and where they are located

Some example workspaces are included in the Examples folder. To use one of them, move it to the main directory and rename
to something appropriate. For example, if you created a layout that included the Fly Dispenser and a FlyPlate 96 well plate,
you might call that PlateLoadWorkspace.py (and again, it should be in the main directory so you script can find it.)

9. Create or edit a script to run to actually do something with the robot!

Again, there are some samples in the Examples folder. They will need to be edited to account for:
* your configuration file name
* the workspace they depend on
* the actual actions they take

10. To use the remote operating mode (Listen Mode):


System Architecture
-------------------

The goal of this platform is to automate various fly handling tasks. To accomplish this, we have
designed a robot with a multi-purpose end effector, which has:

1. Three independent Z axes:
	a) One to pick up and deposit flies
	b) One to focus a downward-pointing camera
	c) One to pick up and deposit other small parts (such as lids)

2. A high-resolution camera (http://www.theimagingsource.com/en_US/products/oem-cameras/usb-cmos-color/dfm72buc02ml/)
   This camera is affixed to the middle Z axis, which serves as its focusing mechanism.

3. An LED ring to illuminate the space underneath the camera

4. The two manipulators (fly and small part) can pick up and put down objects, and are independently controlled.
   Four solenoid valves control the flow of compressed air and vacuum to each manipulator.

Control of the robot is by way of a PC, running Windows.

So there are five different motors to control in the system (X, Y, and 3x Z axes), and four devices to turn on/off
(the four solenoid valves).

We use a 5-axis SmoothieBoard (http://smoothieware.org/smoothieboards) to control the 5 motors, and also the
4 solenoid valves (by way of load switches). The Smoothie connects to the PC by USB, and appears as a COM port
to the operating system. It uses G-Code to communicate with the PC (http://reprap.org/wiki/G-code).

The Python code in this repository translates API calls into the appropriate G-code commands and sends them to
the Smoothie (and waits for replies, as appropriate).

Contributors
-------------------

This code was authored by: Dave Zucker, Tom Alisch, Yong-Li Dich, William Long, and Ben De Bivort
