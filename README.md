# Santa Fe Control Software

This repository contains the software to control the FlySorter fly manipulation platform,
specifically the hardware version Santa Fe.

The software is distributed under the GPL v2.0 license. See the LICENSE file for more details.

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
