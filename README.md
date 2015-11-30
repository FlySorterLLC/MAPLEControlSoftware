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

4. The two manipulators (fly and small part) operate by way of a Venturi tube, and are independently controlled.
   Two solenoid valves control the flow of compressed air to each manipulator, and two solenoid actuators toggle
   between vacuum and positive air pressure.

5. The two manipulators can each rotate around the Z axis (by way of small, hollow shaft stepper motors).

Control of the robot is by way of a PC, running Windows.

So there are seven different motors to control in the system (X, Y, 3x Z axes, plus the two rotating axes for the manipulators).

There are two different motor control schemes in use. The X, Y and two rotating axes are all stepper motors,
and are controlled by a single printrboard (http://reprap.org/wiki/Printrboard). The printrboard connects to
the PC by USB, and appears as a COM port to the operating system. The printrboard uses G-Code to communicate
with the PC (http://reprap.org/wiki/G-code).

The three Z axes are DC motors with encoders, and are controlled by Synaptron boards
(http://www.solutions-cubed.com/products-page/motor-controller/synaptron-micro-1/
with documentation here: http://solutions-cubed.com/app-notes-downloads/#SYNAPU), one per axis. The three Synaptron boards
are all mounted to a carrier circuit board, which also includes a USB-to-serial converter (http://www.solutions-cubed.com/products-page/protocol-converter-interface/bm010/).
Thus, that carrier board also presents itself to the PC as a COM port. The communications protocol is different
(i.e. not G-Code). It is specific to the Synaptron boards, and is documented online (see above link).

