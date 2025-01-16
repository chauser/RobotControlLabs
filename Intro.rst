Introduction

This set of lab notes and exercises was created to help FRC teams develop their 
skills and intuitions concerning the control classes in WPIlib. 

It contains example controllers for elevators, flywheels, and a SimplePosition drivebase
(moves only in a straight line).

In addition to developing understanding of WPILib's controller classes, 
working through the examples also provides practice with using the WPILib
simulation GUI to graph system behaviors and also to manipulate controller
parameters without having to rebuild code every time.

The suggested sequence of system types is: Flywheel, SimplePosition, and Elevator.
I suggest skipping the LQR controller on the first path through each system and
then coming back to those after doing all the system types.

Logistics

Clone the ControlWorkshop git repository to your machine. (URL)
  git clone http:...
  git clone git:...

Attach an XBox-like game controller to your machine.

In VSCode, File|Open Folder the Flywheel folder. (Do not open the RobotControlLabs 
folder.) The Flywheel folder (and the Elevator and SimplePosition folders as well) 
is a complete WPILib project that can be simulated using the WPILib: Simulate Robot Code
choice from the W menu. 

In the Sim GUI window drag the XBox controller from the "System Joysticks" window
to slot [0] of the "Joysticks" window. (It is not clear to me when this is 
remembered and when it has to be done each time the simulation is )


Documentation needed: what controller buttons do what in the various devices.

Lessons needed: what experiments should be run with the various mechanisms? How do 
you set up graphs to observe the behavior of the mechanisms? What should you observe
when running the different controllers? Etc., etc., etc.
