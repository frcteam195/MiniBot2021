# MiniBot2021

Autonomous Control

Modified program to include trajectories for both Course 1 and Course 2.  
Added additional trajectory call to assign which trajectory will be called during autonomous.  
This change allowed us the flexibility to retain both course and to run different autonomous courses by changing one line of code.

Teleop Control

Modified program to work with a GameCube controller
Separated acceleration from steering control
Normalized the trigger (acceleration) control while removing any deadband that existed within the trigger
Normalized the steering control to increase steering to full range for the Romi Robot (controller previously had a range of -.75 to .75)
