# Robot Simulation

## *Description*

This driver station application offers a complete simulation experience, with keyboard support as well as joystick.  It also can demonstrate a built in autonomous with various test which can be observed and selected through the SmartDashboard (included with install).

## *TeleOp*

When the driver station starts up, it is much like the driver station used in FRC.  Start with TeleOperated, then click Enable button.  The simulation window will open and the robot is ready to drive.

TODO: We'll skip the joystick controls until we can use an external configuration file
To use Keyboard, ensure the simulation window has focus, this is a restraint of OSG's keyboard design
The keyboard controls for the robot:
(All keys hold down to accelerate)
w - forward  
s - backwards
a - turn left
d - turn right
arrow keys (up, down, left, right)- strafe; unlike forward and backwards, these keys slow to stop as soon as they are released
x - stop

## *Auton*

To run autonomous requires launching the SmartDashboard.  This can downloaded from FRC, and is not included as part of this installer.  After running SmartDashboard there are some default layouts located in the design folder choose SmartLayout_SwerveV3.xml.  This will layout some of the controls.  The window may need to be enlarged, and these controls can be moved around as needed.

Choose Autonomous:
By default if the AutonTest (upper right area) is not filled, it will populate with 0.0.  The test 0 has a default goal to run down the timer and the robot just sits still.  At this point use the enable button to run the test.  Disable and then change AutonTest to an integer value (SmartDashboard didn't support integer type controls, so ignore that it has a decimal value).

1. AutonTest 1- test move, this tests how many feet to move forward (in the direction relative of where it is an any point).  Running this the first time will populate a new field called TestMove with 1.0 feet.  Because the tolerance is so high, the move may end goal with little to no movement, setting this to a higher value like 10 feet should show how it works.

2. AutonTest 2- test rotate, this test is similar but for rotation (and relative to robots current orientation), it will populate TestRotate with a default of 45 degrees.  Feel free to change the amount to rotate.

3. MoveRotate  3- Set TestMove to 5.0 TestRotate to 90.  This one shows how it can make a composite of the previous goals, by default it runs 4 iterations which can also be set from TestMoveRotateIter variable.

4. TestBoxWaypoints 4- This has TestDistance_ft set to 5 by default it will make a box this feet apart in each direction from the center global coordinates and global orientation.  This means if the robot was driving around it will go back to where these points are from where it currently is.  This one will graze each point without slowing down.

5. TestSmartWaypoint 5- This allows to set x number of waypoints by use of waypoint_count.  For each count will populate wp_x wp_y and wp_speed followed by the waypoint index number (currently in decimal, may want to fix this).  By default it is a count of 1 using speed 1.0 (1.0 full speed) to 0,0 the center.  This allows user to test various patterns of waypoints.  The interesting thing to note here is that it is not necessary to use splines, it will automatically determine the best way to get the waypoint.  There is a lock orientation mode, but this currently avoids changing the orientation of the robot for best results.
