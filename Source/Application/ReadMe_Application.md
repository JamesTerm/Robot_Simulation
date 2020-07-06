# Robot Group

- Robot
- Robot Properties
- SLAM

The *Robot* will pull input and push output.  This is where the drive kinematics resides, it will use properties and if done correct should be able to survive multiple seasons.  The most common component of the robot is the rotary system, and so this is the front end for what is to be the output.

*SLAM* (Simultaneous localization and mapping) is the object that keeps track of it's position on the field.  It will bind to robot to get sensor information (localization) to determine where it really is.  The mapping aspect can be addressed per game as needed.

---

## DriveKinematics

This contains the equations needed to translate the linear and angular velocities from either AI or a teleop controller to the desired velocities needed for the drive systems that use them.  Currently there is the skid steering (i.e. tank drive) and swerve drive setup for a 4 wheel configuration.

I have also included the inverse kinematics here even though these pertain more for SLAM and entities.  For tank drive one can use the inverse kinematics to achieve tank steering control (verses arcade steering control).  It's even possible to mix and match a tank steering feel for swerve.

The swerve equations were given to me from Ether on CD back around December 2011, he also provided the inverse equations.  I've kept the code variable the names the same as they were presented to me.

---

