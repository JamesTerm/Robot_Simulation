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

TODO:
Add inverse kinematics

---

## SLAM

*Simulateous Localization and Mapping*
I'll expand more on this here.  The most important piece is the entity.  This is used for prediction as well as a trapezoidal motion profile.  AI can manipulate a 2D entity and it can then be used to drive to waypoints.  In addition the predicting of the motion profile is the essential ingredient for closed loop programming.  Closed loop feedback from sensors like encoders and vision can be used to calibrate the prediction to where it physically resides at that moment.  1D entities can be used in the same regard for manipulators, we can include them in this group even though they do not necessarily contribute to localization.  

For now we needn't worry about mapping especially because it is hard to find a common ground between seasons, that said... we may wish to consider providing an automated way to plot waypoints onto a virtual field, which is easy because they give us the placement of field elements.  It's even possible to add path algorithms and run cycles from them... for that it could ideal for autonomous and if its possible to recognize robots in the way... we can consider object avoidance algorithms (Like what has been done with rim space / the fringe).

TODO:
Add Entity object

---

## Robot properties

This is mostly reserved.  I may add this back with LUA, but it will much later and evaluated if wanted.  This is more of an automated task, so I may leave this up to the students to work out.  For early iterations I'll just hard code properties to keep it simple and all c++.  One other thing I hope to achieve is to avoid modules from being dependent on it.

---
