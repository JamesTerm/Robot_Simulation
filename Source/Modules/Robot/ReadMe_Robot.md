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

## SLAM *Simulateous Localization and Mapping*

I'll expand more on this here.  The most important piece is the motion_control.  This is used for prediction as well as a trapezoidal motion profile.  AI can manipulate a 2D entity and it can then be used to drive to waypoints.  In addition the predicting of the motion profile is the essential ingredient for closed loop programming.  Closed loop feedback from sensors like encoders and vision can be used to calibrate the prediction to where it physically resides at that moment.  Motion 1D entities can be used in the same regard for manipulators, we can include them in this group even though they do not necessarily contribute to localization.  

For now we needn't worry about mapping especially because it is hard to find a common ground between seasons, that said... we may wish to consider providing an automated way to plot waypoints onto a virtual field, which is easy because they give us the placement of field elements.  It's even possible to add path algorithms and run cycles from them... for that it could ideal for autonomous and if its possible to recognize robots in the way... we can consider object avoidance algorithms (Like what has been done with rim space / the fringe).

So there is a motion control, and entity.  For legacy motion control was ship, and inherited from entity.  Not this time, motion control will be very simple where velocity comes in and a different velocity goes out.  Entity can then take whatever the final velocity is and update the position.  Doing this allows to pick if we want this or not, and which one we want to use.  So then the rotary system can be fit in between like so:

Kinematics->motion control (optional)->rotary system->entity -> position sent to output (for simulation), or input for way-point AI (or both)

The rotary system will manage voltage, and also be open or closed, the actual velocity sent to entity, can pass thru or be a direct feedback from the encoder.  Doing this will be a lot cleaner in knowing exactly how the position will be processed.  Keep in mind each piece is independent, to keep them loosely coupled and swap-able.

*Note: Motion control is not a part of localization (it is a part of the robot module).  All odometry input like encoders will indeed be a part of the localization module, so the rotary system, and entity, are here.*

So now it will be easiest to keep driven methods (like driving to a way point) as part of motion controls, and then we'll need to hook its method to getting the position, like written above we can write the hook in the assembly class (the application that owns all the modules) it can then access the position and provide it as a hook.  Using a hook technique makes it possible to keep no ties between motion control and entity.  (This is important)

### A note about Motion control using physics-  

One thing physics solves is to work out for Entity 2D position's velocity when the direction changes.  Say for example you go full speed and then do a sharp 90 degree turn.  While the kinematics ignore physics, it also makes it possible to wreck havoc on the robot's drive or may cause robot to tip over.  Peter (one of SimBots MEs) would argue, that the robot should be mechanically designed to take on the worst-case stresses, but this would only address the internal gearing, not issues like skids (skids cause team to lose in a pushing match) tipping over, or damage to swerve swivel mounts.  Using motion control with physics takes into consideration the mass's current momentum and the direction at any given moment, since it deals fundamentally with force and torque at the mass entity itself, it can ensure no excessive force will be applied.  One more point to add to this is that a tank drive is more forgiving for excessive forces, where-as swerve drive is not.  I can say that the power house teams like 148 do not do these things and some reasoning may be that they are advanced concepts that students may not be prepared to handle, they would argue that they delegate the responsibility to the driver to practice, and while this can work I've witnessed the amount of stress this puts on the driver and I've seen past year's matches where the driver will inherently drive slower cycles to avoid potential damage.  With motion control it should be easy to drive the robot, and in 2014 was a proof of concept where this pays off for faster cycles.

### A word about the rotary system

This is really a part of the robot and part localization (the part that provides encoder odometry).  In the legacy code it inherited from Ship 1D, and Ship 1D inherited from Entity 1D.  This time around I want to avoid inheritance and any object dependencies... so like with motion control... it will be able to run everything on its own.  Unlike Entity 2D, Entity 1D can handle change of direction properly, so for now I am going to leave it as-is.  I'll look into making a simple version first that integrates well with Entity 1D. Then afterwards I'll look into an advanced version that can use some voltage managing techniques needed for various tasks like reducing latency, or dealing with gravity in one direction.  Given this... I'll shoot for a rotary system (robot) and Entity 1D (localization).  If Entity 1D ever needs the motion control separated like it is for 2D we can always change later...  Rotary will have and output method for voltage to write, and an input method to receive encoder feedback these are optional callbacks both called on its time slice.  Doing this will make it easy to hook to a simulation.

## Robot properties

This is mostly reserved.  I may add this back with LUA, but it will much later and evaluated if wanted.  This is more of an automated task, so I may leave this up to the students to work out.  For early iterations I'll just hard code properties to keep it simple and all c++.  One other thing I hope to achieve is to avoid modules from being dependent on it.

---
