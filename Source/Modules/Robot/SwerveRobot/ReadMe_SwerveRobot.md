# Swerve Robot

- Motion Control
- Kinematics
- Swerve Management
- Odometry Manager
- Simulated Odometry
- Rotary System
- Inverse Kinematics

Swerve Robot connects various modules together to form a super module which essentially is the heart of the Robot module.  This document covers more detail on these sub-modules in more detail to understand their roles as well as some insight on how to diagnose and maintain.

Here is how these work together:

```text
                                   Inverse Kinematics
                                          v
Motion                   Swerve        Odometry   Simulated
Control <- Kinematics <- Management -> Manager -> Odometry
                                ^                  /
                                 \Rotary System-<-/
```

And how Swerve Robot interacts with the external modules

```text

Tele Op Input <--------Motion Control (Swerve Robot) <- Inverse
AI Input-------<----/                                   Kinematics <--- Entity
     \--------------------------> for SLAM ---------------------------->--/
```

## Motion Control

Motion Control is an optional process that can motion profile the acceleration to avoid potential undesired effects such as skidding tipping over and things of this nature.  This is widely accepting by many teams for autonomous work because it can move the robot efficiently and be nice to the odometry for better reading.  This is not as widely accepted for teleop, and usually the teams that support this kind of control like Team 33 Killer Bees, also like to drive with encoders as they explain the pushing match situation, how the response time of a skid in the encoder is faster that a human operator's response which is around 200ms vs. 10ms.  In addition this can be used to have an ideal turn rate that is intuitive to the driver as this was something which was successful in our 2014 robot sky shot.

Motion control in our code example comes in two flavors simple and with physics.  The simple has a simple use case of pure trapezoidal motion profiling, which can handle the same behavior of physics but only for one direction at a time, so if you change directions it cannot manage that, but it does make it easier to debug code especially if you want to debug other aspects of the code... for this, both the Swerve Robot, and the physics have macros that can bypass to use the simple case.  This will help for debug of swerve management or kinematics, but keeping its part transparent.  It is even possible to bypass motion control all together, but currently I do not have a bypass mode for the motion control instead I manually construct a bypass test by putting together these sub-modules for that case.

## Kinimatics

More has been explained in the root read me documents, for this context we are committed to swerve kinematics, so if you want bypass, see test 5 in the assembly application.  This should be cut and dry what it does, except to add that the wheel angles always point in the forward direction that facilitates the direction of the angular velocity, and does not factor in any previous moments of time, nor should it.  These are raw equations that should never need to change.

## Swerve Managment

Swerve Managment is the last piece before applying the final control changes to the robot, which work from intended changes from the kinematics and the present state of the robot from the odometry.  

One of the main issues the management will address is to avoid turning the wheels 180 or greater when they can instead go in reverse on the opposite angle. For example let's suppose the driver was moving forward (heading 0) then wanted to go to (heading 190) from top view this is down and to the left.  Instead of moving the wheel from 0 to 190 going forward, it would move wheel from 0 to 10 going backward.

TODO:  Address strafing motion as this puts stress on the limits of the current algorithm

The other issue this addresses is to throttle down the speed when the wheel angles have not yet made it to their setpoint.  This can allow some motion prior to reaching and also determine a tolerance threshold of when it is considered to allow full speed, usually teams allow up to 2-3 degrees of error for this tolerance.

## Odometry Manager

The conceptual idea here is that the robot has various sensors to help with localization, for our typical case this would be encoders, gyro, and vision *(and perhaps magnetometer and accelerometer)*.  The data from each of these sensors comes here to be evaluated.  The inverse kinematics can work with these readings for the final best probability for where the robot is locally.

## Simulated Odometry

More to be said here, this is the heart of the simulation itself, and the odometry manager can read either this or the actual sensors.  I have a somewhat legacy attempt model of this that can be used in the short term, but for the long term I wish to approach all aspects of physics and some known aspects of motor curves as well to have an accurate model for the encoders and potentiomters used in the swerve drive.  It may even be possible to simulate vision as we attempted this in 2014.

## Rotary System

Here is where we convert from velocity to voltage, this comes in two flavors, velocity controlled and position controlled and in both classes can either be open or closed loop.  For closed loop this uses PID input and there are some other interesting variables to mitigate error.  In addition this also improves latency by taking into account the current acceleration and velocity and making adjustments during the initial start or decline to compensate for extra voltage needed for change.  The position control flavor also included bursting techniques or dealing with gravity by keeping a constant opposing force voltage when needed.

Similar to motion control, this does add a new layer of complexity so there is a Bypass macro which can be used to factor out its laggy nature of reality to make it easier to debug other aspects of the robot, but this of course is only for within the simulation realm.  Luckily the legacy version of this code has been tested enough to be trusted to be reliable, the bypass version should be easy enough to review to understand a simpler overall way to handle the conversion.

## Inverse Kinematics

This piece reads from the odometry manager to simply return the velocity of position and rotation.  Entity 2D can then keep track of this for its localization.  Since I haven't worked with vision yet in this code it is possible that it link to Entity 2D instead of the odometry manager, but in either case has impact on how this works like with kinematics it is simple and should never need to change.  It could be thought as part of the odometry manager, but then the idea is to not tightly couple this object with it so it can be used elsewhere.  I say all this now in case vision integration needs to be re-evaluated on how it works with the design.