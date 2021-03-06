# Application Group

- The assembly testers
- Driver Station
- Robot Assembly

## The assembly testers

This should be the only place that combines modules, as each module should be self contained, and not dependent on any other module.  For visual studio properties, each application can then pull files to make a full executable.  To keep life simple I'm avoiding dlls as the whole project should be small.

### The launcher tests overview

The way this works and has evolved, is a generic launching interface that is similar to that how the driver station operates, both the driver station and robot assembly can make use of this.  This is also generic enough to be compatible with whatever main() setup WPI evolves to use as well.  I'll cover each of the testers here:

- TeleOp_V1 - No Swerve Robot super module yet, and no OSG view
- TeleOp_V2 - OSG View, this is good to see how the sub modules evolved
- TeleOp_V3 - the sub modules are embedded into the Swerve Robot super module, keeps this layer easier to read and manage
- Test05_Test_Bypass_with_OSG - Shows a complete bypass setup and how the UI is not dependent on the swerve velocities
- TeleAuton_V1 - This is V3 teleop with added ability for autonomous and testing
- TeleAuton_V2 - This adds properties integration

### description, explanation of the layout

As hinted from the main Readme, there is a lot of code redundancy here, but for good reason, I feel it best to review and understand version 1 first, then move to version 2, and so on.  If you can see the progression and how each incremental add works, it makes it easier to focus on the newer stuff as you graduate to the next level, plus if something in the higher progressions break we can test the simpler models to diagnose if something new has broken the build.  Within Robot Assembly itself contains even more simpler tests, but these needn't be covered here except to say that it is possible to run simpler tests if any of the teleop tests are broken.

### About TeleAuton_V1

Starting with TeleAuton_V1 on the viewer.SetUpdateCallback() I've added a line to force the delta time to be 0.01 or 10ms for each timeslice call.  This is commented out, but if you set breakpoints and are stepping through code you will want to enable this synthetic timing.  It is even possible to set it at a smaller time and everything can run in slow motion when doing this.  In the assembly there is a run command to run various tests, much similar to the test for the driver station.

### About TeleAuton_V2

This starts properties integration, for the robot modules section of the code, this remains identical, but will change drastically for both input and output.  I will expand upon this a bit later.

## Driver Station

This is a work in progress... the final simulation main project will be this.  I may reserve the test button for keyboard control testing.
Driver station interfaces with a robot tester, which can be swapped out for different kind of robot assembly tests.
Keeping this generic like this will make it easy to test various kinds of assemblies with ease, and adds the ability for keyboard input without needing
some 3rd party solution

TODO:
Use AutonV1 for auton mode
See if it makes sense to route the test to the tester class from the assembly.  I can then specify test argument on smart dashboard and make use of the enable button
to execute them
keyboard driving with entities

## Robot Assembly

This is a work in progress, where we have various tests that can test various object interactions.  This is open for any kind of test which can be command drive, and open for unit testing.

As the modules are being developed there is a tester which can test integration between modules.  I've also added TeleOpV1 which is everything needed to test teleop

TODO I have some tests that will be coming soon:
AutonV2 properties usage with assigning controls


---
