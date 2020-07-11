# Application Group

- Driver Station
- Robot Assembly

This should be the only place that combines modules, as each module should be self contained, and not dependent on any other *object.  Each application can then pull files to make a full executable.  To keep life simple I'm avoid dlls as the whole project should be small, and plans for the output simulation can be interfaced via tcpip.

*This should be modules for sure, but we want to allow for framework and libraries moving forward, however; for now I'm going to see how far we can go without even those (at least with what is written in the public header files)

---

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

---

## Robot Assembly

This is a work in progress, where we have various tests that can test various object interactions.  This is open for any kind of test which can be command drive, and open for unit testing.

As the modules are being developed there is a tester which can test integration between modules.  I've also added TeleOpV1 which is everything needed to test teleop

TODO I have some tests that will be coming soon:
AutonV1 example
Robot properties for kinematics and entity

---
