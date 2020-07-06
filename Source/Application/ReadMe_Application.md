# Application Group

- Driver Station
- Robot Assembly

This should be the only place that combines modules, as each module should be self contained, and not dependent on any other *object.  Each application can then pull files to make a full executable.  To keep life simple I'm avoid dlls as the whole project should be small, and plans for the output simulation can be interfaced via tcpip.

*This should be modules for sure, but we want to allow for framework and libraries moving forward, however; for now I'm going to see how far we can go without even those (at least with what is written in the public header files)

---

## Driver Station

This is a work in progress... the final simulation main project will be this.  I may reserve the test button for keyboard control testing.

TODO:
keyboard driving with entities

---

## Robot Assembly

This is a work in progress, where we have various tests that can test various object interactions.  This is open for any kind of test which can be command drive, and open for unit testing.

TODO I have some tests that will be coming soon:
Swerve Kinematics Test 2
Tank Steering Test 3
SLAM entity
Robot properties for kinematics and entity

---
