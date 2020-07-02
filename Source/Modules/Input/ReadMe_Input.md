
# Input Group

- AI Input
- Input
- Controller Properties

The *Input* with its controller properties defines tele-op or the controller's implementation.  The Launcher will provide the Robot object and then input can bind its controls (specified by the properties)

*AI Input* is the autonomous that manages and executes goals.  If designed properly it should only need SLAM, and may need other sensors from the Robot object (which could be routed through SLAM)

The way the design currently stands is that robot gets the time slices and each input module can get a time slice for its operations and the robot reads from the input to translate what it needs to do for output.

---

*dx_Joystick_Controller* - This is a legacy direct x controller that is self contained with test code.  It is one of the first module examples that demonstrates how a module header file should only have the class with public methods, and private pointer to implementation.  This makes it easy to be pulled to a different application place as needed.  It can be replaced with updated SDK protocols like HID.  The interface itself is self contained.  TODO work-out how to map translations when I get to the joystick binder code.

TODO expand on the projects here
