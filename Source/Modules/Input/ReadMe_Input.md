
# Input Group

- Controller Properties
- Teleop Input
- AI Input

The way the design currently stands is that robot gets the time slices and each input module can get a time slice for its operations and the robot reads from the input to translate what it needs to do for output.

The biggest difference between Teleop and AI is that Teleop input is not dependent on the robot, and does not put commands to it, it is dependent on the controls and operator, where as AI is dependent on sensors and needs access to give commands to the robot.  In other words when in teleop, the updates are pulled from the operator where-as in AI the updates are pushed from AI.  Even though these work differently, AI is still an input and can still operate in the same manner of getting a time slice for its operation.

## Controller Properties

Reserved, currently there is minimal alterations and the inputs are simply hard-coded.  This may be addressed later depending on the need for it.

## Teleop Input

*Teleop Input* with its controller properties defines tele-op or the controller's implementation.  The Launcher will provide the Robot object and then input can bind its controls (specified by the properties)

### dx_Joystick_Controller

The dx_Joystick_Controller is a legacy direct x controller that is self contained with test code.  It is one of the first module examples that demonstrates how a module header file should only have the class with public methods, and private pointer to implementation.  This makes it easy to be pulled to a different application place as needed.  It can be replaced with updated SDK protocols like HID.  The interface itself is self contained.  TODO work-out how to map translations when I get to the joystick binder code (When we get to controller properties)

### OSG keyboard

For simulation, Open Scene Graph has been quite handy for both visual output of motion and keyboard input.  This works because the window itself can capture keyboard presses.  This makes it possible to quickly test the robot if you do not have access to a joystick.  I probably do 80% development using solely the keyboard for teleop.  In the legacy code, the configuration of properties was able to bind either keyboard or joystick buttons on an event driven medium and so they could be used interchangeably as needed.

### Telop Hybrid controls

There is a concept where some controls can be hybrid where they execute AI type goals, but activated via teleop.  For example there can be a button that aligns the robot to target or even auto drives to a waypoint.  In other cases it can be a quick safety precondition check before executing a desired function.  In 2014 skyshot needed the back intake down before it could fire.  This was later solved by a goal hybrid where it checked to ensure it was down, and if not it put it down and then fired automatically in one button press.  For cases like this the game is setup for teleop, but still in theory can share some goals that can also be used for autonomous.  TODO give an example that can remain in scope

## AI Input

*AI Input* is the autonomous that manages and executes goals.  If designed properly it should only need SLAM, and may need other sensors from the Robot object (which could be routed through SLAM).  For our scope of just the drive, all interaction only needs to happen with the motion control.  The localization part of SLAM is essentially the Entity, and motion control is already using this for primitive operations like rotating to a certain angle or driving to a certain way point.  The mapping part of SLAM can remain with this part of the code and may need access to sensors, and is reserved for future development.  The only thing to add for this is that we have some technology in the works which deal with CUDA, point cloud data, Open CV, and mesh, but these things take up significant time and computing resources; however, may be an exciting avenue for off-season.

### Goals

There is an existing technology of goals, which has been used for many seasons and has proven to still be useful.  I have only the main goal class itself defined in the framework.  I would like to at some point evaluate for an improved version which can make use of modern c++ techniques, but for now in the short term will roll out an example, mostly for the sake of showing how to interact with motion control to be able to do everything needed with minimal coupling of classes.

