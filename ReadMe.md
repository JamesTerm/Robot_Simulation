# Robot Simulation

## *Description*

This simulation takes advantage of modern c++ to have modular code at each stage of the simulation.  Doing this makes it possible swap out legacy code with never code with ease, and then keeps each module simple to and allows for easier collaboration.

## *Introduction*

To help avoid confusion I should address a couple of things to be aware of:

- There are multiple projects that are optional to build but one main project
- There is an optional way to avoid needing to build OpenSceneGraph

### Multiple projects

Usually we only have one project to build, and it has everything in it.  This is partially true, as the "Launcher" currently is the Driver Station.  So if you just want to build and run the main program, this is all you need, and may be a good starting point to see what this code has to offer.  The Robot Assembly project is essentially a command driven version of the driver station, but also has various tests within it to test multiple objects to interact.  This is a good project to work with when trying to understand and test linking objects.

The other projects are in each module so that each module can be tested by itself and those can have special unit test cases to ensure the module can handle strategic stresses as deemed fit.  The also show how to use them (without the clutter of other objects as it is in the assembly project).  The physical file organization itself is just how visual studio works by default.

You may ask couldn't all of these modules be projects in one solution where each has its own test in there.  If each module had multiple files this would be a great idea.  But Instead we are using a one-file one-header approach... so it is not necessary to make them like this, instead we pull the files themselves into one solution.  In doing so, makes it possible to do the same technique for the actual robot code.  This keeps the main assembly lean, and only what is needed.

### Optional way to avoid needing to build OpenSceneGraph

To start, the most important thing I wished to avoid is any committed dependency for OpenSceneGraph (a.k.a. osg).  The main reason is that we may decide to swap out osg for a different viewer like unreal, and we may even want to experiment with 3D view as well like they do for the WPI simulation. Given this I've made it as such where you can completely avoid building osg by pulling the drop box binaries and placing them in the OSG_Viewer folder I have detailed instruction in TeleOpV2.cpp that has the link and where to put it.  If you want to build OpenSceneGraph I may put some instruction on this, but for now I'm discouraging this because it will take focus away from the robot code.  That said, I am happy to add instructions for this as needed.

---

## *Launcher*

The launcher is the main code which is responsible for instantiating each module, and will need to link them together.  Hopefully the linking part should be abstract enough to where the properties can manage the details as needed.  The idea is that the linking involved should be able to not be robot specific at this level, and could last many seasons.

---

## *Modules*

Each Module is self contained and can run independently to make this work if it's dependency is not linked yet it can default to some kind of unit test to act as the filler so that it can be maintained without need from any other module.

In "Moduler_Design.ppt" has a high level list of the modules and their general flow of information.  The standard convention will be a method driven callback where it's default implementation can be a test stub.  This makes it possible to avoid any class coupling which should be avoided between each module.

An overall view of the module layout consist of the input group, the robot group, and the output group.

---

### Input Group

- AI Input
- Input
- Controller Properties

The *Input* with its controller properties defines tele-op or the controller's implementation.  The Launcher will provide the Robot object and then input can bind its controls (specified by the properties)

*AI Input* is the autonomous that manages and executes goals.  If designed properly it should only need SLAM, and may need other sensors from the Robot object (which could be routed through SLAM)

The way the design currently stands is that robot gets the time slices and each input module can get a time slice for its operations and the robot reads from the input to translate what it needs to do for output.

### Robot Group

- Robot
- Robot Properties
- SLAM

The *Robot* will pull input and push output.  This is where the drive kinematics resides, it will use properties and if done correct should be able to survive multiple seasons.  The most common component of the robot is the rotary system, and so this is the front end for what is to be the output.

*SLAM* (Simultaneous localization and mapping) is the object that keeps track of it's position on the field.  It will bind to robot to get sensor information (localization) to determine where it really is.  The mapping aspect can be addressed per game as needed.

### Output Group

- Simulated Output
- WPI Output
- Output properties

The simulated output can make use of SLAM data and present it in a way that can help see how the output will behave.  Typically in the past this has been used to monitor the drive from a top view 2D analyses of motion, but could indeed by translated to a 3D representation.  It can alternative pull data from the Robot to show some representation of this data.  The SmartDashboard could be considered a simulated output.

Finally the WPI Output can pull data from the robot to update components as well as give data from sensors back to the robot, where the properties can help map out the slot assignments for each component.
