# Robot Simulation

## *Description*

This simulation takes advantage of modern c++ to have modular code at each stage of the simulation.  Modular code makes it possible to swap out older code modules to newer ones with ease.  The module separation keeps each class simple and orthogonal which allows for easier collaboration.

## *Introduction*

To help avoid confusion I should address a couple of things to be aware of:

- There are multiple projects that are optional to build but one main project
- There is an optional way to avoid needing to build OpenSceneGraph *(See Output group)*
- Multiple readme files see layout guide below on how to navigate... starting here!

This readme starts with a layout guide, and then gives a brief description of each folder section as such:

- Description
- Introduction (you are here)
  - Layout guide
  - Multiple projects
- Launcher
- Modules
  - Input
  - Robot
  - Output
- Properties

### Layout guide

The folder structure consists of the following groups:

- Application *- Much like the assembly files in SolidWorks where it pieces together modules into one app*
- Base *- Framework base code that has proven to be useful for many years*
- Libraries *- Reserved for third party code, home of SmartDashboard*
- Modules *- Complete sub projects with some form of unit testing*
  - Input *- Joystick controls, and AI goals*
  - Robot *- The heart of translating the input, converting processing, then sending to output*
  - Output *- Simulation viewer, WPI interface to robot*
- Properties *- A way to convert a deployable script for robot calibration*

Most of these folders contain a readme with details of each section, so if just starting, I'd suggest finishing this readme first then move on to the Modules folder in this order Input, Output, Robot, Swerve Robot.  This mimics the order of development to see how it got tested and built on to itself with newer layers this way.  After this, briefly browse the Application readme and end with properties.

### Multiple projects

Usually we only have one project to build, and it has everything in it.  This is partially true, as the "Launcher" currently is the *Driver Station*.  So to build and run the main program, this is all that is needed, and may be a good starting point to see what this code has to offer.  The *Robot Assembly* project is essentially a command driven version of the driver station, but also has various tests within it to test multiple objects to interact.  This is a good project to work with when trying to understand and test linking objects.

The other projects are in each module so that each module can be tested by itself and those can have special unit test cases to ensure the module can handle strategic stresses as deemed fit.  They also show how to use them (without the clutter of other objects as it is in the assembly project).  The physical file organization itself is just how visual studio works by default.

One may ask, couldn't all of these modules be projects in one solution where each has its own test in there.  If each module had multiple files this would be a great idea.  But Instead we are using a one-file one-header approach... so, it is not necessary to make them like this, instead we pull the files themselves into one solution.  In doing so, makes it possible to do the same technique for the actual robot code.  This keeps the main assembly lean, and only what is needed.

---

## *Launcher*

The launcher is the main code which is responsible for instantiating each module, and will need to link them together.  Hopefully the linking part should be abstract enough to where the properties can manage the details as needed.  The idea is that the linking involved should be able to not be robot specific at this level, and could last many seasons.

## *Modules*

Each Module is self contained and can run independently to make this work if it's dependency is not linked yet it can default to some kind of unit test to act as the filler so that it can be maintained without need from any other module.

In "Moduler_Design.ppt" has a high level list of the modules and their general flow of information.  The standard convention will be a method driven callback where it's default implementation can be a test stub.  This makes it possible to avoid any class coupling which should be avoided between each module.

An overall view of the module layout consist of the input group, the robot group, and the output group.

### Input Group

- AI Input
- Input

The *Input* with its controller properties defines tele-op or the controller's implementation.  The Launcher will provide the Robot object and then input can bind its controls (specified by the properties)

*AI Input* is the autonomous that manages and executes goals.  If designed properly it should only need SLAM, and may need other sensors from the Robot object (which could be routed through SLAM)

The way the design currently stands is that launcher gives the time slices to the appropriate input.  The launcher manages which input module to use, so for example using the driver station app, the user picks autonomous or teleop, this gets passed to the launcher to pick if its going to pull the controllers, or run AI goals to push to the robot.  In either case, the selected input module can get a time slice for its operations.

### Robot Group

- Robot
- SLAM
- Swerve Robot super module

#### Robot

The *Robot* will receive input methods and push to output.  This is where the drive kinematics resides, it will use properties and if done correct should be able to survive multiple seasons.  The most common component of the robot is the rotary system, and so this is the front end for what is to be the output.

#### SLAM

*SLAM* (Simultaneous localization and mapping) is the object that keeps track of it's position on the field.  It will bind to robot to get sensor information (localization) to determine where it really is.  The mapping aspect can be addressed per game as needed and most-likely not a part of this module.

#### Swerve Robot super module

*Swerve Robot* the super module where it ties together various robot components into one module.  It contains all the odometry needed for localization.  *Note: SLAM is divided up, where localization occurs with odometry in the robot area, and the entity that records the position is managed outside the robot.  Mapping most-likely will be managed in the AI.*

### Output Group

- Simulated Output
- WPI Output

The simulated output can make use of SLAM data and present it in a way that can help see how the output will behave.  Typically in the past this has been used to monitor the drive from a top view 2D analyses of motion, but could indeed by translated to a 3D representation.  It can alternative pull data from the Robot to show some representation of this data.  The SmartDashboard could be considered a simulated output.

Finally the WPI Output can pull data from the robot to update components as well as give data from sensors back to the robot, where the properties can help map out the slot assignments for each component.

#### Optional way to avoid needing to build OpenSceneGraph

To start, the most important thing I wished to avoid is any committed dependency for OpenSceneGraph (a.k.a. osg).  The main reason is that we may decide to swap out osg for a different viewer like unreal, and we may even want to experiment with 3D view as well like they do for the WPI simulation. Given this I've made it as such where anyone can completely avoid building osg by pulling the drop box binaries and placing them in the OSG_Viewer folder.  I have detailed instruction in TeleOpV2.cpp that has the link and where to put it.  If anyone needs to build OpenSceneGraph, I may put some instruction on this, but for now I'm discouraging this because it will take focus away from the robot code.  That said, I am happy to add instructions for this as needed.

## *Properties*

Properties represents a way to convert a deployable script for robot calibration, which is necessary for quick adjustments while in the pit.  This will most-likely be rewritten *(and probably should be)* by the students.  The example I included here will show conceptually how to interface with the modules where the robot does not have to do anything different or have any interaction with any given properties technique.  The way this works is the launcher will launch this first, and pulls the entire script on startup.  To make life good, it may update on a reset or clicking start, in the past it updated when starting, but some parts required a reboot.  So this may go through several iterations of translation, the properties scripting may translate to some form that is 1:1 with the script language, and then each group of callbacks will provide their own native structures to be filled in as needed.
