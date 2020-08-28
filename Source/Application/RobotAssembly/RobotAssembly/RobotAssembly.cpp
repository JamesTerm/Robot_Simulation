#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

//At this point anything can be included or not in any order unlike before as each header represents a module which is self 
//contained.  For this project we'll include all the modules for ease of testing even though each test may only include
//a few select

#include "../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../Modules/Input/JoystickConverter.h"
#include "../../../Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../../Modules/Robot/Entity2D/Entity2D/Entity2D.h"
#include "TeleOpV1.h"  //still a great test which doesn't require OSG (but can be ran separately)
#include "TeleOpV2.h"
//Note these are not needed for tests 1-4
#include "../../../Modules/Robot/MotionControl2D_physics/MotionControl2D_physics/MotionControl2D.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/SwerveRobot_UI.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/Entity_UI.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/Keyboard_State.h"

#pragma endregion
#pragma region _Test01_Tank_Kinematics_with_Joystick_

class Test01_Tank_Kinematics_with_Joystick
{
	//Note: most of the code in here is copied from the examples... I'm not always going to do this, but I am
	//in the beginning as new things are introduced
private:
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Tank_Drive m_robot;
	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
public:
	void init()
	{
		m_joystick.Init();

		using namespace Module::Robot;
		//setup some robot properties
		using properties = Tank_Drive::properties;
		//For 6WD the length is distance between the center wheels to one set of the outer wheels, this is because
		//with the drop center only 4 wheels are touching at any given time (most of the time)
		Vec2D wheel_dimensions(Inches2Meters(12), Inches2Meters(24));
		//Note: In inch units the length of 12 and 24 is roughly 26.83 where it's the
		//hypotenuse of a 12 24 triangle
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD or one set of 4 wheels for 6WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);

		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		const double skid = cos(atan2(wheel_dimensions[1], wheel_dimensions[0]));
		//We'll compute the max angular velocity to use in equation
		//To maintain ability to retain max angular velocity in all cases of moving forward or reverse it ends up being
		//roughly half power when spinning in place... so we could make this higher, but then all the other cases would
		//be clipping.  Note: Please don't spend too much time worrying about this, because this is all skid steering
		//issues, none-of which will be a problem for swerve drive
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
	}
	void operator()() 
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			printf("Button: 2=exit, x azis=turn, y axis=forward/reverse \n");
			dx_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
			bool done = false;
			while (!done)
			{
				if (m_joystick.read_joystick(JoyNum, joyinfo))
				{
					//To use the Kinematics correctly the vector needs to keep the magnitude at 1.0, typically the joy axis
					//goes to 1.0 for all axis so the diagonal result is a greater magnitude.  To fix we'll normalize the 
					//direction with 1.0, and clip via min of the current value.. the result is the joystick has a circle
					//instead of a square on its range of motion.
					Vec2D JoyInput_To_Use(joyinfo.lX, joyinfo.lY);
					Vec2D NormalizedDirection = JoyInput_To_Use;
					double Magnitude =NormalizedDirection.normalize();
					bool test_clipping = false; //just for display to see how this works
					if (Magnitude > 1.0)
					{
						test_clipping = true;
						//clip with the normalized direction
						JoyInput_To_Use = NormalizedDirection;
						//sanity check... for some reason the magnitude is still high with some combinations
						//Even this check may fail, this is probably a result of floating point precision lost
						//however, it's still effective enough, and we can observe that when we remain within the clipping
						//region the limit of the velocities are correct, most of these issues will not be a real problem
						//especially when making the input logarithmic, and even less of a problem by reducing the maximum
						//turning rate
						if (JoyInput_To_Use.length() > 1.0)
						{
							//TODO why is this failing... give it a hacked less magnitude :(
							JoyInput_To_Use = NormalizedDirection * 0.75;
						}
					}

					//Get an input from the controllers to feed in... we'll hard code the x and y axis
					//m_robot.UpdateVelocities(Feet2Meters(m_maxspeed*joyinfo.lY), joyinfo.lX * m_max_heading_rad);

					//Instead... normalized to 1.0 magnitude when needed
					m_robot.UpdateVelocities(Feet2Meters(m_maxspeed * JoyInput_To_Use[1]), JoyInput_To_Use[0] * m_max_heading_rad);

					printf("\r Left=%f Right=%f %s            ",
						Meters2Feet(m_robot.GetLeftVelocity()), Meters2Feet(m_robot.GetRightVelocity()),
						test_clipping ? "[!]" : ""
					);
					//Use smart dashboard to see progress bar representation (gets a better idea of the clipping)
					//Roll the joystick around each direction when doing this... to confirm it's correct
					//set progress bar to 12 to -12 on the range in its properties
					SmartDashboard::PutNumber("Left", Meters2Feet(m_robot.GetLeftVelocity()));
					SmartDashboard::PutNumber("Right", Meters2Feet(m_robot.GetRightVelocity()));
					if (joyinfo.ButtonBank[0] == 2)
						done = true;
				}
				Sleep(22); //about 30 times a sec
			}
		}
		else
		{
			printf("None found\n");
			Sleep(2000);
		}
	}
};
#pragma endregion
#pragma region _Test02_Tank_Kinematics_with_TankSteering_

class Test02_Tank_Kinematics_with_TankSteering
{
private:
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Tank_Drive m_robot;
	Module::Robot::Inv_Tank_Drive m_tank_steering;
	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
public:
	void init()
	{
		m_joystick.Init();

		using namespace Module::Robot;
		//setup some robot properties
		using properties = Tank_Drive::properties;
		//For 6WD see test 1 for more information
		Vec2D wheel_dimensions(Inches2Meters(12), Inches2Meters(24));
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD or one set of 4 wheels for 6WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);
		Inv_Tank_Drive::properties *props2;
		//This works, for test code but not recommended for actual code
		props2 = (Inv_Tank_Drive::properties *)&props;
		m_tank_steering.SetProperties(*props2);

		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		const double skid = cos(atan2(wheel_dimensions[1], wheel_dimensions[0]));
		//See test 1 for more information
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
	}
	void operator()()
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			printf("Button: 2=exit, y axis=left, z axis=right \n");
			dx_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
			bool done = false;
			while (!done)
			{
				if (m_joystick.read_joystick(JoyNum, joyinfo))
				{
					//unlike in previous test... no magnitude check... we can see how well they work together
					//Note the right up/down axis may be different on your controller... just hard code the right one for now
					m_tank_steering.InterpolateVelocities(Feet2Meters(m_maxspeed * joyinfo.lY), Feet2Meters(m_maxspeed *  joyinfo.lZ));

					//Now we can pull the interpolated values to be passed in... Note: we have no strafe, so we can ignore
					m_robot.UpdateVelocities(m_tank_steering.GetLocalVelocityY(), m_tank_steering.GetAngularVelocity());

					printf("\r Left=%f Right=%f             ",
						Meters2Feet(m_robot.GetLeftVelocity()), Meters2Feet(m_robot.GetRightVelocity())	);
					//Use smart dashboard to see progress bar representation
					//set progress bar to 12 to -12 on the range in its properties
					//Tests show the inputs match the final velocities virtually exact
					SmartDashboard::PutNumber("Left_input", m_maxspeed * joyinfo.lY);
					SmartDashboard::PutNumber("Right_input", m_maxspeed * joyinfo.lZ);
					SmartDashboard::PutNumber("Left", Meters2Feet(m_robot.GetLeftVelocity()));
					SmartDashboard::PutNumber("Right", Meters2Feet(m_robot.GetRightVelocity()));
					if (joyinfo.ButtonBank[0] == 2)
						done = true;
				}
				Sleep(22); //about 30 times a sec
			}
		}
		else
		{
			printf("None found\n");
			Sleep(2000);
		}
	}
};
#pragma endregion
#pragma region _Test03_Swerve_Kinematics_with_Joystick_

class Test03_Swerve_Kinematics_with_Joystick
{
private:
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Swerve_Drive m_robot;
	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
public:
	void init()
	{
		m_joystick.Init();

		using namespace Module::Robot;
		//setup some robot properties
		using properties = Swerve_Drive::properties;
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);

		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;  
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
	}
	void operator()()
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			dx_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
			bool done = false;
			while (!done)
			{
				if (m_joystick.read_joystick(JoyNum, joyinfo))
				{
					//In this test we will not have magnitude clipping, but I may change that later
					//Get an input from the controllers to feed in... we'll hard code the x and y axis
					m_robot.UpdateVelocities(Feet2Meters(m_maxspeed*joyinfo.lY), Feet2Meters(m_maxspeed*joyinfo.lX), joyinfo.lZ * m_max_heading_rad);

					//I've added SmartLayout_Swerve1.xml in the design folder to test
					//Use smart dashboard to see progress bar representation (gets a better idea of the clipping)
					//Roll the joystick around each direction when doing this... to confirm it's correct
					//set progress bar to 12 to -12 on the range in its properties
					SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(0)));
					SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(1)));
					SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(2)));
					SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(3)));
					//For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
					//its not perfect, but it gives a good enough direction to tell (especially when going down)
					SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(0)));
					SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(1)));
					SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(2)));
					SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(3)));
					if (joyinfo.ButtonBank[0] == 2)
						done = true;
				}
				Sleep(22); //about 30 times a sec
			}
		}
		else
		{
			printf("None found\n");
			Sleep(2000);
		}
	}
};
#pragma endregion
#pragma region _Test04_Swerve_Kinematics_with_TankSteering_

class Test04_Swerve_Kinematics_with_TankSteering
{
private:
	Module::Input::dx_Joystick m_joystick;
	Module::Robot::Swerve_Drive m_robot;
	Module::Robot::Inv_Tank_Drive m_tank_steering; //yes still tank (for *tank* steering)
	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
public:
	void init()
	{
		m_joystick.Init();

		using namespace Module::Robot;
		//setup some robot properties
		using properties = Swerve_Drive::properties;
		//For test 3 for more information
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);
		Inv_Tank_Drive::properties *props2;
		//This works, for test code but not recommended for actual code
		props2 = (Inv_Tank_Drive::properties *)&props;
		m_tank_steering.SetProperties(*props2);

		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//see test 3 for more info on skid
		const double skid = 1.0;
		//See test 3 for more information
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
	}
	void operator()()
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			printf("Button: 2=exit, x axis=strafe, y axis=left, z axis=right \n");
			dx_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
			bool done = false;
			while (!done)
			{
				if (m_joystick.read_joystick(JoyNum, joyinfo))
				{
					//unlike in previous test... no magnitude check... we can see how well they work together
					//Note the right up/down axis may be different on your controller... just hard code the right one for now
					m_tank_steering.InterpolateVelocities(Feet2Meters(m_maxspeed * joyinfo.lY), Feet2Meters(m_maxspeed *  joyinfo.lZ));

					//Now we can pull the interpolated values to be passed in... Note: we have strafe too!
					m_robot.UpdateVelocities(m_tank_steering.GetLocalVelocityY(), Feet2Meters(m_maxspeed*joyinfo.lX), m_tank_steering.GetAngularVelocity());

					//Use smart dashboard to see progress bar representation
					SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(0)));
					SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(1)));
					SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(2)));
					SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(3)));
					SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(0)));
					SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(1)));
					SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(2)));
					SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(3)));
					if (joyinfo.ButtonBank[0] == 2)
						done = true;
				}
				Sleep(22); //about 30 times a sec
			}
		}
		else
		{
			printf("None found\n");
			Sleep(2000);
		}
	}
};
#pragma endregion
#pragma region _Test05_Test_Bypass_with_OSG_
class Test05_Test_Bypass_with_OSG
{
	//This is essentially a stripped down version of TeleOp V2,
	//This uses a simpler EntityUI, and a bypass for the kinematics to show exactly what goes on from input to when it gets
	//the velocities interpreted, this is an ideal minimal setup for testing various kinds of motion control (or lack thereof)
	//ideal for the start of motion control that deals with physics of a heavy mass
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	Module::Robot::Physics::MotionControl2D m_MotionControl2D;
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Input::Analog_EventEntry m_joystick_options;  //for now a simple one-stop option for all
	Module::Robot::Bypass_Drive m_robot;
	Module::Robot::Bypass_Drive m_Entity_Input;
	
	Module::Output::OSG_Viewer m_viewer;
	Module::Output::Entity_UI m_RobotUI;
	Module::Output::Entity_UI::uEntity_State m_current_state = {};

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	double m_dTime_s=0.016;  //cached so other callbacks can access
    Module::Input::Keyboard_State m_Keyboard;
	#pragma endregion

	void UpdateVariables()
	{
		Module::Localization::Entity2D &entity = m_Entity;
		using namespace Module::Localization;
		Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		//Entity variables-------------------------------------------
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		m_current_state.bits.Pos_m.x = position.x;
		m_current_state.bits.Pos_m.y = position.y;
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		//This is temporary and handy for now, will change once we get AI started
		m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
		m_current_state.bits.Att_r = entity.GetCurrentHeading();
		//kinematic variables-------------------------------------------
	}

	void GetInputSlice()
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		dx_Joystick::JoyState joyinfo = {0}; //setup joy zero'd out

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			m_joystick.read_joystick(JoyNum, joyinfo);
		}
		{
			//Get an input from the controllers to feed in... we'll hard code the x and y axis from both joy and keyboard
			//we simply combine them so they can work inter-changeably (e.g. keyboard for strafing, joy for turning)
			m_robot.UpdateVelocities(
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.lY,m_joystick_options)+m_Keyboard.GetState().bits.m_Y)*-1.0),
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.lX, m_joystick_options)+m_Keyboard.GetState().bits.m_X)),
				(AnalogConversion(joyinfo.lZ, m_joystick_options) +m_Keyboard.GetState().bits.m_Z) * m_max_heading_rad);
			//because of properties to factor we need to interpret the actual velocities resolved from the kinematics by inverse kinematics
			m_Entity_Input.InterpolateVelocities(m_robot.GetLocalVelocityY(),m_robot.GetLocalVelocityX(),m_robot.GetAngularVelocity());

			//Here is how is we can use or not use motion control, some interface... the motion control is already linked to entity
			//so this makes it easy to use

			#if 0
			//Now we can update the entity with this inverse kinematic input
			m_Entity.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
			m_Entity.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
			#else
			//Now we can update the entity with this inverse kinematic input
			m_MotionControl2D.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
			m_MotionControl2D.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
			#endif

			//This comes in handy for testing
			if (joyinfo.ButtonBank[0] == 1)
				Reset();
		}
		
	}

	void TimeSliceLoop(double dTime_s)
	{
		m_dTime_s = dTime_s;
		//Grab kinematic velocities from controller
		GetInputSlice();
		//Update the predicted motion for this time slice
		m_MotionControl2D.TimeSlice(dTime_s);
		m_Entity.TimeSlice(dTime_s);
		UpdateVariables();
	}

	void SetUpHooks(bool enable)
	{
		using namespace Module::Output;
		if (enable)
		{
			OSG_Viewer &viewer = m_viewer;
			viewer.SetSceneCallback([&](void *rootNode, void *geode) { m_RobotUI.UpdateScene(geode, true); });
			//Anytime robot needs updates link it to our current state
			m_RobotUI.SetEntity_Callback([&]() {	return m_current_state.bits; });
			//When viewer updates a frame
			viewer.SetUpdateCallback(
				[&](double dTime_s)
			{
				//any updates can go here to the current state
				TimeSliceLoop(dTime_s);
				//give the robot its time slice to process them
				m_RobotUI.TimeChange(dTime_s);
			});
			viewer.SetKeyboardCallback(
				[&](int key, bool press)
			{
				//printf("key=%d press=%d\n", key, press);
				m_Keyboard.UpdateKeys(m_dTime_s, key, press);
				if (key == ' ' && press == false)
					Reset();
			});
		}
		else
		{
			m_RobotUI.SetEntity_Callback(nullptr);
			m_viewer.SetSceneCallback(nullptr);
			m_viewer.SetUpdateCallback(nullptr);
			m_viewer.SetKeyboardCallback(nullptr);
		}
	}
public:
	Test05_Test_Bypass_with_OSG()
	{
		SetUpHooks(true);
	}
	void Reset()
	{
		m_Entity.Reset();
		m_MotionControl2D.Reset();
		m_Keyboard.Reset();
	}
	void init()
	{
		m_joystick.Init();
		m_joystick_options = { 
			0.3,   //filter dead zone
			1.0,   //additional scale
			1.0,   // curve intensity
			false  //is flipped
			};
		using namespace Module::Robot;
		//setup some robot properties
		using properties = Swerve_Drive::properties;
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
		//Note: We'll skip properties for motion control since we have good defaults
		#pragma region _optional linking of entity to motion control_
		//Now to link up the callbacks for motion control:  Note we can link them up even if we are not using it
		using Vector2D = Module::Robot::Physics::MotionControl2D::Vector2D;
		m_MotionControl2D.Set_UpdateGlobalVelocity([&](const Vector2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y, new_velocity.x);
			});
		m_MotionControl2D.Set_UpdateHeadingVelocity([&](double new_velocity)
		{	m_Entity.SetAngularVelocity(new_velocity);
			});
		m_MotionControl2D.Set_GetCurrentPosition([&]() -> Vector2D
		{	
			//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
			//its worth the hassle
			Vector2D ret = { m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y };
			return ret;
		});
		m_MotionControl2D.Set_GetCurrentHeading([&]() -> double
		{
			return m_Entity.GetCurrentHeading();
		});
		#pragma endregion
		Reset();  //for entity variables
		m_viewer.init();
		m_RobotUI.Initialize();
	}
	void Start()
	{
		m_viewer.StartStreaming();
	}
	void Stop()
	{
		m_viewer.StopStreaming();
	}
	~Test05_Test_Bypass_with_OSG()
	{
		//Make sure we are stopped
		Stop();
		SetUpHooks(false);
	}
};
#pragma endregion

#pragma region _main_

#pragma region _DriverStation Tester_
//We'll be able to pick a driver station tester to use
class DriverStation_Tester
{
public:
	enum Testers
	{
		eTeleOpV1,
		eTeleOpV2,
		eBypassKinematics
	};
private:
	#pragma region _Test objects_
	Application::TeleOp_V1 m_RobotTester_V1;
	Application::TeleOp_V2 m_RobotTester_V2;
	Test05_Test_Bypass_with_OSG m_Entity_Bypass;
	#pragma endregion
	Testers m_Tester_Selection=eTeleOpV2;
	bool m_IsInit = false; //provide a valve at this level
public:
	//Call this before init
	void SelectTester(Testers test)
	{
		m_Tester_Selection = test;
	}
	void Reset()
	{
		switch (m_Tester_Selection)
		{
		case eTeleOpV1:
			m_RobotTester_V1.Reset();
			break;
		case eTeleOpV2:
			m_RobotTester_V2.Reset();
			break;
		case eBypassKinematics:
			m_Entity_Bypass.Reset();
			break;
		}
	}
	void init()
	{
		if (!m_IsInit)
		{
			switch (m_Tester_Selection)
			{
			case eTeleOpV1:
			case eTeleOpV2:
				SmartDashboard::init();
				break;
			}
			switch (m_Tester_Selection)
			{
			case eTeleOpV1:
				m_RobotTester_V1.init();
				break;
			case eTeleOpV2:
				m_RobotTester_V2.init();
				break;
			case eBypassKinematics:
				m_Entity_Bypass.init();
				break;
			}
			m_IsInit = true;
		}
	}
	void Start()
	{
		if (!m_IsInit)
			init();
		switch (m_Tester_Selection)
		{
		case eTeleOpV1:
			m_RobotTester_V1.Start();
			break;
		case eTeleOpV2:
			m_RobotTester_V2.Start();
			break;
		case eBypassKinematics:
			m_Entity_Bypass.Start();
			break;
		}
	}
	void Stop()
	{
		switch (m_Tester_Selection)
		{
		case eTeleOpV1:
			m_RobotTester_V1.Stop();
			break;
		case eTeleOpV2:
			m_RobotTester_V2.Stop();
			break;
		case eBypassKinematics:
			m_Entity_Bypass.Stop();
			break;
		}
	}
};
#pragma endregion

enum tests
{
	eCurrent,
	eTankTest_Joy,
	eTankTest_tank_steering,
	eSwerveTest_Joy,
	eSwerveTest_tank_steering,
	eTeleV1,
	eBypass
};

void tester(const char *csz_test, DriverStation_Tester &RobotTester)
{
	const char * const csz_tests[] =
	{
		"current",
		"tank1",
		"tank2",
		"swerve1",
		"swerve2",
		"tele01",
		"bypass"
	};

	int Test = atoi(csz_test);

	//if the first character is not a number then translate the string
	if (((csz_test[0] < '0') || (csz_test[0] > '9')) && (csz_test[0] != 0))
	{
		Test = -1;
		for (int i = 0; i < _countof(csz_tests); i++)
		{
			if (stricmp(csz_tests[i], csz_test) == 0)
			{
				Test = i;
				break;
			}
		}
		if (Test == -1)
		{
			printf("No match found.  Try:\n");
			for (size_t i = 0; i < _countof(csz_tests); i++)
				printf("%s, ", csz_tests[i]);
			printf("\n");
			return;
		}
	}
	switch (Test)
	{
	case eTankTest_Joy:
	case eTankTest_tank_steering:
	case eSwerveTest_Joy:
	case eSwerveTest_tank_steering:
	case eTeleV1:
		SmartDashboard::init();
	}
	switch (Test)
	{
	case eTankTest_Joy:
	{
		Test01_Tank_Kinematics_with_Joystick test;
		test.init();  //good habit to late bind your classes (if possible), makes them easier to work with
		test();
		printf("complete\n");
	}
		break;
	case eTankTest_tank_steering:
	{
		Test02_Tank_Kinematics_with_TankSteering test;
		test.init();
		test();
		printf("complete\n");
	}
		break;
	case eSwerveTest_Joy:
	{
		Test03_Swerve_Kinematics_with_Joystick test;
		test.init();  //good habit to late bind your classes (if possible), makes them easier to work with
		test();
		printf("complete\n");
	}
		break;
	case eSwerveTest_tank_steering:
	{
		Test04_Swerve_Kinematics_with_TankSteering test;
		test.init();
		test();
		printf("complete\n");
	}
		break;
	case eTeleV1:
		//just change the driver selector and away we go
		RobotTester.SelectTester(DriverStation_Tester::eTeleOpV1);
		RobotTester.Start();
		break;
	case eBypass:
	case eCurrent:
		RobotTester.SelectTester(DriverStation_Tester::eBypassKinematics);
		RobotTester.Start();
		break;
	}
}

void cls(HANDLE hConsole = NULL)
{
	if (!hConsole)
		hConsole = ::GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coordScreen = { 0, 0 }; /* here's where we'll home the cursor */
	BOOL bSuccess;
	DWORD cCharsWritten;
	CONSOLE_SCREEN_BUFFER_INFO csbi; /* to get buffer info */
	DWORD dwConSize; /* number of character cells in the current buffer */
	/* get the number of character cells in the current buffer */
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "GetConsoleScreenBufferInfo");   
	dwConSize = csbi.dwSize.X * csbi.dwSize.Y;   /* fill the entire screen with blanks */
	bSuccess = FillConsoleOutputCharacter(hConsole, (TCHAR) ' ',
		dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputCharacter");   /* get the current text attribute */   
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "ConsoleScreenBufferInfo");   /* now set the buffer's attributes accordingly */   
	bSuccess = FillConsoleOutputAttribute(hConsole, csbi.wAttributes, dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputAttribute");   /* put the cursor at (0, 0) */   
	bSuccess = SetConsoleCursorPosition(hConsole, coordScreen);
	//PERR(bSuccess, "SetConsoleCursorPosition");   return; 
}

static void DisplayHelp()
{
	printf(
		"cls\n"
		"test <test name or number> \n"
		"start\n"
		"stop\n"
		"Help (displays this)\n"
		"\nType \"Quit\" at anytime to exit this application\n"
	);
}

__inline void prompt()
{
	std::cout << ">";
}

bool CommandLineInterface()
{
	bool ret = false;
	#pragma region _CLI_SetUp_
	DisplayHelp();

	char		command[32];
	char		str_1[MAX_PATH];
	char		str_2[MAX_PATH];
	char		str_3[MAX_PATH];
	char		str_4[MAX_PATH];
	char		input_line[128];

	using namespace std;
	cout << endl;
	cout << "Ready." << endl;
	#pragma endregion

	DriverStation_Tester m_RobotTester;

	while (prompt(), cin.getline(input_line, 128))
	{
		//init args
		command[0] = str_1[0] = str_2[0] = str_3[0] = str_4[0] = 0;
		if (sscanf(input_line, "%s %s %s %s %s", command, str_1, str_2, str_3, str_4) >= 1)
		{
			if (!_strnicmp(input_line, "cls", 3))
			{
				cls();
			}
			else if (!_strnicmp(input_line, "test", 4))
			{
				//Run tests if we explicitly ask for them; otherwise start up the main test
				if (str_1[0] != 0)
					tester(str_1,m_RobotTester);
				else
					m_RobotTester.Start();
			}
			else if (!_strnicmp(input_line, "reset", 5))
			{
				m_RobotTester.Reset();
			}
			else if (!_strnicmp(input_line, "start", 5))
			{
				m_RobotTester.Start();
			}
			else if (!_strnicmp(input_line, "stop", 5))
			{
				m_RobotTester.Stop();
			}
			else if (!_strnicmp(input_line, "Exit", 4))
			{
				break;
			}
			else if (!_strnicmp(input_line, "Help", 4))
				DisplayHelp();
			else if (!_strnicmp(input_line, "Quit", 4))
			{
				ret=true;
				break;
			}
			else
				cout << "huh? - try \"help\"" << endl;
		}
	}
	return ret;
}


int main()
{
	CommandLineInterface();
	SmartDashboard::shutdown();
	return 0;
}
#pragma endregion

