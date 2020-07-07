#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <iostream>
#include <math.h>
#include <assert.h>
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

//At this point anything can be included or not in any order unlike before as each header represents a module which is self 
//contained.  For this project we'll include all the modules for ease of testing even though each test may only include
//a few select

#include "../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"
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
			printf("Button: 2=exit, x azis=turn, y axis=forward/reverse \n");
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

#pragma region _main_
int main()
{
	SmartDashboard::init();
	//Reserved... this may become command driven
	//For now we are just typing in the test here
	//Test01_Tank_Kinematics_with_Joystick test;
	Test02_Tank_Kinematics_with_TankSteering test;
	test.init();  //good habit to late bind your classes (if possible), makes them easier to work with
	test();
	SmartDashboard::shutdown();
	return 0;
}
#pragma endregion

