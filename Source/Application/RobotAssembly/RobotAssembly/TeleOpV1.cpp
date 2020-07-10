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

#include "../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../../Modules/Robot/Entity2D/Entity2D/Entity2D.h"

#include "TeleOpV1.h"
#pragma endregion

namespace Application
{

#pragma region _Test_Swerve_Entity_Joystick_

class Test_Swerve_Entity_Joystick
{
private:
	#pragma region _member variables_
	std::future<void> m_TaskState_TimeSliceLoop;  //Use future to monitor task status
	Module::Localization::Entity2D m_Entity;
	bool m_Done = false;
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Swerve_Drive m_robot;
	Module::Robot::Inv_Swerve_Drive m_Entity_Input;

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	#pragma endregion

	void UpdateVariables()
	{
		Module::Localization::Entity2D &entity = m_Entity;
		using namespace Module::Localization;
		Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
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
	}

	void GetInputSlice()
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			dx_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
			if (m_joystick.read_joystick(JoyNum, joyinfo))
			{
				//Get an input from the controllers to feed in... we'll hard code the x and y axis
				m_robot.UpdateVelocities(Feet2Meters(m_maxspeed*joyinfo.lY*-1.0), Feet2Meters(m_maxspeed*joyinfo.lX), joyinfo.lZ * m_max_heading_rad);
				//because of properties to factor we need to interpret the actual velocities resolved from the kinematics by inverse kinematics
				m_Entity_Input.InterpolateVelocities(m_robot.GetIntendedVelocities());
				//Now we can update the entity with this inverse kinematic input
				m_Entity.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
				m_Entity.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
				//This comes in handy for testing
				if (joyinfo.ButtonBank[0] == 1)
					Reset();
			}
		}
	}

	void TimeSliceLoop(DWORD SleepTime)
	{
		while (!m_Done)
		{
			//Grab kinematic velocities from controller
			GetInputSlice();
			//Update the predicted motion for this time slice
			m_Entity.TimeSlice(0.010);
			UpdateVariables();
			Sleep(SleepTime);
		}
	}


public:
	void Reset()
	{
		m_Entity.Reset();
	}

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
		//redundant but reserved to be different
		Inv_Swerve_Drive::properties inv_props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		m_Entity_Input.SetProperties(inv_props);
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
		Reset();  //for entity variables
	}

	void Start()
	{
		m_Done = false;
		//https://stackoverflow.com/questions/9094422/how-to-check-if-a-stdthread-is-still-running
		using namespace std::chrono_literals;
		//It's considered standard practice to wait for 0ms to obtain the current status without really waiting
		const bool in_flight = m_TaskState_TimeSliceLoop.valid() && m_TaskState_TimeSliceLoop.wait_for(0ms) != std::future_status::ready;
		DWORD time_interval = 10; //actual robot time easy to step calculations a bit slower
		//DWORD time_interval = 33;   //better timing with refresh
		//DWORD time_interval = 1000;  //debugging
		if (!in_flight)
			m_TaskState_TimeSliceLoop = std::async(std::launch::async, &Test_Swerve_Entity_Joystick::TimeSliceLoop, this, time_interval);
		_CP_("in_flight =%d", in_flight);
	}
	void Stop()
	{
		m_Done = true;
		if (m_TaskState_TimeSliceLoop.valid())
		{
			//join... wait for task to close:
			size_t TimeOut = 0;
			while (m_TaskState_TimeSliceLoop.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready)
			{
				printf("waiting for m_TaskState_TimeSliceLoop to close %zd\n", TimeOut++);
			}
		}
	}
	~Test_Swerve_Entity_Joystick()
	{
		//Make sure we are stopped
		Stop();
	}
};
#pragma endregion

#pragma region _TeleOp Wrapper methods_
void TeleOp_V1::Reset()
{
	m_Tele_Internal->Reset();
}
void TeleOp_V1::init()
{
	m_Tele_Internal = std::make_shared<Test_Swerve_Entity_Joystick>();
	m_Tele_Internal->init();
}
void TeleOp_V1::Start()
{
	m_Tele_Internal->Start();
}
void TeleOp_V1::Stop()
{
	m_Tele_Internal->Stop();
}
#pragma endregion

}