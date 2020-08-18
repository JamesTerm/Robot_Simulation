#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include "../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../../Modules/Robot/Entity2D/Entity2D/Entity2D.h"
#include "../../../Modules/Robot/MotionControl2D_simple/MotionControl2D/MotionControl2D.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/SwerveRobot_UI.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/Keyboard_State.h"
#include "TeleOpV2.h"

//We can specify the linking in code
//https://www.dropbox.com/s/bn7ffxb7yqinzme/OSG_Viewer_x64.zip?dl=0
//unzip this to here: .\Robot_Simulation\Source\Modules\Output\OSG_Viewer
//if done correctly an x64 folder will be in the OSG_Viewer folder, then you can link and the 
//custom build step will auto copy the dll's needed to run

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib,"../../../Modules/Output/OSG_Viewer/x64/Debug/OSG_Viewer.lib")
#else
#pragma comment(lib,"../../../Modules/Output/OSG_Viewer/x64/Release/OSG_Viewer.lib")
#endif
#endif

#pragma endregion

namespace Application
{

#pragma region _Test_Swerve_Viewer_

class Test_Swerve_Viewer
{
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	Module::Robot::Simple::MotionControl2D m_MotionControl2D;
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Swerve_Drive m_robot;
	Module::Robot::Inv_Swerve_Drive m_Entity_Input;
	
	Module::Output::OSG_Viewer m_viewer;
	Module::Output::SwerveRobot_UI m_RobotUI;
	Module::Output::SwerveRobot_UI::uSwerveRobot_State m_current_state = {};

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
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X", Meters2Feet(position.x));
		m_current_state.bits.Pos_m.x = position.x;
		SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
		m_current_state.bits.Pos_m.y = position.y;
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		//This is temporary and handy for now, will change once we get AI started
		m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		m_current_state.bits.Att_r = entity.GetCurrentHeading();
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
		for (size_t i = 0; i < 8; i++)
			m_current_state.bits.SwerveVelocitiesFromIndex[i] = m_robot.GetIntendedVelocities().Velocity.AsArray[i];
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
				Feet2Meters(m_maxspeed*(joyinfo.lY+m_Keyboard.GetState().bits.m_Y)*-1.0), 
				Feet2Meters(m_maxspeed*(joyinfo.lX+m_Keyboard.GetState().bits.m_X)), 
				(joyinfo.lZ+m_Keyboard.GetState().bits.m_Z) * m_max_heading_rad);
			//because of properties to factor we need to interpret the actual velocities resolved from the kinematics by inverse kinematics
			m_Entity_Input.InterpolateVelocities(m_robot.GetIntendedVelocities());

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
			m_RobotUI.SetSwerveRobot_Callback([&]() {	return m_current_state.bits; });
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
			m_RobotUI.SetSwerveRobot_Callback(nullptr);
			m_viewer.SetSceneCallback(nullptr);
			m_viewer.SetUpdateCallback(nullptr);
			m_viewer.SetKeyboardCallback(nullptr);
		}
	}
public:
	Test_Swerve_Viewer()
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
		//Note: We'll skip properties for motion control since we have good defaults
		#pragma region _optional linking of entity to motion control_
		//Now to link up the callbacks for motion control:  Note we can link them up even if we are not using it
		using Vector2D = Module::Robot::Simple::MotionControl2D::Vector2D;
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
	~Test_Swerve_Viewer()
	{
		//Make sure we are stopped
		Stop();
		SetUpHooks(false);
	}
};
#pragma endregion

#pragma region _TeleOp Wrapper methods_
void TeleOp_V2::Reset()
{
	m_Tele_Internal->Reset();
}
void TeleOp_V2::init()
{
	m_Tele_Internal = std::make_shared<Test_Swerve_Viewer>();
	m_Tele_Internal->init();
}
void TeleOp_V2::Start()
{
	m_Tele_Internal->Start();
}
void TeleOp_V2::Stop()
{
	m_Tele_Internal->Stop();
}
#pragma endregion

}