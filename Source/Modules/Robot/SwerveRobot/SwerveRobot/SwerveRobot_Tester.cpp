#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"

#include "../../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../../Modules/Input/JoystickConverter.h"
#include "../../../../Modules/Robot/Entity2D/Entity2D/Entity2D.h"
#include "../../../../Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.h"
#include "../../../../Modules/Output/OSG_Viewer/OSG_Viewer/SwerveRobot_UI.h"
#include "../../../../Modules/Output/OSG_Viewer/OSG_Viewer/Keyboard_State.h"
#include "../../../../Properties/script_loader.h"
#include "../../../../Properties/RegistryV1.h"
#include "SwerveRobot.h"

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib,"../../../../Modules/Output/OSG_Viewer/x64/Debug/OSG_Viewer.lib")
#else
#pragma comment(lib,"../../../../Modules/Output/OSG_Viewer/x64/Release/OSG_Viewer.lib")
#endif
#endif

//#define __TestPID__

#ifndef __TestPID__
#define cout(x,...) printf(x,__VA_ARGS__);
#else
#define cout(x,...)
#endif

#pragma endregion

void TimeSlice(Module::Robot::SwerveRobot& robot)
{
	robot.SimulatorTimeSlice(0.010);
	robot.TimeSlice(0.010); //update a time slice
}
void Stop(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 200; i++)
	{
		robot.SetLinearVelocity_local(0.0, 0.0);  //stop
		TimeSlice(robot); //update a time slice
		//cout("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		cout("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ Stop\n");

}

void strafe_test(Module::Robot::SwerveRobot &robot)
{
	//Test strafing back and forth, test with simple motion control and bypass rotary system to debug swerve management

	for (size_t i = 0; i < 32; i++)
	{
		robot.SetLinearVelocity_local(0.0, 1.0);  //simple move right
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().x()));
	}
	cout("------------------------------------------------------ strafe right\n");
	for (size_t i = 0; i < 16; i++)
	{
		robot.SetLinearVelocity_local(0.0, -1.0);  //simple move left
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().x()));
	}
	cout("------------------------------------------------------ strafe left\n");
}
void up_down_test(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 32; i++)
	{
		robot.SetLinearVelocity_local(-1.0, 0.0);  //simple move down reverse
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ down\n");

	#if 1
	const size_t no_iterations = 16;
	const double velocity = 1.0;
	#else
	const size_t no_iterations = 100;
	const double velocity = 3.128;
	#endif
	for (size_t i = 0; i < no_iterations; i++)
	{
		robot.SetLinearVelocity_local(velocity, 0.0);  //simple move up forward
		TimeSlice(robot); //update a time slice
		//cout("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		cout("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ Up\n");
}

void stop_test(Module::Robot::SwerveRobot& robot, size_t eoi)
{
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, 0.0);  //stop
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ stop\n");
}

void diagonal_test(Module::Robot::SwerveRobot& robot, double forward, double right, size_t eoi)
{
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(forward, right);  //diagonally
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ stop\n");
}

void strafe_box_test(Module::Robot::SwerveRobot& robot, bool reverse_x=false)
{
	const size_t eoi = 128;
	const double x_mult = reverse_x ? -1.0 : 1.0;
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, x_mult*1.0);  //simple move right
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), 
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ box right\n");
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(1.0, 0.0);  //simple move down
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ box down\n");

	//Stop(robot);
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, x_mult * -1.0);  //simple move left
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ box left\n");
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(-1.0, 0.0);  //simple move up
		TimeSlice(robot); //update a time slice
		cout("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ box up\n");
}

void Test_Turn(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 50; i++)
	{
		robot.SetIntendedOrientation(DEG_2_RAD(45));
		TimeSlice(robot); //update a time slice
		//cout("heading=%.2f\n", RAD_2_DEG( _robot.GetCurrentHeading()));
		cout("angle=%.2f,heading =%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), RAD_2_DEG(robot.GetCurrentHeading()));
	}
	cout("------------------------------------------------------ turn 45\n");
}

void Test_Centripetal(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 100; i++)
	{
		robot.SetLinearVelocity_local(3.12, 0.0);  //up to top speed
		TimeSlice(robot); //update a time slice
		//cout("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		cout("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	cout("------------------------------------------------------ up 45\n");
	for (size_t i = 0; i < 50; i++)
	{
		robot.SetAngularVelocity(8.0);
		TimeSlice(robot); //update a time slice
		//cout("heading=%.2f\n", RAD_2_DEG( _robot.GetCurrentHeading()));
		cout("pos[%.2f,%.2f],heading =%.2f,gyro=%.2f,e=%.2f\n", 
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()), RAD_2_DEG(robot.GetCurrentHeading()),
			RAD_2_DEG(robot.Get_OdometryCurrentHeading()), Meters2Feet(robot.GetCurrentVelocities().Velocity.AsArray[0])
		);
	}
	cout("------------------------------------------------------ turn fast\n");

}
void Test_Centripetal_right(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 100; i++)
	{
		robot.SetLinearVelocity_local(3.12, 0.0);  //up to top speed
		robot.SetAngularVelocity(8.0);
		TimeSlice(robot); //update a time slice
		//cout("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		cout("pos[%.2f,%.2f],heading =%.2f,gyro=%.2f,e=%.2f\n",
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()), RAD_2_DEG(robot.GetCurrentHeading()),
			RAD_2_DEG(robot.Get_OdometryCurrentHeading()), Meters2Feet(robot.GetCurrentVelocities().Velocity.AsArray[0])
		);
	}
	cout("------------------------------------------------------ up 45\n");
}

void Test_Centripetal_left(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 100; i++)
	{
		robot.SetLinearVelocity_local(3.12, 0.0);  //up to top speed
		robot.SetAngularVelocity(-8.0);
		TimeSlice(robot); //update a time slice
		//cout("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		cout("pos[%.2f,%.2f],heading =%.2f,gyro=%.2f,e=%.2f\n",
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()), RAD_2_DEG(robot.GetCurrentHeading()),
			RAD_2_DEG(robot.Get_OdometryCurrentHeading()), Meters2Feet(robot.GetCurrentVelocities().Velocity.AsArray[0])
		);
	}
	cout("------------------------------------------------------ up 45\n");
}

int main()
{
	using namespace Module::Robot;
	Framework::Base::asset_manager properties;
	properties::script_loader script_loader;
	script_loader.load_script(properties);

	#ifdef __TestPID__
	{
		using namespace properties::registry_v1;
		//pick what we want to test... s is speed for encoder, a is angle for potentiometer
		const char* const prefix = csz_sFL_;
		//const char* const prefix = csz_aFL_;
		std::string constructed_name=prefix;
		constructed_name += csz_Rotary_PID_Console_Dump;
		properties.put_bool(constructed_name.c_str(), true);
	}
	#endif

	SwerveRobot _robot;
	//Test without properties too
	#if 0
	_robot.Init();
	#else
	_robot.Init(&properties);
	#endif
	_robot.SetAngularVelocity(0.0);
	#if 1
	//Test_Centripetal(_robot);
	//Test_Centripetal_right(_robot);
	Test_Centripetal_left(_robot);
#endif
	#if 0
	up_down_test(_robot);
	//stop_test(_robot, 100);
	strafe_test(_robot);
	#endif
	#if 0
	Test_Turn(_robot);
	#endif
	#if 0
	strafe_box_test(_robot);
	strafe_box_test(_robot);
	strafe_box_test(_robot,true);
	strafe_box_test(_robot,true);
	#endif
	#if 0
	diagonal_test(_robot,1.6,1.0, 32);
	stop_test(_robot, 32);
	#endif
	_robot.Reset();
}
