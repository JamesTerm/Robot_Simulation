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
#include "SwerveRobot.h"

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib,"../../../../Modules/Output/OSG_Viewer/x64/Debug/OSG_Viewer.lib")
#else
#pragma comment(lib,"../../../../Modules/Output/OSG_Viewer/x64/Release/OSG_Viewer.lib")
#endif
#endif

#pragma endregion

void strafe_test(Module::Robot::SwerveRobot &robot)
{
	//Test strafing back and forth, test with simple motion control and bypass rotary system to debug swerve management

	for (size_t i = 0; i < 32; i++)
	{
		robot.SetLinearVelocity_local(0.0, 1.0);  //simple move right
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().x()));
	}
	printf("------------------------------------------------------\n");
	for (size_t i = 0; i < 16; i++)
	{
		robot.SetLinearVelocity_local(0.0, -1.0);  //simple move left
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().x()));
	}
	printf("------------------------------------------------------\n");
}

void up_down_test(Module::Robot::SwerveRobot& robot)
{
	for (size_t i = 0; i < 32; i++)
	{
		robot.SetLinearVelocity_local(-1.0, 0.0);  //simple move down reverse
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------\n");

	for (size_t i = 0; i < 16; i++)
	{
		robot.SetLinearVelocity_local(1.0, 0.0);  //simple move up forward
		robot.TimeSlice(0.010); //update a time slice
		//printf("position=%.2f, x=%.2f\n", Meters2Feet( _robot.GetCurrentPosition().y()), Meters2Feet( _robot.GetCurrentPosition().x()));
		printf("angle=%.2f,position y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------\n");
}

void stop_test(Module::Robot::SwerveRobot& robot, size_t eoi)
{
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, 0.0);  //stop
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ stop\n");
}

void diagonal_test(Module::Robot::SwerveRobot& robot, double forward, double right, size_t eoi)
{
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(forward, right);  //diagonally
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ stop\n");
}

void strafe_box_test(Module::Robot::SwerveRobot& robot, bool reverse_x=false)
{
	const size_t eoi = 128;
	const double x_mult = reverse_x ? -1.0 : 1.0;
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, x_mult*1.0);  //simple move right
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]), 
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ right\n");
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(1.0, 0.0);  //simple move down
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ down\n");
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(0.0, x_mult * -1.0);  //simple move left
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ left\n");
	for (size_t i = 0; i < eoi; i++)
	{
		robot.SetLinearVelocity_local(-1.0, 0.0);  //simple move up
		robot.TimeSlice(0.010); //update a time slice
		printf("angle=%.2f,position x=%.2f  y=%.2f\n", RAD_2_DEG(robot.GetCurrentVelocities().Velocity.AsArray[4]),
			Meters2Feet(robot.GetCurrentPosition().x()), Meters2Feet(robot.GetCurrentPosition().y()));
	}
	printf("------------------------------------------------------ up\n");
}

int main()
{
	using namespace Module::Robot;
	SwerveRobot _robot;
	_robot.Init();
	_robot.SetAngularVelocity(0.0);

	#if 1
	up_down_test(_robot);
	strafe_test(_robot);
	_robot.SetAngularVelocity(Pi2);
	for (size_t i = 0; i < 16; i++)
	{
		_robot.SetAngularVelocity(Pi2);
		_robot.TimeSlice(0.033); //update a time slice
		//printf("heading=%.2f\n", RAD_2_DEG( _robot.GetCurrentHeading()));
		printf("angle=%.2f,heading =%.2f\n", RAD_2_DEG(_robot.GetCurrentVelocities().Velocity.AsArray[4]), RAD_2_DEG(_robot.GetCurrentHeading()));
	}
	printf("------------------------------------------------------\n");
	#endif
	#if 1
	strafe_box_test(_robot);
	strafe_box_test(_robot);
	strafe_box_test(_robot,true);
	strafe_box_test(_robot,true);
	#endif
	#if 1
	diagonal_test(_robot,1.6,1.0, 32);
	stop_test(_robot, 32);
	#endif
	_robot.Reset();
}
