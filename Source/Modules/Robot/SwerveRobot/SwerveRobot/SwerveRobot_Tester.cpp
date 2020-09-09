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

int main()
{
	using namespace Module::Robot;
	SwerveRobot _robot;
	_robot.Init();
	_robot.SetLinearVelocity_local(1.0, 0.0);  //simple move forward
	for (size_t i = 0; i < 16; i++)
	{
		_robot.TimeSlice(0.010); //update a time slice
		printf("position=%.2f\n",_robot.GetCurrentPosition().y());
	}
	_robot.Reset();
	_robot.SetAngularVelocity(Pi);
	for (size_t i = 0; i < 16; i++)
	{
		_robot.TimeSlice(0.010); //update a time slice
		printf("heading=%.2f\n", _robot.GetCurrentHeading());
	}
	_robot.Reset();
}
