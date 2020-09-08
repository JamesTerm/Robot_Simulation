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

#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../MotionControl2D_simple/MotionControl2D/MotionControl2D.h"
#include "../../MotionControl2D_physics/MotionControl2D_physics/MotionControl2D.h"
#include "SwerveRobot.h"

#define __UseSimpleMotionControl__
#pragma endregion

namespace Module {
	namespace Robot {

class SwerveRobot_Internal
{
private:
	#pragma region _member variables_
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	#ifdef __UseSimpleMotionControl__
	Simple::MotionControl2D m_MotionControl2D;
	#else
	Physics::MotionControl2D m_MotionControl2D;
	#endif
	#pragma endregion
public:
	void Init()
	{

	}
	void SetLinearVelocity_local(double forward, double right)
	{
		m_MotionControl2D.SetLinearVelocity_local(forward, right);
	}
	void SetAngularVelocity(double clockwise)
	{
		m_MotionControl2D.SetAngularVelocity(clockwise);
	}
	void TimeSlice(double d_time_s)
	{
		m_MotionControl2D.TimeSlice(d_time_s);
	}
	//allows entity to be stopped and reset to a position and heading
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_MotionControl2D.Reset(X,Y,heading);
	}
};
}}