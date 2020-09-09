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
#include "RotarySystem.h"
//Useful for diagnostics
#define __UseBypass__
#pragma endregion
namespace Module {
	namespace Robot {
#pragma region _RotaryPosition_Internal_
class RotaryPosition_Internal
{
private:
	double m_Position = 0.0;
	std::function<void(double new_voltage)> m_VoltageCallback;
public:
	void Init()
	{}
	void ShutDown()
	{}
	void SetPosition(double position)
	{
		m_Position = position;
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		m_VoltageCallback(m_Position);
		#else
		#endif
	}
	void Reset(double position = 0.0)
	{}
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
	{
		m_VoltageCallback = callback;
	}
	void SetOdometryCallback(std::function<double()> callback)
	{}
};
#pragma endregion
#pragma region _RotaryVelocity_Internal_
class RotaryVelocity_Internal
{
private:
	double m_Velocity = 0.0;
	std::function<void(double new_voltage)> m_VoltageCallback;
public:
	void Init()
	{}
	void ShutDown()
	{}
	void SetVelocity(double rate)
	{
		m_Velocity = rate;
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		m_VoltageCallback(m_Velocity);
		#else
		#endif
	}
	void Reset(double position = 0.0)
	{
		m_Velocity = 0.0;
	}
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
	{
		m_VoltageCallback = callback;
	}
	void SetOdometryCallback(std::function<double()> callback)
	{}
};
#pragma endregion
#pragma region _wrapper methods_
#pragma region _Rotary Position_
RotarySystem_Position::RotarySystem_Position()
{
	m_rotary_system = std::make_shared<RotaryPosition_Internal>();
}
void RotarySystem_Position::Init()
{
	m_rotary_system->Init();
}
void RotarySystem_Position::ShutDown()
{
	m_rotary_system->ShutDown();
}
void RotarySystem_Position::SetPosition(double position)
{
	m_rotary_system->SetPosition(position);
}
void RotarySystem_Position::TimeSlice(double d_time_s)
{
	m_rotary_system->TimeSlice(d_time_s);
}
void RotarySystem_Position::Reset(double position)
{
	m_rotary_system->Reset(position);
}
void RotarySystem_Position::Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
{
	m_rotary_system->Set_UpdateCurrentVoltage(callback);
}
void RotarySystem_Position::SetOdometryCallback(std::function<double()> callback)
{
	m_rotary_system->SetOdometryCallback(callback);
}
#pragma endregion
#pragma region _Rotary Velocity_
RotarySystem_Velocity::RotarySystem_Velocity()
{
	m_rotary_system = std::make_shared<RotaryVelocity_Internal>();
}
void RotarySystem_Velocity::Init()
{
	m_rotary_system->Init();
}
void RotarySystem_Velocity::ShutDown()
{
	m_rotary_system->ShutDown();
}
void RotarySystem_Velocity::SetVelocity(double rate)
{
	m_rotary_system->SetVelocity(rate);
}
void RotarySystem_Velocity::TimeSlice(double d_time_s)
{
	m_rotary_system->TimeSlice(d_time_s);
}
void RotarySystem_Velocity::Reset()
{
	m_rotary_system->Reset();
}
void RotarySystem_Velocity::Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
{
	m_rotary_system->Set_UpdateCurrentVoltage(callback);
}
void RotarySystem_Velocity::SetOdometryCallback(std::function<double()> callback)
{
	m_rotary_system->SetOdometryCallback(callback);
}

#pragma endregion
#pragma endregion

	}
}