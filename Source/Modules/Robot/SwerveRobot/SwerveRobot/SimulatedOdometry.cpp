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
#include "SimulatedOdometry.h"
//Useful for diagnostics
#define __UseBypass__
#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Simulated Odometry Internal_
class SimulatedOdometry_Internal
{
private:
	SwerveVelocities m_CurrentVelocities;
	std::function<SwerveVelocities()> m_VoltageCallback;
public:
	void Init()
	{
		//reserved
	}
	void ShutDown()
	{
		//reserved
	}
	~SimulatedOdometry_Internal()
	{
		ShutDown();
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		//Input get it from client
		m_VoltageCallback = callback;
	}
	//Run the simulation time-slice
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		//If only life were this simple, but alas robots are not god-ships
		m_CurrentVelocities = m_VoltageCallback();
		#else
		//TODO reserved
		m_CurrentVelocities = m_VoltageCallback();
		#endif
	}
	const SwerveVelocities &GetCurrentVelocities() const
	{
		//Output: contains the current speeds and positions of any given moment of time
		return m_CurrentVelocities;
	}
};
#pragma endregion
#pragma region _wrapper methods_
SimulatedOdometry::SimulatedOdometry()
{
	m_simulator = std::make_shared<SimulatedOdometry_Internal>();
}
void SimulatedOdometry::Init()
{
	m_simulator->Init();
}
void SimulatedOdometry::Shutdown()
{
	m_simulator->ShutDown();
}
void SimulatedOdometry::SetVoltageCallback(std::function<SwerveVelocities()> callback)
{
	m_simulator->SetVoltageCallback(callback);
}
void SimulatedOdometry::TimeSlice(double d_time_s)
{
	m_simulator->TimeSlice(d_time_s);
}
const SwerveVelocities &SimulatedOdometry::GetCurrentVelocities() const
{
	return m_simulator->GetCurrentVelocities();
}

#pragma endregion
}}