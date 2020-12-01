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
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	double m_current_position[4] = {};  //keep track of the pot's position of each angle
	SimulatedOdometry::properties m_properties;

	inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}

public:
	void Init(const SimulatedOdometry::properties *props)
	{
		//Since we define these props as we need them the client code will not need default ones
		if (!props)
		{
			m_properties.swivel_max_speed[0] =
				m_properties.swivel_max_speed[1] =
				m_properties.swivel_max_speed[2] =
				m_properties.swivel_max_speed[3] = 8.0;
		}
		else
			m_properties = *props;
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
		//convert voltages back to velocities
		//for drive this is easy
		for (size_t i = 0; i < 4; i++)
		{
			m_CurrentVelocities.Velocity.AsArray[i] *= m_maxspeed;
		}
		//for position we have to track this ourselves
		for (size_t i = 0; i < 4; i++)
		{
			//go ahead and apply the voltage to the position... this is over-simplified but effective for a bypass
			//from the voltage determine the velocity delta
			const double velocity_delta = m_CurrentVelocities.Velocity.AsArray[i + 4] * m_properties.swivel_max_speed[i];
			m_current_position[i] = NormalizeRotation2( m_current_position[i] + velocity_delta * d_time_s);
			m_CurrentVelocities.Velocity.AsArray[i+4] = m_current_position[i];
		}
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
void SimulatedOdometry::Init(const properties *props)
{
	m_simulator->Init(props);
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