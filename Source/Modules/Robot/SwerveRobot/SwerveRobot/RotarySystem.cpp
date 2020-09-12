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
	double m_maxspeed = 1.0;
	std::function<void(double new_voltage)> m_VoltageCallback=nullptr;
	std::function<double()> m_OdometryCallack = nullptr;
	double m_OpenLoop_position=0.0;  //keep track internally if we are open loop
	static double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel, double EntityMass)
	{
		//pulled from physics 1D, needed for computing the velocity (bypass only) slightly altered to be member free
		double ret;

		//This is how many radians the ship is capable to turn for this given time frame
		double Acceleration = (Restraint / EntityMass); //obtain acceleration

		{
			//first compute which direction to go
			double DistanceDirection = Distance;
			DistanceDirection -= matchVel * DeltaTime_s;
			if (IsZero(DistanceDirection))
			{
				ret = matchVel;
				return ret;
			}

			//Unlike in the 3D physics, we'll need while loops to ensure all of the accumulated turns are normalized, in the 3D physics the
			//Quat is auto normalized to only require one if check here
			while (DistanceDirection > M_PI)
				DistanceDirection -= Pi2;
			while (DistanceDirection < -M_PI)
				DistanceDirection += Pi2;
			double DistanceLength = fabs(DistanceDirection);

			//Ideal speed needs to also be normalized
			double IDS = Distance;
			if (IDS > M_PI)
				IDS -= Pi2;
			else if (IDS < -M_PI)
				IDS += Pi2;

			double IdealSpeed = fabs(IDS / DeltaTime_s);

			if (Restraint != -1)
			{
				//Given the distance compute the time needed
				//Place the division first keeps the multiply small
				double Time = sqrt(2.0*(DistanceLength / Acceleration));
				//With torque and its fixed point nature... it is important to have the jump ahead of the slope so that it doesn't overshoot
				//this can be accomplished by subtracting this delta time and working with that value... this should work very well but it could
				//be possible for a slight overshoot when the delta times slices are irregular. 
				if (Time > DeltaTime_s)
				{
					Time -= DeltaTime_s;
					if (IsZero(Time))
						Time = 0.0;
				}

				//Now compute maximum speed for this time
				double MaxSpeed = Acceleration * Time;
				ret = std::min(IdealSpeed, MaxSpeed);

				if (DistanceDirection < 0)
					ret = -ret;
				ret += matchVel;
			}
			else
			{
				ret = IdealSpeed;  //i.e. god speed
				if (IDS < 0)
					ret = -ret;
			}
		}
		return ret;
	}

	void time_slice_bypass(double d_time_s)
	{
		//This example is the simplest logic needed to handle setpoint.  It first computes the delta of where it is vs. where
		//it needs to be and from this computes the velocity using the s=1/2at^2 equation (solved for time, more details 
		//of this later). Finally the velocity get's normalized for voltage. The actual rotary system code uses PID and other
		//vices to deal with real-world stresses

		//First grab our current position
		const double pot_pos = m_OdometryCallack ? m_OdometryCallack() : m_OpenLoop_position;
		//now our delta always in direction of destination - previous
		const double distance = m_Position - pot_pos;
		//Now to compute the velocity, our units are radians
		//For this to work, we have to be mindful of our max acceleration, the way this method GetVelocityFromDistance_Angular
		//works is that we compute the max torque the motor can supply (Restraint parameter) against the mass that it needs to
		//turn.  We can compute the torque with pretty good accuracy, and the mass can be compensated to account for other loads
		//such as friction and mechanical transfer efficiency etc. This can be tuned empirically and in the real code have PID to
		//work out any error.

		//this is our wheel module weight and really does not count weight of robot or friction
		const double EntityMass = 2.0 * 0.453592; //essentially our load of what we are applying the torque to
		const double stall_torque = 0.38; //using a RS-550 which is plenty of power
		const double torque_restraint = stall_torque * 100.0; //the total torque simple factors any gear reduction (counting the shaft to final gear reduction)
		const double velocity = GetVelocityFromDistance_Angular(distance, torque_restraint, d_time_s, 0.0, EntityMass);
		//almost there now to normalize the voltage we need the max speed, this is provided by the free speed of the motor, and since it is the speed of the
		//swivel itself we factor in the gear reduction.  This should go very quick because the load is light
		const double free_speed_RPM = 19000.0 / 30.0;  //factor in gear reduction
		const double max_speed_rad = free_speed_RPM * Pi2;
		//Note: At some point I should determine why the numbers do not quite align, they are empirically tweaked to have a good acceleration, and
		//should be fine for bypass demo
		double voltage = velocity / 8;
		//avoid oscillation by cutting out minor increments
		if (fabs(voltage) < 0.01)
			voltage = 0.0;
		m_VoltageCallback(voltage);
	}
public:
	void Init()
	{
		//TODO
	}
	void ShutDown()
	{}
	void SetPosition(double position)
	{
		m_Position = position;
	}
	void TimeSlice(double d_time_s)
	{
		//Use our setpoint input m_Position compute needed velocity and apply normalized voltage to voltage callback
		#ifdef __UseBypass__
		time_slice_bypass(d_time_s);
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
	{
		m_OdometryCallack = callback;
	}
};
#pragma endregion
#pragma region _RotaryVelocity_Internal_
class RotaryVelocity_Internal
{
private:
	double m_Velocity = 0.0;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	std::function<void(double new_voltage)> m_VoltageCallback;
public:
	void Init()
	{
		//TODO bind max speed to properties used in main assembly via external properties
		//we'll have other properties, and we'll need to work out the common ones
	}
	void ShutDown()
	{}
	void SetVelocity(double rate)
	{
		m_Velocity = rate;
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		//voltage in the bypass case is a normalized velocity
		m_VoltageCallback(m_Velocity/m_maxspeed);
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