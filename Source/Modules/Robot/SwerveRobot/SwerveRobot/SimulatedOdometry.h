#pragma once
#include <memory>
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace Module {
	namespace Robot {

class SimulatedOdometry_Internal;

class SimulatedOdometry
{
public:
	struct properties
	{
		double swivel_max_speed[4];
	};

	SimulatedOdometry();
	void Init(const properties *props=nullptr);
	void Shutdown();
	//Input: for simulation only, grab the voltages from each rotary system
	void SetVoltageCallback(std::function<SwerveVelocities ()> callback);
	//Run the simulation time-slice
	void TimeSlice(double d_time_s);
	//Output: contains the current speeds and positions of any given moment of time
	const SwerveVelocities &GetCurrentVelocities() const;
private:
	std::shared_ptr<SimulatedOdometry_Internal> m_simulator;
};
	}
}