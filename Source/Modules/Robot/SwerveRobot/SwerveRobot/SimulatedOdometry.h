#pragma once
#include <memory>
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/AssetManager.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace Module {
	namespace Robot {

class SimulatedOdometry_Internal;

class SimulatedOdometry
{
public:
	SimulatedOdometry();
	void Init(const Framework::Base::asset_manager *props=nullptr);
	void ResetPos();
	void Shutdown();
	//Input: for simulation only, grab the voltages from each rotary system
	void SetVoltageCallback(std::function<SwerveVelocities ()> callback);
	//Run the simulation time-slice
	void TimeSlice(double d_time_s);
	//Output: contains the current speeds and positions of any given moment of time
	const SwerveVelocities &GetCurrentVelocities() const;
	//Output: optional advanced sensors for localization
	//If our simulation supports providing localization through vision
	bool Sim_SupportVision() const;
	//This gives best guess, may simulate latency or delay in updates
	Vec2D Vision_GetCurrentPosition() const;
	//If our simulation supports providing localization through gyro magnetometer blend
	bool Sim_SupportHeading() const;
	//This should be very accurate not latent but may be subjected to simulate a glitch in shifted heading
	double GyroMag_GetCurrentHeading() const;
private:
	std::shared_ptr<SimulatedOdometry_Internal> m_simulator;
};
	}
}