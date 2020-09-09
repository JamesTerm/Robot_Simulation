#pragma once
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace Module {
	namespace Robot {
class SwerveManagement
{
private:
	SwerveVelocities m_Velocity;
	std::function< SwerveVelocities ()> m_OdometryCallback=nullptr;
public:
	//Input
	void InterpolateVelocities(const SwerveVelocities &Velocities)
	{
		//TODO
		m_Velocity = Velocities;
	}
	//Output
	const SwerveVelocities &GetIntendedVelocities() const
	{
		return m_Velocity;
	}
	//Thru_put
	const SwerveVelocities &GetOdometryVelocities() const
	{
		if (m_OdometryCallback)
			return m_OdometryCallback();
		else
		{
			return m_Velocity;
		}
	}

	//Odometry callback of each wheel module
	void SetOdometryCallback(std::function< SwerveVelocities ()> callback)
	{
		m_OdometryCallback = callback;
	}
};
	}
}