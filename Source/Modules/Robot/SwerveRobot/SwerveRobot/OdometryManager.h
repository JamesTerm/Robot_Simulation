#pragma once
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"

//This is nothing more than a proxy that either links to WPI for readings or the simulated odometry for its readings
//It does offer up the velocities in one group at one time making it easy to access for multiple clients

namespace Module {
	namespace Robot {
class OdometryManager
{
private:
	SwerveVelocities m_Velocity;
	double m_Heading=0.0;
	std::function<const SwerveVelocities &()> m_OdometryCallback = nullptr;
	std::function<double ()> m_OdometryHeadingCallback = nullptr;
public:
	//Input: pace the updates per time-slice
	void TimeSlice(double d_time_s)
	{
		//Gather the current positions / velocities and cache them here
		if (m_OdometryCallback)
			m_Velocity = m_OdometryCallback();
		if (m_OdometryHeadingCallback)
			m_Heading = m_OdometryHeadingCallback();
	}
	//Output
	const SwerveVelocities &GetIntendedVelocities() const
	{
		return m_Velocity;
	}
	const double GetHeading() const
	{
		return m_Heading;
	}

	//Odometry callback of each wheel module
	void SetOdometryCallback(std::function<const SwerveVelocities &()> callback)
	{
		m_OdometryCallback = callback;
	}
	//This can be a gyro, vision, or inverse kinematics, or even a blend of each
	void SetOdometryHeadingCallback(std::function<double ()> callback)
	{
		m_OdometryHeadingCallback = callback;
	}

};
	}
}