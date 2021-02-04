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
	Vec2D m_Position;
	SwerveVelocities m_Velocity;
	double m_Heading=0.0;
	std::function< SwerveVelocities ()> m_OdometryVelocityCallback = nullptr;
	std::function<double ()> m_OdometryHeadingCallback = nullptr;
	std::function<Vec2D()> m_OdometryPositionCallback = nullptr;
	bool m_SupportHeading = false;  //this needs to be explicitly set since we have to use this callback with a backup
public:
	//provide if the position odometry is supported
	bool SupportPosition() const
	{
		return m_OdometryPositionCallback != nullptr;
	}
	bool SupportHeading() const
	{
		return m_SupportHeading;
	}
	void Set_SupportHeading(bool support_heading)
	{
		m_SupportHeading = support_heading;
	}
	void Reset()
	{
		memset(&m_Velocity, 0, sizeof(SwerveVelocities));
		m_Position = Vec2D(0.0, 0.0);
		m_Heading = 0.0;
	}
	//Input: pace the updates per time-slice
	void TimeSlice(double d_time_s)
	{
		//Gather the current positions / velocities and cache them here
		if (m_OdometryVelocityCallback)
			m_Velocity = m_OdometryVelocityCallback();
		if (m_OdometryHeadingCallback)
			m_Heading = m_OdometryHeadingCallback();
		if (m_OdometryPositionCallback)
			m_Position = m_OdometryPositionCallback();
	}
	//Output
	const SwerveVelocities &GetCurrentVelocities() const
	{
		return m_Velocity;
	}
	double GetHeading() const
	{
		return m_Heading;
	}
	const Vec2D& GetPosition() const
	{
		return m_Position;
	}

	//Callbacks...
	//Odometry callback of each wheel module
	void SetOdometryVelocityCallback(std::function<SwerveVelocities ()> callback)
	{
		m_OdometryVelocityCallback = callback;
	}
	//This can be a gyro, vision, or inverse kinematics, or even a blend of each
	void SetOdometryHeadingCallback(std::function<double ()> callback)
	{
		m_OdometryHeadingCallback = callback;
	}
	void SetOdometryPositionCallback(std::function<Vec2D()> callback)
	{
		m_OdometryPositionCallback = callback;
	}
};
	}
}