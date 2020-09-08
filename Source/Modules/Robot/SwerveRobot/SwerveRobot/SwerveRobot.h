#pragma once
#include <memory>
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"

namespace Module {
	namespace Robot {

class SwerveRobot_Internal;


class SwerveRobot
{
public:
	//Handle property initialization
	void Init();
	//These are local to current orientation
	void SetLinearVelocity_local(double forward, double right);
	//Note: 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetAngularVelocity(double clockwise);
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//allows entity to be stopped and reset to a position and heading
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0);

	//Linker callbacks:  This will link a type of entity object to be updated with the output of this module to the input
	//of the adjoining module.  This is optional (important) because each module should be completely independent of other
	//classes or modules.  These are called on the time slice
	void Set_UpdateGlobalVelocity(std::function<void(const Vec2D &new_velocity)> callback);
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback);
private:
	std::shared_ptr<SwerveRobot_Internal> m_SwerveRobot;
};
	}
}