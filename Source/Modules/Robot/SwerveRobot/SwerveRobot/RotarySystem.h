#pragma once
#include <memory>
#include <functional>

//All rotary systems can fall into either a velocity controlled (like a shooter wheel, or drive)
//or position controlled like an arm or turret, both can be open or closed loop, which can be controlled by the client
//where open loop has no odometry callback.

namespace Module {
	namespace Robot {

class Rotary_Position_Control;
class RotarySystem_Position
{
public:
	//Recommended units: radians or meters for linear
	//This allows setting the desired position
	//If this is angular it is always absolute, so relative can be managed by the caller
	//like written before where 0 is north, pi/2 east, etc.
	void SetPosition(double position);
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//set internal caching to a position, or PID variables as needed
	void Reset(double position = 0.0);
	//Output callback of new voltage to be set to output device (e.g. WPI, simulated odometry etc...)
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback);
	//provide access to the odometry for closed loops (e.g. potentiometer), use nullptr for open loops
	void SetOdometryCallback(std::function<double()> callback);
private:
	std::shared_ptr<RotarySystem_Position> m_rotary_system;
};

class Rotary_Velocity_Control;
class RotarySystem_Velocity
{
public:
	//Recommended units: radians per second
	//Note: For angular 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetVelocity(double rate);
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//clear velocity caching, or PID variables as needed
	void Reset();
	//Output
	//Output callback of new voltage to be set to output device (e.g. WPI, simulated odometry etc...)
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback);
	//provide access to the odometry for closed loops (e.g. encoder), use nullptr for open loops
	void SetOdometryCallback(std::function<double()> callback);
private:
	std::shared_ptr<Rotary_Velocity_Control> m_rotary_system;
};


	}
}