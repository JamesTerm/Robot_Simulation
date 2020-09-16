#pragma once
#include <memory>

//Entity 2D
//This records its current position on a 2 dimensional plane
//In addition it can be moved by tele-op or AI, so methods are available for this
//This also records heading orientation, so methods are provided for this
//The properties include acceleration and deceleration attributes for trapezoidal motion profiling
//this applies to both position and orientation
//For AI, way points can be set to travel to, we also can set an intended orientation

namespace Module {
	namespace Localization	{
//Using Vec2D internally
class Entity1D_internal;
class Entity1D
{
public:
	Entity1D();

	struct properties
	{
		double max_speed; //radians fastest turn rate
		double max_acceleration;  //meters per second square
		double max_deceleration;  //these may be different especially on manipulators that fall faster due to gravity
		bool is_linear;  //Most of the time we are angular and will normalize the rotation
	};
	void SetProperties(const properties &props);

	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//allows entity to be stopped and reset to a position and heading
	void Reset(double position=0.0);

	//Recommended units: Linear meters per second, Angular radians per second
	//There is a hidden driving or driven mode depending on which method was last called So if you set a way point and you call 
	//set velocity you change from driving to driven, and vise versa

	//Note: For angular 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetVelocity(double rate);
	//This allows setting the desired position
	//If this is angular it is always absolute, so relative can be managed by the caller
	//like written before where 0 is north, pi/2 east, etc.
	void SetPosition(double position);
	//Accessors:---------------------------------------------
	bool GetIsDriven() const; //returns if its driving or driven (false by default)
	double Get_SetVelocity() const;  //Returns last velocity requested
	double Get_SetPosition() const; //Get set point; for angular, heading where 0 is north in radians
	double GetCurrentVelocity() const;  //returns the current velocity
	double GetCurrentPosition() const;  //returns the current position; for angular, heading where 0 is north in radians

private:
	std::shared_ptr<Entity1D_internal> m_Entity1D; //implementation
};
	}
}
	