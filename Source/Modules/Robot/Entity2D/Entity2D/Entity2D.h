#pragma once

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
class Entity2D_internal;
class Entity2D
{
public:
	Entity2D();

	struct properties
	{
		double max_speed_linear;  //meters per second of fastest speed available
		double max_speed_angular; //radians fastest turn rate
		double max_acceleration_linear;  //meters per second square
		double max_deceleration_linear;  //these may be different since we may need to stop quicker
		double max_acceleration_angular;  //for both acceleration and deceleration
	};
	void SetProperties(const properties &props);

	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//allows entity to be stopped and reset to a position and heading
	void Reset(double X=0.0, double Y=0.0, double heading=0.0);

	//Recommended units: Linear meters per second, Angular radians per second
	//For most of the methods the heading and linear are independent of each other and for each there is a hidden driving
	//or driven mode depending on which method was last called So if you set a way point (drive to location) and you call 
	//set linear velocity you change from driving to driven for linear only, and vise versa
	//
	//For non-strafe robots, 
	//never use the global method and SetIntendedOrientation works only in local
	//velocity, drive to location has a parameter to indicate this

	//Tele-op methods:-----------------------------------------------------------------------
	//These are local to current orientation
	void SetLinearVelocity_local(double forward, double right);
	//These will travel in a given direction field centric no matter the orientation
	void SetLinearVelocity_global(double north, double east);
	//Note: 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetAngularVelocity(double clockwise);
	//AI methods: ---------------------------------------------------------------------------
	//This allows setting the desired heading directly either relative to the current heading or absolute
	//If relative positive is a clockwise offset from where it is; otherwise the direction is
	//like written before where 0 is north, pi/2 east, etc.
	void SetIntendedOrientation(double intended_orientation, bool absolute = true);
	//Drives to a location at given by coordinates at max speed given
	//It can either stop at location or continue to drive past it once it hits (allows for path driving)
	//max speed is optional where 0.0 means the max speed in properties
	//if can strafe is true caller manages its own orientation as it deems fit; otherwise if it can't strafe
	//it must manage the orientation to always drive forward in the direction toward the way point
	void DriveToLocation(double north, double east, bool stop_at_destination=true, double max_speed = 0.0, bool can_strafe = true);

	//Accessors:---------------------------------------------
	bool GetIsDrivenLinear() const; //returns if its driving or driven (false by default)
	bool GetIsVelocityGlobal() const;  //returns last method called (local / false by default)
	bool GetIsDrivenAngular() const; //returns if its driving or driven (false by default)
	//Use a simple struct to keep methods of return
	struct Vector2D
	{
		double x, y;
	};
	Vector2D Get_SetVelocity_local() const;  //Returns last velocity requested
	Vector2D Get_SetVelocity_global() const;  //these are recorded separately
	double Get_IntendedOrientation() const; //Get set point heading where 0 is north in radians
	Vector2D GetCurrentVelocity() const;  //returns the current velocity in raw global form
	Vector2D GetCurrentPosition() const;  //returns x y coordinates of current position
	double GetCurrentHeading() const;  //returns heading where 0 is north in radians

private:
	std::shared_ptr<Entity2D_internal> m_Entity2D; //implementation
};
	}
}
	