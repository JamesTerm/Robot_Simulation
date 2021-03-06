#pragma once
#include <memory>
#include <functional>
#include "../../../../Base/AssetManager.h"

//Entity 2D
//This records its current position on a 2 dimensional plane
//In addition it can be moved by tele-op or AI, so methods are available for this
//This also records heading orientation, so methods are provided for this
//The properties include acceleration and deceleration attributes for trapezoidal motion profiling
//this applies to both position and orientation
//For AI, way points can be set to travel to, we also can set an intended orientation

namespace Module {
	namespace Robot	{
		namespace Physics {
//Using Vec2D internally
class MotionControl2D_internal;
class MotionControl2D
{
public:
	MotionControl2D();

	struct properties
	{
		double max_speed_linear;  //meters per second of fastest speed available
		double max_speed_angular; //radians fastest turn rate
		double max_acceleration_linear;  //meters per second square
		double max_deceleration_linear;  //these may be different since we may need to stop quicker
		double max_acceleration_angular;  //for both acceleration and deceleration
	};
	void Initialize(const Framework::Base::asset_manager *props=nullptr);

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
	void DriveToLocation(double north, double east, bool absolute=true, bool stop_at_destination=true, double max_speed = 0.0, bool can_strafe = true);
	//Use a simple struct to keep methods simple
	struct Vector2D
	{
		double x, y;
	};
	//Linker callbacks:  This will link a type of entity object to be updated with the output of this module to the input
	//of the adjoining module.  This is optional (important) because each module should be completely independent of other
	//classes or modules.  These are called on the time slice
	void Set_UpdateGlobalVelocity(std::function<void(const Vector2D &new_velocity)> callback);
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback);
	//These are needed for driven functions, optional as well because we have internal method if its not outsourced
	//The dependency on the position and heading is needed for set-point computations
	void Set_GetCurrentPosition(std::function <Vector2D()> callback);  //returns x y coordinates of current position
	void Set_GetCurrentHeading(std::function <double()> callback);  //returns heading where 0 is north in radians

	//Accessors:---------------------------------------------
	bool GetIsDrivenLinear() const; //returns if its driving or driven (false by default)
	bool GetIsVelocityGlobal() const;  //returns last method called (local / false by default)
	bool GetIsDrivenAngular() const; //returns if its driving or driven (false by default)
	Vector2D Get_SetVelocity_local() const;  //Returns last velocity requested
	Vector2D Get_SetVelocity_global() const;  //these are recorded separately
	double Get_IntendedOrientation() const; //Get set point heading where 0 is north in radians
	Vector2D GetCurrentVelocity() const;  //returns the current velocity in raw global form
	Vector2D GetCurrentPosition() const;  //returns x y coordinates of current position
	double GetCurrentHeading() const;  //returns heading where 0 is north in radians
	double GetCurrentAngularVelocity() const; //radians per second

private:
	std::shared_ptr<MotionControl2D_internal> m_MotionControl2D; //implementation
};
		}
	}
}
	