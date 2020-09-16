#pragma once
#include <memory>

//Entity 2D
//This records its current position on a 2 dimensional plane
//This also records heading orientation, so methods are provided for this

namespace Module {
	namespace Localization	{
//Using Vec2D internally
class Entity2D_internal;
class Entity2D
{
public:
	Entity2D();
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//allows entity to be stopped and reset to a position and heading
	void Reset(double X=0.0, double Y=0.0, double heading=0.0);

	//Recommended units: Linear meters per second, Angular radians per second
	//heading and linear are independent of each other
	//

	//moving entity methods:-----------------------------------------------------------------------
	//These are local to current orientation
	void SetLinearVelocity_local(double forward, double right);
	//These will travel in a given direction field centric no matter the orientation
	void SetLinearVelocity_global(double north, double east);
	//Note: 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetAngularVelocity(double clockwise);

	//Accessors:---------------------------------------------
	//Use a simple struct to keep methods of return
	struct Vector2D
	{
		double x, y;
	};
	Vector2D GetCurrentVelocity() const;  //returns the current velocity in raw global form
	Vector2D GetCurrentPosition() const;  //returns x y coordinates of current position
	double GetCurrentHeading() const;  //returns heading where 0 is north in radians
	double GetCurrentAngularVelocity() const; //radians per second

private:
	std::shared_ptr<Entity2D_internal> m_Entity2D; //implementation
};
	}
}
	