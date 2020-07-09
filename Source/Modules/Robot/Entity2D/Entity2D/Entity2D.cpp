#include "../../../../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"

// Standard C++ support
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <bitset>
#include <map>
#include <algorithm>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>

#include "Entity2D.h"

namespace Module {
	namespace Localization	{

#pragma region _Entity 2D Internal_

__inline Vec2D LocalToGlobal(double Heading, const Vec2D &LocalVector)
{
	return Vec2D(sin(Heading)*LocalVector[1] + cos(-Heading)*LocalVector[0],
		cos(Heading)*LocalVector[1] + sin(-Heading)*LocalVector[0]);
}

__inline Vec2D GlobalToLocal(double Heading, const Vec2D &GlobalVector)
{
	return Vec2D(sin(-Heading)*GlobalVector[1] + cos(Heading)*GlobalVector[0],
		cos(-Heading)*GlobalVector[1] + sin(Heading)*GlobalVector[0]);
}

class Entity2D_internal
{
private:
	#pragma region _member varibles_
	using properties = Entity2D::properties;
	//provide some meaningful defaults
	properties m_properties=
	{
		Feet2Meters(12.0),  //meters per second of fastest speed available
		Pi, //radians fastest turn rate
		5,  //max acceleraton meters per second square
		5,  //these may be different since we may need to stop quicker
		Pi2  //for both acceleration and deceleration
	};

	Vec2D m_requested_local;
	Vec2D m_requested_global;
	Vec2D m_current_velocity;
	Vec2D m_requested_position;
	Vec2D m_current_position;
	double m_requested_heading=0.0;  //always absolute relative method solves on the spot
	double m_requested_angular_velocity=0.0;
	double m_current_heading=0.0;
	double m_driven_max_speed=0.0;
	//keep note of last modes
	bool m_is_linear_driven = false;
	bool m_is_angular_driven = false;
	bool m_is_velocity_global = false;
	bool m_stop_at_waypoint = true;
	bool m_can_strafe = true;
	#pragma endregion
	void process_slice(double d_time_s)
	{
		//apply linear velocity first, then anglar

		//For linear determine if we are driving or driven
		if (m_is_linear_driven == false)
		{
			//First break apart current velocity into magnitude and direction
			Vec2D normlized_current = m_current_velocity;
			const double current_magnitude = normlized_current.normalize();  //observe for diagnostics
			const Vec2D DesiredVelocity =(m_is_velocity_global)? m_requested_global : LocalToGlobal(m_current_heading,m_requested_local);
			//break this apart
			Vec2D normalized_request = DesiredVelocity;
			double request_magnitute = normalized_request.normalize();  //always use this direction
			//use current direction if the requested magnitude is zero (because there is no direction then)
			if (request_magnitute == 0.0)
				normalized_request = normlized_current;
			double adjusted_magnitude = current_magnitude;
			//adjust the velocity to match request
			if (current_magnitude < request_magnitute)
			{
				//accelerate and clip as needed
				 adjusted_magnitude = std::min(
					current_magnitude + (m_properties.max_acceleration_linear * d_time_s), m_properties.max_speed_linear
					);
			}
			else if (current_magnitude > request_magnitute)
			{
				//decelarate and clip as needed
				 adjusted_magnitude = std::max(
					current_magnitude - (m_properties.max_deceleration_linear * d_time_s), 0.0
				);
			}
			//now update the current velocity
			m_current_velocity = normalized_request * adjusted_magnitude;
			//from current velocity we can update the position
			m_current_position += m_current_velocity;
		}
		//TODO way point here

		//For angular determine if we are driving or driven
		const bool is_waypoint_managing_heading = m_is_linear_driven && m_can_strafe == false;
		if (!is_waypoint_managing_heading)
		{
			if (m_is_angular_driven == false)
			{

			}
			//TODO update intended orientation
		}
	}
public:
	void TimeSlice(double d_time_s)
	{
		process_slice(d_time_s);  //keep public small and compact
	}

	void SetProperties(const properties &props)
	{
		m_properties = props;
	}
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_requested_local = m_requested_global = m_current_velocity = Vec2D(0.0, 0.0);
		m_current_position[0] = X;
		m_current_position[1] = Y;
		m_requested_heading = m_requested_angular_velocity = 0.0;
		m_current_heading = heading;
		//optional update last modes for consistency
		m_is_linear_driven = m_is_angular_driven = m_is_velocity_global = false;
	}

	//Tele-op methods:-----------------------------------------------------------------------
	void SetLinearVelocity_local(double forward, double right)
	{
		//for ease of debugging, keep this in local here (translate in timeslice)
		m_requested_local[0] = right;
		m_requested_local[1] = forward;
	}
	void SetLinearVelocity_global(double north, double east)
	{
		m_requested_global[0] = east;
		m_requested_global[1] = north;
	}
	void SetAngularVelocity(double clockwise)
	{
		m_requested_angular_velocity = clockwise;
	}
	//AI methods: ---------------------------------------------------------------------------
	void SetIntendedOrientation(double intended_orientation, bool absolute = true)
	{
		m_requested_heading = absolute?intended_orientation:m_current_heading+intended_orientation;
	}
	void DriveToLocation(double north, double east, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true)
	{
		m_requested_position[0] = east, m_requested_position[1] = north;
		m_stop_at_waypoint = stop_at_destination;
		m_driven_max_speed = max_speed;
		m_can_strafe = can_strafe;
	}

	//Accessors:---------------------------------------------
	bool GetIsDrivenLinear() const
	{
		return m_is_linear_driven;
	}
	bool GetIsVelocityGlobal() const
	{
		return m_is_velocity_global;
	}
	bool GetIsDrivenAngular() const
	{
		return m_is_angular_driven;
	}
	Vec2D Get_SetVelocity_local() const
	{
		return m_requested_local;
	}
	Vec2D Get_SetVelocity_global() const
	{
		return m_requested_global;
	}
	Vec2D GetCurrentVelocity() const
	{
		return m_current_velocity;
	}
	Vec2D GetCurrentPosition() const
	{
		return m_current_position;
	}
	double GetCurrentHeading() const
	{
		return m_current_heading;
	}
};
#pragma endregion

#pragma region _wrapper methods_
Entity2D::Entity2D()
{
	m_Entity2D = std::make_shared<Entity2D_internal>();
}
void Entity2D::SetProperties(const properties &props)
{
	m_Entity2D->SetProperties(props);
}
void Entity2D::TimeSlice(double d_time_s)
{
	m_Entity2D->TimeSlice(d_time_s);
}
void Entity2D::Reset(double X, double Y, double heading)
{
	m_Entity2D->Reset(X, Y, heading);
}

//Tele-op methods:-----------------------------------------------------------------------
void Entity2D::SetLinearVelocity_local(double forward, double right)
{
	m_Entity2D->SetLinearVelocity_local(forward, right);
}
void Entity2D::SetLinearVelocity_global(double north, double east)
{
	m_Entity2D->SetLinearVelocity_global(north, east);
}
void Entity2D::SetAngularVelocity(double clockwise)
{
	m_Entity2D->SetAngularVelocity(clockwise);
}
//AI methods: ---------------------------------------------------------------------------
void Entity2D::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_Entity2D->SetIntendedOrientation(intended_orientation, absolute);
}
void Entity2D::DriveToLocation(double north, double east, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_Entity2D->DriveToLocation(north, east, stop_at_destination, max_speed, can_strafe);
}

//Accessors:---------------------------------------------
bool Entity2D::GetIsDrivenLinear() const
{
	return m_Entity2D->GetIsDrivenLinear();
}
bool Entity2D::GetIsVelocityGlobal() const
{
	return m_Entity2D->GetIsVelocityGlobal();
}
bool Entity2D::GetIsDrivenAngular() const
{
	return m_Entity2D->GetIsDrivenAngular();
}
__inline Entity2D::Vector2D as_vector_2d(Vec2D input)
{
	Entity2D::Vector2D ret = { input[0],input[1] };
	return ret;
}

Entity2D::Vector2D Entity2D::Get_SetVelocity_local() const
{
	return as_vector_2d(m_Entity2D->Get_SetVelocity_local());
}
Entity2D::Vector2D Entity2D::Get_SetVelocity_global() const
{
	return as_vector_2d(m_Entity2D->Get_SetVelocity_global());
}
Entity2D::Vector2D Entity2D::GetCurrentVelocity() const
{
	return as_vector_2d(m_Entity2D->GetCurrentVelocity());
}
Entity2D::Vector2D Entity2D::GetCurrentPosition() const
{
	return as_vector_2d(m_Entity2D->GetCurrentPosition());
}
double Entity2D::GetCurrentHeading() const
{
	return m_Entity2D->GetCurrentHeading();
}
#pragma endregion

}}