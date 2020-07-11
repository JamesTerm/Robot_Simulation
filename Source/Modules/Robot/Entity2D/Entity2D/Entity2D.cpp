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
	double m_current_angular_velocity = 0.0;
	double m_current_heading=0.0;
	double m_driven_max_speed=0.0;
	//keep note of last modes
	bool m_is_linear_driven = false;
	bool m_is_angular_driven = false;
	bool m_is_velocity_global = false;
	bool m_stop_at_waypoint = true;
	bool m_can_strafe = true;
	#pragma endregion
	#pragma region _normalize rotation_
	inline void NormalizeRotation(double &Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
	}

	inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}

	inline double NormalizeRotation_HalfPi(double Orientation)
	{
		if (Orientation > PI_2)
			Orientation -= M_PI;
		else if (Orientation < -PI_2)
			Orientation += M_PI;
		return Orientation;
	}

	inline double SaturateRotation(double Rotation)
	{
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation = M_PI;
		else if (Rotation < -M_PI)
			Rotation = -M_PI;
		return Rotation;
	}

	#pragma endregion
	void process_slice(double d_time_s)
	{
		//apply linear velocity first, then angular

		//For linear determine if we are driving or driven
		if (m_is_linear_driven == false)
		{
			//First break apart current velocity into magnitude and direction
			Vec2D normlized_current = m_current_velocity;
			const double current_magnitude = normlized_current.normalize();  //observe for diagnostics
			const Vec2D DesiredVelocity =(m_is_velocity_global)? m_requested_global : LocalToGlobal(m_current_heading,m_requested_local);
			//break this apart
			Vec2D normalized_request = DesiredVelocity;
			double request_magnitude = normalized_request.normalize();  //always use this direction
			//This shouldn't be needed but just in case
			request_magnitude = std::max(std::min(request_magnitude, m_properties.max_speed_linear), 0.0);
			//use current direction if the requested magnitude is zero (because there is no direction then)
			if (request_magnitude == 0.0)
				normalized_request = normlized_current;
			double adjusted_magnitude = current_magnitude;
			//adjust the velocity to match request
			const double acc_increment = (m_properties.max_acceleration_linear * d_time_s);
			const double dec_increment = -(m_properties.max_deceleration_linear * d_time_s);
			//TODO while a 1D I can get perfect handling of switching direction stresses, I can't do that here
			//for now I'll let this go as it will do the correct math most of the time, and will address this
			//again once I start working with feedback
			//Check if we can go a full increment in either direction
			if (current_magnitude + acc_increment < request_magnitude)
				 adjusted_magnitude += acc_increment; //accelerate and clip as needed
			else if (current_magnitude + dec_increment > request_magnitude)
				 adjusted_magnitude += dec_increment; //decelerate and clip as needed
			else if (fabs(current_magnitude - request_magnitude) < acc_increment)
			{
				//we are close enough to the requested to just become it
				adjusted_magnitude = request_magnitude;
			}

			//now update the current velocity
			m_current_velocity = normalized_request * adjusted_magnitude;
			//from current velocity we can update the position
			m_current_position += m_current_velocity * d_time_s;
		}
		//TODO way point here

		//For angular determine if we are driving or driven
		const bool is_waypoint_managing_heading = m_is_linear_driven && m_can_strafe == false;
		if (!is_waypoint_managing_heading)
		{
			if (m_is_angular_driven == false)
			{
				//Very similar trapezoidal profile as with the velocity except we are 1D, so the direction is easier to manage
				//First break apart current heading into magnitude and direction
				const double current_magnitude = fabs(m_current_angular_velocity);
				const double current_direction = current_magnitude>0.0?current_magnitude / m_current_angular_velocity:0.0;  //order does not matter
				double request_magnitude = fabs(m_requested_angular_velocity);
				double direction_to_use = request_magnitude>0.0? request_magnitude / m_requested_angular_velocity : 0.0;
				//This shouldn't be needed but just in case (must be after direction_to_use)
				request_magnitude = std::max(std::min(request_magnitude, m_properties.max_speed_linear), 0.0);
				//use current direction if the requested magnitude is zero (because there is no direction then)
				if (request_magnitude == 0.0)
					direction_to_use = current_direction;
				double adjusted_magnitude = current_magnitude;
				//adjust the velocity to match request
				const double acc_increment = (m_properties.max_acceleration_angular * d_time_s);
				const double dec_increment = -(m_properties.max_acceleration_angular * d_time_s);
				//Check the signs
				if (current_direction * direction_to_use >= 0.0)
				{
					//Check if we can go a full increment in either direction
					if (current_magnitude + acc_increment < request_magnitude)
						adjusted_magnitude += acc_increment; //accelerate and clip as needed
					else if (current_magnitude + dec_increment > request_magnitude)
						adjusted_magnitude += dec_increment; //decelerate and clip as needed
					else if (fabs(current_magnitude - request_magnitude) < acc_increment)
					{
						//we are close enough to the requested to just become it
						adjusted_magnitude = request_magnitude;
					}
				}
				else
				{
					//signs are different, so the checking is as well
					//To keep this easy to debug we'll preserve current_magnitude, and create a combined one
					const double combined_magnitude = current_magnitude + request_magnitude;
					if (combined_magnitude < acc_increment)
						adjusted_magnitude = request_magnitude;  //close enough; otherwise...
					else
					{
						adjusted_magnitude += dec_increment;  //always decelerate when going towards opposite sign
						direction_to_use = current_direction;  //always keep the sign
					}
				}

				//now update the current velocity,  Note: this can go beyond the boundary of 2 pi
				m_current_angular_velocity = direction_to_use * adjusted_magnitude;
				//from current velocity we can update the position
				m_current_heading += m_current_angular_velocity * d_time_s;
				//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
				m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging
			}
			else
			{
			}
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
		m_requested_heading = m_requested_angular_velocity = m_current_angular_velocity=0.0;
		m_current_heading = heading;
		//optional update last modes for consistency
		m_is_linear_driven = m_is_angular_driven = m_is_velocity_global = false;
	}

	//Tele-op methods:-----------------------------------------------------------------------
	void SetLinearVelocity_local(double forward, double right)
	{
		//for ease of debugging, keep this in local here (translate in time slice)
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
		m_is_angular_driven = false;
	}
	//AI methods: ---------------------------------------------------------------------------
	void SetIntendedOrientation(double intended_orientation, bool absolute = true)
	{
		m_requested_heading = absolute?intended_orientation:m_current_heading+intended_orientation;
		m_is_angular_driven = true;
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
	double Get_IntendedOrientation() const
	{
		return m_requested_heading;
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
	double GetCurrentAngularVelocity() const
	{
		return m_current_angular_velocity;
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
double Entity2D::Get_IntendedOrientation() const
{
	return m_Entity2D->Get_IntendedOrientation();
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
double Entity2D::GetCurrentAngularVelocity() const
{
	return m_Entity2D->GetCurrentAngularVelocity();
}
#pragma endregion

}}