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

#include "MotionControl2D.h"

namespace Module {
	namespace Robot	{
		namespace Simple {

#pragma region _Motion Control 2D Internal_

__inline MotionControl2D::Vector2D as_vector_2d(Vec2D input)
{
	MotionControl2D::Vector2D ret = { input[0],input[1] };
	return ret;
}

__inline  Vec2D as_Vec2d( MotionControl2D::Vector2D input)
{
	Vec2D ret(input.x,input.y);
	return ret;
}

class MotionControl2D_internal
{
private:
	#pragma region _member varibles_
	using properties = MotionControl2D::properties;
	//provide some meaningful defaults
	properties m_properties=
	{
		Feet2Meters(12.0),  //meters per second of fastest speed available
		Pi, //radians fastest turn rate
		5,  //max acceleration meters per second square
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

	std::function<void(const MotionControl2D::Vector2D &new_velocity)> m_ExternSetVelocity=nullptr;
	std::function<void(double new_velocity)> m_ExternSetHeadingVelocity=nullptr;
	std::function <MotionControl2D::Vector2D()> m_ExternGetCurrentPosition=nullptr;
	std::function <double()> m_ExternGetCurrentHeading=nullptr;

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
	void process_update_velocity(Vec2D velocity, double d_time_s)
	{
		m_current_velocity = velocity;
		//send this velocity to entity if it exists
		if (m_ExternSetVelocity)
			m_ExternSetVelocity(as_vector_2d(m_current_velocity));
		//If we don't have the callbacks for both set and get... we'll update our own default implementation
		if ((!m_ExternGetCurrentPosition) || (!m_ExternSetVelocity))
		{
			//from current velocity we can update the position
			m_current_position += m_current_velocity * d_time_s;
		}
	}
	void process_update_angular_velocity(double velocity, double d_time_s)
	{
		m_current_angular_velocity = velocity;
		//send this velocity to entity if it exists
		if (m_ExternSetHeadingVelocity)
			m_ExternSetHeadingVelocity(m_current_angular_velocity);
		//If we don't have the callbacks for both set and get... we'll update our own default implementation
		if ((!m_ExternGetCurrentHeading) || (!m_ExternSetHeadingVelocity))
		{
			//from current velocity we can update the position
			m_current_heading += m_current_angular_velocity * d_time_s;
			//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
			m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging

		}
	}
	void process_slice(double d_time_s)
	{
		//apply linear velocity first, then angular

		//For linear determine if we are driving or driven
		if (m_is_linear_driven == false)
		{
			//First break apart current velocity into magnitude and direction
			Vec2D normlized_current = m_current_velocity;
			const double current_magnitude = normlized_current.normalize();  //observe for diagnostics
			const Vec2D DesiredVelocity =(m_is_velocity_global)? m_requested_global : LocalToGlobal(GetCurrentHeading(),m_requested_local);
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
			process_update_velocity(normalized_request * adjusted_magnitude,d_time_s);
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
				process_update_angular_velocity(direction_to_use * adjusted_magnitude,d_time_s);
			}
			else
			{
			}
		}
	}
	void process_bypass(double d_time_s)
	{
		//This is useful for diagnostics, but also shows the input to output conversion used
		if (m_is_linear_driven == false)
		{
			process_update_velocity((m_is_velocity_global) ? m_requested_global : LocalToGlobal(GetCurrentHeading(), m_requested_local), d_time_s);
			process_update_angular_velocity(m_requested_angular_velocity, d_time_s);
		}
	}
public:
	void TimeSlice(double d_time_s)
	{
		//keep public small and compact
		process_slice(d_time_s);
		//process_bypass(d_time_s);
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
		m_requested_heading = absolute?intended_orientation:GetCurrentHeading()+intended_orientation;
		m_is_angular_driven = true;
	}
	void DriveToLocation(double north, double east, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true)
	{
		m_requested_position[0] = east, m_requested_position[1] = north;
		m_stop_at_waypoint = stop_at_destination;
		m_driven_max_speed = max_speed;
		m_can_strafe = can_strafe;
	}
	#pragma region _Accessors_
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
		if (m_ExternGetCurrentPosition)
			return as_Vec2d(m_ExternGetCurrentPosition());
		else
			return m_current_position;
	}
	double GetCurrentHeading() const
	{
		if (m_ExternGetCurrentHeading)
			return m_ExternGetCurrentHeading();
		else
			return m_current_heading;
	}
	double GetCurrentAngularVelocity() const
	{
		return m_current_angular_velocity;
	}
	#pragma endregion
	#pragma region _Callbacks_
	//Optional Linker callbacks:
	void Set_UpdateGlobalVelocity(std::function<void(const MotionControl2D::Vector2D &new_velocity)> callback)
	{
		m_ExternSetVelocity = callback;
	}
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
	{
		m_ExternSetHeadingVelocity = callback;
	}
	void Set_GetCurrentPosition(std::function <MotionControl2D::Vector2D()> callback)
	{
		m_ExternGetCurrentPosition = callback;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_ExternGetCurrentHeading = callback;
	}
	#pragma endregion
};
#pragma endregion

#pragma region _wrapper methods_
MotionControl2D::MotionControl2D()
{
	m_MotionControl2D = std::make_shared<MotionControl2D_internal>();
}
void MotionControl2D::Initialize(const Framework::Base::asset_manager *props)
{
	//TODO Translate properties here
	//m_MotionControl2D->SetProperties(props);
}
void MotionControl2D::TimeSlice(double d_time_s)
{
	m_MotionControl2D->TimeSlice(d_time_s);
}
void MotionControl2D::Reset(double X, double Y, double heading)
{
	m_MotionControl2D->Reset(X, Y, heading);
}

//Tele-op methods:-----------------------------------------------------------------------
void MotionControl2D::SetLinearVelocity_local(double forward, double right)
{
	m_MotionControl2D->SetLinearVelocity_local(forward, right);
}
void MotionControl2D::SetLinearVelocity_global(double north, double east)
{
	m_MotionControl2D->SetLinearVelocity_global(north, east);
}
void MotionControl2D::SetAngularVelocity(double clockwise)
{
	m_MotionControl2D->SetAngularVelocity(clockwise);
}
//AI methods: ---------------------------------------------------------------------------
void MotionControl2D::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_MotionControl2D->SetIntendedOrientation(intended_orientation, absolute);
}
void MotionControl2D::DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_MotionControl2D->DriveToLocation(north, east, stop_at_destination, max_speed, can_strafe);
}

//Accessors:---------------------------------------------
bool MotionControl2D::GetIsDrivenLinear() const
{
	return m_MotionControl2D->GetIsDrivenLinear();
}
bool MotionControl2D::GetIsVelocityGlobal() const
{
	return m_MotionControl2D->GetIsVelocityGlobal();
}
bool MotionControl2D::GetIsDrivenAngular() const
{
	return m_MotionControl2D->GetIsDrivenAngular();
}

MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_local() const
{
	return as_vector_2d(m_MotionControl2D->Get_SetVelocity_local());
}
MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_global() const
{
	return as_vector_2d(m_MotionControl2D->Get_SetVelocity_global());
}
double MotionControl2D::Get_IntendedOrientation() const
{
	return m_MotionControl2D->Get_IntendedOrientation();
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentVelocity() const
{
	return as_vector_2d(m_MotionControl2D->GetCurrentVelocity());
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentPosition() const
{
	return as_vector_2d(m_MotionControl2D->GetCurrentPosition());
}
double MotionControl2D::GetCurrentHeading() const
{
	return m_MotionControl2D->GetCurrentHeading();
}
double MotionControl2D::GetCurrentAngularVelocity() const
{
	return m_MotionControl2D->GetCurrentAngularVelocity();
}
//Optional Linker callbacks:
void MotionControl2D::Set_UpdateGlobalVelocity(std::function<void(const Vector2D &new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateGlobalVelocity(callback);
}
void MotionControl2D::Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateHeadingVelocity(callback);
}
void MotionControl2D::Set_GetCurrentPosition(std::function <Vector2D()> callback)
{
	m_MotionControl2D->Set_GetCurrentPosition(callback);
}
void MotionControl2D::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_MotionControl2D->Set_GetCurrentHeading(callback);
}

#pragma endregion

}}}