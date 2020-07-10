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

#include "Entity1D.h"

namespace Module {
	namespace Localization {

#pragma region _Entity 2D Internal_


class Entity1D_internal
{
private:
	#pragma region _member varibles_
	using properties = Entity1D::properties;
	//provide some meaningful defaults
	properties m_properties =
	{
		Pi, //radians fastest turn rate
		Pi2,  //acceleration
		Pi2,  //deceleration
		false //angular by default
	};

	double m_requested_position = 0.0;
	double m_requested_velocity = 0.0;
	double m_current_velocity = 0.0;
	double m_current_position = 0.0;
	double m_driven_max_speed = 0.0;
	//keep note of last modes
	bool m_is_driven = false;
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
		if (m_is_driven == false)
		{
			//Very similar trapezoidal profile as with the velocity except we are 1D, so the direction is easier to manage
			//First break apart current heading into magnitude and direction
			const double current_magnitude = fabs(m_current_velocity);
			const double current_direction = current_magnitude > 0.0 ? current_magnitude / m_current_velocity : 0.0;  //order does not matter
			const double request_magnitude = fabs(m_requested_velocity);
			double direction_to_use = request_magnitude > 0.0 ? request_magnitude / m_requested_velocity : 0.0;
			//use current direction if the requested magnitude is zero (because there is no direction then)
			if (request_magnitude == 0.0)
				direction_to_use = current_direction;
			double adjusted_magnitude = current_magnitude;
			//adjust the velocity to match request
			if (current_magnitude < request_magnitude)
			{
				//accelerate and clip as needed
				adjusted_magnitude = std::min(
					current_magnitude + (m_properties.max_acceleration * d_time_s), m_properties.max_speed
				);
			}
			else if (current_magnitude > request_magnitude)
			{
				//decelerate and clip as needed
				adjusted_magnitude = std::max(
					current_magnitude - (m_properties.max_deceleration * d_time_s), 0.0
				);
			}
			//now update the current velocity,  Note: this can go beyond the boundary of 2 pi
			m_current_velocity = direction_to_use * adjusted_magnitude;
			//from current velocity we can update the position
			m_current_position += m_current_velocity * d_time_s;
			if (!m_properties.is_linear)
			{
				//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
				m_current_position = NormalizeRotation2(m_current_position);  //written out for ease of debugging
			}
		}
		else
		{
			//TODO
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
	void Reset(double position = 0.0)
	{
		m_requested_position = m_requested_velocity = m_current_velocity = 0.0;
		m_current_position = position;
		//optional update last modes for consistency
		m_is_driven = false;
	}

	void SetVelocity(double rate)
	{
		m_requested_velocity = rate;
		m_is_driven = false;
	}
	void SetPosition(double position)
	{
		m_requested_position =  position;
		m_is_driven = true;
	}

	//Accessors:---------------------------------------------
	bool GetIsDriven() const
	{
		return m_is_driven;
	}
	double Get_SetVelocity() const
	{
		return m_requested_velocity;
	}
	double Get_SetPosition() const
	{
		return m_requested_position;
	}
	double GetCurrentVelocity() const
	{
		return m_current_velocity;
	}
	double GetCurrentPosition() const
	{
		return m_current_position;
	}
};
#pragma endregion

#pragma region _wrapper methods_
Entity1D::Entity1D()
{
	m_Entity1D = std::make_shared<Entity1D_internal>();
}
void Entity1D::SetProperties(const properties &props)
{
	m_Entity1D->SetProperties(props);
}
void Entity1D::TimeSlice(double d_time_s)
{
	m_Entity1D->TimeSlice(d_time_s);
}
void Entity1D::Reset(double position)
{
	m_Entity1D->Reset(position);
}

void Entity1D::SetVelocity(double rate)
{
	m_Entity1D->SetVelocity(rate);
}
//AI methods: ---------------------------------------------------------------------------
void Entity1D::SetPosition(double position)
{
	m_Entity1D->SetPosition(position);
}

//Accessors:---------------------------------------------
bool Entity1D::GetIsDriven() const
{
	return m_Entity1D->GetIsDriven();
}

double Entity1D::Get_SetVelocity() const
{
	return m_Entity1D->Get_SetVelocity();
}
double Entity1D::Get_SetPosition() const
{
	return m_Entity1D->Get_SetPosition();
}
double Entity1D::GetCurrentVelocity() const
{
	return m_Entity1D->GetCurrentVelocity();
}
double Entity1D::GetCurrentPosition() const
{
	return m_Entity1D->GetCurrentPosition();
}
#pragma endregion

	}
}