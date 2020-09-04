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

class Entity2D_internal
{
private:
	#pragma region _member varibles_
	Vec2D m_current_velocity;
	Vec2D m_current_position;
	double m_current_angular_velocity = 0.0;
	double m_current_heading=0.0;
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
		//For linear we always driving for Entity
		{
			//from current velocity we can update the position
			m_current_position += m_current_velocity * d_time_s;
		}

		//For angular always driving for Entity
		{
			
			//from current velocity we can update the position
			m_current_heading += m_current_angular_velocity * d_time_s;
			//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
			m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging
			
		}
	}
public:
	void TimeSlice(double d_time_s)
	{
		process_slice(d_time_s);  //keep public small and compact
	}

	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_current_velocity = Vec2D(0.0, 0.0);
		m_current_position[0] = X;
		m_current_position[1] = Y;
		m_current_angular_velocity=0.0;
		m_current_heading = heading;
	}

	//Tele-op methods:-----------------------------------------------------------------------
	void SetLinearVelocity_local(double forward, double right)
	{
		//Supported... just translate to global
		const Vec2D GlobalVelocity=LocalToGlobal(m_current_heading, Vec2D(right,forward));
		m_current_velocity[0] = right;
		m_current_velocity[1] = forward;
	}
	void SetLinearVelocity_global(double north, double east)
	{
		m_current_velocity[0] = east;
		m_current_velocity[1] = north;
	}
	void SetAngularVelocity(double clockwise)
	{
		m_current_angular_velocity = clockwise;
	}

	//Accessors:---------------------------------------------
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
void Entity2D::TimeSlice(double d_time_s)
{
	m_Entity2D->TimeSlice(d_time_s);
}
void Entity2D::Reset(double X, double Y, double heading)
{
	m_Entity2D->Reset(X, Y, heading);
}

//moving entity methods:-----------------------------------------------------------------------
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
//Accessors:---------------------------------------------
__inline Entity2D::Vector2D as_vector_2d(Vec2D input)
{
	Entity2D::Vector2D ret = { input[0],input[1] };
	return ret;
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