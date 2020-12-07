#include "StdAfx.h"
#include "AI_Input.h"
#include "Goal_Types.h"

using namespace Framework::Base;

namespace Module
{
	namespace Input
	{
//This sets up a good foundational environment and then a class with a goal can inherit from it without needing
//to gather the resources, the default implementation does nothing
class AI_Input_Internal
{
private:
	#pragma region _members_
	AtomicGoal m_EmptyGoal;  //allows a new derived class to quickly provide a goal
	std::function<void(double intended_orientation, bool absolute)> m_SetIntendedOrientation_callback=nullptr;
	std::function<AI_Input::DriveTo_proto> m_DriveToLocation_callback=nullptr;
	std::function <Vec2D()> m_GetCurrentPosition_callback=nullptr;
	std::function <double()> m_GetCurrentHeading_callback=nullptr;
	#pragma endregion
protected:
	friend AI_Input;
	#pragma region _goal interface methods_
	void SetIntendedOrientation(double intended_orientation, bool absolute)
	{
		m_SetIntendedOrientation_callback(intended_orientation, absolute);
	}
	void DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
	{
		m_DriveToLocation_callback(north, east, absolute, stop_at_destination, max_speed, can_strafe);
	}
	Vec2D GetCurrentPosition() const
	{
		return m_GetCurrentPosition_callback();
	}
	double GetCurrentHeading() const
	{
		return m_GetCurrentHeading_callback();
	}
	#pragma endregion
public:
	Goal& GetGoal()
	{
		return m_EmptyGoal;
	}

	#pragma region _callbacks_
	void Set_SetIntendedOrientation(std::function<void(double intended_orientation, bool absolute)> callback)
	{
		m_SetIntendedOrientation_callback = callback;
	}
	void Set_DriveToLocation(std::function<void(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)> callback)
	{
		m_DriveToLocation_callback = callback;
	}
	void Set_GetCurrentPosition(std::function <Vec2D()> callback)
	{
		m_GetCurrentPosition_callback = callback;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_GetCurrentHeading_callback = callback;
	}
	#pragma endregion
};

#pragma region wrapper methods

AI_Input::AI_Input()
{
	m_AI_Input = std::make_shared<AI_Input_Internal>();
}

Framework::Base::Goal& AI_Input::GetGoal()
{
	return m_AI_Input->GetGoal();
}

void AI_Input::Set_SetIntendedOrientation(std::function<void(double intended_orientation, bool absolute)> callback)
{
	m_AI_Input->Set_SetIntendedOrientation(callback);
}
void AI_Input::Set_DriveToLocation(std::function<AI_Input::DriveTo_proto> callback)
{
	m_AI_Input->Set_DriveToLocation(callback);
}
void AI_Input::Set_GetCurrentPosition(std::function <Vec2D()> callback)
{
	m_AI_Input->Set_GetCurrentPosition(callback);
}
void AI_Input::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_AI_Input->Set_GetCurrentHeading(callback);
}

void AI_Input::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_AI_Input->SetIntendedOrientation(intended_orientation, absolute);
}
void AI_Input::DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_AI_Input->DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
}
Vec2D AI_Input::GetCurrentPosition() const
{
	return m_AI_Input->GetCurrentPosition();
}
double AI_Input::GetCurrentHeading() const
{
	return m_AI_Input->GetCurrentHeading();
}

#pragma endregion

	}
}