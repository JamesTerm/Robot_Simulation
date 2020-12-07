#pragma once

#include "../../../Base/Goal.h"
#include "../../../Base/Vec2d.h"

namespace Module
{
	namespace Input
	{

//Forward declare
class AI_Input_Internal;

class AI_Input
{
public:
	//This is a complete foundational interface that bridges with the robot that is then intended to be derived with the actual goal
	AI_Input();
	//Output for assembly file (e.g TeleAutonV1)
	//The goal is written internally where the main loop can give it time slices as well as read the status and terminate if needed
	//One thing to consider is to gracefully exit a goal by issuing a terminate if it is still active when disabling or switching out
	//These can be done internally when object gets destroyed, but it is still possible to add external control to this.
	virtual Framework::Base::Goal& GetGoal();
	#pragma region _callbacks_
	//Callbacks---
	//For the goals to work they are dependent on interfacing with the robot
	//For the drive they will need motion control methods, and localization methods
	//We can add sensor methods in here as well

	//AI motion control methods: ---------------------------------------------------------------------------
	//This allows setting the desired heading directly either relative to the current heading or absolute
	//If relative positive is a clockwise offset from where it is; otherwise the direction is
	//like written before where 0 is north, pi/2 east, etc.
	void Set_SetIntendedOrientation(std::function<void(double intended_orientation,bool absolute)> callback);
	//Drives to a location at given by coordinates at max speed given
	//It can either stop at location or continue to drive past it once it hits (allows for path driving)
	//max speed is optional where 0.0 means the max speed in properties
	//if can strafe is true caller manages its own orientation as it deems fit; otherwise if it can't strafe
	//it must manage the orientation to always drive forward in the direction toward the way point
	using DriveTo_proto = void(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe);
	void Set_DriveToLocation(std::function<DriveTo_proto> callback);

	//AI localization methods:  ------------------------------------------------------------------------------
	//Goals will need to do their own management of where they are to know when the goal is completed, the same is
	//true for other sensors
	void Set_GetCurrentPosition(std::function <Vec2D()> callback);  //returns x y coordinates of current position
	void Set_GetCurrentHeading(std::function <double()> callback);  //returns heading where 0 is north in radians
	#pragma endregion

	#pragma region _goal interface methods_
	//The methods in this section are to assist derived classes access to the foundational resources
	void SetIntendedOrientation(double intended_orientation, bool absolute);
	void DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe);
	Vec2D GetCurrentPosition() const;
	double GetCurrentHeading() const;
	#pragma endregion
private:
	std::shared_ptr<AI_Input_Internal>  m_AI_Input;
};

}}