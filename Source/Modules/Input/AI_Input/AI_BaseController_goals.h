#pragma once

#include <functional>
#include "../../../Base/Misc.h"
#include "Goal_Types.h"
#include "AI_Input.h"

namespace Framework
{
	namespace Base
	{

inline void NormalizeRotation(double& Rotation)
{
	const double Pi2 = M_PI * 2.0;
	//Normalize the rotation
	if (Rotation > M_PI)
		Rotation -= Pi2;
	else if (Rotation < -M_PI)
		Rotation += Pi2;
}


//This will explicitly rotate the ship to a particular heading.  It may be moving or still.
class Goal_Ship_RotateToPosition : public AtomicGoal
{
public:
	Goal_Ship_RotateToPosition(Module::Input::AI_Input* controller, double Heading) : m_Controller(controller), m_Heading(Heading), m_Terminate(false)
	{
		m_Status = eInactive;
	}
	~Goal_Ship_RotateToPosition()
	{
		Terminate(); //more for completion
	}
	virtual void Activate()
	{
		m_Status = eActive;
		//During the activation we'll set the requested intended orientation
		m_Controller->SetIntendedOrientation(m_Heading, true);
	}
	virtual Goal_Status Process(double dTime_s)
	{
		//TODO this may be an inline check
		if (m_Terminate)
		{
			if (m_Status == eActive)
				m_Status = eFailed;
			return m_Status;
		}
		ActivateIfInactive();
		if (m_Status == eActive)
		{
			//It should not be possible for something else to take over.. if so, we can provide accessor
			//if (m_Controller->Get_IntendedOrientation() == m_Heading)
			{
				double rotation_delta = m_Controller->GetCurrentHeading() - m_Heading;
				NormalizeRotation(rotation_delta);
				//TODO check IsStuck for failed case
				if (fabs(rotation_delta) < DEG_2_RAD(2.0)) //<-- hard coded for now, may want to check a property here
					m_Status = eCompleted;
			}
			//else
			//	m_Status = eFailed;  //Some thing else took control of the ship
		}
		return m_Status;
	}
	virtual void Terminate() { m_Terminate = true; }
protected:
	Module::Input::AI_Input* const m_Controller;
	double m_Heading;
private:
	bool m_Terminate;
};


//This is like Goal_Ship_RotateToPosition except it will have a relative heading added to its current heading
class Goal_Ship_RotateToRelativePosition : public Goal_Ship_RotateToPosition
{
public:
	Goal_Ship_RotateToRelativePosition(Module::Input::AI_Input* controller, double Heading) :
		Goal_Ship_RotateToPosition(controller, Heading) {}
	//Note: It is important for client code not to activate this... let process activate it... so that it sets the heading at the correct time and current heading
	virtual void Activate()
	{
		m_Heading += m_Controller->GetCurrentHeading();
		__super::Activate();
	}
};


struct WayPoint
{
	WayPoint() : Power(0.0), Position(0, 0), TurnSpeedScaler(1.0) {}
	double Power;
	Vec2D Position;
	double TurnSpeedScaler;  //This will have a default value if not in script
};

//This is similar to Traverse_Edge in book (not to be confused with its MoveToPosition)
class Goal_Ship_MoveToPosition : public AtomicGoal
{
public:
	/// \param double safestop_tolerance used to set safe stop tolerance, default is a little over an inch
	Goal_Ship_MoveToPosition(Module::Input::AI_Input* controller, const WayPoint& waypoint, bool UseSafeStop = true,
		bool LockOrientation = false, double safestop_tolerance = 0.03) : m_Point(waypoint), m_Controller(controller),
		m_SafeStopTolerance(safestop_tolerance), m_Terminate(false), m_UseSafeStop(UseSafeStop), m_LockOrientation(LockOrientation)
	{
		m_TrajectoryPoint = waypoint.Position;  //set it for default
		m_Status = eInactive;
	}
	~Goal_Ship_MoveToPosition()
	{
		Terminate(); //more for completion
	}
	//optionally set the trajectory point... it is the same as the waypoint by default
	void SetTrajectoryPoint(const Vec2D& TrajectoryPoint)
	{
		m_TrajectoryPoint = TrajectoryPoint;
	}
	virtual void Activate()
	{
		m_Status = eActive;
		//Unlike before the controller does all the work, we only have to monitor the waypoint activity
		m_Controller->DriveToLocation(m_Point.Position.y(), m_Point.Position.x(), true, m_UseSafeStop, m_Point.Power, m_LockOrientation);
	}
	virtual Goal_Status Process(double dTime_s)
	{
		//TODO this may be an inline check
		if (m_Terminate)
		{
			if (m_Status == eActive)
				m_Status = eFailed;
			return m_Status;
		}
		ActivateIfInactive();
		if (m_Status == eActive)
		{
			if (HitWayPoint())
			{
				//Note: This happens implicitly, all we have to do is just update the status
				//m_Controller->SetShipVelocity(0.0);
				m_Status = eCompleted;
			}
		}
		return m_Status;
	}
	virtual void Terminate()
	{
		//TODO this may be an inline check
		m_Terminate = true;
	}

protected:
	//Similar to FlyWayPoints, except it only checks for the tolerance
	bool HitWayPoint()
	{
		//This is depreciated, if we need access to the velocity we can add it

		// Base a tolerance2 for how close we want to get to the way point based on the current velocity,
		// within a second of reaching the way point, just move to the next one
		//Note for FRC... moving at 2mps it will come within an inch of its point with this tolerance
		//double tolerance2 = m_UseSafeStop ? m_SafeStopTolerance : (m_ship.GetPhysics().GetLinearVelocity().length() * 1.0) + 0.1; // (will keep it within one meter even if not moving)

		const double tolerance2 = m_SafeStopTolerance;
		const Vec2d currPos = m_Controller->GetCurrentPosition();
		double position_delta = (m_Point.Position - currPos).length();
		bool ret = position_delta < tolerance2;
#if 0
		printf("\r%f        ", position_delta);
		if (ret)
			printf("completed %f\n", position_delta);
#endif
		return ret;
	}

	WayPoint m_Point;
	Vec2D m_TrajectoryPoint;
	Module::Input::AI_Input* const m_Controller;
	double m_SafeStopTolerance;
	bool m_Terminate;
	bool m_UseSafeStop;
	bool m_LockOrientation;
};

//This is like Goal_Ship_MoveToPosition except it will set the waypoint relative to its current position and orientation
//This will also set the trajectory point x distance (1 meter default) beyond the point to help assist in orientation
class  Goal_Ship_MoveToRelativePosition : public Goal_Ship_MoveToPosition
{
public:
	Goal_Ship_MoveToRelativePosition(Module::Input::AI_Input* controller, const WayPoint& waypoint, bool UseSafeStop = true,
		bool LockOrientation = false, double safestop_tolerance = 0.03) : Goal_Ship_MoveToPosition(controller, waypoint, UseSafeStop, LockOrientation, safestop_tolerance) {}
	//Note: It is important for client code not to activate this... let process activate it... so that it sets the point at the correct time and current position
	virtual void Activate()
	{
		//Construct a way point
		WayPoint wp = m_Point;  //set up all the other fields
		const Vec2d& pos = m_Controller->GetCurrentPosition();

		const Vec2d Local_GoalTarget = m_Point.Position;

		const Vec2d Global_GoalTarget = LocalToGlobal(m_Controller->GetCurrentHeading(), Local_GoalTarget);
		wp.Position = Global_GoalTarget + pos;
		//set the trajectory point
		double lookDir_radians = atan2(Local_GoalTarget[0], Local_GoalTarget[1]);
		const Vec2d LocalTrajectoryOffset(sin(lookDir_radians), cos(lookDir_radians));
		const Vec2d  GlobalTrajectoryOffset = LocalToGlobal(m_Controller->GetCurrentHeading(), LocalTrajectoryOffset);
		SetTrajectoryPoint(wp.Position + GlobalTrajectoryOffset);
		m_Point = wp;  //This should be a one-time assignment
		__super::Activate();
	}
};

class  Goal_Ship_FollowPath : public CompositeGoal
{
public:
	Goal_Ship_FollowPath(Module::Input::AI_Input* controller, std::list<WayPoint> path, bool LoopMode = false, bool UseSafeStop = false) :
		m_Controller(controller), m_Path(path), m_PathCopy(path), m_LoopMode(LoopMode), m_UseSafeStop(UseSafeStop)
	{
		m_Status = eInactive;
	}
	virtual void Activate()
	{
		RemoveAllSubgoals();
		if (!m_Path.empty())
		{
			m_Status = eActive;
			WayPoint point = m_Path.front();
			m_Path.pop_front();
			AddSubgoal(new Goal_Ship_MoveToPosition(m_Controller, point, m_UseSafeStop));
		}
		else
			m_Status = eFailed;
	}
	virtual Goal_Status Process(double dTime_s)
	{
		ActivateIfInactive();
		if (m_Status == eActive)
		{
			m_Status = ProcessSubgoals(dTime_s);
			if (m_Status == eCompleted)
			{
				if (m_Path.empty() && (m_LoopMode))
					m_Path = m_PathCopy;
				if (!m_Path.empty())
					Activate();
			}
		}
		return m_Status;
	}
	virtual void Terminate()
	{
		//ensure its all clean
		RemoveAllSubgoals();
		m_Status = eInactive; //make this inactive
	}
private:
	Module::Input::AI_Input* const m_Controller;
	std::list<WayPoint> m_Path, m_PathCopy;
	bool m_LoopMode, m_UseSafeStop;
};

class  Goal_Wait : public AtomicGoal
{
private:
	double m_TimeAccrued = 0.0;
	double m_TimeToWait;
public:
	Goal_Wait(double seconds) : m_TimeToWait(seconds)
	{
		m_Status = eInactive;
	}
	virtual void Activate()
	{
		m_Status = eActive;
		m_TimeAccrued = 0.0;
	}
	virtual Goal_Status Process(double dTime_s)
	{
		ActivateIfInactive();
		m_TimeAccrued += dTime_s;
		if (m_TimeAccrued > m_TimeToWait)
			m_Status = eCompleted;
		return m_Status;
	}
	virtual void Terminate()
	{
		m_Status = eInactive;
	}
};

//This goal simply will fire an event when all goals are complete
class  Goal_NotifyWhenComplete : public CompositeGoal
{
private:
	std::string m_EventName, m_FailedEventName;  //name to fire when complete
	std::function<void(bool success)> m_Callback;
public:
	Goal_NotifyWhenComplete(std::function<void(bool success)> callback)
	{
		m_Status = eInactive;
		m_Callback = callback;
	}
	//give public access for client to populate goals
	virtual void AddSubgoal(Goal* g) { __super::AddSubgoal(g); }
	//client activates manually when goals are added
	virtual void Activate()
	{
		m_Status = eActive;
	}
	virtual Goal_Status Process(double dTime_s)
	{
		//Client will activate
		if (m_Status == eInactive)
			return m_Status;

		if (m_Status == eActive)
		{
			m_Status = ProcessSubgoals(dTime_s);
			if (m_Status == eCompleted)
			{
				//m_EventMap.Event_Map[m_EventName].Fire(); //Fire the event
				m_Callback(true);
				Terminate();
			}
		}
		else if (m_Status == eFailed)
		{
			//if (m_FailedEventName[0] != 0)
			//	m_EventMap.Event_Map[m_FailedEventName].Fire(); //Fire the event
			m_Callback(false);
			Terminate();
		}
		return m_Status;
	}
	virtual void Terminate()
	{
		//ensure its all clean
		RemoveAllSubgoals();
		m_Status = eInactive; //make this inactive
	}
};

}}