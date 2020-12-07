#include "StdAfx.h"
#include "AI_Input_Example.h"
#include "Goal_Types.h"
#include "SmartDashboard_HelperFunctions.h"
#include "AI_BaseController_goals.h"

using namespace Framework::Base;

namespace Module
{
	namespace Input
	{

//now this goal can be whatever we want with access to robot resources
class AI_Example_internal : public AtomicGoal
{
private:
	AI_Input* m_pParent=nullptr;
	double m_Timer=0.0;
	MultitaskGoal m_Primer;

	#pragma region _foundation goals_
	class goal_clock : public AtomicGoal
	{
	private:
		AI_Example_internal* m_Parent;
	public:
		goal_clock(AI_Example_internal* Parent) : m_Parent(Parent) { m_Status = eInactive; }
		void Activate() { m_Status = eActive; }
		Goal_Status Process(double dTime_s)
		{
			const double AutonomousTimeLimit = 30.0 * 60.0; //level 1 30 minutes
			double& Timer = m_Parent->m_Timer;
			if (m_Status == eActive)
			{
				SmartDashboard::PutNumber("Timer", AutonomousTimeLimit - Timer);
				Timer += dTime_s;
				if (Timer >= AutonomousTimeLimit)
					m_Status = eCompleted;
			}
			return m_Status;
		}
		void Terminate() { m_Status = eFailed; }
	};

	class goal_watchdog : public AtomicGoal
	{
	private:
		AI_Example_internal* m_Parent;
	public:
		goal_watchdog(AI_Example_internal* Parent) : m_Parent(Parent) { m_Status = eInactive; }
		void Activate() { m_Status = eActive; }
		Goal_Status Process(double dTime_s)
		{
			if (m_Status == eActive)
			{
				//const bool SafetyLock = SmartDashboard::GetBoolean("SafetyLock_Drive");
				bool SafetyLock=false;
				try
				{
					SafetyLock = SmartDashboard::GetBoolean("SafetyLock_Drive");
				}
				catch (...)
				{
					//set up default for nothing
					SmartDashboard::PutBoolean("SafetyLock_Drive", false);
				}

				if (SafetyLock)
					m_Status = eFailed;
			}
			return m_Status;
		}
		void Terminate() { m_Status = eFailed; }
	};

	static Goal* Move_Straight(AI_Input* Parent, double length_ft)
	{
		//Construct a way point
		WayPoint wp;
		const Vec2d Local_GoalTarget(0.0, Feet2Meters(length_ft));
		wp.Position = Local_GoalTarget;
		wp.Power = 1.0;
		//Now to setup the goal
		const bool LockOrientation = true;
		const double PrecisionTolerance = Feet2Meters(1.0);
		Goal_Ship_MoveToPosition* goal_drive = NULL;
		goal_drive = new Goal_Ship_MoveToRelativePosition(Parent, wp, true, LockOrientation, PrecisionTolerance);
		return goal_drive;
	}

	static Goal* Rotate(AI_Input* Parent, double Degrees)
	{
		return new Goal_Ship_RotateToRelativePosition(Parent, DEG_2_RAD(Degrees));
	}

	class RobotQuickNotify : public AtomicGoal
	{
	private:
		std::function<void(bool IsOn)> m_Callback;
		bool m_IsOn;
	public:
		RobotQuickNotify(std::function<void(bool IsOn)> callback,bool On) : m_Callback(callback), m_IsOn(On)
		{
			m_Status = eInactive;
		}
		virtual void Activate() { m_Status = eActive; }
		virtual Goal_Status Process(double dTime_s)
		{
			ActivateIfInactive();
			//m_EventMap.EventOnOff_Map[m_EventName.c_str()].Fire(m_IsOn);
			m_Callback(m_IsOn);
			m_Status = eCompleted;
			return m_Status;
		}
	};
	#pragma endregion
	#pragma region _Drive Tests_
	//Drive Tests----------------------------------------------------------------------
	class MoveForward : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_Parent;
	public:
		MoveForward(AI_Input* Parent, bool AutoActivate = false) : Generic_CompositeGoal(AutoActivate), m_Parent(Parent)
		{
			if (!AutoActivate)
				m_Status = eActive;
		}
		virtual void Activate()
		{
			const char* const MoveSmartVar = "TestMove";
			double DistanceFeet = Auton_Smart_GetSingleValue(MoveSmartVar, 1.0); //should be a safe default

			AddSubgoal(new Goal_Wait(0.500));
			AddSubgoal(Move_Straight(m_Parent, DistanceFeet));
			AddSubgoal(new Goal_Wait(0.500));  //allow time for mass to settle
			m_Status = eActive;
		}
	};

	class RotateWithWait : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_Parent;
	public:
		RotateWithWait(AI_Input* Parent, bool AutoActivate = false) : Generic_CompositeGoal(AutoActivate), m_Parent(Parent)
		{
			if (!AutoActivate)
				m_Status = eActive;
		}
		virtual void Activate()
		{
			const char* const RotateSmartVar = "TestRotate";
			const double RotateDegrees = Auton_Smart_GetSingleValue(RotateSmartVar, 45.0); //should be a safe default

			AddSubgoal(new Goal_Wait(0.500));
			AddSubgoal(Rotate(m_Parent, RotateDegrees));
			AddSubgoal(new Goal_Wait(0.500));  //allow time for mass to settle
			m_Status = eActive;
		}
	};

	class TestMoveRotateSequence : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_pParent;
	public:
		TestMoveRotateSequence(AI_Input* Parent) : m_pParent(Parent) 
		{ 
			m_Status = eActive; 
		}
		virtual void Activate()
		{
			double dNoIterations = 4.0;
			double TestRotateDeg = 90.0;
			double TestMoveFeet = 1.0;
			const char* const SmartNames[] = { "TestMoveRotateIter","TestRotate","TestMove" };
			double* const SmartVariables[] = { &dNoIterations,&TestRotateDeg,&TestMoveFeet };
			Auton_Smart_GetMultiValue(3, SmartNames, SmartVariables);
			size_t NoIterations = (size_t)dNoIterations;

			for (size_t i = 0; i < NoIterations; i++)
			{
				AddSubgoal(new MoveForward(m_pParent, true));
				AddSubgoal(new RotateWithWait(m_pParent, true));
			}
			m_Status = eActive;
		}
	};

	static Goal* GiveRobotSquareWayPointGoal(AI_Input* Parent)
	{
		const char* const LengthSetting = "TestDistance_ft";
		const double Length_m = Feet2Meters(Auton_Smart_GetSingleValue(LengthSetting, Feet2Meters(5)));

		std::list <WayPoint> points;
		struct Locations
		{
			double x, y;
		} test[] =
		{
			{Length_m,Length_m},
			{Length_m,-Length_m},
			{-Length_m,-Length_m},
			{-Length_m,Length_m},
			{0,0}
		};
		for (size_t i = 0; i < _countof(test); i++)
		{
			WayPoint wp;
			wp.Position[0] = test[i].x;
			wp.Position[1] = test[i].y;
			//May want to test slower in real life
			//wp.Power = 0.5;
			//This gets you there but the accuracy is rounded to tolerance
			//(because the swivel wheels are challenged more)
			//wp.Power = 0.0;
			//TODO for now the goals work in power, while the robot works with max speed
			//I may change the interface to use power, but for now just use the speed for power
			//In testing this speed does good enough and pretty quick 
			//(may be different repsonse times once the swivel rates change for simulation or in real life)
			wp.Power = 2.5;
			points.push_back(wp);
		}
		//Now to setup the goal
		Goal_Ship_FollowPath* goal = new Goal_Ship_FollowPath(Parent, points, false, true);
		return goal;
	}
	#pragma endregion
	//Reserved Drive Tracking
public:
	enum AutonType
	{
		eDoNothing,
		eJustMoveForward,
		eJustRotate,
		eSimpleMoveRotateSequence,
		eTestBoxWayPoints,
		//eDriveTracking,
		eNoAutonTypes
	};

	AI_Example_internal(AI_Input* parent) : m_pParent(parent), m_Timer(0.0),
		m_Primer(false)  //who ever is done first on this will complete the goals (i.e. if time runs out)
	{
		m_Status = eInactive;
	}
	void Activate()
	{
		m_Primer.AsGoal().Terminate();  //sanity check clear previous session

		AutonType AutonTest = eDoNothing;
		const char* const AutonTestSelection = "AutonTest";
		#if 1
		try
		{
			AutonTest = (AutonType)((size_t)SmartDashboard::GetNumber(AutonTestSelection));
		}
		catch (...)
		{
			//set up default for nothing
			SmartDashboard::PutNumber(AutonTestSelection, (double)eDoNothing);
		}
		#else
		#if !defined __USE_LEGACY_WPI_LIBRARIES__
		SmartDashboard::SetDefaultNumber(AutonTestSelection, 0.0);
		AutonTest = (AutonType)((size_t)SmartDashboard::GetNumber(AutonTestSelection));
		#else
		//for cRIO checked in using zero in lua (default) to prompt the variable and then change to -1 to use it
		if (auton.AutonTest != (size_t)-1)
		{
			SmartDashboard::PutNumber(AutonTestSelection, (double)0.0);
			SmartDashboard::PutBoolean("TestVariables_set", false);
		}
		else
			AutonTest = (AutonType)((size_t)SmartDashboard::GetNumber(AutonTestSelection));
		#endif
		#endif

		printf("Testing=%d \n", AutonTest);
		switch (AutonTest)
		{
		case eJustMoveForward:
			m_Primer.AddGoal(new MoveForward(m_pParent));
			break;
		case eJustRotate:
			m_Primer.AddGoal(new RotateWithWait(m_pParent));
			break;
		case eSimpleMoveRotateSequence:
			m_Primer.AddGoal(new TestMoveRotateSequence(m_pParent));
			break;
		case eTestBoxWayPoints:
			m_Primer.AddGoal(GiveRobotSquareWayPointGoal(m_pParent));
			break;
		//case eDriveTracking:
		//	m_Primer.AddGoal(new DriveTracking(this));
		//	break;
		case eDoNothing:
		case eNoAutonTypes: //grrr windriver and warning 1250
			break;
		}
		m_Primer.AddGoal(new goal_clock(this));
		m_Primer.AddGoal(new goal_watchdog(this));
		m_Status = eActive;
	}

	Goal_Status Process(double dTime_s)
	{
		ActivateIfInactive();
		if (m_Status == eActive)
			m_Status = m_Primer.AsGoal().Process(dTime_s);
		return m_Status;
	}
	void Terminate()
	{
		m_Primer.AsGoal().Terminate();
		m_Status = eFailed;
	}

};

AI_Example::AI_Example()
{
	m_AI_Input = std::make_shared<AI_Example_internal>(this);
}

Goal& AI_Example::GetGoal()
{
	return *m_AI_Input;
}
	}
}