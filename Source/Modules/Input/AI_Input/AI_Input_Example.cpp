#include "StdAfx.h"
#include "AI_Input_Example.h"
#include "Goal_Types.h"
#include "SmartDashboard_HelperFunctions.h"
#include "AutonChooser.h"
#include "AutonSelection.h"
#include "AI_BaseController_goals.h"
#include "DirectAutonChainLog.h"
#include "../../../Properties/RegistryV1.h"
using namespace Framework::Base;

namespace Module
{
	namespace Input
	{
			namespace
			{
				inline bool IsChooserEnabledForCurrentConnection()
				{
					const SmartDashboardConnectionMode mode = SmartDashboard::GetConnectionMode();
					return (mode == SmartDashboardConnectionMode::eDirectConnect) ||
						(mode == SmartDashboardConnectionMode::eNativeLink);
				}
			}

//now this goal can be whatever we want with access to robot resources
class AI_Example_internal : public AtomicGoal
{
private:
	AI_Input* m_pParent=nullptr;
	double m_Timer=0.0;
	MultitaskGoal m_Primer;
	const Framework::Base::asset_manager* m_properties=nullptr;

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
			printf("[MoveForward] Activate TestMove=%g\n", DistanceFeet);
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[MoveForward] Activate TestMove=%g", DistanceFeet);
				AppendDirectAutonChainLog(dbg);
			}

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
			double TestMoveFeet = 5.0;
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
		const double Length_m = Feet2Meters(Auton_Smart_GetSingleValue(LengthSetting, 5.0));

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
			//(may be different response times once the swivel rates change for simulation or in real life)
			wp.Power = 2.5;
			points.push_back(wp);
		}
		//Now to setup the goal
		Goal_Ship_FollowPath* goal = new Goal_Ship_FollowPath(Parent, points, false, true);
		return goal;
	}
	static Goal* SmartWaypoints(AI_Input* Parent, double max_speed)
	{
		const char* const Waypoint_Count = "waypoint_count";
		const size_t wp_count = (size_t)Auton_Smart_GetSingleValue(Waypoint_Count, 1.0);
		std::list <WayPoint> points;
		//we'll push one group at a time to our list (keeping it simple)
		for (size_t i = 0; i < wp_count; i++)
		{
			//generate our group of points
			std::string WP_x="wp_x_";
			WP_x += std::to_string((double)i);
			std::string WP_y = "wp_y_";
			WP_y += std::to_string((double)i);
			std::string WP_speed = "wp_speed_";
			WP_speed += std::to_string((double)i);
			const char* const SmartNames[] = { WP_x.c_str(),WP_y.c_str(),WP_speed.c_str()};
			double X=0.0, Y=0.0, Speed=1.0;
			double* const SmartVariables[] = { &X,&Y,&Speed};
			Auton_Smart_GetMultiValue(3, SmartNames, SmartVariables);
			//Now to setup this way point
			WayPoint wp;
			wp.Position[0] = Feet2Meters(X);
			wp.Position[1] = Feet2Meters(Y);
			wp.Power = Speed * max_speed;
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
		eTestSmartWayPoints,
		//eDriveTracking,
		eNoAutonTypes
	};

	static const AutonChooserOption* GetAutonChooserOptions(size_t& optionCount)
	{
		static const AutonChooserOption kOptions[] =
		{
			{eDoNothing, "Do Nothing"},
			{eJustMoveForward, "Just Move Forward"},
			{eJustRotate, "Just Rotate"},
			{eSimpleMoveRotateSequence, "Move Rotate Sequence"},
			{eTestBoxWayPoints, "Box Waypoints"},
			{eTestSmartWayPoints, "Smart Waypoints"}
		};
		optionCount = _countof(kOptions);
		return kOptions;
	}

	AI_Example_internal(AI_Input* parent) : m_pParent(parent), m_Timer(0.0),
		m_Primer(false)  //who ever is done first on this will complete the goals (i.e. if time runs out)
	{
		m_Status = eInactive;
	}
	void Initialize(const Framework::Base::asset_manager* props)
	{
		m_properties = props;  //just reference it we know it stays active
	}
	void Activate()
	{
		m_Primer.AsGoal().Terminate();  //sanity check clear previous session
		//reset timer
		m_Timer = 0.0;
		AutonType AutonTest = eDoNothing;
		const char* const AutonTestSelection = "AutonTest";
		const char* const AutonChooserBase = "Test/Auton_Selection/AutoChooser";
		size_t autonOptionCount = 0;
		const AutonChooserOption* autonOptions = GetAutonChooserOptions(autonOptionCount);
		// Ian: Keep chooser state on its own key path so Direct chooser work never
		// masks the numeric `AutonTest` baseline that legacy NT and regression
		// testing still depend on.
		auto tryReadAutonSelectionChooser = [&](double& outValue) -> bool
		{
			std::string selectedLabel;
			std::string activeLabel;
			std::string defaultLabel;
			try
			{
				selectedLabel = SmartDashboard::GetString(std::string(AutonChooserBase) + "/selected");
			}
			catch (...)
			{
			}
			try
			{
				activeLabel = SmartDashboard::GetString(std::string(AutonChooserBase) + "/active");
			}
			catch (...)
			{
			}
			try
			{
				defaultLabel = SmartDashboard::GetString(std::string(AutonChooserBase) + "/default");
			}
			catch (...)
			{
			}
			{
				char dbg[512] = {};
				sprintf_s(
					dbg,
					"[AI AutonTest] raw selected='%s' active='%s' default='%s'",
					selectedLabel.c_str(),
					activeLabel.c_str(),
					defaultLabel.c_str());
				AppendDirectAutonChainLog(dbg);
			}

			const bool hasChooserSignal =
				!selectedLabel.empty() ||
				!activeLabel.empty() ||
				!defaultLabel.empty();

			// Ian: Only treat chooser mode as active when the chooser actually has
			// signal. Direct mode can be chooser-capable and still need to fall back
			// to numeric `AutonTest` for baseline or legacy-style runs.
			if (hasChooserSignal)
			{
				const int chooserIndex = ResolveAutonSelectionFromChooser(
					AutonChooserBase,
					AutonTestSelection,
					autonOptions,
					autonOptionCount,
					(int)eDoNothing);
				outValue = static_cast<double>(chooserIndex);
				printf("[AI AutonTest] source=chooser final_index=%d\n", chooserIndex);
				return true;
			}

			return false;
		};

			const char* const AutonTestSelectionScoped = "Test/AutonTest";
		auto tryReadAutonSelectionDouble = [&](double& outValue) -> bool
		{
			std::string rawAutonText;
			std::string rawAutonScopedText;
			if (SmartDashboard::TryGetNumber(AutonTestSelection, outValue))
			{
				printf("[AI AutonTest] source=number value=%g\n", outValue);
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[AI AutonTest] source=number key='%s' value=%g", AutonTestSelection, outValue);
					AppendDirectAutonChainLog(dbg);
				}
				return true;
			}

			if (SmartDashboard::TryGetNumber(AutonTestSelectionScoped, outValue))
			{
				printf("[AI AutonTest] source=number(scoped) value=%g\n", outValue);
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[AI AutonTest] source=number key='%s' value=%g", AutonTestSelectionScoped, outValue);
					AppendDirectAutonChainLog(dbg);
				}
				return true;
			}

			if (SmartDashboard::TryGetString(AutonTestSelection, rawAutonText))
			{
				outValue = atof(rawAutonText.c_str());
				printf("[AI AutonTest] source=string raw='%s' parsed=%g\n", rawAutonText.c_str(), outValue);
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[AI AutonTest] source=string key='%s' raw='%s' parsed=%g", AutonTestSelection, rawAutonText.c_str(), outValue);
					AppendDirectAutonChainLog(dbg);
				}
				return true;
			}

			if (SmartDashboard::TryGetString(AutonTestSelectionScoped, rawAutonScopedText))
			{
				outValue = atof(rawAutonScopedText.c_str());
				printf("[AI AutonTest] source=string(scoped) raw='%s' parsed=%g\n", rawAutonScopedText.c_str(), outValue);
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[AI AutonTest] source=string key='%s' raw='%s' parsed=%g", AutonTestSelectionScoped, rawAutonScopedText.c_str(), outValue);
					AppendDirectAutonChainLog(dbg);
				}
				return true;
			}

			{
				char dbg[256] = {};
				sprintf_s(dbg, "[AI AutonTest] source=missing key='%s' scoped='%s'", AutonTestSelection, AutonTestSelectionScoped);
				AppendDirectAutonChainLog(dbg);
			}

			return false;
		};
		const bool useChooser = IsChooserEnabledForCurrentConnection();
		#if 1
		int autonIndex = ResolveAutonIndex(
			[&](double& outSelection)
			{
				// Ian: Direct mode prefers chooser when present, but numeric fallback
				// must remain alive or the baseline smoke test regresses back to
				// `Do Nothing` whenever chooser widgets are absent.
				if (useChooser)
				{
					if (tryReadAutonSelectionChooser(outSelection))
						return true;
				}
				return tryReadAutonSelectionDouble(outSelection);
			},
			(int)eDoNothing,
			(int)eNoAutonTypes,
			20,
			std::chrono::milliseconds(10));

		if (autonIndex == (int)eDoNothing)
			printf("[AI AutonTest] source=missing_or_default final_index=%d\n", autonIndex);
		printf("[AI AutonTest] final_index=%d\n", autonIndex);
		{
			char dbg[256] = {};
			sprintf_s(dbg, "[AI AutonTest] chooser_enabled=%d final_index=%d", useChooser ? 1 : 0, autonIndex);
			AppendDirectAutonChainLog(dbg);
		}
		if (useChooser)
		{
			PublishAutonChooser(
				AutonChooserBase,
				autonOptions,
				autonOptionCount,
				(int)eDoNothing,
				autonIndex,
				autonIndex,
				false);
		}
		AutonTest = (AutonType)autonIndex;
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
		case eTestSmartWayPoints:
		{
			using namespace properties::registry_v1;
			std::string max_speed_name = csz_CommonDrive_;
			max_speed_name += csz_Ship_1D_MAX_SPEED;
			const double max_speed = m_properties ? m_properties->get_number(max_speed_name.c_str(), 1.0) : 1.0;
			m_Primer.AddGoal(SmartWaypoints(m_pParent,max_speed));
		}
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

void AI_Example::Initialize(const Framework::Base::asset_manager* props)
{
	m_AI_Input->Initialize(props);
}

Goal& AI_Example::GetGoal()
{
	return *m_AI_Input;
}
	}
}
