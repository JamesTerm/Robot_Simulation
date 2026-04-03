#include "StdAfx.h"
#include "AI_Input_Example.h"
#include "Goal_Types.h"
#include "SmartDashboard_HelperFunctions.h"
#include "AutonChooser.h"
#include "AutonSelection.h"
#include "AI_BaseController_goals.h"
#include "DirectAutonChainLog.h"
#include "../../../Properties/RegistryV1.h"
// Ian: ManipulatorPlugin is needed by ActivateTest() to query the active manipulator's
// test goals via GetTestGoals()/CreateTestGoal() for the Test chooser.
#include "../../Robot/Manipulator/Manipulator/ManipulatorPlugin.h"
using namespace Framework::Base;

namespace Module
{
	namespace Input
	{
			namespace
			{
			// Ian: The chooser-based auton selection works through SmartDashboard's
			// query source, which each transport backend implements.  NT4 mode
			// uses NT4Backend::TryGetString to read from the NT4 retained cache.
			// All connection modes that expose a chooser widget should be listed here.
			inline bool IsChooserEnabledForCurrentConnection()
			{
				const SmartDashboardConnectionMode mode = SmartDashboard::GetConnectionMode();
				return (mode == SmartDashboardConnectionMode::eDirectConnect) ||
					(mode == SmartDashboardConnectionMode::eNativeLink) ||
					(mode == SmartDashboardConnectionMode::eNetworkTablesV4);
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
				SmartDashboard::PutNumber("Autonomous/Timer", AutonomousTimeLimit - Timer);
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
					SafetyLock = SmartDashboard::GetBoolean("Autonomous/SafetyLock_Drive");
				}
				catch (...)
				{
					//set up default for nothing
					SmartDashboard::PutBoolean("Autonomous/SafetyLock_Drive", false);
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
	// Ian: Excavator-specific test goals have been moved to ExcavatorGoals.h alongside
	// the ExcavatorArm plugin.  They are contributed dynamically via ManipulatorPlugin's
	// GetTestGoals()/CreateTestGoal() virtual methods — the AI goal system never hard-codes
	// manipulator-specific goal classes.  See ExcavatorArm.cpp for the plugin registration.

	//Reserved Drive Tracking
public:
	// Ian: AutonType — the minimal auton chooser for match use.
	// Only "Do Nothing" (safe default) and "Just Move Forward" (minimal safe auton).
	// All other test goals live in the Test chooser (TestType enum below).
	enum AutonType
	{
		eDoNothing,
		eJustMoveForward,
		eNoAutonTypes
	};

	// Ian: TestType — the full test chooser, available in DriverStation Test mode.
	// Drive tests are always present.  Manipulator-specific test goals are contributed
	// dynamically by the active ManipulatorPlugin via GetTestGoals()/CreateTestGoal().
	// This enum only covers the built-in drive tests.  Plugin goals get indices
	// starting at eNoTestDriveTypes.
	enum TestType
	{
		eTestDoNothing,
		// Drive tests
		eTestJustMoveForward,
		eTestJustRotate,
		eTestMoveRotateSequence,
		eTestBoxWayPoints,
		eTestSmartWayPoints,
		eNoTestDriveTypes  // Ian: sentinel — plugin goals are indexed starting here
	};

	static const AutonChooserOption* GetAutonChooserOptions(size_t& optionCount)
	{
		static const AutonChooserOption kOptions[] =
		{
			{eDoNothing, "Do Nothing"},
			{eJustMoveForward, "Just Move Forward"}
		};
		optionCount = _countof(kOptions);
		return kOptions;
	}

	// Ian: GetDriveTestOptions — returns the static drive-only test options.
	// These are always present regardless of which manipulator is active.
	static const AutonChooserOption* GetDriveTestOptions(size_t& optionCount)
	{
		static const AutonChooserOption kDriveOptions[] =
		{
			{eTestDoNothing, "Do Nothing"},
			{eTestJustMoveForward, "Just Move Forward"},
			{eTestJustRotate, "Just Rotate"},
			{eTestMoveRotateSequence, "Move Rotate Sequence"},
			{eTestBoxWayPoints, "Box Waypoints"},
			{eTestSmartWayPoints, "Smart Waypoints"}
		};
		optionCount = _countof(kDriveOptions);
		return kDriveOptions;
	}

	// Ian: BuildTestChooserOptions — dynamically combines drive options with plugin options.
	// Drive tests get their TestType enum indices.  Plugin tests get indices starting at
	// eNoTestDriveTypes, with the plugin-local index recoverable as (chooserIndex - eNoTestDriveTypes).
	// Returns a vector that the caller must keep alive while the chooser is in use.
	static std::vector<AutonChooserOption> BuildTestChooserOptions(Module::Robot::ManipulatorPlugin* plugin)
	{
		size_t driveCount = 0;
		const AutonChooserOption* driveOptions = GetDriveTestOptions(driveCount);
		std::vector<AutonChooserOption> combined(driveOptions, driveOptions + driveCount);

		if (plugin)
		{
			size_t pluginCount = 0;
			const Module::Robot::TestGoalDescriptor* pluginGoals = plugin->GetTestGoals(pluginCount);
			for (size_t i = 0; i < pluginCount; i++)
			{
				AutonChooserOption opt;
				opt.index = (int)eNoTestDriveTypes + pluginGoals[i].index;
				opt.label = pluginGoals[i].label;
				combined.push_back(opt);
			}
		}
		return combined;
	}

	// Ian: BuildAutonChooserOptions — dynamically combines drive auton options with plugin
	// auton options.  Same pattern as BuildTestChooserOptions but for the Auton chooser.
	// Drive autons (Do Nothing, Just Move Forward) always appear.  Plugin autons (e.g.
	// ExcavatorArm's "Grab and Return") are appended with indices starting at eNoAutonTypes.
	// Returns a vector the caller must keep alive while the chooser is in use.
	static std::vector<AutonChooserOption> BuildAutonChooserOptions(Module::Robot::ManipulatorPlugin* plugin)
	{
		size_t driveCount = 0;
		const AutonChooserOption* driveOptions = GetAutonChooserOptions(driveCount);
		std::vector<AutonChooserOption> combined(driveOptions, driveOptions + driveCount);

		if (plugin)
		{
			size_t pluginCount = 0;
			const Module::Robot::TestGoalDescriptor* pluginGoals = plugin->GetAutonGoals(pluginCount);
			for (size_t i = 0; i < pluginCount; i++)
			{
				AutonChooserOption opt;
				opt.index = (int)eNoAutonTypes + pluginGoals[i].index;
				opt.label = pluginGoals[i].label;
				combined.push_back(opt);
			}
		}
		return combined;
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
	// Ian: m_autonPlugin — cached manipulator plugin pointer for auton goal creation.
	// Set by ActivateAuton() before calling Activate().  Null when no manipulator is active.
	Module::Robot::ManipulatorPlugin* m_autonPlugin = nullptr;
	void Activate()
	{
		m_Primer.AsGoal().Terminate();  //sanity check clear previous session
		//reset timer
		m_Timer = 0.0;
		const char* const AutonTestSelection = "AutonTest";
		const char* const AutonChooserBase = "Autonomous/Auton_Selection/AutoChooser";
		// Ian: Build the auton chooser options dynamically.  Drive auton options are always
		// present.  The manipulator plugin contributes its own options via GetAutonGoals().
		std::vector<AutonChooserOption> autonOptionsVec = BuildAutonChooserOptions(m_autonPlugin);
		const AutonChooserOption* autonOptions = autonOptionsVec.data();
		const size_t autonOptionCount = autonOptionsVec.size();
		const int totalAutonCount = autonOptionCount > 0 ? autonOptionsVec.back().index + 1 : (int)eNoAutonTypes;
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
			totalAutonCount,
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

		// Ian: Auton goal creation — drive goals vs plugin goals are distinguished by index range.
		// Built-in auton types (Do Nothing, Just Move Forward) are handled by the switch.
		// Plugin auton goals (indices >= eNoAutonTypes) are created by the manipulator plugin.
		printf("Testing=%d \n", autonIndex);
		Goal* autonGoal = nullptr;
		if (autonIndex < (int)eNoAutonTypes)
		{
			switch ((AutonType)autonIndex)
			{
			case eJustMoveForward:
				autonGoal = new MoveForward(m_pParent);
				break;
			case eDoNothing:
			case eNoAutonTypes:
				break;
			}
		}
		else if (m_autonPlugin)
		{
			// Ian: Plugin auton goal — recover the plugin-local index by subtracting the drive offset.
			const int pluginLocalIndex = autonIndex - (int)eNoAutonTypes;
			autonGoal = m_autonPlugin->CreateAutonGoal(pluginLocalIndex, m_pParent);
		}

		if (autonGoal)
			m_Primer.AddGoal(autonGoal);
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

	// Ian: CreateDriveTestGoal — factory for the built-in drive test goals (TestType enum).
	// Manipulator-specific goals are NOT handled here — they are created by the plugin's
	// CreateTestGoal() method via ActivateTest().
	Goal* CreateDriveTestGoal(TestType testType)
	{
		switch (testType)
		{
		case eTestJustMoveForward:
			return new MoveForward(m_pParent);
		case eTestJustRotate:
			return new RotateWithWait(m_pParent);
		case eTestMoveRotateSequence:
			return new TestMoveRotateSequence(m_pParent);
		case eTestBoxWayPoints:
			return GiveRobotSquareWayPointGoal(m_pParent);
		case eTestSmartWayPoints:
		{
			using namespace properties::registry_v1;
			std::string max_speed_name = csz_CommonDrive_;
			max_speed_name += csz_Ship_1D_MAX_SPEED;
			const double max_speed = m_properties ? m_properties->get_number(max_speed_name.c_str(), 1.0) : 1.0;
			return SmartWaypoints(m_pParent, max_speed);
		}
		case eTestDoNothing:
		case eNoTestDriveTypes:
		default:
			return nullptr;
		}
	}

	// Ian: PublishTestChooser — seeds the Test chooser with the full option list so the
	// dashboard widget populates as soon as the user selects Test mode.  This must be called
	// BEFORE ActivateTest() to give the user time to make a selection in the chooser widget.
	//
	// Called from TeleAutonV2::SetGameMode(eTest) — which fires when the mode dropdown
	// changes, potentially before Enable is pressed.  The auton chooser gets its seed from
	// TransportSmoke; the test chooser gets its seed here because its option list is dynamic
	// (depends on which manipulator is active).
	void PublishTestChooser(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
	{
		if (!IsChooserEnabledForCurrentConnection())
			return;

		std::vector<AutonChooserOption> testOptionsVec = BuildTestChooserOptions(manipulatorPlugin);
		const char* const TestChooserBase = "Test/Test_Selection/TestChooser";

		// Ian: publishSelected = true here (unlike ActivateTest which passes false).
		// On the initial seed we MUST write the `selected` key so the chooser widget
		// has a valid default.  Once the widget exists, the dashboard client writes its
		// own value into `selected` when the user picks an option.  ActivateTest later
		// reads that client-written value without overwriting it.
		PublishAutonChooser(
			TestChooserBase,
			testOptionsVec.data(),
			testOptionsVec.size(),
			(int)eTestDoNothing,   // defaultIndex
			(int)eTestDoNothing,   // activeIndex  (no test running yet)
			(int)eTestDoNothing,   // selectedIndex (seed to default)
			true);                 // publishSelected = true (initial seed)

		printf("[AI TestMode] Test chooser published with %zu options\n", testOptionsVec.size());
	}

	// Ian: ActivateTest — called from TeleAutonV2 when DriverStation is in Test mode
	// and the user presses Start.  Reads the Test chooser, resolves to an index, and
	// creates the appropriate goal using the same goal framework as Auton.
	// This replaces the old hardcoded test(int) switch.
	//
	// The chooser is dynamically built from:
	//   - Built-in drive test goals (indices 0 .. eNoTestDriveTypes-1)
	//   - Plugin-contributed test goals (indices eNoTestDriveTypes+0, eNoTestDriveTypes+1, ...)
	//
	// If the resolved index falls in the drive range, CreateDriveTestGoal() handles it.
	// If it falls in the plugin range, manipulatorPlugin->CreateTestGoal(pluginLocalIndex).
	//
	// manipulatorPlugin: pointer to the active ManipulatorPlugin (may be nullptr).
	void ActivateTest(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
	{
		m_Primer.AsGoal().Terminate();  //clear previous session
		m_Timer = 0.0;

		// Ian: Build the test chooser options dynamically.  Drive goals are always present.
		// The manipulator plugin contributes its own options via GetTestGoals().
		std::vector<AutonChooserOption> testOptionsVec = BuildTestChooserOptions(manipulatorPlugin);
		const AutonChooserOption* testOptions = testOptionsVec.data();
		const size_t testOptionCount = testOptionsVec.size();

		const char* const TestChooserBase = "Test/Test_Selection/TestChooser";
		const char* const TestLegacyKey = "AutonTest";

		const int totalTestCount = testOptionCount > 0 ? testOptionsVec.back().index + 1 : (int)eNoTestDriveTypes;

		const bool useChooser = IsChooserEnabledForCurrentConnection();
		int testIndex = ResolveAutonIndex(
			[&](double& outSelection)
			{
				if (useChooser)
				{
					std::string selectedLabel;
					try { selectedLabel = SmartDashboard::GetString(std::string(TestChooserBase) + "/selected"); }
					catch (...) {}
					if (!selectedLabel.empty())
					{
						outSelection = static_cast<double>(
							ResolveAutonSelectionFromChooser(TestChooserBase, TestLegacyKey,
								testOptions, testOptionCount, (int)eTestDoNothing));
						printf("[AI TestMode] source=chooser selected='%s' index=%d\n",
							selectedLabel.c_str(), (int)outSelection);
						return true;
					}
				}
				// Fallback: try reading numeric AutonTest key
				if (SmartDashboard::TryGetNumber(TestLegacyKey, outSelection))
				{
					printf("[AI TestMode] source=number value=%g\n", outSelection);
					return true;
				}
				return false;
			},
			(int)eTestDoNothing,
			totalTestCount,
			20,
			std::chrono::milliseconds(10));

		printf("[AI TestMode] final_index=%d\n", testIndex);
		{
			char dbg[256] = {};
			sprintf_s(dbg, "[AI TestMode] chooser_enabled=%d final_index=%d", useChooser ? 1 : 0, testIndex);
			AppendDirectAutonChainLog(dbg);
		}

		// Publish the Test chooser state
		if (useChooser)
		{
			PublishAutonChooser(
				TestChooserBase,
				testOptions,
				testOptionCount,
				(int)eTestDoNothing,
				testIndex,
				testIndex,
				false);
		}

		printf("Test=%d \n", testIndex);

		// Ian: Create the goal — drive goals vs plugin goals are distinguished by index range.
		Goal* testGoal = nullptr;
		if (testIndex < (int)eNoTestDriveTypes)
		{
			testGoal = CreateDriveTestGoal((TestType)testIndex);
		}
		else if (manipulatorPlugin)
		{
			// Ian: Plugin goal — recover the plugin-local index by subtracting the drive offset.
			const int pluginLocalIndex = testIndex - (int)eNoTestDriveTypes;
			testGoal = manipulatorPlugin->CreateTestGoal(pluginLocalIndex);
		}

		if (testGoal)
			m_Primer.AddGoal(testGoal);

		m_Primer.AddGoal(new goal_clock(this));
		m_Primer.AddGoal(new goal_watchdog(this));
		m_Status = eActive;
	}

	// Ian: PublishAutonChooserOptions — seeds the Auton chooser with the full option list
	// including any manipulator-contributed auton routines.  Called from TeleAutonV2 when
	// switching to Auton mode or during initialization.  Similar to PublishTestChooser but
	// for the Auton chooser path.
	void PublishAutonChooserOptions(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
	{
		if (!IsChooserEnabledForCurrentConnection())
			return;

		std::vector<AutonChooserOption> autonOptionsVec = BuildAutonChooserOptions(manipulatorPlugin);
		const char* const AutonChooserBase = "Autonomous/Auton_Selection/AutoChooser";

		PublishAutonChooser(
			AutonChooserBase,
			autonOptionsVec.data(),
			autonOptionsVec.size(),
			(int)eDoNothing,    // defaultIndex
			(int)eDoNothing,    // activeIndex (no auton running yet)
			(int)eDoNothing,    // selectedIndex (seed to default)
			true);              // publishSelected = true (initial seed)

		printf("[AI AutonMode] Auton chooser published with %zu options\n", autonOptionsVec.size());
	}

	// Ian: ActivateAuton — stores the manipulator plugin pointer and calls Activate().
	// This is the entry point for auton mode when a manipulator may contribute auton goals.
	// Called from TeleAutonV2 instead of the raw GetGoal().Activate() to thread the plugin.
	void ActivateAuton(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
	{
		m_autonPlugin = manipulatorPlugin;
		Activate();
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

void AI_Example::ActivateTest(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
{
	m_AI_Input->ActivateTest(manipulatorPlugin);
}

void AI_Example::PublishTestChooser(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
{
	m_AI_Input->PublishTestChooser(manipulatorPlugin);
}

void AI_Example::PublishAutonChooserOptions(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
{
	m_AI_Input->PublishAutonChooserOptions(manipulatorPlugin);
}

void AI_Example::ActivateAuton(Module::Robot::ManipulatorPlugin* manipulatorPlugin)
{
	m_AI_Input->ActivateAuton(manipulatorPlugin);
}
	}
}
