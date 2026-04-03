#pragma once
// Ian: ExcavatorGoals.h — test goals for the ExcavatorArm manipulator.
// Ported from Curivator_Goals.cpp (ArmMoveToPosition, ArmGrabSequence, ClawGrabSequence).
//
// Architecture change (oscillation fix): Goals now hold DIRECT references to the virtual joints
// (Ship_1D entities) and call SetIntendedPosition() ONCE in Activate(), matching the Curivator's
// Goal_Rotary_MoveToPosition pattern exactly.  The previous approach wrote SmartDashboard keys
// every frame, which repeatedly called SetIntendedPosition() and reset m_LockShipToPosition=false
// each frame, interfering with Ship_1D's convergence behavior and causing oscillation.
//
// The Curivator pattern: Goal_Rotary_MoveToPosition (Common/Rotary_System.cpp:1210-1266)
//   - Activate(): SetIntendedPosition(position) ONCE
//   - Process(): GetActualPos() vs tolerance → completed; SetRequestedVelocity(0) on completion
// Our analog: Goal_Ship1D_MoveToPosition does exactly the same on Ship_1D (the base of our
// virtual joints).  Move_ArmAndBucket wraps 4 of these in a MultitaskGoal.
//
// Behavior parity notes (ported from legacy ArmGrabSequence behavior):
//   - Per-axis tolerance: Matches CurivatorRobot.lua v1.14 (tolerance=0.15 for all virtual joints)
//     instead of v1.0 default (0.60).  Configurable per-axis via Move_ArmAndBucket parameters.
//   - RobotArmHoldStill: Goal_SetArmPosition now inserts a hold-still sequence AFTER
//     Move_ArmAndBucket, matching legacy SetArmWaypoint (Curivator_Goals.cpp:691-693).
//     Pattern: LockPosition → wait 0.4s → FreezeArm → wait 0.2s → UnfreezeArm → UnlockPosition.
//   - Bucket speed override: ArmGrabSequence step 5 passes bucket_speed=0.5 for slow scoop.
//
// Turret goals are omitted for this iteration.

#include "../../../../Base/Goal.h"
#include "../../../../Modules/Input/AI_Input/Goal_Types.h"
#include "../../../../Modules/Input/AI_Input/AI_BaseController_goals.h"
#include "../../../../Modules/Input/AI_Input/DirectAutonChainLog.h"
#include "../../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "../../../../Modules/Robot/SwerveRobot/SwerveRobot/Ship1D_Legacy.h"

#include <cmath>
#include <array>
#include <functional>

namespace Module {
namespace Robot {
namespace ExcavatorGoals {

using namespace Framework::Base;

// Ian: Default per-axis tolerances matching CurivatorRobot.lua v1.14.
// The legacy lua set tolerance=0.15 for all four virtual joints (arm_xpos, arm_ypos,
// bucket_angle, clasp_angle).  Version 1.0 used 0.6 from arm_pos_common, but v1.14
// tightened all four to 0.15.  Using the tighter values for better behavior parity.
// Note: Legacy lua key "tolerance" maps to Rotary_Props::PrecisionTolerance via
// SCRIPT_INIT_DOUBLE (Rotary_System.cpp:1059).
constexpr double kDefaultTolerance_XPos = 0.15;
constexpr double kDefaultTolerance_YPos = 0.15;
constexpr double kDefaultTolerance_BucketAngle = 0.15;
constexpr double kDefaultTolerance_ClaspAngle = 0.15;

// Ian: Goal_Ship1D_MoveToPosition — exact analog of Curivator's Goal_Rotary_MoveToPosition
// (Common/Rotary_System.cpp:1210-1266) but targeting Ship_1D instead of Rotary_Position_Control.
//
// Key behaviors matching the Curivator:
//   - Activate(): saves default max speeds, applies speed ratios, calls SetIntendedPosition ONCE
//   - Process(): checks GetIntendedPosition()==m_Position (detects external hijack),
//     then checks |GetPos_m() - m_Position| < tolerance → completed
//   - On completion: SetRequestedVelocity(0.0) to stop, restore max speeds
//   - Terminate(): sets m_Terminate flag for clean abort
//
// This is the fundamental building block.  Move_ArmAndBucket creates a MultitaskGoal of 4 of
// these (one per virtual joint) so all joints converge in parallel — same as Curivator line 455.
class Goal_Ship1D_MoveToPosition : public AtomicGoal
{
private:
	Legacy::Ship_1D& m_ship;
	double m_Position;
	double m_Tolerance;
	double m_MaxForwardSpeedRatio;
	double m_MaxReverseSpeedRatio;
	double m_DefaultForwardSpeed = 0.0;
	double m_DefaultReverseSpeed = 0.0;
	bool m_Terminate = false;
public:
	Goal_Ship1D_MoveToPosition(Legacy::Ship_1D& ship, double position, double tolerance = 0.15,
		double MaxForwardSpeedRatio = 1.0, double MaxReverseSpeedRatio = 1.0)
		: m_ship(ship), m_Position(position), m_Tolerance(tolerance)
		, m_MaxForwardSpeedRatio(MaxForwardSpeedRatio), m_MaxReverseSpeedRatio(MaxReverseSpeedRatio)
	{
		m_Status = eInactive;
	}
	~Goal_Ship1D_MoveToPosition()
	{
		Terminate();  // more for completion — matches Curivator
	}
	virtual void Activate() override
	{
		m_Status = eActive;
		m_DefaultForwardSpeed = m_ship.GetMaxSpeedForward();
		m_DefaultReverseSpeed = m_ship.GetMaxSpeedReverse();
		m_ship.SetMaxSpeedForward(m_MaxForwardSpeedRatio * m_DefaultForwardSpeed);
		m_ship.SetMaxSpeedReverse(m_MaxReverseSpeedRatio * m_DefaultReverseSpeed);
		// Ian: THE critical line — SetIntendedPosition is called ONCE, not every frame.
		// Ship_1D's position-chasing physics (TimeChange lines 428-469) will smoothly
		// converge to this waypoint.  m_LockShipToPosition is set to false ONCE here,
		// enabling the waypoint-seeking code path.
		m_ship.SetIntendedPosition(m_Position);
	}
	virtual Goal_Status Process(double dTime_s) override
	{
		if (m_Terminate)
		{
			if (m_Status == eActive)
				m_Status = eFailed;
			return m_Status;
		}
		ActivateIfInactive();
		if (m_Status == eActive)
		{
			// Ian: Check that nobody else hijacked the intended position
			// (matches Curivator Rotary_System.cpp line 1244)
			if (m_ship.GetIntendedPosition() == m_Position)
			{
				const double position_delta = m_ship.GetPos_m() - m_Position;
				if (fabs(position_delta) < m_Tolerance)
				{
					m_Status = eCompleted;
					m_ship.SetRequestedVelocity(0.0);  // stop it
					// Restore speeds
					m_ship.SetMaxSpeedForward(m_DefaultForwardSpeed);
					m_ship.SetMaxSpeedReverse(m_DefaultReverseSpeed);
				}
			}
			else
			{
				printf("Goal_Ship1D_MoveToPosition failed — position hijacked\n");
				m_Status = eFailed;
			}
		}
		return m_Status;
	}
	virtual void Terminate() override
	{
		m_Terminate = true;
	}
};

// Ian: Move_ArmAndBucket — creates a MultitaskGoal with 4 Goal_Ship1D_MoveToPosition goals
// (one per virtual joint) so all joints converge in parallel.
// Matches Curivator_Goals.cpp Move_ArmAndBucket (line 452-461).
//
// The speed ratio parameters match the Curivator's interface: default 1.0 = use full max speed.
// The Curivator's ArmMoveToPosition (line 706-741) calls this with all ratios = 1.0.
//
// Per-axis tolerances: Each axis gets its own tolerance value matching the legacy lua v1.14
// defaults (0.15 for all four).  This replaces the previous single hardcoded 0.60 tolerance.
inline Goal* Move_ArmAndBucket(
	Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
	Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
	double length_in, double height_in, double bucket_angle_deg, double clasp_angle_deg,
	double length_tolerance = kDefaultTolerance_XPos,
	double height_tolerance = kDefaultTolerance_YPos,
	double bucket_tolerance = kDefaultTolerance_BucketAngle,
	double clasp_tolerance = kDefaultTolerance_ClaspAngle,
	double length_speed = 1.0, double height_speed = 1.0,
	double bucket_speed = 1.0, double clasp_speed = 1.0)
{
	MultitaskGoal* goal = new MultitaskGoal(true);
	// Ian: Flat-lined (not nested MultitaskGoals) — easier to debug.
	// Matches Curivator line 456-460 comment: "easier to debug keeping it more flat lined"
	goal->AddGoal(new Goal_Ship1D_MoveToPosition(armXpos, length_in, length_tolerance, length_speed, length_speed));
	goal->AddGoal(new Goal_Ship1D_MoveToPosition(armYpos, height_in, height_tolerance, height_speed, height_speed));
	goal->AddGoal(new Goal_Ship1D_MoveToPosition(bucketAngle, bucket_angle_deg, bucket_tolerance, bucket_speed, bucket_speed));
	goal->AddGoal(new Goal_Ship1D_MoveToPosition(claspAngle, clasp_angle_deg, clasp_tolerance, clasp_speed, clasp_speed));
	return goal;
}

// Ian: Goal_ArmHoldStill — port of Curivator's RobotArmHoldStill (Curivator_Goals.cpp:538-552).
// Executed AFTER each Move_ArmAndBucket to let the arm settle before advancing to the next step.
//
// Legacy sequence (LIFO, added in reverse):
//   1. Robot_LockPosition ON   — makes IK use last-captured positions instead of live values
//   2. Wait 0.4s               — let the arm coast to a stop
//   3. Robot_FreezeArm ON      — zero all velocities on all 8 axes (4 virtual + 4 physical)
//   4. Wait 0.2s               — hold still
//   5. Robot_FreezeArm OFF     — unfreeze
//   6. Robot_LockPosition OFF  — unlock
//
// In the modern architecture, FreezeArm and LockPosition are boolean members on ExcavatorArm
// (not event-map dispatched).  We access them via SmartDashboard keys that the ExcavatorArm
// reads each frame — the same mechanism used for EnableArmAutoPosition.
//
// However, ExcavatorArm currently reads FreezeArm/LockPosition as internal members set by the
// CommandScheduler, not from SmartDashboard.  To avoid adding new SmartDashboard coupling, we
// use a callback-based approach: the caller provides lambdas that set FreezeArm/LockPosition
// on the ExcavatorArm instance.
class Goal_ArmNotify : public AtomicGoal
{
private:
	std::function<void()> m_action;
public:
	Goal_ArmNotify(std::function<void()> action) : m_action(std::move(action))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override { m_Status = eActive; }
	virtual Goal_Status Process(double dTime_s) override
	{
		ActivateIfInactive();
		m_action();
		m_Status = eCompleted;
		return m_Status;
	}
};

// Ian: Goal_ArmHoldStill — composite goal that performs the legacy RobotArmHoldStill sequence.
// Uses callbacks to set FreezeArm/LockPosition on the ExcavatorArm instance, matching the
// legacy event-dispatched approach (Robot_FreezeArm / Robot_LockPosition events).
class Goal_ArmHoldStill : public Generic_CompositeGoal
{
public:
	using SetBoolFn = std::function<void(bool)>;

	Goal_ArmHoldStill(SetBoolFn setFreezeArm, SetBoolFn setLockPosition)
		: Generic_CompositeGoal(true)  // AutoActivate
		, m_setFreezeArm(std::move(setFreezeArm))
		, m_setLockPosition(std::move(setLockPosition))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		if (m_Status == eActive) return;
		// Ian: LIFO order — added in reverse of execution.
		// Execution: LockPosition(true) → Wait(0.4) → FreezeArm(true) → Wait(0.2) → FreezeArm(false) → LockPosition(false)
		AddSubgoal(new Goal_ArmNotify([this]() { m_setLockPosition(false); }));
		AddSubgoal(new Goal_ArmNotify([this]() { m_setFreezeArm(false); }));
		AddSubgoal(new Goal_Wait(0.2));
		AddSubgoal(new Goal_ArmNotify([this]() { m_setFreezeArm(true); }));
		AddSubgoal(new Goal_Wait(0.4));
		AddSubgoal(new Goal_ArmNotify([this]() { m_setLockPosition(true); }));
		m_Status = eActive;
	}
private:
	SetBoolFn m_setFreezeArm;
	SetBoolFn m_setLockPosition;
};

// Ian: Goal_SetArmPosition — drives ExcavatorArm virtual joints to a target pose.
// Refactored to use Move_ArmAndBucket (4 parallel Goal_Ship1D_MoveToPosition goals)
// instead of SmartDashboard every-frame writes.  This matches the Curivator's
// ArmMoveToPosition (line 682) which delegates to Move_ArmAndBucket.
//
// This is a Generic_CompositeGoal wrapping Move_ArmAndBucket + Goal_ArmHoldStill so it
// integrates seamlessly with ArmGrabSequence's sequential flow.
//
// Legacy SetArmWaypoint::Activate() (Curivator_Goals.cpp:691-693):
//   AddSubgoal(new RobotArmHoldStill(m_Parent));
//   AddSubgoal(Move_ArmAndBucket(...));
// Execution order: Move_ArmAndBucket first, then RobotArmHoldStill to settle.
//
// Ian: AutoActivate=true is REQUIRED because when Goal_SetArmPosition is used as a
// subgoal inside ArmGrabSequence (a Generic_CompositeGoal), ProcessSubgoals() calls
// Process() on the front subgoal but never calls Activate() on it explicitly.
// Generic_CompositeGoal::Process() only calls ActivateIfInactive() when AutoActivate
// is true.  Without this, the subgoal stays eInactive forever, returns eInactive to
// ProcessSubgoals, which propagates up and kills the entire sequence.
//
// The legacy Curivator's SetArmWaypoint (Curivator_Goals.cpp:661) worked around this
// by calling Activate() in its constructor.  AutoActivate=true is the cleaner modern
// equivalent — it lets the goal framework handle activation timing correctly.
class Goal_SetArmPosition : public Generic_CompositeGoal
{
private:
	Legacy::Ship_1D& m_armXpos;
	Legacy::Ship_1D& m_armYpos;
	Legacy::Ship_1D& m_bucketAngle;
	Legacy::Ship_1D& m_claspAngle;
	double m_targetXpos;
	double m_targetYpos;
	double m_targetBucketAngle;
	double m_targetClaspAngle;
	// Per-axis speed ratios (default 1.0 = full speed)
	double m_lengthSpeed;
	double m_heightSpeed;
	double m_bucketSpeed;
	double m_claspSpeed;
	// FreezeArm/LockPosition callbacks for hold-still behavior
	Goal_ArmHoldStill::SetBoolFn m_setFreezeArm;
	Goal_ArmHoldStill::SetBoolFn m_setLockPosition;
public:
	Goal_SetArmPosition(
		Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
		Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
		double xpos, double ypos, double bucketAngleDeg, double claspAngleDeg,
		Goal_ArmHoldStill::SetBoolFn setFreezeArm = nullptr,
		Goal_ArmHoldStill::SetBoolFn setLockPosition = nullptr,
		double lengthSpeed = 1.0, double heightSpeed = 1.0,
		double bucketSpeed = 1.0, double claspSpeed = 1.0)
		: Generic_CompositeGoal(true)  // Ian: AutoActivate — see comment above
		, m_armXpos(armXpos), m_armYpos(armYpos)
		, m_bucketAngle(bucketAngle), m_claspAngle(claspAngle)
		, m_targetXpos(xpos), m_targetYpos(ypos)
		, m_targetBucketAngle(bucketAngleDeg), m_targetClaspAngle(claspAngleDeg)
		, m_lengthSpeed(lengthSpeed), m_heightSpeed(heightSpeed)
		, m_bucketSpeed(bucketSpeed), m_claspSpeed(claspSpeed)
		, m_setFreezeArm(std::move(setFreezeArm)), m_setLockPosition(std::move(setLockPosition))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		if (m_Status == eActive) return;
		// Ian: Ensure Layer 2 is active — write EnableArmAutoPosition=true
		SmartDashboard::PutBoolean("Manipulator/EnableArmAutoPosition", true);
		// Ian: Write target positions under Test/ folder group for telemetry visibility.
		// These are informational only — the virtual joints are driven directly by
		// Goal_Ship1D_MoveToPosition, not through SmartDashboard.
		SmartDashboard::PutNumber("Test/target_arm_xpos", m_targetXpos);
		SmartDashboard::PutNumber("Test/target_arm_ypos", m_targetYpos);
		SmartDashboard::PutNumber("Test/target_bucket_angle", m_targetBucketAngle);
		SmartDashboard::PutNumber("Test/target_clasp_angle", m_targetClaspAngle);
		// Ian: LIFO order — Move_ArmAndBucket executes first, then HoldStill to settle.
		// Matches legacy SetArmWaypoint::Activate() (Curivator_Goals.cpp:691-693).
		if (m_setFreezeArm && m_setLockPosition)
		{
			AddSubgoal(new Goal_ArmHoldStill(m_setFreezeArm, m_setLockPosition));
		}
		AddSubgoal(Move_ArmAndBucket(
			m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			m_targetXpos, m_targetYpos, m_targetBucketAngle, m_targetClaspAngle,
			kDefaultTolerance_XPos, kDefaultTolerance_YPos,
			kDefaultTolerance_BucketAngle, kDefaultTolerance_ClaspAngle,
			m_lengthSpeed, m_heightSpeed, m_bucketSpeed, m_claspSpeed));
		m_Status = eActive;
	}
};

// Ian: Factory — reads target from SmartDashboard Test/ keys, creates Goal_SetArmPosition.
// Ported from Curivator_Goals.cpp ArmMoveToPosition (line 690-702).
// Keys are grouped under Test/ for dashboard layout (e.g. Test/target_arm_xpos).
// Uses direct SmartDashboard reads (not Auton_Smart_GetMultiValue which would double-prefix).
inline Goal* TestArmMove(
	Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
	Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle)
{
	double xpos = 25.0, ypos = -7.0, bucket = 78.0, clasp = 13.0;
	SmartDashboard::TryGetNumber("Test/target_arm_xpos", xpos);
	SmartDashboard::TryGetNumber("Test/target_arm_ypos", ypos);
	SmartDashboard::TryGetNumber("Test/target_bucket_angle", bucket);
	SmartDashboard::TryGetNumber("Test/target_clasp_angle", clasp);
	return new Goal_SetArmPosition(armXpos, armYpos, bucketAngle, claspAngle,
		xpos, ypos, bucket, clasp);
}

// Ian: ArmGrabSequence — sequential pickup: hover -> approach -> grab -> retract.
// Ported from Curivator_Goals.cpp ArmGrabSequence (line 755-785), without turret.
// Uses the same starting/hover/pickup positions from the original Curivator constants.
//
// Behavior parity with legacy:
//   - Each step uses Goal_SetArmPosition which includes RobotArmHoldStill (0.6s settle)
//   - Step 5 (scoop) passes bucket_speed=0.5 matching legacy line 825: bucket_Angle_deg_speed=0.5
//   - Tolerance tightened to 0.15 per-axis (legacy v1.14 lua values)
class ArmGrabSequence : public Generic_CompositeGoal
{
private:
	Legacy::Ship_1D& m_armXpos;
	Legacy::Ship_1D& m_armYpos;
	Legacy::Ship_1D& m_bucketAngle;
	Legacy::Ship_1D& m_claspAngle;
	double m_length_in;
	double m_height_in;
	// FreezeArm/LockPosition callbacks passed through to each Goal_SetArmPosition
	Goal_ArmHoldStill::SetBoolFn m_setFreezeArm;
	Goal_ArmHoldStill::SetBoolFn m_setLockPosition;
public:
	ArmGrabSequence(
		Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
		Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
		double length_in, double height_in,
		Goal_ArmHoldStill::SetBoolFn setFreezeArm = nullptr,
		Goal_ArmHoldStill::SetBoolFn setLockPosition = nullptr)
		: m_armXpos(armXpos), m_armYpos(armYpos)
		, m_bucketAngle(bucketAngle), m_claspAngle(claspAngle)
		, m_length_in(length_in), m_height_in(height_in)
		, m_setFreezeArm(std::move(setFreezeArm))
		, m_setLockPosition(std::move(setLockPosition))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		if (m_Status == eActive) return;
		// Ian: Positions ported from Curivator_Goals.cpp Robot_TesterCode path.
		// StartingPosition = {13.0, 4.0, 60.0, 5.0} (xpos, ypos, bucket, clasp)
		// HoverPosition    = {39.0, 0.0, 90.0, 45.0}
		// PickupPosition   = {39.0, -20.0, 90.0, 45.0}
		const double StartXpos = 13.0, StartYpos = 4.0, StartBucket = 60.0, StartClasp = 5.0;
		const double PickupBucket = 90.0, PickupClasp = 45.0;
		// Sequence (LIFO — added in reverse execution order for Generic_CompositeGoal):
		// 1. Return to starting position
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			StartXpos, StartYpos, StartBucket, StartClasp,
			m_setFreezeArm, m_setLockPosition));
		// 2. Hover — lift from pickup to hover height
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			m_length_in, 0.0, PickupBucket, PickupClasp,
			m_setFreezeArm, m_setLockPosition));
		// 3. Pickup position — open clasp at pickup
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			m_length_in, m_height_in, PickupBucket, PickupClasp,
			m_setFreezeArm, m_setLockPosition));
		// 4. Close clasp (grab)
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			m_length_in, m_height_in, PickupBucket, -7.0,
			m_setFreezeArm, m_setLockPosition));
		// 5. Rotate bucket slowly (approach) — bucket_speed=0.5 matching legacy line 825
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			m_length_in, m_height_in, 40.0, -7.0,
			m_setFreezeArm, m_setLockPosition,
			1.0, 1.0, 0.5, 1.0));  // bucket_speed=0.5
		// 6. Start from starting position
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			StartXpos, StartYpos, 50.0, -7.0,
			m_setFreezeArm, m_setLockPosition));
		m_Status = eActive;
	}
};

inline Goal* TestArmGrabSequence(
	Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
	Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
	Goal_ArmHoldStill::SetBoolFn setFreezeArm = nullptr,
	Goal_ArmHoldStill::SetBoolFn setLockPosition = nullptr)
{
	double length_in = 38.0, height_in = -20.0;
	SmartDashboard::TryGetNumber("Test/testarm_length", length_in);
	SmartDashboard::TryGetNumber("Test/testarm_height", height_in);
	return new ArmGrabSequence(armXpos, armYpos, bucketAngle, claspAngle,
		length_in, height_in,
		std::move(setFreezeArm), std::move(setLockPosition));
}

// Ian: ClawGrabSequence — close clasp test.
// Ported from Curivator_Goals.cpp ClawGrabSequence (line 838-853).
class ClawGrabSequence : public Generic_CompositeGoal
{
private:
	Legacy::Ship_1D& m_armXpos;
	Legacy::Ship_1D& m_armYpos;
	Legacy::Ship_1D& m_bucketAngle;
	Legacy::Ship_1D& m_claspAngle;
	Goal_ArmHoldStill::SetBoolFn m_setFreezeArm;
	Goal_ArmHoldStill::SetBoolFn m_setLockPosition;
public:
	ClawGrabSequence(
		Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
		Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
		Goal_ArmHoldStill::SetBoolFn setFreezeArm = nullptr,
		Goal_ArmHoldStill::SetBoolFn setLockPosition = nullptr)
		: m_armXpos(armXpos), m_armYpos(armYpos)
		, m_bucketAngle(bucketAngle), m_claspAngle(claspAngle)
		, m_setFreezeArm(std::move(setFreezeArm))
		, m_setLockPosition(std::move(setLockPosition))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		if (m_Status == eActive) return;
		// Wait to observe voltage behavior, then close clasp
		AddSubgoal(new Goal_Wait(5.0));
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			19.14, -1.60, 86.457, 10.0,
			m_setFreezeArm, m_setLockPosition));
		AddSubgoal(new Goal_SetArmPosition(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			19.14, -1.60, 86.457, 30.0,
			m_setFreezeArm, m_setLockPosition));
		m_Status = eActive;
	}
};

// Ian: Goal_LogMessage — lightweight atomic goal that prints a diagnostic message at Activate
// time, then immediately completes.  Used to trace phase transitions inside composite goal
// chains without affecting behavior.  Logs to both console (printf) and the auton chain log
// file so we can correlate with SmartDashboard telemetry after the fact.
class Goal_LogMessage : public AtomicGoal
{
private:
	std::string m_message;
	Module::Input::AI_Input* m_controller;  // optional, for position snapshot
public:
	Goal_LogMessage(const char* message, Module::Input::AI_Input* controller = nullptr)
		: m_message(message), m_controller(controller)
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		m_Status = eActive;
		if (m_controller)
		{
			Vec2D pos = m_controller->GetCurrentPosition();
			printf("[AutonLog] %s  pos=(%f, %f)\n", m_message.c_str(), pos.x(), pos.y());
			char dbg[512] = {};
			sprintf_s(dbg, "[AutonLog] %s  pos=(%f, %f)", m_message.c_str(), pos.x(), pos.y());
			Module::Input::AppendDirectAutonChainLog(dbg);
		}
		else
		{
			printf("[AutonLog] %s\n", m_message.c_str());
			Module::Input::AppendDirectAutonChainLog("[AutonLog] " + m_message);
		}
	}
	virtual Goal_Status Process(double dTime_s) override
	{
		ActivateIfInactive();
		// Immediately complete — this goal only exists to emit a log line
		m_Status = eCompleted;
		return m_Status;
	}
	virtual void Terminate() override {}
};

// Ian: AutonGrabAndReturn — full autonomous routine that combines driving with arm manipulation.
// Sequence:
//   0. Drive to center (0,0) — normalizes starting position
//   1. Drive to waypoint 1 (forward-right diagonal)
//   2. Perform arm grab sequence
//   3. Drive to waypoint 2 (forward-left diagonal)
//   4. Perform arm grab sequence
//   5. Drive back to center (0,0)
//
// This creates a triangle pattern on the field which looks visually interesting.
// The arm grab sequence at each stop uses the same ArmGrabSequence (hover -> approach ->
// grab -> retract) from the test goals, giving a realistic "pick up game pieces" autonomous.
//
// Design: This goal lives in ExcavatorGoals.h because it uses ExcavatorArm-specific arm
// sequences (ArmGrabSequence).  The AI system creates it via ManipulatorPlugin::CreateAutonGoal()
// — the loose coupling point.  Future manipulators will override CreateAutonGoal() with their
// own autonomous routines without touching the AI module.
//
// Ian: Diagnostic logging — this goal is heavily instrumented because waypoint arrival is the
// most likely failure point.  Goal_LogMessage markers fire at each phase boundary so the
// console + auton chain log show exactly which step the robot is on.  Process() prints a
// throttled position summary once per second so you can watch convergence without flooding.
// Disable the #if 1 blocks once the auton is confirmed working.
class AutonGrabAndReturn : public Generic_CompositeGoal
{
private:
	Module::Input::AI_Input* m_controller;
	Legacy::Ship_1D& m_armXpos;
	Legacy::Ship_1D& m_armYpos;
	Legacy::Ship_1D& m_bucketAngle;
	Legacy::Ship_1D& m_claspAngle;
	Goal_ArmHoldStill::SetBoolFn m_setFreezeArm;
	Goal_ArmHoldStill::SetBoolFn m_setLockPosition;
	// Ian: Diagnostic — accumulated time for throttled position logging
	double m_logTimer = 0.0;
	int m_phaseIndex = 0;  // tracks which phase we're in (increments as log markers complete)
public:
	AutonGrabAndReturn(
		Module::Input::AI_Input* controller,
		Legacy::Ship_1D& armXpos, Legacy::Ship_1D& armYpos,
		Legacy::Ship_1D& bucketAngle, Legacy::Ship_1D& claspAngle,
		Goal_ArmHoldStill::SetBoolFn setFreezeArm = nullptr,
		Goal_ArmHoldStill::SetBoolFn setLockPosition = nullptr)
		: m_controller(controller)
		, m_armXpos(armXpos), m_armYpos(armYpos)
		, m_bucketAngle(bucketAngle), m_claspAngle(claspAngle)
		, m_setFreezeArm(std::move(setFreezeArm))
		, m_setLockPosition(std::move(setLockPosition))
	{
		m_Status = eInactive;
	}
	virtual void Activate() override
	{
		if (m_Status == eActive) return;

		// Ian: Waypoints in meters (absolute field coordinates).
		// Vec2D is (x=east, y=north).
		// Waypoint 1: forward-right diagonal (~8ft out, NE quadrant)
		// Waypoint 2: forward-left diagonal (~8ft out, NW quadrant)
		// Creates a triangle drive pattern that looks visually appealing.
		const double wp1_east  =  Feet2Meters(5.0);   // ~1.5m east
		const double wp1_north =  Feet2Meters(6.0);   // ~1.8m north
		const double wp2_east  = -Feet2Meters(5.0);   // ~1.5m west
		const double wp2_north =  Feet2Meters(6.0);   // ~1.8m north

		// Ian: ArmGrabSequence parameters — same as TestArmGrabSequence defaults.
		// length_in = 38" reach, height_in = -20" (reaching down to ground).
		const double grab_length_in = 38.0;
		const double grab_height_in = -20.0;

		// Ian: Power (speed) for waypoint driving.  2.5 matches the box waypoint test
		// (AI_Input_Example.cpp:250) — fast enough to be interesting, not reckless.
		const double drivePower = 2.5;

		// Ian: safestop_tolerance — how close to a waypoint before we consider it "hit".
		// Feet2Meters(1.0) = ~0.3m was too tight — the robot circled waypoints endlessly
		// because the physics simulation's turning radius and momentum cause overshoot on
		// absolute waypoints.  The relative-position variant (Goal_Ship_MoveToRelativePosition)
		// works with 0.03m tolerance because it sets the target from the robot's current
		// position — no accumulated error.  For absolute waypoints the robot approaches
		// from various angles, so we need ~3ft (~0.9m) to reliably "hit" and proceed.
		// Tune this down once PID/steering is improved; tune up if the robot still circles.
		const double wpTolerance = Feet2Meters(3.0);

		// Ian: Subgoals added in LIFO order (reverse execution) for Generic_CompositeGoal.
		// Execution order:
		//   0. Log + Drive to center (normalize start)
		//   1. Log + Wait settle
		//   2. Log + Drive to waypoint 1
		//   3. Log + Arm grab sequence at waypoint 1
		//   4. Log + Drive to waypoint 2
		//   5. Log + Arm grab sequence at waypoint 2
		//   6. Log + Drive back to center
		//   7. Log + Wait settle (final)

		// --- 7. Final settle ---
		AddSubgoal(new Goal_Wait(0.500));
		AddSubgoal(new Goal_LogMessage("Phase 7: Final settle (0.5s wait)", m_controller));

		// --- 6. Drive back to center ---
		{
			WayPoint wp;
			wp.Position = Vec2D(0.0, 0.0);
			wp.Power = drivePower;
			AddSubgoal(new Goal_Ship_MoveToPosition(m_controller, wp, true, true, wpTolerance));
			AddSubgoal(new Goal_LogMessage("Phase 6: Driving back to center (0,0)", m_controller));
		}

		// --- 5. Arm grab sequence at waypoint 2 ---
		AddSubgoal(new ArmGrabSequence(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			grab_length_in, grab_height_in, m_setFreezeArm, m_setLockPosition));
		AddSubgoal(new Goal_LogMessage("Phase 5: Arm grab sequence at waypoint 2", m_controller));

		// --- 4. Drive to waypoint 2 ---
		{
			WayPoint wp;
			wp.Position = Vec2D(wp2_east, wp2_north);
			wp.Power = drivePower;
			AddSubgoal(new Goal_Ship_MoveToPosition(m_controller, wp, true, true, wpTolerance));
			AddSubgoal(new Goal_LogMessage("Phase 4: Driving to waypoint 2 (west)", m_controller));
		}

		// --- 3. Arm grab sequence at waypoint 1 ---
		AddSubgoal(new ArmGrabSequence(m_armXpos, m_armYpos, m_bucketAngle, m_claspAngle,
			grab_length_in, grab_height_in, m_setFreezeArm, m_setLockPosition));
		AddSubgoal(new Goal_LogMessage("Phase 3: Arm grab sequence at waypoint 1", m_controller));

		// --- 2. Drive to waypoint 1 ---
		{
			WayPoint wp;
			wp.Position = Vec2D(wp1_east, wp1_north);
			wp.Power = drivePower;
			AddSubgoal(new Goal_Ship_MoveToPosition(m_controller, wp, true, true, wpTolerance));
			AddSubgoal(new Goal_LogMessage("Phase 2: Driving to waypoint 1 (east)", m_controller));
		}

		// --- 1. Initial settle ---
		AddSubgoal(new Goal_Wait(0.500));
		AddSubgoal(new Goal_LogMessage("Phase 1: Initial settle (0.5s wait)", m_controller));

		// --- 0. Drive to center first (normalize starting position) ---
		{
			WayPoint wp;
			wp.Position = Vec2D(0.0, 0.0);
			wp.Power = drivePower;
			AddSubgoal(new Goal_Ship_MoveToPosition(m_controller, wp, true, true, wpTolerance));
			AddSubgoal(new Goal_LogMessage("Phase 0: Driving to center to normalize start", m_controller));
		}

		m_Status = eActive;
		{
			Vec2D startPos = m_controller->GetCurrentPosition();
			printf("[AutonGrabAndReturn] Activated: start=(%f,%f) wp1=(%f,%f) wp2=(%f,%f) tol=%f\n",
				startPos.x(), startPos.y(), wp1_east, wp1_north, wp2_east, wp2_north, wpTolerance);
			char dbg[512] = {};
			sprintf_s(dbg, "[AutonGrabAndReturn] Activated: start=(%f,%f) wp1=(%f,%f) wp2=(%f,%f) tol=%f",
				startPos.x(), startPos.y(), wp1_east, wp1_north, wp2_east, wp2_north, wpTolerance);
			Module::Input::AppendDirectAutonChainLog(dbg);
		}
	}

	// Ian: Override Process to add throttled position logging every ~1 second.
	// This lets us watch the robot converge on waypoints in real time without flooding
	// the console.  Disable the #if 1 block once auton is confirmed working.
	virtual Goal_Status Process(double dTime_s) override
	{
		ActivateIfInactive();
		if (m_Status == eInactive)
			return m_Status;

#if 1
		// Ian: Throttled diagnostic — print position once per second
		m_logTimer += dTime_s;
		if (m_logTimer >= 1.0)
		{
			m_logTimer = 0.0;
			Vec2D pos = m_controller->GetCurrentPosition();
			printf("[AutonGrabAndReturn] heartbeat: pos=(%f, %f) status=%d\n",
				pos.x(), pos.y(), (int)m_Status);
		}
#endif

		if (m_Status == eActive)
			m_Status = ProcessSubgoals(dTime_s);

		return m_Status;
	}
};

}  // namespace ExcavatorGoals
}  // namespace Robot
}  // namespace Module
