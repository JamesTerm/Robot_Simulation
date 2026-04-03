// Ian: ExcavatorArm.cpp — ported from Curivator_Robot.cpp (2015–2019 NASA SRR excavator).
// The geometry math is a faithful port; variable names match the original so future maintainers
// can cross-reference against the CAD-derived Curivator source.  Only the framework wiring has
// changed: old Rotary_Position_Control → modern RotarySystem_Position, old SmartDashboard calls
// remain identical (they route through SmartDashboardDirectPublishSink to NT4).
//
// The forward kinematics compute tip position from shaft extensions.  The inverse kinematics
// (ComputeArmPosition) go the other direction for goal-seeking in Phase 2.

#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Misc.h"
#include "../../../../Base/AssetManager.h"
#include "../../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "ExcavatorArm.h"
#include "../../../../Modules/Output/OSG_Viewer/OSG_Viewer/ManipulatorArm_UI.h"
// Ian: ExcavatorGoals.h must be included HERE, outside any namespace block, because it
// transitively pulls in standard headers (<chrono>, <cmath>, etc.) that would otherwise
// be parsed inside Module::Robot and produce "Module::Robot::std::ratio" compile errors.
#include "ExcavatorGoals.h"

#include <algorithm>
#include <cmath>

namespace Module {
namespace Robot {

using namespace ExcavatorGeometry;

#pragma region _Helpers_

// Ian: LawOfCosines — given all three side lengths of a triangle, returns the angle opposite
// side c.  This is the workhorse of all the dart-to-angle conversions.  Ported verbatim from
// Curivator_Robot.cpp line 35.
double ExcavatorArm::LawOfCosines(double a, double b, double c)
{
	// c^2 = a^2 + b^2 - 2ab*cos(C)
	// Rearranged: cos(C) = (a^2 + b^2 - c^2) / (2ab)
	const double cos_C = -1.0 * (((c * c) - (b * b) - (a * a)) / (2.0 * a * b));
	return std::acos(cos_C);
}

double ExcavatorArm::GetDistance(double x1, double y1, double x2, double y2)
{
	const double dx = std::fabs(x2 - x1);
	const double dy = std::fabs(y2 - y1);
	return std::sqrt((dx * dx) + (dy * dy));
}

double ExcavatorArm::EnforceShaftLimits(double value, double minRange, double maxRange)
{
	double ret = std::max(std::min(value, maxRange), minRange);
	// Check for NaN — if the acos gave us garbage, fall back to center
	if (!(value < 0.0 || value > 0.0 || value == 0.0))
		ret = ((maxRange - minRange) / 2.0) + minRange;
	return ret;
}

#pragma endregion

#pragma region _Construction and Init_

ExcavatorArm::ExcavatorArm()
{
	m_simulated_positions.fill(0.0);
	m_starting_positions.fill(0.0);
	m_virtualJoints.fill(nullptr);
}

const char* ExcavatorArm::GetJointName(ExcavatorJoint joint)
{
	switch (joint)
	{
	case ExcavatorJoint::eBigArm: return "BigArm";
	case ExcavatorJoint::eBoom:   return "Boom";
	case ExcavatorJoint::eBucket: return "Bucket";
	case ExcavatorJoint::eClasp:  return "Clasp";
	default: return "Unknown";
	}
}

void ExcavatorArm::SetDefaultJointProps(ExcavatorJoint joint, double minRange, double maxRange, double startingPos)
{
	size_t idx = static_cast<size_t>(joint);
	rotary_properties& p = m_joint_props[idx];
	p.Init();  // fill with sane defaults

	// Entity1D — linear actuator modeled as a 1D entity
	p.entity_props.m_StartingPosition = startingPos;
	p.entity_props.m_Mass = 1.5;           // kg — reasonable for a dart + load
	p.entity_props.m_Dimension = 0.0254;   // 1 inch in meters (not critical for open loop)
	p.entity_props.m_IsAngular = false;     // linear actuator

	// ======================================================================
	// Ian: ALL Ship_1D and Rotary_Props values below are ported 1:1 from CurivatorRobot.lua.
	// The students tuned these PID gains, tolerances, dead zones, and predict values during
	// matches — they are match-tested and correct.  Do NOT change them without comparing against
	// the lua reference (lines 237-373) and Curivator_Robot.cpp (lines 1212-1262).
	//
	// ROOT CAUSE FIX: Previously PID_Up/PID_Down defaulted to {0,0,0} from Init(), so the PID
	// produced zero ErrorOffset every frame.  The position-tracking velocity from
	// GetVelocityFromDistance_Linear was also suppressed by missing dead zones and tolerance.
	// Setting these gains restores the full PID loop that drives the voltage callbacks.
	// ======================================================================

	// --- Per-joint Ship_1D properties from lua ---
	// Ian: BigArm/Boom have max_speed=6.0, Bucket/Clasp have max_speed=3.0.
	// BigArm/Boom max_accel_forward/reverse=5/25 respectively (slower, heavier linkage).
	// Bucket/Clasp max_accel_forward/reverse=500 (fast, lightweight end-effector).
	// IMPORTANT: MaxSpeed_Reverse MUST be negative — Ship_1D::SetRequestedVelocity clamps
	// negative velocities to MAX(Velocity, MaxSpeed_Reverse).  If positive, negative velocities
	// get clamped to the positive value, breaking retraction.
	double maxSpeed = 3.0;
	double maxAccelFwd = 500.0;
	double maxAccelRev = 500.0;
	switch (joint)
	{
	case ExcavatorJoint::eBigArm:
		maxSpeed = 6.0;
		maxAccelFwd = 5.0;
		maxAccelRev = 5.0;
		break;
	case ExcavatorJoint::eBoom:
		maxSpeed = 6.0;
		maxAccelFwd = 25.0;
		maxAccelRev = 25.0;
		break;
	case ExcavatorJoint::eBucket:
	case ExcavatorJoint::eClasp:
		maxSpeed = 3.0;
		maxAccelFwd = 500.0;
		maxAccelRev = 500.0;
		break;
	default:
		break;
	}
	p.ship_props.MAX_SPEED = maxSpeed;
	p.ship_props.MaxSpeed_Forward = maxSpeed;
	p.ship_props.MaxSpeed_Reverse = -maxSpeed;
	p.ship_props.ACCEL = 10.0;             // all joints: accel=10.0, brake=10.0
	p.ship_props.BRAKE = 10.0;
	p.ship_props.MaxAccelForward = maxAccelFwd;
	p.ship_props.MaxAccelReverse = maxAccelRev;
	p.ship_props.MinRange = minRange;
	p.ship_props.MaxRange = maxRange;
	p.ship_props.UsingRange = true;        // all joints: using_range=1
	p.ship_props.DistanceDegradeScaler = 1.0;

	// Ian: ROOT CAUSE FIX for Layer 2 zero-voltage bug.
	// Robot_Simulation's RotarySystem.cpp (line ~297) uses MinLimitRange/MaxLimitRange to set the
	// PID output range: SetOutputRange(MinLimitRange * 0.99, MaxLimitRange * 0.99).
	// The Curivator instead used symmetric ±MaxSpeed for output range (different #if path).
	// When MinLimitRange/MaxLimitRange default to 0.0, the PID output range becomes (0,0), and the
	// I-term clamping code divides by m_I (which is 0) → NaN m_ErrorOffset → NaN voltage → clamped
	// to 0.  This silently killed ALL voltage for Bucket/Clasp (and partially BigArm/Boom).
	// Fix: set symmetric limits so the PID output range spans [-max, +max], matching Curivator's
	// ±MaxSpeed approach.  MinLimitRange = -MaxLimitRange ensures m_minimumOutput < 0, avoiding
	// the 0/0 division in PID I-clamp code.
	p.rotary_props.MinLimitRange = -maxRange;
	p.rotary_props.MaxLimitRange = maxRange;

	// --- Rotary_Props: closed loop with potentiometer feedback ---
	// Ian: With eClosed, GetActualPos() reads from the odometry callback (our
	// m_simulated_positions) instead of Ship_1D's internal GetPos_m().  Without this, the PID
	// never sees the "real" simulated position and runs open loop.
	// This matches the Curivator's Robot_TesterCode path where Potentiometer_Tester2 provided
	// feedback through GetRotaryCurrentPorV → GetPotentiometerCurrentPosition → GetPos_m().
	p.rotary_props.VoltageScaler = 1.0;       // all joints: voltage_multiply not set → defaults 1.0
	p.rotary_props.EncoderToRS_Ratio = 1.0;   // all joints: encoder_to_wheel_ratio not set → 1.0
	p.rotary_props.LoopState = rotary_properties::Rotary_Props::eClosed;

	// --- PID gains: pid_up={p=100, i=0, d=25}, pid_down={p=100, i=0, d=25} for ALL joints ---
	// Ian: P=100 is correct — the PID operates on position error scaled against the time slice.
	// The students tuned these during matches via lua; they wouldn't need to change code.
	p.rotary_props.ArmGainAssist.PID_Up[0] = 100.0;   // P
	p.rotary_props.ArmGainAssist.PID_Up[1] = 0.0;     // I
	p.rotary_props.ArmGainAssist.PID_Up[2] = 25.0;    // D
	p.rotary_props.ArmGainAssist.PID_Down[0] = 100.0;
	p.rotary_props.ArmGainAssist.PID_Down[1] = 0.0;
	p.rotary_props.ArmGainAssist.PID_Down[2] = 25.0;

	// --- use_pid_up_only='y' for ALL joints ---
	// Ian: When true, the PID always uses PID_Up gains regardless of direction.  The Curivator
	// darts are symmetric (same force extending/retracting), so one set of gains suffices.
	p.rotary_props.ArmGainAssist.UsePID_Up_Only = true;

	// --- use_aggressive_stop='yes' for ALL joints ---
	// Ian: Enables adverse force to assist stopping.  Prevents overshoot at setpoints.
	p.rotary_props.UseAggressiveStop = true;

	// --- tolerance_count=1 for ALL joints ---
	p.rotary_props.ArmGainAssist.ToleranceConsecutiveCount = 1.0;

	// --- Per-joint tolerance, dead zones, and velocity predict ---
	// Ian: BigArm/Boom have wider tolerances (0.3/0.15) and dead zones (0.23/0.37) because
	// they are heavy joints with mechanical backlash.  Bucket/Clasp have tight tolerances
	// (0.0125) and no dead zones because they are lightweight end-effector linkages.
	// VelocityPredictUp/Down are only set for BigArm/Boom (0.200 ms each); they compensate
	// for potentiometer lag on the heavier joints.
	switch (joint)
	{
	case ExcavatorJoint::eBigArm:
		p.rotary_props.PrecisionTolerance = 0.3;
		p.rotary_props.Positive_DeadZone = 0.23;
		p.rotary_props.Negative_DeadZone = -0.23;     // MUST be negative form!
		p.rotary_props.ArmGainAssist.VelocityPredictUp = 0.200;
		p.rotary_props.ArmGainAssist.VelocityPredictDown = 0.200;
		break;
	case ExcavatorJoint::eBoom:
		p.rotary_props.PrecisionTolerance = 0.15;
		p.rotary_props.Positive_DeadZone = 0.37;
		p.rotary_props.Negative_DeadZone = -0.37;     // MUST be negative form!
		p.rotary_props.ArmGainAssist.VelocityPredictUp = 0.200;
		p.rotary_props.ArmGainAssist.VelocityPredictDown = 0.200;
		break;
	case ExcavatorJoint::eBucket:
		p.rotary_props.PrecisionTolerance = 0.0125;
		// No dead zones for bucket/clasp (default 0.0 from Init)
		// No velocity predict for bucket/clasp (default 0.0 from Init)
		break;
	case ExcavatorJoint::eClasp:
		p.rotary_props.PrecisionTolerance = 0.0125;
		// No dead zones for bucket/clasp
		// No velocity predict for bucket/clasp
		break;
	default:
		break;
	}

	m_starting_positions[idx] = startingPos;
}

// Ian: SetDefaultVirtualJointProps — configures a virtual 3D-position joint (Ship_1D).
// These match the Curivator's lua configs for arm_xpos, arm_ypos, bucket_angle, clasp_angle.
// Note: the virtual joints are Ship_1D entities, NOT RotarySystem_Position.  They have no
// potentiometer, no voltage output, no PID — they just track a desired position that the
// user moves with keyboard and the IK reads.  The FK feeds back actual values for telemetry.
void ExcavatorArm::SetDefaultVirtualJointProps(VirtualJoint joint, double minRange, double maxRange,
	double startingPos, double maxSpeed, double accel, double brake, bool usingRange)
{
	using namespace Legacy;
	size_t idx = static_cast<size_t>(joint);
	Ship_1D_Properties& p = m_virtualJointProps[idx];

	static const char* names[] = { "arm_xpos", "arm_ypos", "bucket_angle", "clasp_angle" };
	const char* name = names[idx];

	// Ian: Match the Curivator lua config exactly.  MaxAccelForward/Reverse are set very high
	// (10000 = "god mode") in the original for xpos, ypos, bucket_angle, clasp_angle.
	p = Ship_1D_Properties(name, 1.5, 0.0254,
		maxSpeed, accel, brake,
		10000.0,  // MaxAccelForward — "god mode" per Curivator lua
		10000.0,  // MaxAccelReverse
		usingRange, minRange, maxRange,
		false);   // IsAngular=false (all positions are linear/degree units, not radians)
	p.AsEntityProps().m_StartingPosition = startingPos;
}

void ExcavatorArm::Init(const Framework::Base::asset_manager& /*properties*/)
{
	// Ian: Physical dart ranges (inches of shaft extension) — ported from CurivatorRobot.lua.
	// BigArm/Boom: min_range=1, max_range=10 (lua lines 252-253, 290-291) with pot_offset=1.
	// Bucket: min_range=0, max_range=12 (lua line 327-328).
	// Clasp:  min_range=0, max_range=7  (lua line 357-358).
	// Starting positions (6, 6, 6, 3.5) are mid-stroke for all joints.
	// Phase 2 will read these from the asset_manager using prefixed keys (e.g. "Excavator_BigArm_").
	SetDefaultJointProps(ExcavatorJoint::eBigArm,  1.0,  10.0, 6.0);
	SetDefaultJointProps(ExcavatorJoint::eBoom,    1.0,  10.0, 6.0);
	SetDefaultJointProps(ExcavatorJoint::eBucket,  0.0,  12.0, 6.0);
	SetDefaultJointProps(ExcavatorJoint::eClasp,   0.0,   7.0, 3.5);

	for (size_t i = 0; i < kExcavatorJointCount; i++)
	{
		m_joints[i].Init(i, &m_joint_props[i]);

		// Ian: For Phase 1 open-loop simulation, we wire the voltage output to a simple integrator.
		// The voltage drives position change (voltage * dt * speed_scale → position delta).
		// This mimics what the old Curivator sim did inside Potentiometer_Tester2.
		const size_t idx = i;  // capture for lambda
		m_simulated_positions[i] = m_starting_positions[i];

		m_joints[i].Set_UpdateCurrentVoltage([this, idx](double voltage)
		{
			// Ian: Voltage trace for diagnostic tests — fires unconditionally
			if (m_voltageTraceCallback)
				m_voltageTraceCallback(idx, voltage);
			// Ian: When voltage is detached, output zero regardless of PID.  The PID still
			// runs internally (avoiding integral windup on re-enable) but no motion occurs.
			if (m_voltageDetached)
				return;
			// Simple integrator: voltage ∈ [-1, 1] scaled by max speed and frame dt.
			// We accumulate into m_simulated_positions and clamp to range.
			// Uses m_lastDTime_s (cached before TimeSlice call) instead of a hardcoded dt.
			const auto& sp = m_joint_props[idx].ship_props;
			const double speed = sp.MAX_SPEED;
			const double delta = voltage * speed * m_lastDTime_s;
			m_simulated_positions[idx] += delta;
			m_simulated_positions[idx] = std::max(sp.MinRange, std::min(sp.MaxRange, m_simulated_positions[idx]));
		});

		// Odometry callback returns current simulated position
		m_joints[i].SetOdometryVelocityCallback([this, idx]() -> double
		{
			return m_simulated_positions[idx];
		});
	}

	// ====== Layer 2: Virtual 3D-position joints ======
	// Ian: These match the Curivator's arm_xpos, arm_ypos, bucket_angle, clasp_angle from the lua.
	// Starting positions are the mathematically ideal values for the physical darts at mid-stroke.
	// All values from CurivatorRobot.lua lines 374-494.
	//                               minRange  maxRange   startingPos                  maxSpeed   accel  brake  usingRange
	SetDefaultVirtualJointProps(VirtualJoint::eArmXpos,      12.79,  55.29, 32.801521314123598,   6.0,  10.0,  10.0, false);
	SetDefaultVirtualJointProps(VirtualJoint::eArmYpos,     -20.0,   40.0,  -0.97606122071131374, 6.0,  10.0,  10.0, true);
	SetDefaultVirtualJointProps(VirtualJoint::eBucketAngle,   0.0,  180.0,  78.070524788111342,  36.66,  50.0,  50.0, false);
	SetDefaultVirtualJointProps(VirtualJoint::eClaspAngle,   -7.0,  100.0,  13.19097419,         36.66,   5.0,   5.0, true);

	for (size_t i = 0; i < kVirtualJointCount; i++)
	{
		// Ian: Ship_1D construction — these are pure position-tracking entities.
		// They have their own physics (mass, accel) so keyboard input moves them smoothly.
		// No voltage output, no potentiometer — just Ship_1D::TimeChange() integrating position.
		static const char* jointNames[] = { "arm_xpos", "arm_ypos", "bucket_angle", "clasp_angle" };
		m_virtualJointStorage[i] = std::make_unique<Legacy::Ship_1D>(jointNames[i]);
		m_virtualJoints[i] = m_virtualJointStorage[i].get();
		m_virtualJoints[i]->Initialize(&m_virtualJointProps[i]);
	}

	// Ian: Provide good FK defaults before first frame, matching Curivator_Robot::Bucket::ResetPos().
	Update3DPositioningPosition(18.0, 0.0, 80.0);

	// Ian: Publish the initial enable_arm_auto_position value to SmartDashboard once, so the key
	// exists for the user to toggle.  The high-frequency UpdateVariables() only reads it back via
	// TryGetBoolean — no repeated PutBoolean in the hot path.
	SmartDashboard::PutBoolean("Manipulator/EnableArmAutoPosition", m_enable_arm_auto_position);

	// Ian: Phase 2 — initialize the command scheduler.  Registers diagnostic commands and
	// publishes initial .type properties for Scheduler/Subsystem/Command NT keys.
#if ENABLE_COMMAND_SCHEDULER
	InitScheduler();
#endif

	// Ian: Reset() must be called here to sync Ship_1D's internal m_Position with
	// m_simulated_positions.  Without this, Ship_1D::GetPos_m() starts at 0.0 (from construction)
	// while m_simulated_positions starts at startingPos (e.g. 6.0).  If any TimeSlice fires before
	// TeleAutonV2::Reset() — which can happen because the OSG viewer thread starts in init() before
	// Reset() — the PID sees error = 6.0, outputs max voltage, and the arm drifts wildly.
	// Reset() calls ResetPosition(startingPos) on each joint's Ship_1D, setting m_Position to match.
	Reset();
}

void ExcavatorArm::Reset()
{
	// Layer 2: Reset virtual 3D-position joints to their starting positions FIRST
	// (we need their positions to compute consistent physical starting positions via IK)
	for (size_t i = 0; i < kVirtualJointCount; i++)
	{
		if (m_virtualJoints[i])
			m_virtualJoints[i]->ResetPos();
	}

	// Ian: Compute physical starting positions from virtual starting positions via IK.
	// The physical shaft lengths MUST correspond to the virtual joint positions so that
	// FK(shaft_lengths) produces the same 3D position the virtual joints expect.
	// Without this, actual != desired on reset and the PID sees a phantom error.
	{
		IKInput ikIn;
		ikIn.GlobalHeight = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->GetPos_m();
		ikIn.GlobalDistance = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->GetPos_m();
		ikIn.BucketAngle_deg = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->GetPos_m();
		ikIn.ClaspOpeningAngle_deg = m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->GetPos_m();
		IKOutput ikOut = ComputeArmPosition(ikIn);

		// Ian: Apply the same inversion as TimeSlice Layer 2 (Curivator_Robot.cpp lines 730-731).
		// BigArm and Boom darts are wired in reverse: shaft = maxRange - ikShaft + minRange.
		const auto& bigArmSp = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBigArm)].ship_props;
		const auto& boomSp = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBoom)].ship_props;

		m_starting_positions[static_cast<size_t>(ExcavatorJoint::eBigArm)] =
			bigArmSp.MaxRange - ikOut.BigArmShaftLength + bigArmSp.MinRange;
		m_starting_positions[static_cast<size_t>(ExcavatorJoint::eBoom)] =
			boomSp.MaxRange - ikOut.BoomShaftLength + boomSp.MinRange;
		m_starting_positions[static_cast<size_t>(ExcavatorJoint::eBucket)] = ikOut.BucketShaftLength;
		m_starting_positions[static_cast<size_t>(ExcavatorJoint::eClasp)] = ikOut.ClaspShaftLength;
	}

	// Layer 1: Reset physical dart joints to IK-derived starting positions
	for (size_t i = 0; i < kExcavatorJointCount; i++)
	{
		m_simulated_positions[i] = m_starting_positions[i];
		m_joints[i].Reset(m_starting_positions[i]);
	}

	// Reset lock position cache
	m_Last_xpos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)] ?
		m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->GetPos_m() : 0.0;
	m_Last_ypos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)] ?
		m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->GetPos_m() : 0.0;
	m_Last_bucket_angle = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)] ?
		m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->GetPos_m() : 0.0;
	m_Last_clasp_angle = m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)] ?
		m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->GetPos_m() : 0.0;

	m_FreezeArm = false;
	m_LockPosition = false;

	ComputeForwardKinematics();
}

#pragma endregion

#pragma region _Joint Input_

// Ian: SetJointInput — stores normalized [-1, 1] velocity for a joint.  Applied in TimeSlice()
// to move m_simulated_positions (the setpoint), which the PID then chases.
// Keys 1/2 = BigArm ±, 3/4 = Boom ±, 5/6 = Bucket ±, 7/8 = Clasp ± (wired in TeleAutonV2).
void ExcavatorArm::SetJointInput(size_t jointIndex, double normalizedVelocity)
{
	if (jointIndex < kExcavatorJointCount)
		m_jointInputs[jointIndex] = std::max(-1.0, std::min(1.0, normalizedVelocity));
}

// Ian: Set3DPositionInput — stores normalized [-1, 1] velocity for a virtual 3D-position joint.
// Applied in TimeSlice() to drive the virtual joints when EnableArmAutoPosition=true.
// Keys: j/k = xpos ±, l/; = ypos ±, u/i = bucket_angle ±  (wired in TeleAutonV2).
void ExcavatorArm::Set3DPositionInput(size_t posIndex, double normalizedVelocity)
{
	if (posIndex < kVirtualJointCount)
		m_3DPosInputs[posIndex] = std::max(-1.0, std::min(1.0, normalizedVelocity));
}

// Ian: Getters for desired 3D positions — read from virtual joints' GetPos_m().
// These correspond to arm_xpos_desired / arm_ypos_desired / bucket_angle_desired in SmartDashboard.
double ExcavatorArm::GetDesired_ArmXpos() const
{
	auto* j = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)];
	return j ? j->GetPos_m() : 0.0;
}
double ExcavatorArm::GetDesired_ArmYpos() const
{
	auto* j = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)];
	return j ? j->GetPos_m() : 0.0;
}
double ExcavatorArm::GetDesired_BucketAngle() const
{
	auto* j = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)];
	return j ? j->GetPos_m() : 0.0;
}

#pragma endregion

#pragma region _TimeSlice_

// Ian: Update3DPositioningPosition — stores the FK-computed global bucket position for virtual
// joint feedback.  Matches Curivator_Robot_Control::Update3DPositioningPosition() (h:439-441).
// Called by ComputeForwardKinematics() after the Bucket FK computes the global tip position.
void ExcavatorArm::Update3DPositioningPosition(double BucketDistance, double BucketHeight, double BucketAngle_deg)
{
	m_3DPos_BucketDistance = BucketDistance;
	m_3DPos_BucketHeight = BucketHeight;
	m_3DPos_BucketAngle = BucketAngle_deg;
}

void ExcavatorArm::TimeSlice(double dTime_s)
{
	// Ian: Cache frame dt for the voltage callback (called from within the PID's TimeChange).
	m_lastDTime_s = dTime_s;

	// Ian: Two-layer control loop, faithful port of Curivator_Robot.cpp lines 700-753.
	//
	// ALL joints (physical + virtual) tick every frame regardless of mode.
	// When EnableArmAutoPosition=true, virtual joints are MASTER: their GetPos_m() gives
	// the desired 3D position, IK computes shaft lengths, SetIntendedPosition drives the
	// physical joints' PID.  FK feedback writes actual position back.
	// When EnableArmAutoPosition=false, keyboard inputs 1-8 drive m_simulated_positions
	// directly (Layer 1 only, dart tuning mode).

	// --- Read enable_arm_auto_position each frame ---
	// Ian: This is a dedicated boolean, not a SmartDashboard variable.  No fighting with
	// time slices.  Matches Curivator_Robot.cpp line 708: if (props.EnableArmAutoPosition).
	// Set via SetEnableArmAutoPosition() or keyboard toggle.
	const bool enableArmAutoPosition = m_enable_arm_auto_position;

	if (enableArmAutoPosition)
	{
		// ====== Layer 2 active: Virtual joints are MASTER ======
		//
		// Ian: CRITICAL ORDERING — must match the Curivator's TimeChange (lines 701-740):
		//   1. ALL entities TimeChange (physical + virtual)
		//   2. Read virtual joint positions → IK → SetIntendedPosition on physical joints
		//
		// The physical joints' PID processes the PREVIOUS frame's SetIntendedPosition during
		// their TimeChange.  The new SetIntendedPosition is picked up on the NEXT frame.
		// If we call SetIntendedPosition BEFORE TimeChange (as we had it before), the PID
		// sees a brand-new target and must process it in the same frame — the internal
		// SetPos_m(NewPosition) at RotarySystem.cpp:538 resets Ship_1D's position to the
		// actual position, erasing any waypoint progress, which caused zero voltage for joints
		// without VelocityPredictUp (Bucket, Clasp).

		// Step 1a: Drive virtual joints from keyboard input (j/k, l/;, u/i, etc.).
		// SetRequestedVelocity_FromNormalized scales by max speed and manages flood control.
		for (size_t i = 0; i < kVirtualJointCount; i++)
		{
			if (m_virtualJoints[i])
				m_virtualJoints[i]->SetRequestedVelocity_FromNormalized(m_3DPosInputs[i]);
		}

		// Step 1b: Tick ALL entities — physical joints first (PID processes PREVIOUS frame's
		// SetIntendedPosition), then virtual joints integrate keyboard velocity into position.
		// This matches Curivator_Robot.cpp line 702: mp_Arm[i]->AsEntity1D().TimeChange(dTime_s)
		// which iterates ALL arm entities (physical + virtual) in one loop.
		for (size_t i = 0; i < kExcavatorJointCount; i++)
			m_joints[i].TimeSlice(dTime_s);
		for (size_t i = 0; i < kVirtualJointCount; i++)
		{
			if (m_virtualJoints[i])
			{
				// Ian: TimeChange is protected on Ship_1D but public on Entity1D (base class).
				// The Curivator calls it via AsEntity1D().TimeChange() — same pattern here.
				static_cast<Legacy::Entity1D*>(m_virtualJoints[i])->TimeChange(dTime_s);
			}
		}

		// Step 2: Read desired 3D position from virtual joints (with LockPosition support)
		const double xpos = !m_LockPosition ?
			m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->GetPos_m() : m_Last_xpos;
		const double ypos = !m_LockPosition ?
			m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->GetPos_m() : m_Last_ypos;
		const double bucket_angle = !m_LockPosition ?
			m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->GetPos_m() : m_Last_bucket_angle;
		const double clasp_angle = !m_LockPosition ?
			m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->GetPos_m() : m_Last_clasp_angle;

		// Cache for LockPosition (no harm in always assigning — Curivator_Robot.cpp line 715)
		m_Last_xpos = xpos;
		m_Last_ypos = ypos;
		m_Last_bucket_angle = bucket_angle;
		m_Last_clasp_angle = clasp_angle;

		// Step 3: Run IK to get shaft lengths from desired 3D position
		// Ian: IK input is (height=ypos, distance=xpos, bucket_angle, clasp_angle)
		// matching Curivator_Robot.cpp line 725: ComputeArmPosition(ypos, xpos, bucket_angle, clasp_angle, ...)
		IKInput ikInput;
		ikInput.GlobalHeight = ypos;
		ikInput.GlobalDistance = xpos;
		ikInput.BucketAngle_deg = bucket_angle;
		ikInput.ClaspOpeningAngle_deg = clasp_angle;
		IKOutput ikOut = ComputeArmPosition(ikInput);

		// Ian: Invert BigArm and Boom shaft lengths — the darts are wired in reverse.
		// Curivator_Robot.cpp lines 730-731:
		//   Boom_ShaftLength = maxRange - Boom_ShaftLength + minRange
		//   BigArm_ShaftLength = maxRange - BigArm_ShaftLength + minRange
		const auto& bigArmSp = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBigArm)].ship_props;
		const auto& boomSp = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBoom)].ship_props;
		const double BigArm_Inverted = bigArmSp.MaxRange - ikOut.BigArmShaftLength + bigArmSp.MinRange;
		const double Boom_Inverted = boomSp.MaxRange - ikOut.BoomShaftLength + boomSp.MinRange;

		// Step 4: Set intended positions for NEXT frame's PID to process
		if (!m_FreezeArm)
		{
			// Ian: SetIntendedPosition — sets ONLY the PID's intended position without modifying
			// m_Position or m_simulated_positions.  The PID drives voltage to reach the shaft
			// length on the NEXT TimeSlice call.
			m_joints[static_cast<size_t>(ExcavatorJoint::eBigArm)].SetIntendedPosition(BigArm_Inverted);
			m_joints[static_cast<size_t>(ExcavatorJoint::eBoom)].SetIntendedPosition(Boom_Inverted);
			m_joints[static_cast<size_t>(ExcavatorJoint::eBucket)].SetIntendedPosition(ikOut.BucketShaftLength);
			m_joints[static_cast<size_t>(ExcavatorJoint::eClasp)].SetIntendedPosition(ikOut.ClaspShaftLength);
		}
		else
		{
			// Ian: FreezeArm — stop all motion on both layers (Curivator_Robot.cpp lines 742-751).
			// Physical joints: set requested velocity to 0 (stops PID from chasing).
			// Virtual joints: also stop so they don't drift while frozen.
			// Note: We can't directly call SetRequestedVelocity on RotarySystem_Position
			// (it's not exposed), so we freeze by setting intended position to current position.
			for (size_t i = 0; i < kExcavatorJointCount; i++)
				m_joints[i].SetIntendedPosition(m_simulated_positions[i]);
			// Virtual joints freeze via SetRequestedVelocity(0.0)
			for (size_t i = 0; i < kVirtualJointCount; i++)
			{
				if (m_virtualJoints[i])
					m_virtualJoints[i]->SetRequestedVelocity(0.0);
			}
		}
	}
	else
	{
		// ====== Layer 1 only: Direct dart control (EnableArmAutoPosition=false) ======
		// Ian: Keyboard keys 1-8 drive m_simulated_positions directly.  No PID runs in this
		// mode — we just move the simulated positions and let FK compute the result.
		//
		// The PID was previously running here via SetPosition + TimeSlice, but the PID fought
		// the direct changes: Ship_1D's internal m_Position lagged behind m_simulated_positions
		// (SetPosition only sets m_IntendedPosition, not Ship_1D's m_Position), so the PID saw
		// an error and generated voltage that UNDID the direct changes via the voltage callback.
		// Net result was zero FK movement.
		//
		// In Layer 1, the user IS the controller — m_simulated_positions is the truth.
		// The PID is only needed in Layer 2 where IK sets intended positions and the PID drives
		// voltage to match.  We still keep the joints synced (SetPosition) so that switching to
		// Layer 2 mode doesn't produce a discontinuity.

		for (size_t i = 0; i < kExcavatorJointCount; i++)
		{
			if (m_jointInputs[i] != 0.0)
			{
				const auto& sp = m_joint_props[i].ship_props;
				const double delta = m_jointInputs[i] * sp.MAX_SPEED * dTime_s;
				m_simulated_positions[i] += delta;
				m_simulated_positions[i] = std::max(sp.MinRange, std::min(sp.MaxRange, m_simulated_positions[i]));
			}
		}

		// Ian: Sync the joints to the new simulated positions WITHOUT running the PID.
		// SetPosition updates both the intended position and the RotaryPosition_Internal's
		// m_Position so the joint state is consistent.  We skip TimeSlice (no PID) because
		// direct control doesn't need closed-loop feedback.
		for (size_t i = 0; i < kExcavatorJointCount; i++)
			m_joints[i].SetPosition(m_simulated_positions[i]);
	}

	// Recompute forward kinematics from current dart positions (both modes)
	ComputeForwardKinematics();

	// Ian: Phase 2 — update the command scheduler after all physics and FK have run.
	// The active command's execute() may set new intended positions for NEXT frame.
#if ENABLE_COMMAND_SCHEDULER
	UpdateScheduler(dTime_s);
#endif
}

#pragma endregion

#pragma region _Forward Kinematics_

void ExcavatorArm::ComputeForwardKinematics()
{
	// Ian: Each joint reads its simulated shaft extension and computes the resulting angles and
	// positions.  This is a faithful port of the TimeChange() methods from BigArm, Boom, Bucket,
	// and Clasp in Curivator_Robot.cpp.  Variable names are kept identical for traceability.

	//=== BigArm FK (Curivator_Robot.cpp line 192) ===
	{
		const double pos = m_simulated_positions[static_cast<size_t>(ExcavatorJoint::eBigArm)];
		const double minR = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBigArm)].ship_props.MinRange;
		const double maxR = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBigArm)].ship_props.MaxRange;
		// Ian: position is inverted due to dart nature — subtract range from position
		const double ShaftExtension_in = maxR - pos + minR;
		const double FullActuatorLength = ShaftExtension_in + BigArm_DistanceDartPivotToTip + BigArm_DistanceFromTipDartToClevis;
		const double BigAngleDartInterface = LawOfCosines(BigArm_AngleToDartPivotInterface_Length, BigArm_DartToArmDistance, FullActuatorLength);
		m_fk.BigArmAngle = BigAngleDartInterface - BigArm_AngleToDartPivotInterface;
		m_fk.BigArmLength = std::cos(m_fk.BigArmAngle) * BigArm_Radius;
		m_fk.BigArmHeight = std::sin(m_fk.BigArmAngle) * BigArm_Radius;
	}

	//=== Boom FK (Curivator_Robot.cpp line 248) ===
	{
		const double pos = m_simulated_positions[static_cast<size_t>(ExcavatorJoint::eBoom)];
		const double minR = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBoom)].ship_props.MinRange;
		const double maxR = m_joint_props[static_cast<size_t>(ExcavatorJoint::eBoom)].ship_props.MaxRange;
		// Ian: inverted like BigArm
		const double ShaftExtension_in = maxR - pos + minR;
		const double FullActuatorLength = ShaftExtension_in + Boom_DistanceDartPivotToTip + Boom_DistanceFromTipDartToClevis;
		const double BigAngleDartInterface = LawOfCosines(Boom_AngleToDartPivotInterface_Length, Boom_DartToArmDistance, FullActuatorLength);
		const double local_BoomAngle = M_PI - BigAngleDartInterface + Boom_AngleToDartPivotInterface;
		// Convert to global: subtract big arm angle contribution
		m_fk.BoomAngle = local_BoomAngle - ((PI_2 - m_fk.BigArmAngle) + Boom_AngleBigArmToDartPivot);

		const double LocalBoomLength = std::sin(m_fk.BoomAngle) * Boom_Radius;
		m_fk.BoomLength = LocalBoomLength + m_fk.BigArmLength;
		const double LocalBoomHeight = std::cos(m_fk.BoomAngle) * Boom_Radius;
		m_fk.BoomHeight = m_fk.BigArmHeight - LocalBoomHeight;
	}

	//=== Bucket FK (Curivator_Robot.cpp line 315) ===
	{
		const double ShaftExtension_in = m_simulated_positions[static_cast<size_t>(ExcavatorJoint::eBucket)];
		const double FullActuatorLength = ShaftExtension_in + Bucket_LAB_housingLength;
		const double RockerBoomAngle = LawOfCosines(Bucket_BRP_To_LAB, Bucket_RockerBoomLength, FullActuatorLength);
		const double QuadRockerBoomAngle = Bucket_BRP_LABtoBRP_BP_Angle - RockerBoomAngle;
		const double RockerInterfaceY = (std::cos(QuadRockerBoomAngle) * Bucket_RockerBoomLength) - Bucket_BRP_To_BP;
		const double RockerInterfaceX = std::sin(QuadRockerBoomAngle) * Bucket_RockerBoomLength;
		const double QuadBisectLength = std::sqrt((RockerInterfaceX * RockerInterfaceX) + (RockerInterfaceY * RockerInterfaceY));
		const double BucketPivotUpperAngle = LawOfCosines(Bucket_BRP_To_BP, QuadBisectLength, Bucket_RockerBoomLength);
		const double BucketPivotLowerAngle = LawOfCosines(Bucket_BP_To_BucketRP, QuadBisectLength, Bucket_RockerBucketLength);
		const double BucketPivotAngle = BucketPivotUpperAngle + BucketPivotLowerAngle;
		const double BucketCoMPivotAngleHorz = BucketPivotAngle + Bucket_BucketRPtoBucketCoM_Angle - DEG_2_RAD(90) - (DEG_2_RAD(90) - Bucket_HorizontalToBRP_BP_Angle);
		const double BoomAngle = m_fk.BoomAngle;

		m_fk.Bucket_globalBRP_BP_height = std::sin(Bucket_HorizontalToBRP_BP_Angle - BoomAngle) * Bucket_BRP_To_BP;
		const double GlobalCoMHeight = m_fk.Bucket_globalBRP_BP_height + (std::sin(BucketCoMPivotAngleHorz - BoomAngle) * Bucket_BP_To_BucketCoM);
		m_fk.Bucket_globalBRP_BP_distance = std::cos(Bucket_HorizontalToBRP_BP_Angle - BoomAngle) * Bucket_BRP_To_BP;
		const double GlobalCoMDistance = m_fk.Bucket_globalBRP_BP_distance + (std::cos(BucketCoMPivotAngleHorz - BoomAngle) * Bucket_BP_To_BucketCoM);

		// Ian: Store global CoM position for the side-view UI circle rendering.
		// These match the original Curivator_Robot::Bucket::GetCoMHeight()/GetCoMDistance():
		//   CoMHeight  = BoomHeight - GlobalCoMHeight   (screen Y)
		//   CoMDistance = BoomLength + GlobalCoMDistance  (screen X)
		m_fk.CoMHeight = m_fk.BoomHeight - GlobalCoMHeight;
		m_fk.CoMDistance = m_fk.BoomLength + GlobalCoMDistance;

		const double LocalTipHeight = m_fk.Bucket_globalBRP_BP_height + (std::sin(BucketCoMPivotAngleHorz + Bucket_CoMtoTip_Angle - BoomAngle) * Bucket_BP_to_BucketTip);
		m_fk.BucketTipHeight = m_fk.BoomHeight - LocalTipHeight;

		const double LocalBucketAngle = DEG_2_RAD(180) - (BucketCoMPivotAngleHorz + Bucket_CoMtoTip_Angle) - Bucket_BPTip_to_BucketInterface_Angle;
		m_fk.BucketAngle = LocalBucketAngle + BoomAngle;

		const double GlobalDistance = m_fk.Bucket_globalBRP_BP_distance + (std::cos(BucketCoMPivotAngleHorz + Bucket_CoMtoTip_Angle - BoomAngle) * Bucket_BP_to_BucketTip);
		m_fk.BucketLength = GlobalDistance + m_fk.BoomLength;

		m_fk.BucketRoundEndHeight = m_fk.BoomHeight - (GlobalCoMHeight + Bucket_CoM_Radius);
	}

	//=== Clasp FK (Curivator_Robot.cpp line 428) ===
	{
		const double ShaftExtension_in = m_simulated_positions[static_cast<size_t>(ExcavatorJoint::eClasp)];
		const double FullActuatorLength = ShaftExtension_in + Clasp_LAC_housingLength;
		const double ClaspLA_Interface_Angle = LawOfCosines(Clasp_BRP_To_LAC, Clasp_CP_To_LAC, FullActuatorLength);
		const double ClaspLA_Interface_Angle_Horizontal = (DEG_2_RAD(90) - (ClaspLA_Interface_Angle + Clasp_BoomAngleToLAC_Angle));
		const double Clasp_MidlineSegment_Angle = Clasp_LA_Interface_to_Midline_Angle + ClaspLA_Interface_Angle_Horizontal;

		const double BoomAngle = m_fk.BoomAngle;
		const double ClaspPivotHeight = m_fk.Bucket_globalBRP_BP_height;
		const double Clasp_CP_To_LAC_Height = std::sin(ClaspLA_Interface_Angle_Horizontal - BoomAngle) * Clasp_CP_To_LAC;
		const double Clasp_MidlineSegment_Height = std::sin(Clasp_MidlineSegment_Angle - BoomAngle) * Clasp_MidlineSegment;
		const double localClasp_MidlineHeight = ClaspPivotHeight - Clasp_CP_To_LAC_Height + Clasp_MidlineSegment_Height;
		m_fk.ClaspMidlineHeight = m_fk.BoomHeight - localClasp_MidlineHeight;

		const double ClaspPivotDistance = m_fk.Bucket_globalBRP_BP_distance;
		const double Clasp_CP_To_LAC_Distance = std::cos(ClaspLA_Interface_Angle_Horizontal - BoomAngle) * Clasp_CP_To_LAC;
		const double Clasp_MidlineSegment_Distance = std::cos(Clasp_MidlineSegment_Angle - BoomAngle) * Clasp_MidlineSegment;
		const double localClasp_MidlineDistance = ClaspPivotDistance - Clasp_CP_To_LAC_Distance + Clasp_MidlineSegment_Distance;
		m_fk.ClaspMidlineDistance = m_fk.BoomLength + localClasp_MidlineDistance;

		const double Clasp_MidlineToEdge_Angle_Horizontal = Clasp_MidlineSegment_Angle - (DEG_2_RAD(180) - Clasp_MidLineToEdge_Angle);
		const double localSideFromHorizontal_Angle = DEG_2_RAD(180) - (Clasp_MidlineToEdge_Angle_Horizontal + Clasp_BottomToSideAngle);
		m_fk.ClaspAngle = localSideFromHorizontal_Angle + BoomAngle;

		// Clasp min height = min of inner/outer tip
		const double InnerTipHeight = m_fk.ClaspMidlineHeight - (std::sin(Clasp_MidlineToEdge_Angle_Horizontal - BoomAngle) * Clasp_BottomEdgeLength_Half);
		const double OuterTipHeight = std::sin(Clasp_MidlineToEdge_Angle_Horizontal - BoomAngle) * Clasp_BottomEdgeLength_Half + m_fk.ClaspMidlineHeight;
		m_fk.ClaspMinHeight = std::min(InnerTipHeight, OuterTipHeight);
	}

	// Ian: Feed back the FK-computed global bucket position to the virtual joints' feedback
	// members.  Matches Curivator_Robot.cpp line 360:
	//   Update3DPositioningPosition(globalBucketDistance, min(globalTipHeight, globalRoundEndHeight), globalBucketAngle_deg)
	// This provides the "sensor" reading for the 3D position display and telemetry.
	Update3DPositioningPosition(
		m_fk.BucketLength,                                              // globalBucketDistance
		std::min(m_fk.BucketTipHeight, m_fk.BucketRoundEndHeight),     // min(tipHeight, roundEndHeight)
		RAD_2_DEG(m_fk.BucketAngle)                                    // globalBucketAngle_deg
	);
}

#pragma endregion

#pragma region _Inverse Kinematics_

// Ian: ComputeArmPosition — given a desired end-effector pose (height, distance, bucket angle,
// clasp opening angle), computes the 4 shaft lengths needed.  This is a faithful port of
// Curivator_Robot::ComputeArmPosition() from line 828.
ExcavatorArm::IKOutput ExcavatorArm::ComputeArmPosition(const IKInput& input)
{
	IKOutput out = {};
	const double BucketAngle = DEG_2_RAD(input.BucketAngle_deg);

	// Find the bucket pivot point (global coordinates) from the desired tip position
	// Ian: CRITICAL — must use Bucket_BPTip_to_BucketInterface_Angle here, NOT Bucket_BPBT_ToBucketRP_Angle.
	// The Curivator reference (line 834) uses Bucket_BPTip_to_BucketInterface_Angle (12.4°).
	// Using the wrong constant (78.2°) produces wildly incorrect pivot points and breaks IK↔FK round-trip.
	const double BucketPivotUsingTip_y = std::sin(BucketAngle + Bucket_BPTip_to_BucketInterface_Angle) * Bucket_BP_to_BucketTip + input.GlobalHeight;
	const double BucketCOMtoVertical_Angle = (BucketAngle + Bucket_BPTip_to_BucketInterface_Angle - DEG_2_RAD(90)) + Bucket_CoMtoTip_Angle;
	const double BucketPivotUsingCOM_y = std::cos(BucketCOMtoVertical_Angle) * Bucket_BP_To_BucketCoM + input.GlobalHeight + Bucket_CoM_Radius;
	const double BucketPivotPoint_y = std::max(BucketPivotUsingTip_y, BucketPivotUsingCOM_y);
	const double BucketPivotPoint_x = std::cos(BucketAngle + Bucket_BPTip_to_BucketInterface_Angle) * Bucket_BP_to_BucketTip + input.GlobalDistance;

	// Solve big arm and boom angles from the pivot point and known arm lengths
	const double OriginToBP = std::sqrt((BucketPivotPoint_x * BucketPivotPoint_x) + (BucketPivotPoint_y * BucketPivotPoint_y));
	const double BigArmUpper_Angle = LawOfCosines(BigArm_Radius, OriginToBP, Boom_Radius_BP);
	const double BigArmLower_Angle = std::atan2(BucketPivotPoint_y, BucketPivotPoint_x);
	const double BigArmAngle = BigArmUpper_Angle + BigArmLower_Angle;
	const double BigArmBoomBP_Angle = LawOfCosines(Boom_Radius_BP, BigArm_Radius, OriginToBP);
	const double BoomAngle = BigArmBoomBP_Angle - (DEG_2_RAD(90) - BigArmAngle) + Boom_BP_To_RBP_RadiusAngle;

	// BigArm shaft length
	const double BigArmLAInterface_height = std::sin(BigArmAngle + BigArm_AngleToDartPivotInterface) * BigArm_AngleToDartPivotInterface_Length;
	const double BigArmLAInterface_length = std::cos(BigArmAngle + BigArm_AngleToDartPivotInterface) * BigArm_AngleToDartPivotInterface_Length;
	const double BigArm_LA_Length_xLeg = std::fabs(BigArmLAInterface_length - BigArm_DartToArmDistance);
	const double BigArm_LA_Length = std::sqrt((BigArm_LA_Length_xLeg * BigArm_LA_Length_xLeg) + (BigArmLAInterface_height * BigArmLAInterface_height));
	out.BigArmShaftLength = EnforceShaftLimits(BigArm_LA_Length - BigArm_DistanceDartPivotToTip - Boom_DistanceFromTipDartToClevis);

	// Boom shaft length
	const double BoomLeverAngle = BoomAngle + Boom_BP_To_Lever_angle - DEG_2_RAD(180);
	const double BoomLAInterface_height = std::cos(BoomLeverAngle) * Boom_AngleToDartPivotInterface_Length;
	const double BoomLAInterface_length = std::sin(BoomLeverAngle) * Boom_AngleToDartPivotInterface_Length;
	const double BoomLA_Mount_Angle = (DEG_2_RAD(90) - BigArmAngle) + Boom_AngleBigArmToDartPivot;
	const double BoomLA_Mount_height = std::cos(BoomLA_Mount_Angle) * Boom_DartToArmDistance;
	const double BoomLA_Mount_length = std::sin(BoomLA_Mount_Angle) * Boom_DartToArmDistance;
	const double Boom_LA_Length_xLeg = std::fabs(BoomLAInterface_length - BoomLA_Mount_length);
	const double Boom_LA_Length_yLeg = std::fabs(BoomLAInterface_height + BoomLA_Mount_height);
	const double Boom_LA_Length = std::sqrt((Boom_LA_Length_xLeg * Boom_LA_Length_xLeg) + (Boom_LA_Length_yLeg * Boom_LA_Length_yLeg));
	out.BoomShaftLength = EnforceShaftLimits(Boom_LA_Length - Boom_DistanceDartPivotToTip - Boom_DistanceFromTipDartToClevis);

	// Bucket shaft length — 4-bar linkage IK
	const double BucketRBP_Angle = (DEG_2_RAD(180) - Bucket_BRP_LABtoBRP_BP_Angle) + (BoomAngle - Bucket_BoomAngleToLAB_Angle);
	const double RockerBoomPivotPoint_y = std::cos(BucketRBP_Angle) * Bucket_BRP_To_BP + BucketPivotPoint_y;
	const double RockerBoomPivotPoint_x = BucketPivotPoint_x - std::sin(BucketRBP_Angle) * Bucket_BRP_To_BP;

	const double Vertical_ToBucketRP_Angle = Bucket_BPBT_ToBucketRP_Angle - (DEG_2_RAD(90) - (BucketAngle + Bucket_BPTip_to_BucketInterface_Angle));
	const double RockerBucketPivotPoint_y = BucketPivotPoint_y - std::cos(Vertical_ToBucketRP_Angle) * Bucket_BP_To_BucketRP;
	const double RockerBucketPivotPoint_x = std::sin(Vertical_ToBucketRP_Angle) * Bucket_BP_To_BucketRP + BucketPivotPoint_x;
	const double brp_bucketrp_segment_length = GetDistance(RockerBoomPivotPoint_x, RockerBoomPivotPoint_y, RockerBucketPivotPoint_x, RockerBucketPivotPoint_y);

	const double RockerBoomUpperAngle = LawOfCosines(Bucket_RockerBoomLength, brp_bucketrp_segment_length, Bucket_RockerBucketLength);
	double RockerBoomLowerAngle = LawOfCosines(Bucket_BRP_To_BP, brp_bucketrp_segment_length, Bucket_BP_To_BucketRP);

	const double SegmentAngleFromVertical = DEG_2_RAD(90) + std::atan2(RockerBucketPivotPoint_y - RockerBoomPivotPoint_y, RockerBucketPivotPoint_x - RockerBoomPivotPoint_x);
	if (SegmentAngleFromVertical < BucketRBP_Angle)
		RockerBoomLowerAngle = RockerBoomLowerAngle * -1.0;

	const double RockerBoomFromVertical_Angle = BucketRBP_Angle + RockerBoomUpperAngle + RockerBoomLowerAngle;
	const double RockerPivotLAInterface_y = RockerBoomPivotPoint_y - std::cos(RockerBoomFromVertical_Angle) * Bucket_RockerBoomLength;
	const double RockerPivotLAInterface_x = std::sin(RockerBoomFromVertical_Angle) * Bucket_RockerBoomLength + RockerBoomPivotPoint_x;

	const double VerticalToLAB_Angle = BoomAngle - Bucket_BoomAngleToLAB_Angle;
	const double BoomLAMount_y = std::cos(VerticalToLAB_Angle) * Bucket_BRP_To_LAB + RockerBoomPivotPoint_y;
	const double BoomLAMount_x = RockerBoomPivotPoint_x - (std::sin(VerticalToLAB_Angle) * Bucket_BRP_To_LAB);
	const double Bucket_LA_Length = GetDistance(BoomLAMount_x, BoomLAMount_y, RockerPivotLAInterface_x, RockerPivotLAInterface_y);
	out.BucketShaftLength = EnforceShaftLimits(Bucket_LA_Length - Bucket_LAB_housingLength);

	// Clasp shaft length
	const double ClaspOpeningAngle = DEG_2_RAD(input.ClaspOpeningAngle_deg);
	const double CPMT_ToHorizontal = BucketAngle - (ClaspOpeningAngle - Clasp_CPMT_ToSide_Angle);
	const double MidlineTip_y = BucketPivotPoint_y - std::sin(CPMT_ToHorizontal) * Clasp_CP_To_MidlineTip;
	const double MidlineTip_x = BucketPivotPoint_x - std::cos(CPMT_ToHorizontal) * Clasp_CP_To_MidlineTip;
	const double MidlineToHorizontal = Clasp_MidLineToEdge_Angle + BucketAngle - ClaspOpeningAngle;
	const double ClaspLAInterface_y = std::sin(MidlineToHorizontal) * Clasp_MidlineSegment + MidlineTip_y;
	const double ClaspLAInterface_x = std::cos(MidlineToHorizontal) * Clasp_MidlineSegment + MidlineTip_x;
	const double LAMountVertical_Angle = BoomAngle + Clasp_BoomAngleToLAC_Angle;
	const double LAClaspMount_y = std::cos(LAMountVertical_Angle) * Clasp_BRP_To_LAC + BucketPivotPoint_y;
	const double LAClaspMount_x = BucketPivotPoint_x - std::sin(LAMountVertical_Angle) * Clasp_BRP_To_LAC;
	const double Clasp_LA_Length = GetDistance(LAClaspMount_x, LAClaspMount_y, ClaspLAInterface_x, ClaspLAInterface_y);
	out.ClaspShaftLength = EnforceShaftLimits(Clasp_LA_Length - Clasp_LAC_housingLength, 0.75, 6.0);

	return out;
}

#pragma endregion

#pragma region _CommandScheduler_

#if ENABLE_COMMAND_SCHEDULER

// ====================================================================
// Phase 2: Lightweight CommandScheduler (embedded in ExcavatorArm)
// ====================================================================
// Ian: This implements WPILib-compatible Scheduler/Subsystem/Command NT keys so dashboards
// can display subsystem status.  Commands adapt Curivator diagnostic goals (Curivator_Goals.cpp)
// to the modern ExcavatorArm interface.  Only one command runs at a time.
//
// NT key layout under /SmartDashboard/:
//   Scheduler/         .type="Scheduler"   Names=string[]
//   ExcavatorArm/      .type="Subsystem"   .hasDefault .default .hasCommand .command
//   <CmdName>/         .type="Command"     running=bool
//
// The .type topics carry property JSON {"SmartDashboard":"<type>"} so dashboards
// can identify the widget type from the announce message.

void ExcavatorArm::RegisterCommand(const std::string& name,
	std::function<void()> initialize,
	std::function<void(double)> execute,
	std::function<bool()> isFinished,
	std::function<void(bool)> end)
{
	CommandBehavior cmd;
	cmd.name = name;
	cmd.id = static_cast<int>(m_commands.size());
	cmd.initialize = std::move(initialize);
	cmd.execute = std::move(execute);
	cmd.isFinished = std::move(isFinished);
	cmd.end = std::move(end);
	m_commands.push_back(std::move(cmd));
}

void ExcavatorArm::InitScheduler()
{
	if (m_schedulerInitialized)
		return;
	m_schedulerInitialized = true;

	// Ian: Register diagnostic command behaviors adapted from Curivator_Goals.cpp.
	// Each command captures 'this' to read/write arm state.  The callbacks mirror the
	// WPILib Command lifecycle: initialize() → execute(dt) loop → isFinished() → end(interrupted).

	// --- Command 1: MoveArmToPosition ---
	// Ian: Adapted from Curivator_Goals.cpp ArmMoveToPosition (line 682).
	// Reads target position from SmartDashboard keys (target_arm_xpos, target_arm_ypos,
	// target_bucket_angle, target_clasp_angle) on initialize.  Drives virtual joints via
	// SetIntendedPosition each frame.  Finishes when FK position converges within tolerance.
	// Requires EnableArmAutoPosition=true (Layer 2 active).
	RegisterCommand("MoveArmToPosition",
		/*initialize*/ [this]()
		{
			// Read target from SmartDashboard (matching Curivator's Auton_Smart_GetMultiValue)
			double xpos = 25.0, ypos = -7.0, bucket = 78.0, clasp = 13.0;
			SmartDashboard::TryGetNumber("target_arm_xpos", xpos);
			SmartDashboard::TryGetNumber("target_arm_ypos", ypos);
			SmartDashboard::TryGetNumber("target_bucket_angle", bucket);
			SmartDashboard::TryGetNumber("target_clasp_angle", clasp);
			m_cmdTarget.targetXpos = xpos;
			m_cmdTarget.targetYpos = ypos;
			m_cmdTarget.targetBucketAngle = bucket;
			m_cmdTarget.targetClaspAngle = clasp;
			// Ensure Layer 2 is active
			m_enable_arm_auto_position = true;
			SmartDashboard::PutBoolean("Manipulator/EnableArmAutoPosition", true);
		},
		/*execute*/ [this](double /*dTime_s*/)
		{
			// Ian: Drive virtual joints to target positions.  The virtual joints' Ship_1D
			// physics handle acceleration/deceleration; TimeSlice's Layer 2 loop reads their
			// GetPos_m() → IK → SetIntendedPosition on physical joints.
			// We use SetIntendedPosition on the virtual joints directly (like the Curivator's
			// Goal_Rotary_MoveToPosition did via Arm.SetIntendedPosition).
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->SetIntendedPosition(m_cmdTarget.targetXpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->SetIntendedPosition(m_cmdTarget.targetYpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->SetIntendedPosition(m_cmdTarget.targetBucketAngle);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->SetIntendedPosition(m_cmdTarget.targetClaspAngle);
		},
		/*isFinished*/ [this]() -> bool
		{
			// Converged when FK position is within tolerance of target
			const double dx = std::fabs(m_3DPos_BucketDistance - m_cmdTarget.targetXpos);
			const double dy = std::fabs(m_3DPos_BucketHeight - m_cmdTarget.targetYpos);
			const double da = std::fabs(m_3DPos_BucketAngle - m_cmdTarget.targetBucketAngle);
			return (dx < m_cmdTarget.positionTolerance) &&
			       (dy < m_cmdTarget.positionTolerance) &&
			       (da < m_cmdTarget.angleTolerance);
		},
		/*end*/ [this](bool /*interrupted*/)
		{
			// Ian: On completion or interruption, freeze arm to prevent drift.
			// Matches Curivator's ArmMoveToPosition::StopAuton() behavior.
			m_FreezeArm = true;
			m_LockPosition = true;
		}
	);

	// --- Command 2: ArmGrabSequence ---
	// Ian: Adapted from Curivator_Goals.cpp ArmGrabSequence (line 755).
	// Multi-phase sequence: hover → approach pickup → close clasp → retract to start.
	// Each phase drives virtual joints to preset positions and advances when converged.
	// Uses the non-Robot_TesterCode position constants (real robot values).
	RegisterCommand("ArmGrabSequence",
		/*initialize*/ [this]()
		{
			m_grabPhase = 0;
			m_enable_arm_auto_position = true;
			SmartDashboard::PutBoolean("Manipulator/EnableArmAutoPosition", true);
			// Phase 0: Move to hover position
			// Curivator_Goals.cpp line 777: hover at (startX, hoverY, hoverBucket, hoverClasp)
			m_cmdTarget.targetXpos = 25.0;  // CurivatorGoal_HoverPosition[0] (non-tester)
			m_cmdTarget.targetYpos = 0.0;   // CurivatorGoal_HoverPosition[1]
			m_cmdTarget.targetBucketAngle = 90.0;  // CurivatorGoal_HoverPosition[2]
			m_cmdTarget.targetClaspAngle = 45.0;   // CurivatorGoal_HoverPosition[3]
		},
		/*execute*/ [this](double /*dTime_s*/)
		{
			// Drive virtual joints to current phase target
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->SetIntendedPosition(m_cmdTarget.targetXpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->SetIntendedPosition(m_cmdTarget.targetYpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->SetIntendedPosition(m_cmdTarget.targetBucketAngle);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->SetIntendedPosition(m_cmdTarget.targetClaspAngle);

			// Check if current phase target is reached, then advance
			const double dx = std::fabs(m_3DPos_BucketDistance - m_cmdTarget.targetXpos);
			const double dy = std::fabs(m_3DPos_BucketHeight - m_cmdTarget.targetYpos);
			const double da = std::fabs(m_3DPos_BucketAngle - m_cmdTarget.targetBucketAngle);
			const bool converged = (dx < m_cmdTarget.positionTolerance) &&
			                       (dy < m_cmdTarget.positionTolerance) &&
			                       (da < m_cmdTarget.angleTolerance);

			if (converged)
			{
				m_grabPhase++;
				// Ian: Phase progression adapted from Curivator_Goals.cpp ArmGrabSequence::Activate().
				// The original used a goal stack with SetArmWaypoint at each step.  We flatten
				// it into phases with explicit targets.
				switch (m_grabPhase)
				{
				case 1:
					// Phase 1: Approach pickup position
					// CurivatorGoal_PickupPosition = {25, -5, 90, 45} (non-tester)
					m_cmdTarget.targetXpos = 25.0;
					m_cmdTarget.targetYpos = -5.0;
					m_cmdTarget.targetBucketAngle = 90.0;
					m_cmdTarget.targetClaspAngle = 45.0;
					break;
				case 2:
					// Phase 2: Rotate bucket to scoop (close clasp)
					// Curivator_Goals.cpp line 770: bucket=40, clasp=close
					m_cmdTarget.targetBucketAngle = 40.0;
					m_cmdTarget.targetClaspAngle = 5.0;
					break;
				case 3:
					// Phase 3: Retract to starting position
					// CurivatorGoal_StartingPosition = {18, 4, 70, 5} (non-tester)
					m_cmdTarget.targetXpos = 18.0;
					m_cmdTarget.targetYpos = 4.0;
					m_cmdTarget.targetBucketAngle = 65.0;
					m_cmdTarget.targetClaspAngle = 5.0;
					break;
				default:
					// Phase 4+: sequence complete (isFinished will return true)
					break;
				}
			}
		},
		/*isFinished*/ [this]() -> bool
		{
			return m_grabPhase >= 4;
		},
		/*end*/ [this](bool /*interrupted*/)
		{
			m_FreezeArm = true;
			m_LockPosition = true;
			m_grabPhase = 0;
		}
	);

	// --- Command 3: HomeArm ---
	// Ian: Moves arm to the safe starting position (stowed).  This is the simplest command —
	// drives to the Curivator starting position and finishes.  Useful for recovery after
	// testing other commands.
	RegisterCommand("HomeArm",
		/*initialize*/ [this]()
		{
			m_enable_arm_auto_position = true;
			SmartDashboard::PutBoolean("Manipulator/EnableArmAutoPosition", true);
			// CurivatorGoal_StartingPosition (non-tester): {18, 4, 70, 5}
			m_cmdTarget.targetXpos = 18.0;
			m_cmdTarget.targetYpos = 4.0;
			m_cmdTarget.targetBucketAngle = 70.0;
			m_cmdTarget.targetClaspAngle = 5.0;
		},
		/*execute*/ [this](double /*dTime_s*/)
		{
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->SetIntendedPosition(m_cmdTarget.targetXpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->SetIntendedPosition(m_cmdTarget.targetYpos);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->SetIntendedPosition(m_cmdTarget.targetBucketAngle);
			if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)])
				m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->SetIntendedPosition(m_cmdTarget.targetClaspAngle);
		},
		/*isFinished*/ [this]() -> bool
		{
			const double dx = std::fabs(m_3DPos_BucketDistance - m_cmdTarget.targetXpos);
			const double dy = std::fabs(m_3DPos_BucketHeight - m_cmdTarget.targetYpos);
			const double da = std::fabs(m_3DPos_BucketAngle - m_cmdTarget.targetBucketAngle);
			return (dx < m_cmdTarget.positionTolerance) &&
			       (dy < m_cmdTarget.positionTolerance) &&
			       (da < m_cmdTarget.angleTolerance);
		},
		/*end*/ [this](bool /*interrupted*/)
		{
			m_FreezeArm = true;
			m_LockPosition = true;
		}
	);

	// Ian: Publish initial SmartDashboard keys for target position (so they appear in the dashboard
	// for the user to edit before running MoveArmToPosition).  These match the Curivator's
	// Auton_Smart_GetMultiValue defaults at Curivator_Goals.cpp line 696.
	SmartDashboard::PutNumber("target_arm_xpos", 25.0);
	SmartDashboard::PutNumber("target_arm_ypos", -7.0);
	SmartDashboard::PutNumber("target_bucket_angle", 78.0);
	SmartDashboard::PutNumber("target_clasp_angle", 13.0);

	// Ian: Publish .type properties for the Scheduler, Subsystem, and Command widgets.
	// These must be set ONCE at init so the announce messages carry the correct properties.
	// The type hint is for the ".type" key itself (always a string).
	SmartDashboard::SetTopicProperties("Scheduler/.type", /*String=*/4, "{\"SmartDashboard\":\"Scheduler\"}");
	SmartDashboard::SetTopicProperties("ExcavatorArm/.type", /*String=*/4, "{\"SmartDashboard\":\"Subsystem\"}");
	for (const auto& cmd : m_commands)
	{
		SmartDashboard::SetTopicProperties(cmd.name + "/.type", /*String=*/4, "{\"SmartDashboard\":\"Command\"}");
	}
}

bool ExcavatorArm::ScheduleCommand(const std::string& commandName)
{
	// Find the command by name
	for (size_t i = 0; i < m_commands.size(); i++)
	{
		if (m_commands[i].name == commandName)
		{
			// Interrupt current command if one is running
			if (m_activeCommandIndex >= 0 && m_commandRunning)
			{
				auto& current = m_commands[static_cast<size_t>(m_activeCommandIndex)];
				if (current.end)
					current.end(/*interrupted=*/true);
			}
			// Start the new command
			m_activeCommandIndex = static_cast<int>(i);
			m_commandRunning = true;
			m_FreezeArm = false;  // Unfreeze arm for command control
			m_LockPosition = false;
			if (m_commands[i].initialize)
				m_commands[i].initialize();
			return true;
		}
	}
	return false;
}

void ExcavatorArm::CancelCommand()
{
	if (m_activeCommandIndex >= 0 && m_commandRunning)
	{
		auto& current = m_commands[static_cast<size_t>(m_activeCommandIndex)];
		if (current.end)
			current.end(/*interrupted=*/true);
		m_commandRunning = false;
		m_activeCommandIndex = -1;
	}
}

void ExcavatorArm::UpdateScheduler(double dTime_s)
{
	if (m_activeCommandIndex < 0 || !m_commandRunning)
		return;

	auto& cmd = m_commands[static_cast<size_t>(m_activeCommandIndex)];

	// Execute the command's per-frame callback
	if (cmd.execute)
		cmd.execute(dTime_s);

	// Check if the command has finished
	if (cmd.isFinished && cmd.isFinished())
	{
		if (cmd.end)
			cmd.end(/*interrupted=*/false);
		m_commandRunning = false;
		m_activeCommandIndex = -1;
	}
}

void ExcavatorArm::PublishSchedulerTelemetry()
{
	// Ian: Publish Scheduler widget keys.
	// WPILib expects: Scheduler/.type="Scheduler", Scheduler/Names=string[], Scheduler/Ids=int[],
	// Scheduler/Cancel=int[].  We publish Names and .type now; Ids/Cancel deferred until
	// PublishIntArray is plumbed through the SmartDashboard chain (requires dashboard→server
	// writes for Cancel, which isn't implemented yet).
	{
		SmartDashboard::PutString("Scheduler/.type", "Scheduler");
		// Build Names array: only the currently running command (matching WPILib behavior)
		std::vector<std::string> names;
		if (m_activeCommandIndex >= 0 && m_commandRunning)
			names.push_back(m_commands[static_cast<size_t>(m_activeCommandIndex)].name);
		SmartDashboard::PutStringArray("Scheduler/Names", names);
	}

	// Ian: Publish Subsystem widget keys for "ExcavatorArm".
	// WPILib expects: ExcavatorArm/.type="Subsystem", .hasDefault, .default, .hasCommand, .command
	{
		SmartDashboard::PutString("ExcavatorArm/.type", "Subsystem");
		SmartDashboard::PutBoolean("ExcavatorArm/.hasDefault", false);
		SmartDashboard::PutString("ExcavatorArm/.default", "");
		const bool hasCommand = (m_activeCommandIndex >= 0 && m_commandRunning);
		SmartDashboard::PutBoolean("ExcavatorArm/.hasCommand", hasCommand);
		SmartDashboard::PutString("ExcavatorArm/.command",
			hasCommand ? m_commands[static_cast<size_t>(m_activeCommandIndex)].name : "");
	}

	// Ian: Publish Command widget keys for each registered command.
	// WPILib expects: <CmdName>/.type="Command", <CmdName>/running=bool
	for (size_t i = 0; i < m_commands.size(); i++)
	{
		const auto& cmd = m_commands[i];
		const bool running = (static_cast<int>(i) == m_activeCommandIndex) && m_commandRunning;
		SmartDashboard::PutString(cmd.name + "/.type", "Command");
		SmartDashboard::PutBoolean(cmd.name + "/running", running);
	}
}

#endif // ENABLE_COMMAND_SCHEDULER

#pragma endregion

#pragma region _Telemetry_

void ExcavatorArm::PublishTelemetry()
{
	// Ian: Publish under "Manipulator/" prefix — already recognized as a folder group in
	// SmartDashboard.  These match the telemetry that the original Curivator published.

	// Ian: Joint shaft positions (inches) and potentiometer telemetry.
	// _Raw = position in shaft units (inches), Pot_Raw = normalized [0, 1] across the joint's range.
	// This matches the Curivator's Robot_TesterCode path in GetRotaryCurrentPorV() where:
	//   _Raw  = m_Potentiometer[index].GetPotentiometerCurrentPosition() (ship units)
	//   Pot_Raw = (result - MinRange) / (MaxRange - MinRange)  (normalized)
	for (size_t i = 0; i < kExcavatorJointCount; i++)
	{
		std::string prefix = "Manipulator/";
		prefix += GetJointName(static_cast<ExcavatorJoint>(i));
		const double pos = m_simulated_positions[i];
		SmartDashboard::PutNumber(prefix + "_ShaftPos", pos);
		// _Raw: position in shaft units (like the TesterCode path)
		SmartDashboard::PutNumber(prefix + "_Raw", pos);
		// Pot_Raw: normalized 0–1 across the joint's range
		const auto& sp = m_joint_props[i].ship_props;
		const double range = sp.MaxRange - sp.MinRange;
		const double normalized = (range > 0.0) ? (pos - sp.MinRange) / range : 0.0;
		SmartDashboard::PutNumber(prefix + "Pot_Raw", normalized);
	}

	// Forward kinematics results
	SmartDashboard::PutNumber("Manipulator/BigArm_Angle", RAD_2_DEG(m_fk.BigArmAngle));
	SmartDashboard::PutNumber("Manipulator/BigArm_Length", m_fk.BigArmLength);
	SmartDashboard::PutNumber("Manipulator/BigArm_Height", m_fk.BigArmHeight);

	SmartDashboard::PutNumber("Manipulator/Boom_Angle", RAD_2_DEG(m_fk.BoomAngle));
	SmartDashboard::PutNumber("Manipulator/Boom_Length", m_fk.BoomLength);
	SmartDashboard::PutNumber("Manipulator/Boom_Height", m_fk.BoomHeight);

	SmartDashboard::PutNumber("Manipulator/Bucket_Angle", RAD_2_DEG(m_fk.BucketAngle));
	SmartDashboard::PutNumber("Manipulator/Bucket_Length", m_fk.BucketLength);
	SmartDashboard::PutNumber("Manipulator/Bucket_TipHeight", m_fk.BucketTipHeight);
	SmartDashboard::PutNumber("Manipulator/Bucket_RoundEndHeight", m_fk.BucketRoundEndHeight);

	SmartDashboard::PutNumber("Manipulator/Clasp_Angle", RAD_2_DEG(m_fk.ClaspAngle));
	SmartDashboard::PutNumber("Manipulator/Clasp_Distance", m_fk.ClaspMidlineDistance);
	SmartDashboard::PutNumber("Manipulator/Clasp_Height", m_fk.ClaspMidlineHeight);
	SmartDashboard::PutNumber("Manipulator/Clasp_MinHeight", m_fk.ClaspMinHeight);

	// Ian: 3D position feedback — these are the FK-computed actual bucket position,
	// matching the Curivator's SmartDashboard output at lines 720-723 and 362-368.
	SmartDashboard::PutNumber("Manipulator/arm_xpos", m_3DPos_BucketDistance);
	SmartDashboard::PutNumber("Manipulator/arm_ypos", m_3DPos_BucketHeight);
	SmartDashboard::PutNumber("Manipulator/bucket_angle", m_3DPos_BucketAngle);

	// Ian: Virtual joint desired positions — what the user has commanded via keyboard.
	// Only meaningful when EnableArmAutoPosition=true, but we always publish for visibility.
	if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)])
		SmartDashboard::PutNumber("Manipulator/arm_xpos_desired", m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)]->GetPos_m());
	if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)])
		SmartDashboard::PutNumber("Manipulator/arm_ypos_desired", m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)]->GetPos_m());
	if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)])
		SmartDashboard::PutNumber("Manipulator/bucket_angle_desired", m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->GetPos_m());
	if (m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)])
		SmartDashboard::PutNumber("Manipulator/clasp_angle_desired", m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)]->GetPos_m());

	// Ian: Read back any SmartDashboard toggle of enable_arm_auto_position.
	// The key was published once in Init(); here we only read.
	{
		bool val;
		if (SmartDashboard::TryGetBoolean("Manipulator/EnableArmAutoPosition", val))
			m_enable_arm_auto_position = val;
	}

	// Ian: Phase 2 — publish Scheduler/Subsystem/Command NT keys for dashboard widgets.
#if ENABLE_COMMAND_SCHEDULER
	PublishSchedulerTelemetry();
#endif
}

#pragma endregion

#pragma region _CreateUI_

// Ian: Factory method — creates the excavator arm side-view visualization.
// The ManipulatorArm_UI is fully self-contained; TeleAutonV2 sets up the state callback
// to map our ForwardKinematicsResult into the ArmState struct each frame.
std::unique_ptr<Module::Output::ManipulatorUI_Plugin> ExcavatorArm::CreateUI()
{
	return std::make_unique<Module::Output::ManipulatorArm_UI>();
}

// Ian: SetupUI — wires the ManipulatorArm_UI's state callback to our FK data.
// This is where the mapping from ForwardKinematicsResult to ArmState happens.
// TeleAutonV2 calls this once after CreateUI(), passing the non-owning pointer.
void ExcavatorArm::SetupUI(Module::Output::ManipulatorUI_Plugin* ui)
{
	// Downcast to our concrete UI type — safe because CreateUI() created it
	auto* armUI = static_cast<Module::Output::ManipulatorArm_UI*>(ui);
	if (!armUI) return;

	// Ian: The callback captures 'this' — it's called every frame from the OSG update callback
	// to read the latest FK results.  The mapping is straightforward because ArmState was
	// designed to mirror ForwardKinematicsResult.
	armUI->SetArmState_Callback([this]() -> Module::Output::ManipulatorArm_UI::ArmState
	{
		const auto& fk = m_fk;
		Module::Output::ManipulatorArm_UI::ArmState state;
		state.BigArmLength = fk.BigArmLength;
		state.BigArmHeight = fk.BigArmHeight;
		state.BoomAngle = fk.BoomAngle;
		state.BoomLength = fk.BoomLength;
		state.BoomHeight = fk.BoomHeight;
		state.BucketLength = fk.BucketLength;
		state.BucketTipHeight = fk.BucketTipHeight;
		state.BucketAngle = fk.BucketAngle;
		state.BucketRoundEndHeight = fk.BucketRoundEndHeight;
		state.Bucket_globalBRP_BP_height = fk.Bucket_globalBRP_BP_height;
		state.Bucket_globalBRP_BP_distance = fk.Bucket_globalBRP_BP_distance;
		state.ClaspMidlineDistance = fk.ClaspMidlineDistance;
		state.ClaspMidlineHeight = fk.ClaspMidlineHeight;
		state.CoMDistance = fk.CoMDistance;
		state.CoMHeight = fk.CoMHeight;
		// Ian: BucketAngleContinuity — matches legacy Curivator_Robot::GetBucketAngleContinuity()
		// (Curivator_Robot.cpp:995):  |virtualJoint_bucketAngle - FK_actual_bucketAngle| in degrees.
		// m_3DPos_BucketAngle is the FK result (degrees), the virtual joint holds the desired value.
		{
			const double desiredBucketAngle = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]
				? m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)]->GetPos_m() : 0.0;
			state.BucketAngleContinuity = std::fabs(desiredBucketAngle - m_3DPos_BucketAngle);
		}
		return state;
	});
}

#pragma endregion

#pragma region _Test Goal Contribution_
// Ian: Test goal contribution — ExcavatorArm self-describes its diagnostic test goals
// for the Test chooser.  This is the loose-coupling contract: the AI goal system never
// hard-codes excavator-specific enums.  Each new manipulator (excavator, intake, shooter)
// overrides GetTestGoals() and CreateTestGoal() to plug in its own tests.
// (ExcavatorGoals.h is included at the top of this file, outside the namespace block.)

const TestGoalDescriptor* ExcavatorArm::GetTestGoals(size_t& count) const
{
	// Ian: Plugin-local indices — these are private to ExcavatorArm and never leak
	// into the AI_Input enum space.  The AI system adds a base offset when building
	// the combined chooser.
	enum ExcavatorTestGoalIndex
	{
		eArmMoveToPosition = 0,
		eArmGrabSequence,
		eClawGrab,
		eCount
	};
	static const TestGoalDescriptor kGoals[] =
	{
		{eArmMoveToPosition, "Arm Move To Position"},
		{eArmGrabSequence,   "Arm Grab Sequence"},
		{eClawGrab,          "Claw Grab"}
	};
	count = _countof(kGoals);
	return kGoals;
}

Framework::Base::Goal* ExcavatorArm::CreateTestGoal(int pluginLocalIndex)
{
	// Ian: Goals need direct references to virtual joints (Ship_1D entities) to call
	// SetIntendedPosition() once in Activate() — matching the Curivator's pattern.
	// We extract the 4 virtual joint pointers here and pass them to the goal factories.
	Legacy::Ship_1D* xpos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)];
	Legacy::Ship_1D* ypos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)];
	Legacy::Ship_1D* bucket = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)];
	Legacy::Ship_1D* clasp = m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)];
	if (!xpos || !ypos || !bucket || !clasp)
	{
		printf("ExcavatorArm::CreateTestGoal — virtual joints not initialized\n");
		return nullptr;
	}
	// Ian: FreezeArm/LockPosition callbacks — capture 'this' to set the boolean members
	// directly on ExcavatorArm.  This matches the legacy event-dispatched approach where
	// RobotQuickNotify fires "Robot_FreezeArm" / "Robot_LockPosition" events.
	// See Curivator_Robot.cpp:1000 (FreezeArm) and :1006 (LockPosition).
	auto setFreezeArm = [this](bool isOn) { m_FreezeArm = isOn; };
	auto setLockPosition = [this](bool isOn) { m_LockPosition = isOn; };
	switch (pluginLocalIndex)
	{
	case 0: return ExcavatorGoals::TestArmMove(*xpos, *ypos, *bucket, *clasp);
	case 1: return ExcavatorGoals::TestArmGrabSequence(*xpos, *ypos, *bucket, *clasp, setFreezeArm, setLockPosition);
	case 2: return new ExcavatorGoals::ClawGrabSequence(*xpos, *ypos, *bucket, *clasp, setFreezeArm, setLockPosition);
	default: return nullptr;
	}
}

// Ian: Auton goal contribution — ExcavatorArm provides autonomous routines for the Auton
// chooser that combine drive waypoints with arm grab sequences.  Same loose-coupling pattern
// as test goals: the AI system never imports ExcavatorGoals.h.
const TestGoalDescriptor* ExcavatorArm::GetAutonGoals(size_t& count) const
{
	enum ExcavatorAutonGoalIndex
	{
		eGrabAndReturn = 0,
		eCount
	};
	static const TestGoalDescriptor kGoals[] =
	{
		{eGrabAndReturn, "Grab and Return"}
	};
	count = _countof(kGoals);
	return kGoals;
}

Framework::Base::Goal* ExcavatorArm::CreateAutonGoal(int pluginLocalIndex, Module::Input::AI_Input* controller)
{
	Legacy::Ship_1D* xpos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmXpos)];
	Legacy::Ship_1D* ypos = m_virtualJoints[static_cast<size_t>(VirtualJoint::eArmYpos)];
	Legacy::Ship_1D* bucket = m_virtualJoints[static_cast<size_t>(VirtualJoint::eBucketAngle)];
	Legacy::Ship_1D* clasp = m_virtualJoints[static_cast<size_t>(VirtualJoint::eClaspAngle)];
	if (!xpos || !ypos || !bucket || !clasp)
	{
		printf("ExcavatorArm::CreateAutonGoal — virtual joints not initialized\n");
		return nullptr;
	}
	if (!controller)
	{
		printf("ExcavatorArm::CreateAutonGoal — controller is null\n");
		return nullptr;
	}
	auto setFreezeArm = [this](bool isOn) { m_FreezeArm = isOn; };
	auto setLockPosition = [this](bool isOn) { m_LockPosition = isOn; };
	switch (pluginLocalIndex)
	{
	case 0: return new ExcavatorGoals::AutonGrabAndReturn(
		controller, *xpos, *ypos, *bucket, *clasp, setFreezeArm, setLockPosition);
	default: return nullptr;
	}
}

#pragma endregion

}  // namespace Robot
}  // namespace Module
