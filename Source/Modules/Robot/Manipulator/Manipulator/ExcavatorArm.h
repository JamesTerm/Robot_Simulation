// Ian: Set to 1 to enable the embedded CommandScheduler that publishes WPILib-compatible
// Scheduler/Subsystem/Command NT keys.  Currently disabled: the official SmartDashboard 2026
// does not render these widgets, and the NT4 path has not been validated end-to-end.
// Re-enable once a compatible dashboard (Shuffleboard or updated SmartDashboard) is available.
#define ENABLE_COMMAND_SCHEDULER 0

#pragma once
// Ian: ExcavatorArm — first concrete ManipulatorPlugin.  Ports the Curivator excavator's geometry
// math (forward kinematics, inverse kinematics) into the modern Robot_Simulation framework using
// RotarySystem_Position for each joint.  All physical constants come from the original CAD-derived
// values in Curivator_Robot.cpp.
//
// Two control layers (matching the original Curivator architecture):
//   Layer 1 — Physical dart joints (BigArm, Boom, Bucket, Clasp): RotarySystem_Position with
//             PID, potentiometer feedback via odometry callback, voltage integrator.
//   Layer 2 — Virtual 3D-position joints (arm_xpos, arm_ypos, bucket_angle, clasp_angle):
//             Ship_1D entities whose GetPos_m() returns the desired bucket tip XY position and
//             angle.  When EnableArmAutoPosition=true, these are the MASTER: each frame their
//             positions are read, IK computes shaft lengths, and SetIntendedPosition drives the
//             physical joints.  FK feedback writes the actual bucket position back to these
//             virtual joints via Update3DPositioningPosition().
//
// When EnableArmAutoPosition=false, Layer 2 is inactive and Layer 1 joints are controlled
// directly via keyboard keys 1-8 (for dart tuning).
//
// Turret is omitted (it was velocity-controlled, not position).
//
// The geometry math is self-contained — the only external dependency is RotarySystem_Position for
// the simulated actuator physics (position control, voltage output, simulated odometry).

#include "ManipulatorPlugin.h"
#include "../../../../Modules/Robot/SwerveRobot/SwerveRobot/RotarySystem.h"
#include "../../../../Modules/Robot/SwerveRobot/SwerveRobot/Ship1D_Legacy.h"
#include "../../../../Base/AssetManager.h"
#include "../../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <array>
#include <string>
#include <cmath>
#include <functional>
#include <vector>

namespace Module {
namespace Robot {

// Ian: Geometry constants ported verbatim from Curivator_Robot.cpp — all values in inches, matching
// the original CAD.  Grouped by joint for readability.  Do NOT convert to meters; the forward and
// inverse kinematics operate entirely in inches, and only the final telemetry output converts.
namespace ExcavatorGeometry
{
	//--- BigArm ---
	constexpr double BigArm_Radius                          = 39.77287247;
	constexpr double BigArm_DartLengthAt90                  = 29.72864858;
	constexpr double BigArm_DartToArmDistance                = 9.0;
	constexpr double BigArm_DistanceFromTipDartToClevis      = 2.0915;
	constexpr double BigArm_DistanceDartPivotToTip           = 17.225;
	constexpr double BigArm_ConnectionOffset                 = 5.39557923;
	constexpr double BigArm_DistanceBigArmPivotToDartPivot   = 26.01076402;
	constexpr double BigArm_AngleToDartPivotInterface        = 11.7190273 * M_PI / 180.0;
	constexpr double BigArm_AngleToDartPivotInterface_Length = 26.56448983;
	constexpr double BigArm_DartPerpendicularAngle           = 61.03779676;

	//--- Boom ---
	constexpr double Boom_Radius                             = 23.03394231;
	constexpr double Boom_Radius_BP                          = 26.03003642;
	constexpr double Boom_BP_To_RBP_RadiusAngle              = 0.35809296 * M_PI / 180.0;
	constexpr double Boom_BP_To_Lever_angle                  = 175.16494932 * M_PI / 180.0;
	constexpr double Boom_DartToArmDistance                   = 18.51956156;
	constexpr double Boom_DistanceFromTipDartToClevis         = 2.0915;
	constexpr double Boom_DistanceDartPivotToTip              = 11.5;
	constexpr double Boom_AngleToDartPivotInterface           = 4.83505068 * M_PI / 180.0;
	constexpr double Boom_AngleToDartPivotInterface_Length    = 6.87954395;
	constexpr double Boom_AngleBigArmToDartPivot              = 36.18122057 * M_PI / 180.0;

	//--- Bucket (4-bar rocker linkage) ---
	constexpr double Bucket_BRP_To_LAB                       = 17.2528303;
	constexpr double Bucket_LAB_housingLength                = 12.4;
	constexpr double Bucket_RockerBoomLength                 = 6.49874999;
	constexpr double Bucket_BRP_LABtoBRP_BP_Angle            = 175.01 * M_PI / 180.0;
	constexpr double Bucket_BRP_To_BP                        = 3.0;
	constexpr double Bucket_BP_To_BucketRP                   = 7.79685753;
	constexpr double Bucket_RockerBucketLength               = 7.3799994;
	constexpr double Bucket_BP_To_BucketCoM                  = 7.70049652;
	constexpr double Bucket_BoomAngleToLAB_Angle             = 8.09856217 * M_PI / 180.0;
	constexpr double Bucket_HorizontalToBRP_BP_Angle         = Bucket_BRP_LABtoBRP_BP_Angle + Bucket_BoomAngleToLAB_Angle - (M_PI / 2.0);
	constexpr double Bucket_BucketRPtoBucketCoM_Angle        = 46.01897815 * M_PI / 180.0;
	// Ian: pre-computed constant positions of the BRP-to-BP offset used by both Bucket and Clasp FK
	// Note: std::sin/cos are not constexpr in C++17 (MSVC).  Using 'const' at namespace scope
	// gives internal linkage per TU, avoiding ODR violations without requiring C++17 'inline'.
	const double Bucket_localConstantBRP_BP_height       = std::sin(Bucket_HorizontalToBRP_BP_Angle) * Bucket_BRP_To_BP;
	const double Bucket_localConstantBRP_BP_distance     = std::cos(Bucket_HorizontalToBRP_BP_Angle) * Bucket_BRP_To_BP;
	constexpr double Bucket_CoMtoTip_Angle                   = 32.1449117 * M_PI / 180.0;
	constexpr double Bucket_BPBT_ToBucketRP_Angle            = Bucket_CoMtoTip_Angle + Bucket_BucketRPtoBucketCoM_Angle;
	constexpr double Bucket_BP_to_BucketTip                  = 13.12746417;
	constexpr double Bucket_BPTip_to_BucketInterface_Angle   = 12.4082803 * M_PI / 180.0;
	constexpr double Bucket_CoM_Radius                       = 5.0;

	//--- Clasp (clamshell jaw) ---
	constexpr double Clasp_BRP_To_LAC                        = 10.97993151;
	constexpr double Clasp_LAC_housingLength                 = 7.4;
	constexpr double Clasp_CP_To_LAC                         = 3.11799058;
	constexpr double Clasp_BoomAngleToLAC_Angle              = 9.58288198 * M_PI / 180.0;
	constexpr double Clasp_MidlineSegment                    = 10.31720455;
	constexpr double Clasp_LA_Interface_to_Midline_Angle     = 125.84975925 * M_PI / 180.0;
	constexpr double Clasp_MidlineToEdge_Angle               = 101.61480361 * M_PI / 180.0;
	constexpr double Clasp_BottomToSideAngle                 = 98.23909595 * M_PI / 180.0;
	constexpr double Clasp_BottomEdgeLength                  = 1.507182;
	constexpr double Clasp_BottomEdgeLength_Half             = Clasp_BottomEdgeLength / 2.0;
	constexpr double Clasp_CP_To_MidlineTip                  = 12.40350045;
	constexpr double Clasp_CPMT_ToSide_Angle                 = 8.09713426 * M_PI / 180.0;
	constexpr double Clasp_MidLineToEdge_Angle               = Clasp_MidlineToEdge_Angle + Clasp_BottomToSideAngle - M_PI;
}

// Joint indices — order matches Curivator's SpeedControllerDevices minus Turret
enum class ExcavatorJoint : size_t
{
	eBigArm = 0,
	eBoom,
	eBucket,
	eClasp,
	eCount
};
constexpr size_t kExcavatorJointCount = static_cast<size_t>(ExcavatorJoint::eCount);

// Ian: Virtual 3D position joint indices — these are "meta-joints" whose position represents
// the desired bucket tip XY position and angle.  They match the Curivator's eArm_Xpos, eArm_Ypos,
// eBucket_Angle, eClasp_Angle.  When EnableArmAutoPosition=true, IK converts these into shaft
// lengths for the physical joints.
enum class VirtualJoint : size_t
{
	eArmXpos = 0,     // horizontal distance from pivot (inches)
	eArmYpos,         // height from pivot (inches, negative = below)
	eBucketAngle,     // bucket angle (degrees)
	eClaspAngle,      // clasp opening angle (degrees)
	eCount
};
constexpr size_t kVirtualJointCount = static_cast<size_t>(VirtualJoint::eCount);

class ExcavatorArm : public ManipulatorPlugin
{
public:
	ExcavatorArm();
	~ExcavatorArm() override = default;

	// --- ManipulatorPlugin interface ---
	void Init(const Framework::Base::asset_manager& properties) override;
	void TimeSlice(double dTime_s) override;
	void PublishTelemetry() override;
	void Reset() override;
	const char* GetName() const override { return "ExcavatorArm"; }

	// Ian: CreateUI() — factory method returns the excavator arm side-view visualization.
	// The returned ManipulatorArm_UI is fully self-contained; TeleAutonV2 just needs to set
	// the state callback and wire it into the viewer's scene/update callbacks.
	std::unique_ptr<Module::Output::ManipulatorUI_Plugin> CreateUI() override;
	void SetupUI(Module::Output::ManipulatorUI_Plugin* ui) override;

	// Voltage detach
	void SetVoltageDetached(bool detach) override { m_voltageDetached = detach; }
	bool GetVoltageDetached() const override { return m_voltageDetached; }

	// Ian: Direct setter for unit tests and runtime control — sets the dedicated
	// enable_arm_auto_position boolean.  No SmartDashboard involvement.
	void SetEnableArmAutoPosition(bool val) { m_enable_arm_auto_position = val; }

	// Ian: Getters for unit tests — these correspond to the SmartDashboard telemetry values:
	//   arm_xpos / arm_ypos / bucket_angle  (actual, from FK)
	//   arm_xpos_desired / arm_ypos_desired / bucket_angle_desired  (desired, from virtual joints)
	double GetActual_ArmXpos() const { return m_3DPos_BucketDistance; }
	double GetActual_ArmYpos() const { return m_3DPos_BucketHeight; }
	double GetActual_BucketAngle() const { return m_3DPos_BucketAngle; }
	double GetDesired_ArmXpos() const;
	double GetDesired_ArmYpos() const;
	double GetDesired_BucketAngle() const;

	// Ian: Diagnostic getters for unit tests — expose internal state to trace Layer 2 chain.
	double GetSimulatedPosition(size_t jointIndex) const
	{
		return (jointIndex < kExcavatorJointCount) ? m_simulated_positions[jointIndex] : 0.0;
	}
	double GetStartingPosition(size_t jointIndex) const
	{
		return (jointIndex < kExcavatorJointCount) ? m_starting_positions[jointIndex] : 0.0;
	}

	// Ian: PID monitor callback for diagnostic tests — hooks into the legacy PID's per-frame output.
	// Signature: void(Voltage, Position, PredictedPosition, CurrentVelocity, EncoderVelocity, ErrorOffset)
	using PID_Monitor_proto = RotarySystem_Position::PID_Monitor_proto;
	void SetPIDMonitorCallback(size_t jointIndex, std::function<PID_Monitor_proto> callback)
	{
		if (jointIndex < kExcavatorJointCount)
			m_joints[jointIndex].SetExternal_PID_Monitor_Callback(callback);
	}

	// Ian: Voltage trace callback for diagnostic tests — called from the voltage callback lambda
	// with (jointIndex, voltage).  This fires unconditionally (unlike PID_Monitor which requires
	// PID_Console_Dump=true).
	using VoltageTrace_proto = void(size_t jointIndex, double voltage);
	void SetVoltageTraceCallback(std::function<VoltageTrace_proto> callback)
	{
		m_voltageTraceCallback = callback;
	}

	// Layer 1: Joint input — keyboard/joystick drives physical dart joints directly.
	// Only effective when EnableArmAutoPosition=false (dart tuning mode).
	void SetJointInput(size_t jointIndex, double normalizedVelocity) override;
	size_t GetJointCount() const override { return kExcavatorJointCount; }

	// Layer 2: 3D positioning input — keyboard/joystick drives virtual position joints.
	// Only effective when EnableArmAutoPosition=true.
	// posIndex: 0=xpos, 1=ypos, 2=bucket_angle, 3=clasp_angle
	void Set3DPositionInput(size_t posIndex, double normalizedVelocity) override;
	size_t Get3DPositionCount() const override { return kVirtualJointCount; }

	// Ian: Test goal contribution — ExcavatorArm provides its own diagnostic test goals
	// to the Test chooser via the ManipulatorPlugin loose-coupling interface.
	// GetTestGoals() returns descriptors for: Arm Move To Position, Arm Grab Sequence, Claw Grab.
	// CreateTestGoal() instantiates the corresponding goal from ExcavatorGoals.h.
	const TestGoalDescriptor* GetTestGoals(size_t& count) const override;
	Framework::Base::Goal* CreateTestGoal(int pluginLocalIndex) override;

	// Ian: Auton goal contribution — ExcavatorArm provides autonomous routines that combine
	// drive + arm manipulation for the Auton chooser.  Currently: "Grab and Return" — drives to
	// two waypoints performing arm grab sequences, then returns to center.
	// Future manipulators will override these with their own autonomous routines.
	const TestGoalDescriptor* GetAutonGoals(size_t& count) const override;
	Framework::Base::Goal* CreateAutonGoal(int pluginLocalIndex, Module::Input::AI_Input* controller) override;

	// Ian: GetVirtualJoint — provides direct access to virtual joint Ship_1D entities.
	// Used by ExcavatorGoals to create Goal_Ship1D_MoveToPosition goals that hold references
	// to the joints (matching Curivator's pattern where goals reference Robot_Arm directly).
	// Returns nullptr if index is out of range or joint not initialized.
	Legacy::Ship_1D* GetVirtualJoint(VirtualJoint joint) const
	{
		const size_t idx = static_cast<size_t>(joint);
		return (idx < kVirtualJointCount) ? m_virtualJoints[idx] : nullptr;
	}

	// --- Forward kinematics results (inches) ---
	// These are computed each TimeSlice and cached for telemetry and later IK use
	struct ForwardKinematicsResult
	{
		// BigArm
		double BigArmAngle = 0.0;      // radians
		double BigArmLength = 0.0;     // horizontal distance from pivot (inches)
		double BigArmHeight = 0.0;     // vertical distance from pivot (inches)
		// Boom
		double BoomAngle = 0.0;        // radians, global from vertical
		double BoomLength = 0.0;       // global horizontal distance (inches)
		double BoomHeight = 0.0;       // global height (inches)
		// Bucket
		double BucketLength = 0.0;     // global distance to tip (inches)
		double BucketTipHeight = 0.0;  // global height of bucket tip (inches)
		double BucketRoundEndHeight = 0.0;
		double BucketAngle = 0.0;      // global bucket angle (radians)
		double Bucket_globalBRP_BP_height = 0.0;
		double Bucket_globalBRP_BP_distance = 0.0;
		// Ian: CoM position needed for the bucket circle rendering in the side-view UI.
		// These are global coordinates (relative to arm origin): CoMHeight is positive downward
		// from BoomHeight, CoMDistance is positive rightward from BoomLength.
		// The UI callback maps these to ManipulatorArm_UI::ArmState::CoMDistance/CoMHeight.
		double CoMHeight = 0.0;        // BoomHeight - GlobalCoMHeight (inches)
		double CoMDistance = 0.0;      // BoomLength + GlobalCoMDistance (inches)
		// Clasp
		double ClaspMidlineHeight = 0.0;
		double ClaspMidlineDistance = 0.0;
		double ClaspAngle = 0.0;       // radians
		double ClaspMinHeight = 0.0;
	};

	const ForwardKinematicsResult& GetFK() const { return m_fk; }

	// --- Inverse kinematics ---
	// Given a desired end-effector pose, compute the 4 shaft lengths
	struct IKInput
	{
		double GlobalHeight;         // inches, positive above pivot
		double GlobalDistance;        // inches, from pivot to bucket tip
		double BucketAngle_deg;      // degrees
		double ClaspOpeningAngle_deg; // degrees
	};
	struct IKOutput
	{
		double BigArmShaftLength;
		double BoomShaftLength;
		double BucketShaftLength;
		double ClaspShaftLength;
	};
	static IKOutput ComputeArmPosition(const IKInput& input);

	// ====================================================================
	// Phase 2: Lightweight CommandScheduler (embedded in concrete plugin)
	// ====================================================================
#if ENABLE_COMMAND_SCHEDULER
	// Ian: This implements the WPILib Command/Subsystem/Scheduler NT key patterns so
	// dashboards (ours and official) can display subsystem status and allow command
	// start/cancel.  The commands wrap Curivator-style diagnostic goals adapted to
	// the modern ExcavatorArm interface.
	//
	// Design:
	//   - One subsystem: "ExcavatorArm"
	//   - Multiple named commands: each is a struct with name + callbacks for
	//     initialize/execute/isFinished/end.  Only one runs at a time (subsystem is
	//     single-requirement, matching WPILib's default scheduler behavior).
	//   - The scheduler publishes three groups of NT keys each frame:
	//       Scheduler widget:  Names=string[], Ids=int[], Cancel=int[], .type="Scheduler"
	//       Subsystem widget:  .hasDefault, .default, .hasCommand, .command, .type="Subsystem"
	//       Command widgets:   running=bool, .type="Command"  (one per registered command)

	// A command's behavior callbacks.  Loosely coupled to the arm — the callbacks
	// capture 'this' to read/write arm state.
	struct CommandBehavior
	{
		std::string name;
		int id = 0;                                // unique command ID
		std::function<void()> initialize;          // called once when command starts
		std::function<void(double)> execute;       // called every frame with dTime_s
		std::function<bool()> isFinished;           // return true when command is done
		std::function<void(bool)> end;             // called on finish (true) or cancel (false)
	};

	// Schedule a command by name.  If another command is running on this subsystem, it is
	// interrupted first (end(false)).  Returns false if the command name is not found.
	bool ScheduleCommand(const std::string& commandName);
	// Cancel the currently running command (if any).
	void CancelCommand();
#endif // ENABLE_COMMAND_SCHEDULER

private:
	// Ian: LawOfCosines — given three side lengths, returns the angle opposite side c.
	// Ported from Curivator_Robot.cpp.  Used extensively by both FK and IK.
	static double LawOfCosines(double a, double b, double c);
	static double GetDistance(double x1, double y1, double x2, double y2);
	static double EnforceShaftLimits(double value, double minRange = 0.75, double maxRange = 11.0);

	// ====== Layer 1: Physical dart joints ======
	std::array<RotarySystem_Position, kExcavatorJointCount> m_joints;
	std::array<rotary_properties, kExcavatorJointCount> m_joint_props;

	// Ian: Simulated positions — the voltage output drives a simple integrator that models the
	// dart extending/retracting.  These hold the current shaft extension (inches).  In 3D mode,
	// the IK sets intended positions on the joints' PID; the PID outputs voltage; the voltage
	// integrator moves these simulated positions.  The PID reads them back via the odometry callback.
	std::array<double, kExcavatorJointCount> m_simulated_positions = {};
	std::array<double, kExcavatorJointCount> m_starting_positions = {};

	// ====== Layer 2: Virtual 3D-position joints ======
	// Ian: These are Ship_1D entities matching the Curivator's m_ArmXpos, m_ArmYpos, m_BucketAngle,
	// m_ClaspAngle.  When EnableArmAutoPosition=true, they are the MASTER controllers.  Keyboard
	// input drives their requested velocity (j/k=xpos, l/;=ypos, u/i=bucket_angle).  Each frame,
	// their GetPos_m() returns the desired 3D position, IK converts to shaft lengths, and
	// SetIntendedPosition drives the physical joints.  FK feedback (Update3DPositioningPosition)
	// writes the actual bucket position back so the PID loop can correct.
	//
	// These are constructed with LoopState=eNone (no potentiometer) EXCEPT when feedback is
	// desired.  In the original Curivator, xpos had LoopState=eNone and using_range=0 (commented
	// out is_closed), while ypos, bucket_angle, and clasp_angle were similar.  The FK callback
	// provides the "sensor" reading.
	std::array<Legacy::Ship_1D*, kVirtualJointCount> m_virtualJoints = {};
	std::array<std::unique_ptr<Legacy::Ship_1D>, kVirtualJointCount> m_virtualJointStorage;
	std::array<Legacy::Ship_1D_Properties, kVirtualJointCount> m_virtualJointProps;

	// Ian: 3D position feedback from FK — these are written by ComputeForwardKinematics() and
	// read by the virtual joints' GetRotaryCurrentPorV if they use closed-loop feedback.
	// Matches Curivator_Robot_Control::m_3DPos_BucketDistance/Height/Angle.
	double m_3DPos_BucketDistance = 0.0;
	double m_3DPos_BucketHeight = 0.0;
	double m_3DPos_BucketAngle = 0.0;

	// Ian: enable_arm_auto_position — dedicated boolean matching Curivator_Robot.cpp line 1262:
	//   SCRIPT_TEST_BOOL_YES(props.EnableArmAutoPosition,"enable_arm_auto_position");
	// This is the authoritative source of truth.  SmartDashboard publishes it as read-only
	// telemetry but does NOT write back — avoids SmartDashboard fighting with TimeSlice updates.
	// When true, Layer 2 is active (virtual joints drive IK which drives physical joints).
	// When false, Layer 1 is active (keyboard directly drives darts).
	// Default=true matching the Curivator lua (line 191: enable_arm_auto_position='y').
	bool m_enable_arm_auto_position = true;

	// Ian: FreezeArm — when true, all arm motion stops (both layers).  Matches Curivator's m_FreezeArm.
	bool m_FreezeArm = false;

	// Ian: LockPosition — when true, the virtual joints' positions are frozen to the last known
	// values.  Matches Curivator's m_LockPosition.
	bool m_LockPosition = false;
	double m_Last_xpos = 0.0;
	double m_Last_ypos = 0.0;
	double m_Last_bucket_angle = 0.0;
	double m_Last_clasp_angle = 0.0;

	// Cached FK results
	ForwardKinematicsResult m_fk;

	// Forward kinematics computation — updates m_fk from current joint positions
	void ComputeForwardKinematics();

	// Ian: Update3DPositioningPosition — called by ComputeForwardKinematics() to feed back the
	// FK-computed global bucket position to the virtual joints' feedback members.  Matches
	// Curivator_Robot_Control::Update3DPositioningPosition().
	void Update3DPositioningPosition(double BucketDistance, double BucketHeight, double BucketAngle_deg);

	// Ian: Voltage detach — when true, voltage callbacks output 0.0 regardless of PID output.
	// Toggled via SmartDashboard "Manipulator/VoltageDetached" from TeleAutonV2.
	bool m_voltageDetached = false;

	// Ian: Cached frame dt for the voltage callback.  The voltage callback is called from within
	// m_rotary_legacy->TimeChange() which doesn't pass dt, so we store it before calling TimeSlice.
	// Previously hardcoded to 0.016 (~60Hz) which was inaccurate for 0.02 (50Hz) test frames.
	double m_lastDTime_s = 0.02;

	// Ian: Joint input from keyboard/joystick — normalized [-1, 1] velocity per joint.
	// Applied each TimeSlice to m_simulated_positions when EnableArmAutoPosition=false.
	std::array<double, kExcavatorJointCount> m_jointInputs = {};

	// Ian: 3D position input from keyboard/joystick — normalized [-1, 1] velocity per virtual joint.
	// Applied each TimeSlice to virtual joints when EnableArmAutoPosition=true.
	std::array<double, kVirtualJointCount> m_3DPosInputs = {};

	// Ian: Voltage trace callback for diagnostic tests
	std::function<VoltageTrace_proto> m_voltageTraceCallback = nullptr;

	// Joint name table for telemetry
	static const char* GetJointName(ExcavatorJoint joint);

	// Helper to populate default rotary_properties for a linear actuator joint
	void SetDefaultJointProps(ExcavatorJoint joint, double minRange, double maxRange, double startingPos);

	// Helper to populate default Ship_1D_Properties for a virtual 3D position joint
	void SetDefaultVirtualJointProps(VirtualJoint joint, double minRange, double maxRange,
		double startingPos, double maxSpeed, double accel, double brake, bool usingRange);

	// ====== Phase 2: CommandScheduler state ======
#if ENABLE_COMMAND_SCHEDULER
	// Ian: The scheduler is embedded in the concrete plugin (ManipulatorPlugin.h lines 8-9),
	// not a separate class.  This keeps the command behaviors tightly coupled to the arm's
	// IK/FK/virtual-joint internals without exposing them through an abstract interface.
	// Only one command runs at a time (single-requirement subsystem).

	// Ian: Registry of all available commands.  Populated once in InitScheduler().
	// Index into this vector == command ID for NT key publishing.
	std::vector<CommandBehavior> m_commands;

	// Ian: Active command tracking.  m_activeCommandIndex < 0 means no command is running.
	// m_commandRunning is the per-frame "is executing" flag published as Command/running.
	int m_activeCommandIndex = -1;
	bool m_commandRunning = false;

	// Ian: Scheduler initialization flag — prevents double-registration if Init() is called
	// multiple times (e.g., during viewer restarts).
	bool m_schedulerInitialized = false;

	// Ian: Scheduler lifecycle methods.  Called from Init/TimeSlice/PublishTelemetry respectively.
	void InitScheduler();
	void UpdateScheduler(double dTime_s);
	void PublishSchedulerTelemetry();

	// Ian: Command behavior target state — shared by the command callbacks.
	// MoveArmToPosition reads these from SmartDashboard on initialize(), execute() drives
	// virtual joints toward them, isFinished() checks convergence.
	struct CommandTargetState
	{
		double targetXpos = 25.0;
		double targetYpos = -7.0;
		double targetBucketAngle = 78.0;
		double targetClaspAngle = 13.0;
		double positionTolerance = 1.0;    // inches — convergence threshold for XY
		double angleTolerance = 3.0;       // degrees — convergence threshold for angles
	};
	CommandTargetState m_cmdTarget;

	// Ian: ArmGrabSequence phase tracking — the grab sequence progresses through discrete
	// phases (hover → approach → grab → retract), advancing when the current phase's
	// position targets are reached.
	int m_grabPhase = 0;

	// Ian: Helper — register a single command behavior into m_commands.
	void RegisterCommand(const std::string& name,
		std::function<void()> initialize,
		std::function<void(double)> execute,
		std::function<bool()> isFinished,
		std::function<void(bool)> end);
#endif // ENABLE_COMMAND_SCHEDULER
};

}  // namespace Robot
}  // namespace Module
