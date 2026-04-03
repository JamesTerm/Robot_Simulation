#pragma once
// Ian: ManipulatorPlugin is the hot-swappable base class for year-specific manipulators (excavator,
// intake, shooter, etc.).  Test_Swerve_Properties holds a unique_ptr<ManipulatorPlugin>; nullptr
// means "no manipulator this year."  Concrete implementations register all their own
// RotarySystem_Position joints, publish their own SmartDashboard telemetry, and will later
// publish Command/Subsystem NT keys for the official SmartDashboard to recognise.
//
// Phase 1 delivers geometry + telemetry only.  Phase 2 adds the lightweight CommandScheduler
// embedded right in the concrete plugin (not a separate class).

#include <string>
#include <functional>
#include <memory>

// Forward declaration — concrete plugins may not need all of these
namespace Framework { namespace Base { class asset_manager; class Goal; } }
// Ian: Forward-declare the UI base class so CreateUI() can return it without pulling in OSG_Viewer
// headers.  The concrete plugin includes the actual UI header in its .cpp file.
namespace Module { namespace Output { class ManipulatorUI_Plugin; } }
// Ian: Forward-declare AI_Input so CreateAutonGoal() can accept a controller pointer without
// pulling in AI_Input.h (which would create a circular dependency).
namespace Module { namespace Input { class AI_Input; } }

namespace Module {
namespace Robot {

// Ian: TestGoalDescriptor — lightweight description of a test goal that a manipulator plugin
// contributes to the Test chooser.  Decoupled from AutonChooserOption (which lives in the
// AI_Input module) to avoid circular dependencies.  The AI goal system maps these into
// AutonChooserOption entries at chooser-build time.
struct TestGoalDescriptor
{
	int index = 0;           // Plugin-local index (passed back to CreateTestGoal)
	const char* label = "";  // Human-readable label for the chooser UI
};

class ManipulatorPlugin
{
public:
	virtual ~ManipulatorPlugin() = default;

	// Called once after the asset_manager is populated (from script_loader).
	// The plugin reads its own properties from `properties` using its own key prefix.
	virtual void Init(const Framework::Base::asset_manager& properties) = 0;

	// Per-frame physics update — called from TimeSliceLoop after the swerve robot updates.
	virtual void TimeSlice(double dTime_s) = 0;

	// Publish SmartDashboard telemetry — called from UpdateVariables() every frame.
	virtual void PublishTelemetry() = 0;

	// Reset joint positions to their starting configuration.
	virtual void Reset() = 0;

	// Optional: wire up any odometry / voltage callbacks that depend on the simulation.
	// Called from SetUpHooks(true).  Default does nothing.
	virtual void SetUpHooks() {}

	// Optional: tear down hooks.  Called from SetUpHooks(false).  Default does nothing.
	virtual void TearDownHooks() {}

	// Human-readable name for diagnostics (e.g. "ExcavatorArm", "PowerIntake").
	virtual const char* GetName() const = 0;

	// Ian: Factory method — each manipulator plugin creates its own visualization object.
	// This keeps the rendering fully encapsulated: swapping the manipulator plugin automatically
	// swaps the renderer.  Returns nullptr if the plugin has no visualization.
	// The returned object implements ManipulatorUI_Plugin (defined in OSG_Viewer DLL).
	// TeleAutonV2 calls this once during SetUpHooks and owns the returned pointer.
	// Ian: CreateUI() is NOT defined inline because returning unique_ptr<ManipulatorUI_Plugin>
	// would force every translation unit that includes this header to see the complete type
	// (unique_ptr's destructor needs it).  The default body lives in ManipulatorPlugin.cpp.
	virtual std::unique_ptr<Module::Output::ManipulatorUI_Plugin> CreateUI();

	// Ian: Wire the UI's state callback to this plugin's FK data.  Called by TeleAutonV2 after
	// CreateUI() returns the UI object.  The plugin knows its own concrete UI type and FK struct,
	// so it can set up the correct state callback without TeleAutonV2 needing to know either.
	// The ui pointer is non-owning (TeleAutonV2 owns the unique_ptr).
	virtual void SetupUI(Module::Output::ManipulatorUI_Plugin* /*ui*/) {}

	// Ian: Voltage detach — when true, voltage callbacks output 0.0 regardless of PID output.
	// Toggled via SmartDashboard "Manipulator/VoltageDetached" from TeleAutonV2.
	virtual void SetVoltageDetached(bool /*detach*/) {}
	virtual bool GetVoltageDetached() const { return false; }

	// Ian: Joint input — allows keyboard/joystick to drive individual joints.
	// normalizedVelocity ∈ [-1, 1]: positive = extend/advance, negative = retract.
	// jointIndex is plugin-specific (e.g. 0=BigArm, 1=Boom, 2=Bucket, 3=Clasp for ExcavatorArm).
	virtual void SetJointInput(size_t /*jointIndex*/, double /*normalizedVelocity*/) {}
	// Returns the number of controllable joints (for input binding loops).
	virtual size_t GetJointCount() const { return 0; }

	// Ian: 3D positioning input — allows keyboard/joystick to drive the virtual 3D position
	// joints (arm_xpos, arm_ypos, bucket_angle, clasp_angle).  These only have effect when
	// EnableArmAutoPosition is true.  normalizedVelocity ∈ [-1, 1].
	// posIndex: 0=xpos, 1=ypos, 2=bucket_angle, 3=clasp_angle
	virtual void Set3DPositionInput(size_t /*posIndex*/, double /*normalizedVelocity*/) {}
	virtual size_t Get3DPositionCount() const { return 0; }

	// Ian: Test goal contribution — each manipulator plugin self-describes its test goals.
	// GetTestGoals() returns an array of TestGoalDescriptor entries that the AI goal system
	// appends to the Test chooser after the built-in drive test options.
	// CreateTestGoal() is called with the plugin-local index to instantiate the actual Goal.
	//
	// This is the CRITICAL loose-coupling point: the AI goal system never hard-codes
	// manipulator-specific enums or goal classes.  Each new manipulator (excavator, intake,
	// shooter) just overrides these two methods to plug in its own tests.
	//
	// Default: no test goals (base manipulators with no test sequences).
	virtual const TestGoalDescriptor* GetTestGoals(size_t& count) const { count = 0; return nullptr; }
	virtual Framework::Base::Goal* CreateTestGoal(int /*pluginLocalIndex*/) { return nullptr; }

	// Ian: Auton goal contribution — same loose-coupling pattern as test goals, but for the
	// Auton chooser (match autonomous mode).  Each manipulator plugin can contribute auton
	// routines that combine drive + manipulator actions.  The AI goal system appends these
	// after the built-in drive-only auton options ("Do Nothing", "Just Move Forward").
	//
	// CreateAutonGoal receives an AI_Input pointer so the plugin can create drive goals
	// (Goal_Ship_MoveToPosition, Goal_Ship_FollowPath, etc.) that integrate with arm sequences.
	// This keeps the AI module free of manipulator-specific includes.
	//
	// Default: no auton goals (base manipulators with no autonomous routines).
	virtual const TestGoalDescriptor* GetAutonGoals(size_t& count) const { count = 0; return nullptr; }
	virtual Framework::Base::Goal* CreateAutonGoal(int /*pluginLocalIndex*/, Module::Input::AI_Input* /*controller*/) { return nullptr; }
};

}  // namespace Robot
}  // namespace Module
