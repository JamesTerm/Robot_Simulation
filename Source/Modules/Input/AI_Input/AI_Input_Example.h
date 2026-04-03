#pragma once

#include "AI_Input.h"
#include "../../../Base/AssetManager.h"

// Ian: Forward-declare ManipulatorPlugin — only used as a pointer parameter.
namespace Module { namespace Robot { class ManipulatorPlugin; } }

namespace Module
{
	namespace Input
	{
//Forward declare
class AI_Example_internal;

class AI_Example : public AI_Input
{
public:
	AI_Example();
	//our autonomous really should have access to the properties, especially to make tweaks of the autonomous settings in script
	void Initialize(const Framework::Base::asset_manager* props = nullptr);
	virtual Framework::Base::Goal& GetGoal() override;
	// Ian: PublishTestChooser — seeds the Test chooser with the full option list so the
	// dashboard widget populates immediately when the user switches to Test mode.
	// Must be called BEFORE ActivateTest so the user has time to make a selection.
	// Called from TeleAutonV2::SetGameMode(eTest) — fires as soon as the mode dropdown
	// changes, before Enable is pressed.
	void PublishTestChooser(Module::Robot::ManipulatorPlugin* manipulatorPlugin = nullptr);
	// Ian: ActivateTest — starts a Test-mode goal sequence using the Test chooser.
	// Called from TeleAutonV2 when DriverStation enters Test mode and presses Start.
	// The manipulatorPlugin pointer (may be nullptr) is used to query which
	// manipulator-specific test goals should appear in the chooser.
	void ActivateTest(Module::Robot::ManipulatorPlugin* manipulatorPlugin = nullptr);
	// Ian: PublishAutonChooserOptions — seeds the Auton chooser with the full option list.
	// The auton chooser is now dynamic: when a manipulator plugin is active, its auton goals
	// (e.g. "Grab and Return") are appended after the built-in drive options.
	// Called from TeleAutonV2::SetGameMode(eAuton) or init.
	void PublishAutonChooserOptions(Module::Robot::ManipulatorPlugin* manipulatorPlugin = nullptr);
	// Ian: ActivateAuton — starts the auton goal sequence using the Auton chooser.
	// manipulatorPlugin is needed to create plugin-contributed auton goals.
	// Called from TeleAutonV2 when DriverStation enters Auton mode and presses Start.
	void ActivateAuton(Module::Robot::ManipulatorPlugin* manipulatorPlugin = nullptr);
private:
	std::shared_ptr<AI_Example_internal> m_AI_Input;
};

	}
}