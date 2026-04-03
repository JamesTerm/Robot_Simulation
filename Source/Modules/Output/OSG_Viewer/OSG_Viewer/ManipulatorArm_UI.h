#pragma once
// Ian: ManipulatorArm_UI — side-view line rendering of the excavator arm kinematic chain.
// Ported from Curivator_Robot_UI (Curivator_Robot.cpp lines 1816-2100).  Implements the
// ManipulatorUI_Plugin interface so that TeleAutonV2 can use it polymorphically — swapping
// the manipulator plugin automatically swaps the renderer.
//
// The caller provides a callback that returns ArmState (FK results) each frame.
// UpdateScene needs rootNode (not just geode) because the arm geometry uses a
// PositionAttitudeTransform to position the side view in screen space.

#include "ManipulatorUI_Plugin.h"

#include <functional>
#include <memory>

namespace Module
{
	namespace Output {
class ManipulatorArm_UI_Internal;

class OSG_View_API ManipulatorArm_UI : public ManipulatorUI_Plugin
{
public:
	// Ian: ArmState mirrors the FK results needed for rendering.  TeleAutonV2 fills this
	// from ExcavatorArm::ForwardKinematicsResult each frame via the callback.
	// All positions are in inches (matching the Curivator convention); the renderer scales
	// by 10x to get pixel coordinates, same as the original.
	struct ArmState
	{
		// BigArm
		double BigArmLength = 0.0;      // horizontal distance from pivot (inches)
		double BigArmHeight = 0.0;      // vertical distance from pivot (inches)
		// Boom
		double BoomAngle = 0.0;         // radians, global from vertical
		double BoomLength = 0.0;        // global horizontal distance (inches)
		double BoomHeight = 0.0;        // global height (inches)
		// Bucket
		double BucketLength = 0.0;      // global distance to tip (inches)
		double BucketTipHeight = 0.0;   // global height of bucket tip (inches)
		double BucketAngle = 0.0;       // global bucket angle (radians)
		double BucketRoundEndHeight = 0.0;
		double Bucket_globalBRP_BP_height = 0.0;
		double Bucket_globalBRP_BP_distance = 0.0;
		// Clasp
		double ClaspMidlineDistance = 0.0;
		double ClaspMidlineHeight = 0.0;
		// CoM (for the bucket circle rendering)
		double CoMDistance = 0.0;
		double CoMHeight = 0.0;
		// Ian: BucketAngleContinuity — |desired_bucket_angle - FK_actual_bucket_angle| in degrees.
		// Used by LinesUpdate to blend vertex colors [7] (bucket tip) and [8] (bucket angle)
		// from normal colors to red when geometry hits limits.
		// Matches legacy Curivator_Robot::GetBucketAngleContinuity() (Curivator_Robot.cpp:995).
		double BucketAngleContinuity = 0.0;
	};

	ManipulatorArm_UI();
	~ManipulatorArm_UI() override;

	// Set the state callback before calling Initialize.
	void SetArmState_Callback(std::function<ArmState()> callback);

	// --- ManipulatorUI_Plugin interface ---
	void Initialize() override;
	void UpdateScene(void* rootNode, void* geode, bool AddOrRemove) override;
	void TimeChange(double dTime_s) override;

private:
	std::shared_ptr<ManipulatorArm_UI_Internal> m_Arm;
};

}}
