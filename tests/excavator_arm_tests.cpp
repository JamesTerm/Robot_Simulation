// Ian: Unit tests for ExcavatorArm — validates both control layers against expected behavior.
// Start with Layer 1 (direct dart control via keys 1-8) since that's the simpler path and the
// user reports those keys don't work.  Then test Layer 2 (virtual 3D joints + IK).
//
// The ExcavatorArm is self-contained: Init/Reset/SetJointInput/TimeSlice are all we need.
// SmartDashboard must be initialized for TryGetBoolean/PutNumber calls in TimeSlice.

#include <gtest/gtest.h>

#include "Base/Base_Includes.h"
#include "Base/Misc.h"
#include "Base/AssetManager.h"
#include "Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "Modules/Output/OSG_Viewer/OSG_Viewer/ManipulatorUI_Plugin.h"
#include "Modules/Robot/Manipulator/Manipulator/ExcavatorArm.h"

using Module::Robot::ExcavatorArm;
using Module::Robot::ExcavatorJoint;
using Module::Robot::VirtualJoint;
using Module::Robot::kExcavatorJointCount;
using Module::Robot::kVirtualJointCount;

// ============================================================================
// Test fixture — creates and initializes an ExcavatorArm for each test
// ============================================================================

class ExcavatorArmTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Ian: SmartDashboard must be initialized before ExcavatorArm::Init because TimeSlice
		// calls TryGetBoolean and PublishTelemetry calls PutNumber.
		SmartDashboard::init();

		Framework::Base::asset_manager props;
		m_arm.Init(props);
		// After Init, Reset is already called internally — arm should be at starting positions
	}

	void TearDown() override
	{
		SmartDashboard::shutdown();
	}

	// Convenience: run N time slices at given dt
	void RunTimeSlices(int count, double dt = 0.02)
	{
		for (int i = 0; i < count; i++)
			m_arm.TimeSlice(dt);
	}

	// Get FK result shorthand
	const ExcavatorArm::ForwardKinematicsResult& FK() const { return m_arm.GetFK(); }

	ExcavatorArm m_arm;

	// Starting positions from the hardcoded defaults in ExcavatorArm.cpp
	static constexpr double kBigArmStart = 6.0;
	static constexpr double kBoomStart = 6.0;
	static constexpr double kBucketStart = 6.0;
	static constexpr double kClaspStart = 3.5;
};

// ============================================================================
// Layer 1 Tests — Direct dart control (EnableArmAutoPosition=false)
// ============================================================================

TEST_F(ExcavatorArmTest, InitialFKIsStable)
{
	// After Init+Reset, running TimeSlice with no input should produce no movement.
	// Ian: EnableArmAutoPosition defaults to true, so we must set it false for Layer 1 tests.
	// Since SmartDashboard key doesn't exist, TryGetBoolean returns false and the member
	// stays at its default (true).  We need to publish the key.
	m_arm.SetEnableArmAutoPosition(false);

	auto fk_before = FK();
	RunTimeSlices(50);
	auto fk_after = FK();

	// BigArm angle should be essentially unchanged
	EXPECT_NEAR(fk_before.BigArmAngle, fk_after.BigArmAngle, 0.001)
		<< "BigArm angle drifted with no input in Layer 1";
	EXPECT_NEAR(fk_before.BoomAngle, fk_after.BoomAngle, 0.001)
		<< "Boom angle drifted with no input in Layer 1";
}

TEST_F(ExcavatorArmTest, Layer1_PositiveInputExtendsBigArm)
{
	// In Layer 1 (EnableArmAutoPosition=false), SetJointInput(BigArm, +1.0) should
	// increase the simulated position (shaft extends).
	m_arm.SetEnableArmAutoPosition(false);

	// Run one slice with no input to establish baseline
	RunTimeSlices(1);
	double baselineBigArmAngle = FK().BigArmAngle;

	// Apply positive input to BigArm
	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), 1.0);
	RunTimeSlices(50);  // 1 second at 50Hz

	// The BigArm angle should have changed (shaft extended = arm pivots)
	double newBigArmAngle = FK().BigArmAngle;
	EXPECT_NE(baselineBigArmAngle, newBigArmAngle)
		<< "BigArm FK didn't change after positive input — number keys may not be working";

	// More specifically: extending the BigArm shaft pushes the arm upward (increases angle)
	// Let's just verify the FK changed at all for now
	double angleDelta = newBigArmAngle - baselineBigArmAngle;
	// Print the actual delta for debugging
	printf("  BigArm angle: baseline=%.4f, after=%.4f, delta=%.4f rad (%.2f deg)\n",
		baselineBigArmAngle, newBigArmAngle, angleDelta, angleDelta * 180.0 / M_PI);
}

TEST_F(ExcavatorArmTest, Layer1_NegativeInputRetractsBigArm)
{
	m_arm.SetEnableArmAutoPosition(false);

	RunTimeSlices(1);
	double baselineBigArmAngle = FK().BigArmAngle;

	// Apply negative input (retract)
	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), -1.0);
	RunTimeSlices(50);

	double newBigArmAngle = FK().BigArmAngle;
	EXPECT_NE(baselineBigArmAngle, newBigArmAngle)
		<< "BigArm FK didn't change after negative input";

	// Retracting should move opposite to extending
	double retractDelta = newBigArmAngle - baselineBigArmAngle;
	printf("  BigArm retract: baseline=%.4f, after=%.4f, delta=%.4f rad (%.2f deg)\n",
		baselineBigArmAngle, newBigArmAngle, retractDelta, retractDelta * 180.0 / M_PI);
}

TEST_F(ExcavatorArmTest, Layer1_ExtendAndRetractAreOpposite)
{
	m_arm.SetEnableArmAutoPosition(false);

	// Extend BigArm from starting position
	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), 1.0);
	RunTimeSlices(25);
	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), 0.0);
	RunTimeSlices(1);
	double extendedAngle = FK().BigArmAngle;

	// Reset and retract
	m_arm.Reset();
	m_arm.SetEnableArmAutoPosition(false);
	RunTimeSlices(1);
	double resetAngle = FK().BigArmAngle;

	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), -1.0);
	RunTimeSlices(25);
	m_arm.SetJointInput(static_cast<size_t>(ExcavatorJoint::eBigArm), 0.0);
	RunTimeSlices(1);
	double retractedAngle = FK().BigArmAngle;

	double extendDelta = extendedAngle - resetAngle;
	double retractDelta = retractedAngle - resetAngle;

	printf("  Extend delta=%.4f, Retract delta=%.4f (should be opposite signs)\n",
		extendDelta, retractDelta);

	// Extend and retract should move in opposite directions
	EXPECT_NE(0.0, extendDelta) << "Extend produced no movement";
	EXPECT_NE(0.0, retractDelta) << "Retract produced no movement";
	// Their product should be negative (opposite signs)
	EXPECT_LT(extendDelta * retractDelta, 0.0)
		<< "Extend and retract moved in the SAME direction";
}

TEST_F(ExcavatorArmTest, Layer1_AllJointsRespond)
{
	// Verify all 4 joints respond to positive input
	m_arm.SetEnableArmAutoPosition(false);

	for (size_t j = 0; j < kExcavatorJointCount; j++)
	{
		m_arm.Reset();
		m_arm.SetEnableArmAutoPosition(false);
		RunTimeSlices(1);

		auto fk_before = FK();

		m_arm.SetJointInput(j, 1.0);
		RunTimeSlices(25);
		m_arm.SetJointInput(j, 0.0);

		auto fk_after = FK();

		// At least one FK field should have changed
		bool changed = (fk_before.BigArmAngle != fk_after.BigArmAngle) ||
			(fk_before.BoomAngle != fk_after.BoomAngle) ||
			(fk_before.BucketAngle != fk_after.BucketAngle) ||
			(fk_before.ClaspAngle != fk_after.ClaspAngle) ||
			(fk_before.BucketTipHeight != fk_after.BucketTipHeight) ||
			(fk_before.BucketLength != fk_after.BucketLength);

		EXPECT_TRUE(changed) << "Joint " << j << " produced no FK change with +1.0 input";
	}
}

// ============================================================================
// Layer 2 Tests — Virtual 3D position joints (EnableArmAutoPosition=true)
// ============================================================================

TEST_F(ExcavatorArmTest, Layer2_StableWithNoInput)
{
	// Layer 2 is the default (EnableArmAutoPosition=true).  With no input the arm should be stable.
	m_arm.SetEnableArmAutoPosition(true);

	auto fk_before = FK();
	RunTimeSlices(100);  // 2 seconds
	auto fk_after = FK();

	EXPECT_NEAR(fk_before.BigArmAngle, fk_after.BigArmAngle, 0.01)
		<< "BigArm angle drifted in Layer 2 with no input";
	EXPECT_NEAR(fk_before.BoomAngle, fk_after.BoomAngle, 0.01)
		<< "Boom angle drifted in Layer 2 with no input";
	EXPECT_NEAR(fk_before.BucketTipHeight, fk_after.BucketTipHeight, 0.1)
		<< "Bucket tip height drifted in Layer 2 with no input";
}

TEST_F(ExcavatorArmTest, Layer2_ActualMatchesDesiredOnReset)
{
	// Ian: Simplest possible Layer 2 test — after reset, do NOTHING.
	// The actual 3D positions (from FK) should match the desired 3D positions (from virtual
	// joints) within tolerance.  If they don't, the starting shaft positions and starting
	// virtual joint positions are inconsistent and everything downstream is broken.
	m_arm.SetEnableArmAutoPosition(true);

	// Run one frame so FK is computed
	RunTimeSlices(1);

	double actual_xpos = m_arm.GetActual_ArmXpos();
	double actual_ypos = m_arm.GetActual_ArmYpos();
	double actual_bucket = m_arm.GetActual_BucketAngle();

	double desired_xpos = m_arm.GetDesired_ArmXpos();
	double desired_ypos = m_arm.GetDesired_ArmYpos();
	double desired_bucket = m_arm.GetDesired_BucketAngle();

	printf("  arm_xpos:      actual=%.4f  desired=%.4f  delta=%.4f\n",
		actual_xpos, desired_xpos, actual_xpos - desired_xpos);
	printf("  arm_ypos:      actual=%.4f  desired=%.4f  delta=%.4f\n",
		actual_ypos, desired_ypos, actual_ypos - desired_ypos);
	printf("  bucket_angle:  actual=%.4f  desired=%.4f  delta=%.4f\n",
		actual_bucket, desired_bucket, actual_bucket - desired_bucket);

	EXPECT_NEAR(actual_xpos, desired_xpos, 1.0)
		<< "arm_xpos actual vs desired mismatch on reset";
	EXPECT_NEAR(actual_ypos, desired_ypos, 1.0)
		<< "arm_ypos actual vs desired mismatch on reset";
	EXPECT_NEAR(actual_bucket, desired_bucket, 2.0)
		<< "bucket_angle actual vs desired mismatch on reset";
}

TEST_F(ExcavatorArmTest, Layer2_AutoOn_NoInputNoMovement)
{
	// Ian: With EnableArmAutoPosition=true and no input, actual positions should not change.
	// If they do, the PID is fighting a mismatch between actual and desired at startup.
	m_arm.SetEnableArmAutoPosition(true);
	RunTimeSlices(1);

	double start_xpos = m_arm.GetActual_ArmXpos();
	double start_ypos = m_arm.GetActual_ArmYpos();
	double start_bucket = m_arm.GetActual_BucketAngle();

	printf("  Before: xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		start_xpos, start_ypos, start_bucket);

	RunTimeSlices(100);  // 2 seconds, no input

	double end_xpos = m_arm.GetActual_ArmXpos();
	double end_ypos = m_arm.GetActual_ArmYpos();
	double end_bucket = m_arm.GetActual_BucketAngle();

	printf("  After:  xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		end_xpos, end_ypos, end_bucket);
	printf("  Deltas: xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		end_xpos - start_xpos, end_ypos - start_ypos, end_bucket - start_bucket);

	EXPECT_NEAR(start_xpos, end_xpos, 0.5)
		<< "arm_xpos moved with no input (auto=true)";
	EXPECT_NEAR(start_ypos, end_ypos, 0.5)
		<< "arm_ypos moved with no input (auto=true)";
	EXPECT_NEAR(start_bucket, end_bucket, 1.0)
		<< "bucket_angle moved with no input (auto=true)";
}

TEST_F(ExcavatorArmTest, Layer2_AutoOff_NoInputNoMovement)
{
	// Ian: Same test but with EnableArmAutoPosition=false — should also do nothing.
	m_arm.SetEnableArmAutoPosition(false);
	RunTimeSlices(1);

	double start_xpos = m_arm.GetActual_ArmXpos();
	double start_ypos = m_arm.GetActual_ArmYpos();
	double start_bucket = m_arm.GetActual_BucketAngle();

	printf("  Before: xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		start_xpos, start_ypos, start_bucket);

	RunTimeSlices(100);  // 2 seconds, no input

	double end_xpos = m_arm.GetActual_ArmXpos();
	double end_ypos = m_arm.GetActual_ArmYpos();
	double end_bucket = m_arm.GetActual_BucketAngle();

	printf("  After:  xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		end_xpos, end_ypos, end_bucket);
	printf("  Deltas: xpos=%.4f  ypos=%.4f  bucket=%.4f\n",
		end_xpos - start_xpos, end_ypos - start_ypos, end_bucket - start_bucket);

	EXPECT_NEAR(start_xpos, end_xpos, 0.5)
		<< "arm_xpos moved with no input (auto=false)";
	EXPECT_NEAR(start_ypos, end_ypos, 0.5)
		<< "arm_ypos moved with no input (auto=false)";
	EXPECT_NEAR(start_bucket, end_bucket, 1.0)
		<< "bucket_angle moved with no input (auto=false)";
}

// ============================================================================
// IK/FK Round-trip Test
// ============================================================================

TEST_F(ExcavatorArmTest, IK_FK_RoundTrip)
{
	// Given the starting virtual joint positions, IK should produce shaft lengths
	// that when fed into FK produce the same bucket tip position.
	ExcavatorArm::IKInput ikIn;
	ikIn.GlobalHeight = -0.97606122071131374;  // arm_ypos start
	ikIn.GlobalDistance = 32.801521314123598;    // arm_xpos start
	ikIn.BucketAngle_deg = 78.070524788111342;  // bucket_angle start
	ikIn.ClaspOpeningAngle_deg = 13.19097419;    // clasp_angle start

	ExcavatorArm::IKOutput ikOut = ExcavatorArm::ComputeArmPosition(ikIn);

	printf("  IK output: BigArm=%.4f, Boom=%.4f, Bucket=%.4f, Clasp=%.4f\n",
		ikOut.BigArmShaftLength, ikOut.BoomShaftLength,
		ikOut.BucketShaftLength, ikOut.ClaspShaftLength);

	// All shaft lengths should be within valid ranges
	EXPECT_GE(ikOut.BigArmShaftLength, 0.75);
	EXPECT_LE(ikOut.BigArmShaftLength, 11.0);
	EXPECT_GE(ikOut.BoomShaftLength, 0.75);
	EXPECT_LE(ikOut.BoomShaftLength, 11.0);
	EXPECT_GE(ikOut.BucketShaftLength, 0.0);
	EXPECT_LE(ikOut.BucketShaftLength, 12.0);
	EXPECT_GE(ikOut.ClaspShaftLength, 0.0);
	EXPECT_LE(ikOut.ClaspShaftLength, 7.0);
}

// ============================================================================
// Stability / drift test
// ============================================================================

TEST_F(ExcavatorArmTest, Layer2_NoDriftOver10Seconds)
{
	// With EnableArmAutoPosition=true and no input, the arm should not drift over time
	m_arm.SetEnableArmAutoPosition(true);

	RunTimeSlices(10);  // settle
	double startHeight = FK().BucketTipHeight;
	double startDistance = FK().BucketLength;

	RunTimeSlices(500);  // 10 seconds at 50Hz

	double endHeight = FK().BucketTipHeight;
	double endDistance = FK().BucketLength;

	printf("  10s drift: height %.4f -> %.4f (delta %.6f), distance %.4f -> %.4f (delta %.6f)\n",
		startHeight, endHeight, endHeight - startHeight,
		startDistance, endDistance, endDistance - startDistance);

	EXPECT_NEAR(startHeight, endHeight, 0.5)
		<< "Bucket tip height drifted more than 0.5 inches over 10 seconds";
	EXPECT_NEAR(startDistance, endDistance, 0.5)
		<< "Bucket tip distance drifted more than 0.5 inches over 10 seconds";
}

// ============================================================================
// Layer 2 Diagnostic — trace the bucket_angle chain step-by-step
// Ian: This test traces each step of the Layer 2 control chain for bucket_angle
// to find exactly where setpoint changes stop producing arm movement.
//
// The chain:
//   (a) Set3DPositionInput(2, +1.0)  →  m_3DPosInputs[2] = 1.0
//   (b) TimeSlice → SetRequestedVelocity_FromNormalized(1.0) on bucket_angle Ship_1D
//   (c) Ship_1D::TimeChange integrates position → GetPos_m() should change from 78.07
//   (d) IK(new bucket_angle) → new shaft lengths → differ from starting
//   (e) SetIntendedPosition on physical joints → PID sees error
//   (f) PID outputs voltage → voltage callback moves m_simulated_positions
//   (g) FK reads new m_simulated_positions → actual bucket angle changes
// ============================================================================

TEST_F(ExcavatorArmTest, Layer2_Diagnostic_BucketAngleChain)
{
	m_arm.SetEnableArmAutoPosition(true);

	// Hook up voltage trace to see what the PID is actually outputting
	struct VoltageTrace {
		std::array<double, kExcavatorJointCount> lastVoltage = {};
		std::array<int, kExcavatorJointCount> callCount = {};
	};
	VoltageTrace vTrace;
	m_arm.SetVoltageTraceCallback([&vTrace](size_t jointIdx, double voltage) {
		if (jointIdx < kExcavatorJointCount)
		{
			vTrace.lastVoltage[jointIdx] = voltage;
			vTrace.callCount[jointIdx]++;
			// Print first few calls for Bucket joint
			if (jointIdx == static_cast<size_t>(ExcavatorJoint::eBucket) && vTrace.callCount[jointIdx] <= 5)
			{
				printf("    Bucket voltage[%d]: %.8f\n", vTrace.callCount[jointIdx], voltage);
			}
		}
	});

	// Step 0: Run one frame with no input to establish baseline
	RunTimeSlices(1);
	printf("  Baseline: Bucket voltage callback fired %d times, last voltage=%.8f\n",
		vTrace.callCount[static_cast<size_t>(ExcavatorJoint::eBucket)],
		vTrace.lastVoltage[static_cast<size_t>(ExcavatorJoint::eBucket)]);
	// Reset counts for next phase
	for (size_t i = 0; i < kExcavatorJointCount; i++)
		vTrace.callCount[i] = 0;
	const double baseline_desired = m_arm.GetDesired_BucketAngle();
	const double baseline_actual = m_arm.GetActual_BucketAngle();
	const double baseline_bucket_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBucket));
	const double baseline_bigarm_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBigArm));
	const double baseline_boom_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBoom));

	printf("\n=== Layer 2 Diagnostic: Bucket Angle Chain ===\n");
	printf("Step 0 - Baseline:\n");
	printf("  desired_bucket_angle (virtual joint GetPos_m): %.6f deg\n", baseline_desired);
	printf("  actual_bucket_angle  (FK feedback):            %.6f deg\n", baseline_actual);
	printf("  simulated_positions: BigArm=%.6f, Boom=%.6f, Bucket=%.6f, Clasp=%.6f\n",
		m_arm.GetSimulatedPosition(0), m_arm.GetSimulatedPosition(1),
		m_arm.GetSimulatedPosition(2), m_arm.GetSimulatedPosition(3));
	printf("  starting_positions:  BigArm=%.6f, Boom=%.6f, Bucket=%.6f, Clasp=%.6f\n",
		m_arm.GetStartingPosition(0), m_arm.GetStartingPosition(1),
		m_arm.GetStartingPosition(2), m_arm.GetStartingPosition(3));

	// Verify baseline consistency: desired and actual should roughly match on reset
	ASSERT_NEAR(baseline_desired, baseline_actual, 5.0)
		<< "FAIL at Step 0: desired/actual bucket angle mismatch at baseline";

	// Step (a): Set input to +1.0 for bucket_angle
	printf("\nStep (a): Set3DPositionInput(2, +1.0)\n");
	m_arm.Set3DPositionInput(static_cast<size_t>(VirtualJoint::eBucketAngle), 1.0);

	// Step (b-g): Run ONE frame and trace everything
	// Ian: Use dt=0.01 to match the Curivator's ~100Hz loop rate.  The physics parameters
	// (MaxAccel=500 for Bucket/Clasp) were tuned for this rate.  At dt=0.02, the
	// GetVelocityFromDistance_Linear function computes zero velocity for small displacements
	// because sqrt(2*dist/accel) <= dt.
	const double test_dt = 0.01;
	printf("\nStep (b-g): Running frame 1 at dt=%.3f ...\n", test_dt);
	m_arm.TimeSlice(test_dt);

	printf("\n--- Frame 2 ---\n");
	m_arm.TimeSlice(test_dt);

	printf("\n--- Frame 3 ---\n");
	m_arm.TimeSlice(test_dt);

	const double after1_desired = m_arm.GetDesired_BucketAngle();
	const double after1_actual = m_arm.GetActual_BucketAngle();
	const double after1_bucket_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBucket));
	const double after1_bigarm_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBigArm));
	const double after1_boom_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBoom));

	printf("  (b-c) desired_bucket_angle: %.6f -> %.6f  (delta=%.6f)\n",
		baseline_desired, after1_desired, after1_desired - baseline_desired);
	printf("  (d) IK check - compute IK for new desired position:\n");

	// Manually run IK for the new desired values to see what shaft lengths we'd expect
	ExcavatorArm::IKInput ikIn;
	ikIn.GlobalDistance = m_arm.GetDesired_ArmXpos();
	ikIn.GlobalHeight = m_arm.GetDesired_ArmYpos();
	ikIn.BucketAngle_deg = after1_desired;
	ikIn.ClaspOpeningAngle_deg = 13.19;  // clasp unchanged
	ExcavatorArm::IKOutput ikOut = ExcavatorArm::ComputeArmPosition(ikIn);

	// Apply inversion for BigArm/Boom (same as TimeSlice Layer 2)
	// BigArm: range [1.0, 10.0], Boom: range [1.0, 10.0]
	double ik_bigarm_inverted = 10.0 - ikOut.BigArmShaftLength + 1.0;
	double ik_boom_inverted = 10.0 - ikOut.BoomShaftLength + 1.0;

	printf("       IK shaft lengths: BigArm=%.6f (inverted=%.6f), Boom=%.6f (inverted=%.6f)\n",
		ikOut.BigArmShaftLength, ik_bigarm_inverted, ikOut.BoomShaftLength, ik_boom_inverted);
	printf("       IK shaft lengths: Bucket=%.6f, Clasp=%.6f\n",
		ikOut.BucketShaftLength, ikOut.ClaspShaftLength);

	printf("  (e-f) simulated_positions after 1 frame:\n");
	printf("       BigArm: %.6f -> %.6f (delta=%.6f)\n",
		baseline_bigarm_shaft, after1_bigarm_shaft, after1_bigarm_shaft - baseline_bigarm_shaft);
	printf("       Boom:   %.6f -> %.6f (delta=%.6f)\n",
		baseline_boom_shaft, after1_boom_shaft, after1_boom_shaft - baseline_boom_shaft);
	printf("       Bucket: %.6f -> %.6f (delta=%.6f)\n",
		baseline_bucket_shaft, after1_bucket_shaft, after1_bucket_shaft - baseline_bucket_shaft);
	printf("  (g) actual_bucket_angle: %.6f -> %.6f (delta=%.6f)\n",
		baseline_actual, after1_actual, after1_actual - baseline_actual);

	// Now run more frames with the same input held (3 already ran above)
	printf("\n  Running 97 more frames (total = 100 frames / 1 second) ...\n");
	for (int i = 0; i < 97; i++)
		m_arm.TimeSlice(test_dt);

	const double after50_desired = m_arm.GetDesired_BucketAngle();
	const double after50_actual = m_arm.GetActual_BucketAngle();
	const double after50_bucket_shaft = m_arm.GetSimulatedPosition(
		static_cast<size_t>(ExcavatorJoint::eBucket));

	printf("  After 100 frames (1 sec):\n");
	printf("    desired_bucket_angle: %.6f (delta from baseline=%.6f)\n",
		after50_desired, after50_desired - baseline_desired);
	printf("    actual_bucket_angle:  %.6f (delta from baseline=%.6f)\n",
		after50_actual, after50_actual - baseline_actual);
	printf("    bucket_shaft: %.6f (delta from baseline=%.6f)\n",
		after50_bucket_shaft, after50_bucket_shaft - baseline_bucket_shaft);
	printf("    simulated: BigArm=%.6f, Boom=%.6f, Bucket=%.6f, Clasp=%.6f\n",
		m_arm.GetSimulatedPosition(0), m_arm.GetSimulatedPosition(1),
		m_arm.GetSimulatedPosition(2), m_arm.GetSimulatedPosition(3));

	// === ASSERTIONS: Each step must produce a nonzero delta ===

	// (c) Virtual joint position must have changed
	EXPECT_NE(baseline_desired, after50_desired)
		<< "CHAIN BREAK at step (c): Virtual joint GetPos_m() didn't change — "
		<< "SetRequestedVelocity_FromNormalized or Ship_1D::TimeChange is broken";

	// (f) At least the bucket simulated position should have changed
	EXPECT_NE(baseline_bucket_shaft, after50_bucket_shaft)
		<< "CHAIN BREAK at step (f): m_simulated_positions[Bucket] didn't change — "
		<< "PID produced zero voltage or voltage callback not firing";

	// (g) Actual bucket angle should have changed
	EXPECT_NE(baseline_actual, after50_actual)
		<< "CHAIN BREAK at step (g): FK actual bucket angle didn't change — "
		<< "FK not reading m_simulated_positions correctly";

	// Direction check: positive input should increase bucket angle
	if (after50_desired != baseline_desired)
	{
		printf("\n  Direction: desired moved %s (%.4f deg)\n",
			(after50_desired > baseline_desired) ? "POSITIVE" : "NEGATIVE",
			after50_desired - baseline_desired);
	}
	if (after50_actual != baseline_actual)
	{
		printf("  Direction: actual moved %s (%.4f deg)\n",
			(after50_actual > baseline_actual) ? "POSITIVE" : "NEGATIVE",
			after50_actual - baseline_actual);
	}
}
