#pragma once

#pragma region _Description_
//This is a quick reference to *all* the property names that are used in the assets, note that the actual asset names used do not
//have to be exactly matching here as you may need to prefix the name with a named instance

//It may turn out that we deprecate legacy code in which case a new version of the registry can be written
//since this is name space protected with a version it can co-exist on partial assets that still need it
#pragma endregion

namespace properties
{
namespace registry_v1
{
//put a csz (Const String Zero terminated) prefix as a constant char * variable for asset key names
#define Rg_(x) const char * const csz_##x = #x;

	Rg_(Build_bypass_simulation)

#pragma region _prefix section_
	//instance named prefix for each wheel front, and rear... of left and right, left is port side, right starboard
	//or from top view with front facing up, s = speed, a = wheel angle
	Rg_(FL_) Rg_(sFL_) Rg_(aFL_)  //front left
	Rg_(FR_) Rg_(sFR_) Rg_(aFR_)  //front right
	Rg_(RL_) Rg_(sRL_) Rg_(aRL_)  //rear left
	Rg_(RR_) Rg_(sRR_) Rg_(aRR_)  //rear right
	//One setting that applies to each
	Rg_(CommonDrive_)
	Rg_(CommonSwivel_)
#pragma endregion

#pragma region _Kinematics and motion control_
	//Entity properties
	Rg_(Drive_WheelBase_in)
	Rg_(Drive_TrackWidth_in)
	Rg_(Drive_WheelDiameter_in)
	Rg_(Drive_UseFieldCentric)

	//Motion 2D properties
	Rg_(Motion2D_max_speed_linear)
	Rg_(Motion2D_max_speed_angular)
	Rg_(Motion2D_max_acceleration_linear)
	Rg_(Motion2D_max_deceleration_linear)
	Rg_(Motion2D_max_acceleration_angular)
#pragma endregion

#pragma region _Ship_2D legacy_
	Rg_(Ship2D_dHeading)
	Rg_(Ship2D_EngineRampForward) Rg_(Ship2D_EngineRampReverse) Rg_(Ship2D_EngineRampAfterBurner)
	Rg_(Ship2D_EngineDeceleration) Rg_(Ship2D_EngineRampStrafe)
	Rg_(Ship2D_MAX_SPEED) Rg_(Ship2D_ENGAGED_MAX_SPEED)
	Rg_(Ship2D_ACCEL) Rg_(Ship2D_BRAKE) Rg_(Ship2D_STRAFE) Rg_(Ship2D_AFTERBURNER_ACCEL) Rg_(Ship2D_AFTERBURNER_BRAKE)

	Rg_(Ship2D_MaxAccelLeft) Rg_(Ship2D_MaxAccelRight) Rg_(Ship2D_MaxAccelForward) Rg_(Ship2D_MaxAccelReverse)
	Rg_(Ship2D_MaxAccelForward_High) Rg_(Ship2D_MaxAccelReverse_High)
	Rg_(Ship2D_MaxTorqueYaw) Rg_(Ship2D_MaxTorqueYaw_High)
	Rg_(Ship2D_MaxTorqueYaw_SetPoint) Rg_(Ship2D_MaxTorqueYaw_SetPoint_High)
	Rg_(Ship2D_Rotation_Tolerance)
	Rg_(Ship2D_Rotation_ToleranceConsecutiveCount)
	Rg_(Ship2D_Rotation_TargetDistanceScaler)
#pragma endregion

#pragma region _rotary_properties legacy_
	//Entity1D
	Rg_(Entity1D_StartingPosition)
	Rg_(Entity1D_Mass)
	Rg_(Entity1D_Dimension)
	Rg_(Entity1D_IsAngular) //bool

	//Ship1D
	Rg_(Ship_1D_MAX_SPEED)
	Rg_(Ship_1D_MaxSpeed_Forward)
	Rg_(Ship_1D_MaxSpeed_Reverse)
	Rg_(Ship_1D_ACCEL)
	Rg_(Ship_1D_BRAKE)
	Rg_(Ship_1D_MaxAccel_simulation)
	Rg_(Ship_1D_MaxAccelForward)
	Rg_(Ship_1D_MaxAccelReverse)
	Rg_(Ship_1D_MinRange)
	Rg_(Ship_1D_MaxRange)
	Rg_(Ship_1D_DistanceDegradeScaler)
	Rg_(Ship_1D_UsingRange) //bool

	//Rotary
	Rg_(Rotary_VoltageScaler)
	Rg_(Rotary_EncoderToRS_Ratio)
	Rg_(Rotary_EncoderPulsesPerRevolution)
	Rg_(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
	Rg_(Rotary_PrecisionTolerance)
	//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
	Rg_(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
	Rg_(Rotary_InverseMaxAccel)
	Rg_(Rotary_InverseMaxDecel)
	Rg_(Rotary_Positive_DeadZone)
	Rg_(Rotary_Negative_DeadZone)
	Rg_(Rotary_MinLimitRange)
	Rg_(Rotary_MaxLimitRange)
	Rg_(Rotary_Feedback_DiplayRow)
	Rg_(Rotary_LoopState) //enum LoopStates... put as double, get as int
	Rg_(Rotary_PID_Console_Dump) //bool
	Rg_(Rotary_UseAggressiveStop) //bool
	Rg_(Rotary_EncoderReversed_Wheel) //bool
	Rg_(Rotary_AverageReadingsCount) //size_t

	//Rotary_Arm_GainAssist_
	Rg_(Rotary_Arm_GainAssist_PID_Up)  //double[3]; append _p _i _d to the name for each element
	Rg_(Rotary_Arm_GainAssist_PID_Down) //double[3]; append _p _i _d to the name for each element
	Rg_(Rotary_Arm_GainAssist_InverseMaxAccel_Up)
	Rg_(Rotary_Arm_GainAssist_InverseMaxDecel_Up)
	Rg_(Rotary_Arm_GainAssist_InverseMaxAccel_Down)
	Rg_(Rotary_Arm_GainAssist_InverseMaxDecel_Down)
	Rg_(Rotary_Arm_GainAssist_SlowVelocityVoltage)
	Rg_(Rotary_Arm_GainAssist_SlowVelocity)
	Rg_(Rotary_Arm_GainAssist_GainAssistAngleScaler)
	Rg_(Rotary_Arm_GainAssist_ToleranceConsecutiveCount)
	Rg_(Rotary_Arm_GainAssist_VelocityPredictUp)
	Rg_(Rotary_Arm_GainAssist_VelocityPredictDown)
	Rg_(Rotary_Arm_GainAssist_PulseBurstTimeMs)
	Rg_(Rotary_Arm_GainAssist_PulseBurstRange)
	Rg_(Rotary_Arm_GainAssist_UsePID_Up_Only) //bool

	//Voltage_Stall_Safety_
	Rg_(Rotary_Voltage_Stall_Safety_ErrorThreshold)
	Rg_(Rotary_Voltage_Stall_Safety_OnBurstLevel)
	Rg_(Rotary_Voltage_Stall_Safety_PulseBurstTimeOut) //size_t
	Rg_(Rotary_Voltage_Stall_Safety_StallCounterThreshold) //size_t

	//Rotary_Pot_
	Rg_(Rotary_Pot_min_limit)
	Rg_(Rotary_Pot_max_limit)
	Rg_(Rotary_Pot_limit_tolerance)
	Rg_(Rotary_Pot_offset)
	Rg_(Rotary_Pot_range_flipped)
	//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
	Rg_(Rotary_Pot_curve_pot) //PotPolyTerms
#pragma endregion

#pragma region _simulation odometry legacy_

	Rg_(EncoderSimulation_UseEncoder2) //currently only 2 or 3 is supported, can change if we have more 3 is more accurate, 2 is simpler
	Rg_(EncoderSimulation_EncoderScaler)
//struct EncoderSimulation_Props
//{
//all doubles, and only one instance... easy peasy
	Rg_(EncoderSimulation_Wheel_Mass)  //This is a total mass of all the wheels and gears for one side
	Rg_(EncoderSimulation_COF_Efficiency)
	Rg_(EncoderSimulation_GearReduction)  //In reciprocal form of spread sheet   driving gear / driven gear
	Rg_(EncoderSimulation_TorqueAccelerationDampener) //ratio 1.0 no change
	Rg_(EncoderSimulation_DriveWheelRadius) //in meters
	Rg_(EncoderSimulation_NoMotors)  //Used to get total torque
	Rg_(EncoderSimulation_PayloadMass)  //The robot weight in kg
	Rg_(EncoderSimulation_SpeedLossConstant)
	Rg_(EncoderSimulation_DriveTrainEfficiency)
//	struct Motor_Specs
	//{
	Rg_(EncoderSimulation_FreeSpeed_RPM)
	Rg_(EncoderSimulation_Stall_Torque_NM)
	Rg_(EncoderSimulation_Stall_Current_Amp)
	Rg_(EncoderSimulation_Free_Current_Amp)
	//} motor;
//};
	//Use the CommonDrive_ prefix for SwerveEncoders_Simulator4 encoders
	//and CommonSwivel_ prefix for Potentiometer_Tester4 to override these defaults
	Rg_(Pot4_free_speed_rad) //radians per second of motor
	Rg_(Pot4_stall_torque_NM) //Nm
	Rg_(Pot4_gear_reduction)
	Rg_(Pot4_gear_box_effeciency) //This will account for the friction
	Rg_(Pot4_mass)
	//Use SolidWorks and get the cube root of the volume which gives a rough diameter error on the side of larger
	//divide the diameter for the radius
	Rg_(Pot4_RadiusOfConcentratedMass)
	//The dead zone defines the opposing force to be added to the mass we'll clip it down to match the velocity
	Rg_(Pot4_dead_zone) //this is the amount of voltage to get motion can be tested
	Rg_(Pot4_anti_backlash_scaler) //Any momentum beyond the steady state will be consumed 1.0 max is full consumption
#pragma endregion

#pragma region _Controls Section_
	Rg_(AxisStrafe_)
	Rg_(AxisForward_)
	Rg_(AxisTurn_)

	Rg_(Control_Key)
	Rg_(Control_Joy)
	Rg_(Control_FilterRange)
	Rg_(Control_Multiplier)
	Rg_(Control_CurveIntensity)
	Rg_(Control_IsFlipped)  //bool
#pragma endregion

#undef Reg

}}