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

//instance named prefix for each wheel front, and rear... of left and right, left is port side, right starboard
//or from top view with front facing up
Rg_(FL_)  //front left
Rg_(FR_)  //front right
Rg_(RL_)  //rear left
Rg_(RR_)  //rear right

//Motion 2D properties
Rg_(Motion2D_max_speed_linear)
Rg_(Motion2D_max_speed_angular)
Rg_(Motion2D_max_acceleration_linear)
Rg_(Motion2D_max_deceleration_linear)
Rg_(Motion2D_max_acceleration_angular)

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
Rg_(Ship_1D_MaxAccelForward)
Rg_(Ship_1D_MaxAccelReverse)
Rg_(Ship_1D_MinRange)
Rg_(Ship_1D_MaxRange)
Rg_(Ship_1D_DistanceDegradeScalar)
Rg_(Ship_1D_UsingRange) //bool

//Rotary
Rg_(Rotary_VoltageScalar)
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

//Rotary_Arm_GainAssist_
Rg_(Rotary_Arm_GainAssist_PID_Up)  //double[3]; append _p _i _d to the name for each element
Rg_(Rotary_Arm_GainAssist_PID_Down) //double[3]; append _p _i _d to the name for each element
Rg_(Rotary_Arm_GainAssist_InverseMaxAccel_Up)
Rg_(Rotary_Arm_GainAssist_InverseMaxDecel_Up)
Rg_(Rotary_Arm_GainAssist_InverseMaxAccel_Down)
Rg_(Rotary_Arm_GainAssist_InverseMaxDecel_Down)
Rg_(Rotary_Arm_GainAssist_SlowVelocityVoltage)
Rg_(Rotary_Arm_GainAssist_SlowVelocity)
Rg_(Rotary_Arm_GainAssist_GainAssistAngleScalar)
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
#pragma endregion

#undef Reg

}}