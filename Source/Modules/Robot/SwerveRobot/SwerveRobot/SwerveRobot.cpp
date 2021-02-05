#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"

#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../MotionControl2D_simple/MotionControl2D/MotionControl2D.h"
#include "../../MotionControl2D_physics/MotionControl2D_physics/MotionControl2D.h"
#include "SwerveManagement.h"
#include "RotarySystem.h"
#include "SimulatedOdometry.h"
#include "OdometryManager.h"
#include "SwerveRobot.h"
#include "../../../../Properties/RegistryV1.h"

//#define __UseSimpleMotionControl__
#pragma endregion

namespace Module {
	namespace Robot {

void Default_Velocity_PID_Monitor(double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler)
{
	printf("v=%.2f p=%.2f e=%.2f eo=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, ErrorOffset, CalibratedScaler);
}
void Default_Position_PID_Monitor(double Voltage, double Position, double PredictedPosition, double CurrentVelocity, double Encoder_Velocity, double ErrorOffset)
{
	printf("v=%.2f y=%.2f py=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, Position, PredictedPosition, CurrentVelocity, Encoder_Velocity, ErrorOffset);
}

class SwerveRobot_Internal
{
private:
	#pragma region _member variables_
	//going down the list of each object to the next
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	#ifdef __UseSimpleMotionControl__
	Simple::MotionControl2D m_MotionControl2D;
	#else
	Physics::MotionControl2D m_MotionControl2D;
	#endif
	Swerve_Drive m_robot;  //keep track of our intended velocities... kinematics
	SwerveManagement m_swerve_mgmt;
	RotarySystem_Position m_Swivel[4];
	RotarySystem_Velocity m_Drive[4];
	SimulatedOdometry m_Simulation;
	OdometryManager m_Odometry;
	Inv_Swerve_Drive m_Entity_Input;

	SwerveVelocities m_Voltage;
	//For entity info if not external
	Vec2D m_current_position;
	Vec2D m_current_velocity;
	double m_current_angular_velocity=0.0;
	double m_current_heading = 0.0;

	std::function<void(const Vec2D &new_velocity)> m_ExternSetVelocity = nullptr;
	std::function<void(double new_velocity)> m_ExternSetHeadingVelocity = nullptr;
	std::function <Vec2D()> m_ExternGetCurrentPosition = nullptr;
	std::function <double()> m_ExternGetCurrentHeading = nullptr;
	std::function<SwerveRobot::PID_Velocity_proto> m_PID_Velocity_callback = Default_Velocity_PID_Monitor;
	std::function<SwerveRobot::PID_Position_proto> m_PID_Position_callback = Default_Position_PID_Monitor;
	std::function<Robot::SwerveVelocities ()> m_PhysicalOdometry=nullptr;
#pragma endregion
	void SetHooks(bool enable)
	{
		#ifdef __UseSimpleMotionControl__
		using namespace Simple;
		#else
		using namespace Physics;
		#endif

		//Oh what fun 22 hooks!
		if (enable)
		{
			#define HOOK(x,y,z) \
					x(\
					[&](y)\
					{\
						z;\
					});

			//Link the swerve management to the odometry
			HOOK(m_swerve_mgmt.SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities());
			HOOK(m_MotionControl2D.Set_GetCurrentHeading, , return GetCurrentHeading());
			//used for way points and motion control
			m_MotionControl2D.Set_GetCurrentPosition(
				[&]()
			{
				Vec2D postion = GetCurrentPosition();
				MotionControl2D::Vector2D ret = { postion[0],postion[1] };
				return ret;
			});
			HOOK(m_Odometry.SetOdometryHeadingCallback,, return GetCurrentHeading());
			//I could use the macro, but I can see this one needing a break point from time-to-time
			m_Odometry.SetOdometryVelocityCallback(
				[&]()
				{
					return m_PhysicalOdometry?m_PhysicalOdometry():m_Simulation.GetCurrentVelocities();
				});
			HOOK(m_Simulation.SetVoltageCallback,, return m_Voltage);
			#pragma region _Drive hooks_
			//These have to be unrolled unfortunately
			HOOK(m_Drive[0].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[0] = new_voltage);
			HOOK(m_Drive[1].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[1] = new_voltage);
			HOOK(m_Drive[2].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[2] = new_voltage);
			HOOK(m_Drive[3].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[3] = new_voltage);

			HOOK(m_Drive[0].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[0]);
			HOOK(m_Drive[1].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[1]);
			HOOK(m_Drive[2].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[2]);
			HOOK(m_Drive[3].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[3]);

			#define PID_parms double V, double  CV, double  EV, double  EO, double  CS
			HOOK(m_Drive[0].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Velocity_callback(V, CV, EV, EO, CS));
			HOOK(m_Drive[1].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Velocity_callback(V, CV, EV, EO, CS));
			HOOK(m_Drive[2].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Velocity_callback(V, CV, EV, EO, CS));
			HOOK(m_Drive[3].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Velocity_callback(V, CV, EV, EO, CS));
			#undef PID_parms
			#pragma endregion
			#pragma region _Swivel hooks_
			HOOK(m_Swivel[0].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[4 + 0] = new_voltage);
			HOOK(m_Swivel[1].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[4 + 1] = new_voltage);
			HOOK(m_Swivel[2].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[4 + 2] = new_voltage);
			HOOK(m_Swivel[3].Set_UpdateCurrentVoltage, double new_voltage, m_Voltage.Velocity.AsArray[4 + 3] = new_voltage);

			HOOK(m_Swivel[0].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 0]);
			HOOK(m_Swivel[1].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 1]);
			HOOK(m_Swivel[2].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 2]);
			HOOK(m_Swivel[3].SetOdometryVelocityCallback, , return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 3]);

			#define PID_parms double V, double Pos, double PP, double  CV, double  EV, double  EO
			HOOK(m_Swivel[0].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Position_callback(V, Pos, PP, CV, EV, EO));
			HOOK(m_Swivel[1].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Position_callback(V, Pos, PP, CV, EV, EO));
			HOOK(m_Swivel[2].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Position_callback(V, Pos, PP, CV, EV, EO));
			HOOK(m_Swivel[3].SetExternal_PID_Monitor_Callback, PID_parms, m_PID_Position_callback(V, Pos, PP, CV, EV, EO));

			#pragma endregion
			//done with these macros
			#undef HOOK
			#undef PID_parms
		}
		else
		{
			m_swerve_mgmt.SetOdometryVelocityCallback(nullptr);
			m_MotionControl2D.Set_GetCurrentHeading(nullptr);
			m_MotionControl2D.Set_GetCurrentPosition(nullptr);
			m_Odometry.SetOdometryHeadingCallback(nullptr);
			m_Odometry.SetOdometryVelocityCallback(nullptr);
			m_Odometry.SetOdometryPositionCallback(nullptr);
			m_Simulation.SetVoltageCallback(nullptr);
			for (size_t i = 0; i < 4; i++)
			{
				m_Drive[i].Set_UpdateCurrentVoltage(nullptr);
				m_Drive[i].SetOdometryVelocityCallback(nullptr);
				m_Drive[i].SetExternal_PID_Monitor_Callback(Default_Velocity_PID_Monitor);
				m_Swivel[i].Set_UpdateCurrentVoltage(nullptr);
				m_Swivel[i].SetOdometryVelocityCallback(nullptr);
				m_Swivel[i].SetExternal_PID_Monitor_Callback(Default_Position_PID_Monitor);
			}
		}
	}
	inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}
	void init_rotary_properties(const Framework::Base::asset_manager* asset_properties,
		rotary_properties& update, const char* prefix)
	{
		//Update the properties if we have asset properties, the updates will only update with explicit entries in the database
		//by use of default value parameter
		using namespace ::properties::registry_v1;
		if (asset_properties)
		{
			double ftest = 0.0;  //use to test if an asset exists
			std::string constructed_name;
			#define GET_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = asset_properties->get_number(constructed_name.c_str(), y);

			#define GET_INT(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = asset_properties->get_number_int(constructed_name.c_str(), y);

			#define GET_SIZE_T(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = asset_properties->get_number_size_t(constructed_name.c_str(), y);

			#define GET_NUMBER_suffix(x,y,z) \
			constructed_name = prefix, constructed_name += csz_##x; \
			constructed_name += #z;\
			y = asset_properties->get_number(constructed_name.c_str(), y);

			#define GET_BOOL(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = asset_properties->get_bool(constructed_name.c_str(), y);

			#pragma region _Entity1D_
			//entity 1D
			rotary_properties::Entity1D_Props& e1d = update.entity_props;
			//constructed_name = prefix, constructed_name += csz_Entity1D_StartingPosition;
			//e1d.m_StartingPosition = asset_properties->get_number(constructed_name.c_str(), e1d.m_StartingPosition);
			GET_NUMBER(Entity1D_StartingPosition, e1d.m_StartingPosition);
			GET_NUMBER(Entity1D_Mass, e1d.m_Mass);
			GET_NUMBER(Entity1D_Dimension, e1d.m_Dimension);
			GET_BOOL(Entity1D_IsAngular,e1d.m_IsAngular);
			#pragma endregion
			#pragma region _Ship1D_
			rotary_properties::Ship_1D_Props& _Ship_1D = update.ship_props;
			GET_NUMBER(Ship_1D_MAX_SPEED, _Ship_1D.MAX_SPEED);
			//IF we have this asset assign it to forward and reverse
			if (asset_properties->get_number_native(constructed_name.c_str(), ftest))
			{
				_Ship_1D.MaxSpeed_Forward = _Ship_1D.MAX_SPEED;
				_Ship_1D.MaxSpeed_Reverse = -_Ship_1D.MAX_SPEED;
			}
			GET_NUMBER(Ship_1D_MaxSpeed_Forward, _Ship_1D.MaxSpeed_Forward);
			GET_NUMBER(Ship_1D_MaxSpeed_Reverse, _Ship_1D.MaxSpeed_Reverse);
			GET_NUMBER(Ship_1D_ACCEL, _Ship_1D.ACCEL);
			GET_NUMBER(Ship_1D_BRAKE, _Ship_1D.BRAKE);
			GET_NUMBER(Ship_1D_MaxAccelForward, _Ship_1D.MaxAccelForward);
			//IF we have this asset assign it to reverse; Note: reverse is not negative
			if (asset_properties->get_number_native(constructed_name.c_str(), ftest))
				_Ship_1D.MaxAccelReverse = _Ship_1D.MaxAccelForward;

			GET_NUMBER(Ship_1D_MaxAccelReverse, _Ship_1D.MaxAccelReverse);
			GET_NUMBER(Ship_1D_MinRange, _Ship_1D.MinRange);
			GET_NUMBER(Ship_1D_MaxRange, _Ship_1D.MaxRange);
			GET_NUMBER(Ship_1D_DistanceDegradeScaler, _Ship_1D.DistanceDegradeScaler);
			GET_BOOL(Ship_1D_UsingRange, _Ship_1D.UsingRange) //bool
			#pragma endregion

			#pragma region _Rotary_
			rotary_properties::Rotary_Props& Rotary_ = update.rotary_props;
			rotary_properties::Rotary_Props::Rotary_Arm_GainAssist_Props& arm = update.rotary_props.ArmGainAssist;
			GET_NUMBER(Rotary_VoltageScaler, Rotary_.VoltageScaler);
			GET_NUMBER(Rotary_EncoderToRS_Ratio, Rotary_.EncoderToRS_Ratio);
			GET_NUMBER(Rotary_EncoderPulsesPerRevolution, Rotary_.EncoderPulsesPerRevolution);
			//GET_NUMBER(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
			GET_NUMBER_suffix(Rotary_PID, Rotary_.PID[0], _p);
			GET_NUMBER_suffix(Rotary_PID, Rotary_.PID[1], _i);
			GET_NUMBER_suffix(Rotary_PID, Rotary_.PID[2], _d);
			//Copy PID to arm assist
			for (size_t i = 0; i < 3; i++)
				arm.PID_Up[i] = arm.PID_Down[i] = Rotary_.PID[i];

			GET_NUMBER(Rotary_PrecisionTolerance, Rotary_.PrecisionTolerance);
			//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
			//GET_NUMBER(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
			GET_NUMBER_suffix(Rotary_Voltage_Terms, Rotary_.Voltage_Terms.Term[0], _c);
			GET_NUMBER_suffix(Rotary_Voltage_Terms, Rotary_.Voltage_Terms.Term[1], _t1);
			GET_NUMBER_suffix(Rotary_Voltage_Terms, Rotary_.Voltage_Terms.Term[2], _t2);
			GET_NUMBER_suffix(Rotary_Voltage_Terms, Rotary_.Voltage_Terms.Term[3], _t3);
			GET_NUMBER_suffix(Rotary_Voltage_Terms, Rotary_.Voltage_Terms.Term[4], _t4);
			//Some properties will spawn other defaults (so the order matters on those)
			GET_NUMBER(Rotary_InverseMaxAccel, Rotary_.InverseMaxAccel);
			if (asset_properties->get_number_native(constructed_name.c_str(), ftest))
			{
				Rotary_.InverseMaxDecel = Rotary_.InverseMaxAccel;	//set up deceleration to be the same value by default
				arm.InverseMaxAccel_Up = arm.InverseMaxAccel_Down = arm.InverseMaxDecel_Up = arm.InverseMaxDecel_Down = Rotary_.InverseMaxAccel;
			}

			GET_NUMBER(Rotary_InverseMaxDecel, Rotary_.InverseMaxDecel);
			if (asset_properties->get_number_native(constructed_name.c_str(), ftest))
				arm.InverseMaxDecel_Up = arm.InverseMaxDecel_Down = Rotary_.InverseMaxDecel;

			GET_NUMBER(Rotary_Positive_DeadZone, Rotary_.Positive_DeadZone);
			GET_NUMBER(Rotary_Negative_DeadZone, Rotary_.Negative_DeadZone);
			//Ensure the negative settings are negative
			if (Rotary_.Negative_DeadZone > 0.0)
				Rotary_.Negative_DeadZone = -Rotary_.Negative_DeadZone;

			GET_NUMBER(Rotary_MinLimitRange, Rotary_.MinLimitRange);
			GET_NUMBER(Rotary_MaxLimitRange, Rotary_.MaxLimitRange);
			GET_SIZE_T(Rotary_Feedback_DiplayRow, Rotary_.Feedback_DiplayRow);
			//Enum types have to be casted back explicitly... oh well
			//GET_INT(Rotary_LoopState, Rotary_.LoopState);
			constructed_name = prefix, constructed_name += csz_Rotary_LoopState;
			Rotary_.LoopState = (rotary_properties::Rotary_Props::LoopStates)
				asset_properties->get_number_int(constructed_name.c_str(), Rotary_.LoopState);
			GET_BOOL(Rotary_PID_Console_Dump, Rotary_.PID_Console_Dump); //bool
			GET_BOOL(Rotary_UseAggressiveStop, Rotary_.UseAggressiveStop); //bool
			GET_BOOL(Rotary_EncoderReversed_Wheel, Rotary_.EncoderReversed_Wheel); //bool
			GET_SIZE_T(Rotary_AverageReadingsCount, Rotary_.AverageReadingsCount); //size_t
			#pragma endregion
			#pragma region _Rotary arm gain / pot _
			//GET_NUMBER(Rotary_Arm_GainAssist_PID_Up, arm.)  //double[3]; append _p _i _d to the name for each element
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, arm.PID_Up[0], _p);
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, arm.PID_Up[1], _i);
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, arm.PID_Up[2], _d);
			//GET_NUMBER(Rotary_Arm_GainAssist_PID_Down) //double[3]; append _p _i _d to the name for each element
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Down, arm.PID_Down[0], _p);
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Down, arm.PID_Down[1], _i);
			GET_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Down, arm.PID_Down[2], _d);

			GET_NUMBER(Rotary_Arm_GainAssist_InverseMaxAccel_Up, arm.InverseMaxAccel_Up);
			GET_NUMBER(Rotary_Arm_GainAssist_InverseMaxDecel_Up, arm.InverseMaxDecel_Up);
			GET_NUMBER(Rotary_Arm_GainAssist_InverseMaxAccel_Down, arm.InverseMaxAccel_Down);
			GET_NUMBER(Rotary_Arm_GainAssist_InverseMaxDecel_Down, arm.InverseMaxDecel_Down);
			GET_NUMBER(Rotary_Arm_GainAssist_SlowVelocityVoltage, arm.SlowVelocityVoltage);
			GET_NUMBER(Rotary_Arm_GainAssist_SlowVelocity, arm.SlowVelocity);
			GET_NUMBER(Rotary_Arm_GainAssist_GainAssistAngleScaler, arm.GainAssistAngleScaler);
			GET_NUMBER(Rotary_Arm_GainAssist_ToleranceConsecutiveCount, arm.ToleranceConsecutiveCount);
			GET_NUMBER(Rotary_Arm_GainAssist_VelocityPredictUp, arm.VelocityPredictUp);
			GET_NUMBER(Rotary_Arm_GainAssist_VelocityPredictDown, arm.VelocityPredictDown);
			GET_NUMBER(Rotary_Arm_GainAssist_PulseBurstTimeMs, arm.PulseBurstTimeMs);
			GET_NUMBER(Rotary_Arm_GainAssist_PulseBurstRange, arm.PulseBurstRange)
			GET_BOOL(Rotary_Arm_GainAssist_UsePID_Up_Only,arm.UsePID_Up_Only); //bool
			//TODO pot properties need to be included (once we go beyond the simulation)
			//GET_NUMBER(Rotary_Pot_offset, val.pot_offset);
			#pragma endregion
			//finished with the macros
			#undef GET_NUMBER
			#undef GET_BOOL
			//TODO finish list
		}
	}
	void init_rotary_properties(const Framework::Base::asset_manager* asset_properties,
		rotary_properties& update, bool IsSwivel)
	{
		using namespace properties::registry_v1;
		const char* prefix = IsSwivel ? csz_CommonSwivel_ : csz_CommonDrive_;
		init_rotary_properties(asset_properties, update, prefix);
	}
	void init_rotary_properties(const Framework::Base::asset_manager* asset_properties,
		rotary_properties &update,size_t index,bool IsSwivel)
	{
		using namespace properties::registry_v1;
		//form our prefix, it will use the naming convention in Vehicle Drive.h
		const char* const prefix_table[2][4] =
		{
			{csz_sFL_,csz_sFR_,csz_sRL_,csz_sRR_},
			{csz_aFL_,csz_aFR_,csz_aRL_,csz_aRR_}
		};
		assert(index < 4);
		const char* const prefix = prefix_table[IsSwivel ? 1 : 0][index];
		init_rotary_properties(asset_properties, update, prefix);
	}
public:
	SwerveRobot_Internal()
	{
		SetHooks(true);
	}
	void Shutdown()
	{
		SetHooks(false);
	}
	~SwerveRobot_Internal()
	{
		Shutdown();
	}
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_MotionControl2D.Reset(X, Y, heading);
		m_robot.ResetPos();
		m_Simulation.ResetPos();
		m_Entity_Input.ResetPos();
		m_Odometry.Reset();
		for (size_t i = 0; i < 4; i++)
		{
			m_Drive[i].Reset();
			m_Swivel[i].Reset();
		}
		m_Voltage = {};
		m_current_position = m_current_velocity = Vec2D(0.0,0.0);
		m_current_heading = 0.0;
	}
	void Init(const Framework::Base::asset_manager *asset_properties)
	{
		if ((asset_properties) && (asset_properties->get_bool(properties::registry_v1::csz_Build_hook_simulation,false)))
		{
			if (m_Simulation.Sim_SupportHeading())
			{
				m_Odometry.Set_SupportHeading(true);
				m_Odometry.SetOdometryHeadingCallback(
					[&]()
				{
					return m_Simulation.GyroMag_GetCurrentHeading();
				});
			}
			if (m_Simulation.Sim_SupportVision())
			{
				m_Odometry.SetOdometryPositionCallback(
					[&]()
				{
					return m_Simulation.Vision_GetCurrentPosition();
				});
			}
		}
		//Go ahead and grab all the default properties first, then if we have asset properties fill in the ones we have and pass
		//the asset properties down to children

		#pragma region _drive dimensions properties for Kinematics_
		//setup some robot properties
		m_robot.Init(asset_properties);
		m_Entity_Input.Init(asset_properties);

		#pragma endregion

		m_MotionControl2D.Initialize(asset_properties);

		#pragma region _default rotary properties_
		//Provide rotary properties
		rotary_properties props_rotary_drive, props_rotary_swivel;
		//grab defaults
		props_rotary_drive.Init();
		props_rotary_swivel.Init();

		//Default rotary properties... use what we can from other systems to match these
		//Since the drive rotary system is using linear velocity, we can pull the same characteristics
		{
			//Entity props
			rotary_properties::Entity1D_Props &rw_drv_entity = props_rotary_drive.entity_props;
			rotary_properties::Entity1D_Props &rw_swl_entity = props_rotary_swivel.entity_props;
			rw_drv_entity.m_Mass = rw_swl_entity.m_Mass = 3.0 * 0.453592; //pounds to kilograms
			rw_drv_entity.m_Dimension = rw_swl_entity.m_Dimension = Inches2Meters(6); //wheel diameter... used for RPS to linear conversions
			//Note: for legacy rotary system position control, GetVelocityFromDistance_Angular() is called if m_IsAngular is true
			rw_swl_entity.m_IsAngular = true; //must have for rotary system to properly implement swerve management direction
		}
		{
			rotary_properties::Ship_1D_Props &rw_drv_ship = props_rotary_drive.ship_props;
			rotary_properties::Ship_1D_Props &rw_swl_ship = props_rotary_swivel.ship_props;
			//make use of modern c++ way to populate them...
			rw_swl_ship =
			{
				//double MAX_SPEED;
				//2,  //These match Curivator
				//double MaxSpeed_Forward, MaxSpeed_Reverse;
				//2,-2,
				8,8,-8,  //used in bypass
				//double ACCEL, BRAKE;
				10.0,10.0,
				//double MaxAccelForward, MaxAccelReverse;
				//7.0,7.0, //These match Curivator
				38.0,38.0, //used in bypass
				//double MinRange, MaxRange;
				0.0,0.0,
				//This is used to avoid overshoot when trying to rotate to a heading
				//double DistanceDegradeScaler;
				1.0,
				//bool UsingRange;
				false
			};
			rw_drv_ship =
			{
				//double MAX_SPEED;
				Feet2Meters(12),  //These match motion control
				//double MaxSpeed_Forward, MaxSpeed_Reverse;
				Feet2Meters(12),-Feet2Meters(12),
				//double ACCEL, BRAKE;
				10.0,10.0,
				//double MaxAccelForward, MaxAccelReverse;
				5.0,5.0, //These match motion control
				//double MinRange, MaxRange;
				0.0,0.0,
				//This may be needed for simulation copy
				//double DistanceDegradeScaler;
				1.0,
				//bool UsingRange;
				false
			};
		}
		{
			rotary_properties::Rotary_Props &rw_drv_rotary = props_rotary_drive.rotary_props;
			rotary_properties::Rotary_Props &rw_swl_rotary = props_rotary_swivel.rotary_props;
			rw_drv_rotary.UseAggressiveStop = rw_swl_rotary.UseAggressiveStop = true;
			//Go ahead an close the loop with a default 100 0 25
			//Even for best case simulation this is needed when using the reset button
			//(While wheel angles could be reset as well, it is better they remain compatible to physical case)
			rw_swl_rotary.LoopState = rotary_properties::Rotary_Props::eClosed;
			rw_swl_rotary.PID[0] = 100.0;
			rw_swl_rotary.PID[2] = 25.0;
		}

		#pragma endregion
		//all cases come here to init rotary systems with correct properties
		//first test the common to all properties		
		init_rotary_properties(asset_properties, props_rotary_drive, false);
		init_rotary_properties(asset_properties, props_rotary_swivel, true);
		
		for (size_t i = 0; i < 4; i++)
		{
			//since we are writing individual properties we'll need a local variable here:
			rotary_properties module_rotary = props_rotary_drive;
			init_rotary_properties(asset_properties, module_rotary, i, false);
			m_Drive[i].Init(i,&module_rotary);
			module_rotary = props_rotary_swivel;
			init_rotary_properties(asset_properties, module_rotary, i, true);
			m_Swivel[i].Init(i+4,&module_rotary);
		}
		m_Simulation.Init(asset_properties);
		Reset();
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseSimpleMotionControl__
		using namespace Simple;
		#else
		using namespace Physics;
		#endif
		using namespace Framework::Base;
		m_MotionControl2D.TimeSlice(d_time_s);
		MotionControl2D::Vector2D intended_global_velocity = m_MotionControl2D.GetCurrentVelocity();
		Vec2D intended_local_velocity = GlobalToLocal(GetCurrentHeading(),
			Vec2D(intended_global_velocity.x,intended_global_velocity.y));
		//break down the motion control to its kinematic components
		m_robot.UpdateVelocities(intended_local_velocity.y(), intended_local_velocity.x(), m_MotionControl2D.GetCurrentAngularVelocity());
		//pass this to the final swerve management
		m_swerve_mgmt.InterpolateVelocities(m_robot.GetIntendedVelocities());

		//Update the odometry, note: this must proceed all rotary system loops because of latency, will have the latest positions before
		//the next iteration
		//m_Simulation.TimeSlice(d_time_s); //was here now moved into its own call
		m_Odometry.TimeSlice(d_time_s);

		//our final intended velocities get sent to the input as velocity to the rotary systems
		for (size_t i = 0; i < 4; i++)
		{
			m_Drive[i].SetVelocity(m_swerve_mgmt.GetIntendedVelocities().Velocity.AsArray[i]);
			m_Swivel[i].SetPosition(m_swerve_mgmt.GetIntendedVelocities().Velocity.AsArray[4 + i]);
			//rotary system in place... invoke its time change
			m_Drive[i].TimeSlice(d_time_s);
			m_Swivel[i].TimeSlice(d_time_s);
			//These will hook their updates to here in m_Voltage
		}
		//We'll go ahead and maintain an internal state of the position and heading even if this gets managed
		//We can bind to our entity here if client supports it
		//externally since it doesn't cost any overhead
		m_Entity_Input.InterpolateVelocities(m_Odometry.GetCurrentVelocities());
		//send this velocity to entity if it exists
		m_current_velocity = LocalToGlobal(GetCurrentHeading(), Vec2d(m_Entity_Input.GetLocalVelocityX(), m_Entity_Input.GetLocalVelocityY()));
		if (m_ExternSetVelocity)
			m_ExternSetVelocity(m_current_velocity);
		m_current_position += m_current_velocity * d_time_s;
		//now for our turning
		m_current_angular_velocity = m_Entity_Input.GetAngularVelocity();
		//send this velocity to entity if it exists
		if (m_ExternSetHeadingVelocity)
			m_ExternSetHeadingVelocity(m_current_angular_velocity);
		//If we don't have the callbacks for both set and get... we'll update our own default implementation
		if ((!m_ExternGetCurrentHeading) || (!m_ExternSetHeadingVelocity))
		{
			//from current velocity we can update the position
			m_current_heading += m_current_angular_velocity * d_time_s;
			//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
			m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging
		}
	}
	void SimulatorTimeSlice(double dTime_s)
	{
		//The simulation already is hooked to m_Voltage its ready to simulate
		//This call is skipped in real robot code as it physically happens instead
		m_Simulation.TimeSlice(dTime_s);
	}
	#pragma region _mutators_
	void SetLinearVelocity_local(double forward, double right)
	{
		m_MotionControl2D.SetLinearVelocity_local(forward, right);
	}
	void SetLinearVelocity_global(double north, double east)
	{
		m_MotionControl2D.SetLinearVelocity_global(north, east);
	}
	void SetAngularVelocity(double clockwise)
	{
		m_MotionControl2D.SetAngularVelocity(clockwise);
	}

	void SetIntendedOrientation(double intended_orientation, bool absolute = true)
	{
		m_MotionControl2D.SetIntendedOrientation(intended_orientation, absolute);
	}
	void DriveToLocation(double north, double east, bool absolute = true, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true)
	{
		m_MotionControl2D.DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
	}
	#pragma endregion
	#pragma region _accessors_
	//accessors
	Vec2D GetCurrentPosition() const
	{
		if (m_ExternGetCurrentPosition)
			return m_ExternGetCurrentPosition();
		else
			return m_current_position;
	}
	const Vec2D& Get_OdometryCurrentPosition() const
	{
		return m_Odometry.GetPosition();
	}
	double GetCurrentHeading() const
	{
		if (m_ExternGetCurrentHeading)
			return m_ExternGetCurrentHeading();
		else
			return m_current_heading;
	}
	double Get_OdometryCurrentHeading() const
	{
		return m_Odometry.GetHeading();
	}
	const SwerveVelocities &GetCurrentVelocities() const
	{
		return m_Odometry.GetCurrentVelocities();
	}
	const SwerveVelocities &GetSimulatedVelocities() const
	{
		return m_Simulation.GetCurrentVelocities();
	}
	const SwerveVelocities &GetCurrentVoltages() const
	{
		return m_Voltage;
	}
	const SwerveVelocities &GetIntendedVelocities() const
	{
		return m_robot.GetIntendedVelocities();
	}
	bool GetIsDrivenLinear() const
	{
		return m_MotionControl2D.GetIsDrivenLinear();
	}
	bool GetIsDrivenAngular() const
	{
		return m_MotionControl2D.GetIsDrivenAngular();
	}
	double Get_IntendedOrientation() const
	{
		return m_MotionControl2D.Get_IntendedOrientation();
	}
	bool Get_SupportOdometryPosition() const
	{
		return m_Odometry.SupportPosition();
	}
	bool Get_SupportOdometryHeading() const
	{
		return m_Odometry.SupportHeading();
	}
	#pragma endregion
	#pragma region _callbacks_
	void Set_UpdateGlobalVelocity(std::function<void(const Vec2D &new_velocity)> callback)
	{
		m_ExternSetVelocity = callback;
	}
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
	{
		m_ExternSetHeadingVelocity = callback;
	}
	void Set_GetCurrentPosition(std::function <Vec2D()> callback)
	{
		m_ExternGetCurrentPosition = callback;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_ExternGetCurrentHeading = callback;
	}
	void SetExternal_Velocity_PID_Monitor_Callback(std::function<SwerveRobot::PID_Velocity_proto> callback)
	{
		m_PID_Velocity_callback = callback;
	}
	void SetExternal_Position_PID_Monitor_Callback(std::function<SwerveRobot::PID_Position_proto> callback)
	{
		m_PID_Position_callback = callback;
	}
	void SetPhysicalOdometry(std::function<Robot::SwerveVelocities ()> callback)
	{
		m_PhysicalOdometry=callback;
	}
	#pragma endregion
};
#pragma region _wrapper methods_
SwerveRobot::SwerveRobot()
{
	m_SwerveRobot = std::make_shared<SwerveRobot_Internal>();
}
void SwerveRobot::Init(const Framework::Base::asset_manager* asset_properties)
{
	m_SwerveRobot->Init(asset_properties);
}
void SwerveRobot::Shutdown()
{
	m_SwerveRobot->Shutdown();
}
void SwerveRobot::SetLinearVelocity_local(double forward, double right)
{
	m_SwerveRobot->SetLinearVelocity_local(forward, right);
}
void SwerveRobot::SetLinearVelocity_global(double north, double east)
{
	m_SwerveRobot->SetLinearVelocity_global(north, east);
}
void SwerveRobot::SetAngularVelocity(double clockwise)
{
	m_SwerveRobot->SetAngularVelocity(clockwise);
}
void SwerveRobot::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_SwerveRobot->SetIntendedOrientation(intended_orientation, absolute);
}
void SwerveRobot::DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_SwerveRobot->DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
}
void SwerveRobot::TimeSlice(double d_time_s)
{
	m_SwerveRobot->TimeSlice(d_time_s);
}
void SwerveRobot::SimulatorTimeSlice(double dTime_s)
{
	m_SwerveRobot->SimulatorTimeSlice(dTime_s);
}
void SwerveRobot::Reset(double X, double Y, double heading)
{
	m_SwerveRobot->Reset(X, Y, heading);
}
Vec2D SwerveRobot::GetCurrentPosition() const
{
	return m_SwerveRobot->GetCurrentPosition();
}
const Vec2D& SwerveRobot::Get_OdometryCurrentPosition() const
{
	return m_SwerveRobot->Get_OdometryCurrentPosition();
}
double SwerveRobot::GetCurrentHeading() const
{
	return m_SwerveRobot->GetCurrentHeading();
}
double SwerveRobot::Get_OdometryCurrentHeading() const
{
	return m_SwerveRobot->Get_OdometryCurrentHeading();
}
void SwerveRobot::Set_UpdateGlobalVelocity(std::function<void(const Vec2D &new_velocity)> callback)
{
	m_SwerveRobot->Set_UpdateGlobalVelocity(callback);
}
void SwerveRobot::Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
{
	m_SwerveRobot->Set_UpdateHeadingVelocity(callback);
}
void SwerveRobot::Set_GetCurrentPosition(std::function <Vec2D()> callback)
{
	m_SwerveRobot->Set_GetCurrentPosition(callback);
}
void SwerveRobot::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_SwerveRobot->Set_GetCurrentHeading(callback);
}
void SwerveRobot::SetExternal_Velocity_PID_Monitor_Callback(std::function<PID_Velocity_proto> callback)
{
	m_SwerveRobot->SetExternal_Velocity_PID_Monitor_Callback(callback);
}
void SwerveRobot::SetExternal_Position_PID_Monitor_Callback(std::function<PID_Position_proto> callback)
{
	m_SwerveRobot->SetExternal_Position_PID_Monitor_Callback(callback);
}
void SwerveRobot::SetPhysicalOdometry(std::function<Robot::SwerveVelocities ()> callback)
{
	m_SwerveRobot->SetPhysicalOdometry(callback);
}
const SwerveVelocities &SwerveRobot::GetCurrentVelocities() const
{
	return m_SwerveRobot->GetCurrentVelocities();
}
const SwerveVelocities &SwerveRobot::GetSimulatedVelocities() const
{
	return m_SwerveRobot->GetSimulatedVelocities();
}
const SwerveVelocities &SwerveRobot::GetCurrentVoltages() const
{
	return m_SwerveRobot->GetCurrentVoltages();
}
const SwerveVelocities &SwerveRobot::GetIntendedVelocities() const
{
	return m_SwerveRobot->GetIntendedVelocities();
}
bool SwerveRobot::GetIsDrivenLinear() const
{
	return m_SwerveRobot->GetIsDrivenLinear();
}
bool SwerveRobot::GetIsDrivenAngular() const
{
	return m_SwerveRobot->GetIsDrivenAngular();
}
double SwerveRobot::Get_IntendedOrientation() const
{
	return m_SwerveRobot->Get_IntendedOrientation();
}
bool SwerveRobot::Get_SupportOdometryPosition() const
{
	return m_SwerveRobot->Get_SupportOdometryPosition();
}
bool SwerveRobot::Get_SupportOdometryHeading() const
{
	return m_SwerveRobot->Get_SupportOdometryHeading();
}
#pragma endregion

}}