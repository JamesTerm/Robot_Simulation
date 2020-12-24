#include "../Base/Base_Includes.h"

#include "script_loader.h"
#include "RegistryV1.h"

namespace properties
{
class script_loader_impl
{
private:
	Framework::Base::asset_manager* m_assets=nullptr;
	void TestIndivualWheels()
	{
		Framework::Base::asset_manager& assets = *m_assets;
		using namespace properties::registry_v1;
		//In this simple example we will simply hard code some of the assets in the registry
		//TODO Add properties that will help test various pieces of code
		// 1. add tweaking of controllers
		// 2. add actual values for legacy simulation
		// 3. add properties for WPI card assignments
		// I may abandon these above and instead get a Lua example... we'll see
		for (size_t i = 0; i < 4; i++)
		{
			//form our prefix, it will use the naming convention in Vehicle Drive.h
			const char* const prefix_table[2][4] =
			{
				{csz_sFL_,csz_sFR_,csz_sRL_,csz_sRR_},
				{csz_aFL_,csz_FR_,csz_aRL_,csz_aRR_}
			};
			const char* const prefix = prefix_table[0][i];
			std::string constructed_name;
			//we'll go down the list
			using namespace Framework::Base;
			//entity 1D
			constructed_name = prefix, constructed_name += csz_Entity1D_StartingPosition;
			assets.put_number(constructed_name.c_str(), 0.01);
			constructed_name = prefix, constructed_name += csz_Entity1D_Mass;
			assets.put_number(constructed_name.c_str(), 1.5);
			//for testing... we can intentionally leave out properties like this
			//constructed_name = prefix, constructed_name += csz_Entity1D_Dimension;
			//assets.put_number(constructed_name.c_str(), .1524);
			constructed_name = prefix, constructed_name += csz_Entity1D_IsAngular;
			assets.put_bool(constructed_name.c_str(), false);
		}
	}
	void TestCurivator()
	{
		#pragma region _LUA conversion_
		//Not the fastest robot, but is the most tested and great base line
		#pragma region _units conversions_
		static const double Inches2Meters = 0.0254;
		static const double Feet2Meters = 0.3048;
		static const double Meters2Feet = 3.2808399;
		static const double Meters2Inches = 39.3700787;
		static const double OunceInchToNewton = 0.00706155183333;
		static const double Pounds2Kilograms = 0.453592;
		static const double Deg2Rad = (1.0 / 180.0) * Pi;
		#pragma endregion

		const double wheel_diameter_Curivator_in = 7.95;
		const double g_wheel_diameter_in = wheel_diameter_Curivator_in;   //This will determine the correct distance try to make accurate too
		const double WheelBase_Width_Curivator_In = 42.26;
		const double WheelBase_Length_Curivator_In = 38.46;
		static const double WheelBase_Length_In = WheelBase_Length_Curivator_In;
		static const double WheelBase_Width_In = WheelBase_Width_Curivator_In;

		const double WheelTurningDiameter_In = sqrt((WheelBase_Width_In * WheelBase_Width_In) + (WheelBase_Length_In * WheelBase_Length_In));
		const double DriveGearSpeed_Curivator = (255.15 / 60.0) * Pi * g_wheel_diameter_in * Inches2Meters * 0.9;
		const double DriveGearSpeed = DriveGearSpeed_Curivator;
		static const double Drive_MaxAccel = 5.0;
		//Swerve wheels means no skid
		const double skid_curivator = 1.0;
		const double skid = skid_curivator;
		const double gMaxTorqueYaw = (2 * Drive_MaxAccel * Meters2Inches / WheelTurningDiameter_In) * skid;
		const int EncoderLoop = 1;

		struct averaged_motors
		{
			const double wheel_mass = 1.5;
			const double cof_efficiency = 0.9;
			const double gear_reduction = 1.0;
			const double torque_on_wheel_radius = Inches2Meters * 1.8;
			const double drive_wheel_radius = Inches2Meters * 4;
			const double number_of_motors = 1;
			const double payload_mass = 200 * Pounds2Kilograms;
			const double speed_loss_constant = 0.81;
			const double drive_train_effciency = 0.9;

			const double free_speed_rpm = 263.88;
			const double stall_torque = 34;
			const double stall_current_amp = 84;
			const double free_current_amp = 0.4;
		};

		#pragma region _ship 2d props_
		//Ship props:
		const double Mass = 25; //Weight kg
		const double MaxAccelLeft = 20; 
		const double MaxAccelRight = 20;
		const double MaxAccelForward = Drive_MaxAccel; 
		const double MaxAccelReverse = Drive_MaxAccel;
		const double MaxAccelForward_High = Drive_MaxAccel * 2; 
		const double MaxAccelReverse_High = Drive_MaxAccel * 2;
		const double MaxTorqueYaw = gMaxTorqueYaw; //Note Bradley had 0.78 reduction to get the feel
		const double MaxTorqueYaw_High = gMaxTorqueYaw * 5;
		const double MaxTorqueYaw_SetPoint = gMaxTorqueYaw * 2;
		const double MaxTorqueYaw_SetPoint_High = gMaxTorqueYaw * 10;
		const double rotation_tolerance = Deg2Rad * 2;
		const double rotation_distance_scalar = 1.0;

		const double MAX_SPEED = DriveGearSpeed;
		const double ACCEL = 10; //Thruster Acceleration m / s2(1g = 9.8)
		const double BRAKE = ACCEL;
		//Turn Rates(radians / sec) This is always correct do not change
		const double heading_rad = (2 * DriveGearSpeed * Meters2Inches / WheelTurningDiameter_In) * skid;
		#pragma endregion

		struct	Dimensions 
		{
			double Length = 0.9525; double Width = 0.6477;
		}; //These are 37.5 x 25.5 inches(This is not used except for UI ignore)

		//Rotary System
		const bool is_closed = 1;
		const bool is_closed_swivel = 1;

		//show_pid_dump_wheel = { fl = 0, fr = 0, rl = 0, rr = 0 },
		//show_pid_dump_swivel = { fl = 0, fr = 0, rl = 0, rr = 0 },

		//ds_display_row = -1,
		//where length is in 5 inches in, and width is 3 on each side(can only go 390 degrees a second)
		struct wheel_base_dimensions { double length_in = WheelBase_Length_In; double width_in = WheelBase_Width_In; };

		//This encoders / PID will only be used in autonomous if we decide to go steal balls
		const double wheel_diameter_in = g_wheel_diameter_in;
		struct wheel_pid { double p = 200; double i = 0; double d = 50; };
		struct swivel_pid { double p = 100; double i = 0; double d = 50; };
		const double latency = 0.0;
		const double heading_latency = 0.0;
		const double drive_to_scale = 0.50; //For 4 to 10 50 % gives a 5 inch tolerance
		//strafe_to_scale = 4 / 20, --In autonomous we need the max to match the max forward and reverse
		//This is obtainer from encoder RPM's of 1069.2 and Wheel RPM's 427.68 (both high and low have same ratio)
		struct force_voltage
		{
			double t4 = 0; double t3 = 0; double t2 = 0; double t1 = 0; double c = 1;
		};
		bool reverse_steering = false;
		const double inv_max_accel = 1 / 15.0; //solved empirically
		//motor_specs = averaged_motors;
		averaged_motors motor_specs;
		
		struct wheel_common 
		{
			int is_closed = EncoderLoop;
			bool show_pid_dump = false;
			//ds_display_row = -1;
			struct pid { double p = 200; double i = 0; double d = 25; } _pid;
			//Note: removed in Encoder Simulator v3
			//curve_voltage =
			//{t4 = 3.1199; t3 = -4.4664; t2 = 2.2378; t1 = 0.1222; c = 0};
			const double encoder_pulses_per_revolution = 560 / 4;
			const double encoder_to_wheel_ratio = 1.0;
			const bool encoder_reversed_wheel = false;
			const double max_speed = 8.91 * Feet2Meters;
			const double accel = 10.0;						//We may indeed have a two button solution(match with max accel)
			const double brake = 10.0;
			const double max_accel_forward = Drive_MaxAccel;			//These are in radians, just go with what feels right
			const double max_accel_reverse = Drive_MaxAccel;
			const bool using_range = false;	//Warning Only use range if we have a potentiometer!
			const double inv_max_accel = 1 / 15.0;  //solved empirically
			const bool use_aggressive_stop = true;
		};

		struct swivel_common
		{
			bool is_closed = 0;
			bool show_pid_dump = false;
			//ds_display_row = -1;
			const bool use_pid_up_only = true;
			struct pid_up { double p = 100; double i = 0; double d = 25; } _pid_swivel;
			//pid_down = {p = 100; i = 0; d = 25};
			const double tolerance = 0.03;
			const int tolerance_count = 1;
			const double voltage_multiply = 1.0;			//May be reversed
			//this may be 184: 84 * 36 : 20... using 180 as the ring is 3.8571428571428571428571428571429
			const double encoder_to_wheel_ratio = 1.0;
			const double max_speed = 2.0;	//100 rpm... with a 12 : 36 reduction in radians
			const double accel = 10.0;						//We may indeed have a two button solution(match with max accel)
			const double brake = 10.0;
			const bool using_range = 0;	//Warning Only use range if we have a potentiometer!
			const double max_range_deg = 45;
			const double min_range_deg = -45;
			const double starting_position = 0.0;
			const double pot_offset = -45.0 * Deg2Rad;
			const bool use_aggressive_stop = true;
		};
		#pragma endregion

		#pragma region _setup put vars_
		Framework::Base::asset_manager& assets = *m_assets;
		using namespace properties::registry_v1;
		std::string constructed_name;
		const char* prefix = nullptr;

		#define PUT_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_number(constructed_name.c_str(), y);
		#define PUT_NUMBER_suffix(x,y,z) \
			constructed_name = prefix, constructed_name += csz_##x; \
			constructed_name += #z;\
			assets.put_number(constructed_name.c_str(), y);

		#define PUT_BOOL(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_bool(constructed_name.c_str(), y);
		#pragma endregion
		#pragma region _Put wheel_common_
		{
			prefix = csz_CommonDrive_;
			wheel_common val;
			//There is no Entity 1D set in the LUA--
			//Ship1D------------------------
			//The ones commented out are not in the LUA (client will retain default values)
			PUT_NUMBER(Ship_1D_MAX_SPEED, val.max_speed);
			//PUT_NUMBER(Ship_1D_MaxSpeed_Forward)
			//PUT_NUMBER(Ship_1D_MaxSpeed_Reverse)
			PUT_NUMBER(Ship_1D_ACCEL, val.accel);
			PUT_NUMBER(Ship_1D_BRAKE, val.brake);
			PUT_NUMBER(Ship_1D_MaxAccelForward, val.max_accel_forward);
			PUT_NUMBER(Ship_1D_MaxAccelReverse, val.max_accel_reverse);
			//PUT_NUMBER(Ship_1D_MinRange)
			//PUT_NUMBER(Ship_1D_MaxRange)
			//PUT_NUMBER(Ship_1D_DistanceDegradeScalar)
			PUT_BOOL(Ship_1D_UsingRange,val.using_range) //bool
			//Rotary--------------------------
			//PUT_NUMBER(Rotary_VoltageScalar, val.VoltageScalar);
			PUT_NUMBER(Rotary_EncoderToRS_Ratio, val.encoder_to_wheel_ratio);
			PUT_NUMBER(Rotary_EncoderPulsesPerRevolution, val.encoder_pulses_per_revolution);
			//PUT_NUMBER(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
			PUT_NUMBER_suffix(Rotary_PID, val._pid.p, _p);
			PUT_NUMBER_suffix(Rotary_PID, val._pid.i, _i);
			PUT_NUMBER_suffix(Rotary_PID, val._pid.d, _d);
			//PUT_NUMBER(Rotary_PrecisionTolerance, val.PrecisionTolerance);
			//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
			//PUT_NUMBER(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
			PUT_NUMBER(Rotary_InverseMaxAccel, val.inv_max_accel);
			//PUT_NUMBER(Rotary_InverseMaxDecel, val.InverseMaxDecel);
			//PUT_NUMBER(Rotary_Positive_DeadZone, val.Positive_DeadZone);
			//PUT_NUMBER(Rotary_Negative_DeadZone, val.Negative_DeadZone);
			//PUT_NUMBER(Rotary_MinLimitRange, val.MinLimitRange);
			//PUT_NUMBER(Rotary_MaxLimitRange, val.MaxLimitRange);
			//PUT_NUMBER(Rotary_Feedback_DiplayRow, Rotary_.Feedback_DiplayRow);
			//The logic of loop states. From LUA point of view, was if I didn't have it... it was none = 0, if I do and it is false
			//it's open which = 1, if "is_closed" is true which = 2 and if "is_closed2" is true (for position only) = 3
			//Note: Not having it means I can't even try to read it
			PUT_NUMBER(Rotary_LoopState, val.is_closed ? 2.0 : 1.0); //enum LoopStates... put as double, get as int
			PUT_BOOL(Rotary_PID_Console_Dump, val.show_pid_dump); //bool
			PUT_BOOL(Rotary_UseAggressiveStop, val.use_aggressive_stop); //bool
			PUT_BOOL(Rotary_EncoderReversed_Wheel, val.encoder_reversed_wheel); //bool
		}
		#pragma endregion
		#pragma region _Average Drive Motor_
		{
			prefix = "";
			//TODO Simulator 3 does not bleed off properly... must fix
			PUT_BOOL(EncoderSimulation_UseEncoder2,true); //use encoder 3 simulation if false
			averaged_motors val;

			PUT_NUMBER(EncoderSimulation_Wheel_Mass, val.wheel_mass);
			PUT_NUMBER(EncoderSimulation_COF_Efficiency, val.cof_efficiency);
			PUT_NUMBER(EncoderSimulation_GearReduction, val.gear_reduction);
			PUT_NUMBER(EncoderSimulation_TorqueAccelerationDampener, val.torque_on_wheel_radius);
			PUT_NUMBER(EncoderSimulation_DriveWheelRadius, val.drive_wheel_radius);
			PUT_NUMBER(EncoderSimulation_NoMotors, val.number_of_motors);
			PUT_NUMBER(EncoderSimulation_PayloadMass, val.payload_mass);
			PUT_NUMBER(EncoderSimulation_SpeedLossConstant, val.speed_loss_constant);
			PUT_NUMBER(EncoderSimulation_DriveTrainEfficiency, val.drive_train_effciency);
			//	struct Motor_Specs
			PUT_NUMBER(EncoderSimulation_FreeSpeed_RPM, val.free_speed_rpm);
			PUT_NUMBER(EncoderSimulation_Stall_Torque_NM, val.stall_torque);
			PUT_NUMBER(EncoderSimulation_Stall_Current_Amp, val.stall_current_amp);
			PUT_NUMBER(EncoderSimulation_Free_Current_Amp, val.free_current_amp);
		}
		#pragma endregion
		#pragma region _Put swivel_common_
		{
			prefix = csz_CommonSwivel_;
			swivel_common val;
			//Entity1D----------------------
			PUT_NUMBER(Entity1D_StartingPosition, val.starting_position);
			//Ship1D------------------------
			//The ones commented out are not in the LUA (client will retain default values)
			PUT_NUMBER(Ship_1D_MAX_SPEED, val.max_speed);
			//PUT_NUMBER(Ship_1D_MaxSpeed_Forward)
			//PUT_NUMBER(Ship_1D_MaxSpeed_Reverse)
			PUT_NUMBER(Ship_1D_ACCEL, val.accel);
			PUT_NUMBER(Ship_1D_BRAKE, val.brake);
			//PUT_NUMBER(Ship_1D_MaxAccelForward, val.max_accel_forward);
			//PUT_NUMBER(Ship_1D_MaxAccelReverse, val.max_accel_reverse);
			PUT_NUMBER(Ship_1D_MinRange, Deg2Rad * val.min_range_deg);
			PUT_NUMBER(Ship_1D_MaxRange, Deg2Rad * val.max_range_deg);
			//PUT_NUMBER(Ship_1D_DistanceDegradeScalar)
			PUT_BOOL(Ship_1D_UsingRange,val.using_range) //bool
			//Rotary--------------------------
			PUT_NUMBER(Rotary_VoltageScaler, val.voltage_multiply);
			PUT_NUMBER(Rotary_EncoderToRS_Ratio, val.encoder_to_wheel_ratio);
			//PUT_NUMBER(Rotary_EncoderPulsesPerRevolution, val.encoder_pulses_per_revolution);
			PUT_BOOL(Rotary_Arm_GainAssist_UsePID_Up_Only, val.use_pid_up_only);
			//PUT_NUMBER(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.p, _p);
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.i, _i);
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.d, _d);
			PUT_NUMBER(Rotary_PrecisionTolerance, val.tolerance);
			PUT_NUMBER(Rotary_Arm_GainAssist_ToleranceConsecutiveCount, val.tolerance_count);
			//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
			//PUT_NUMBER(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
			//PUT_NUMBER(Rotary_InverseMaxAccel, val.inv_max_accel);
			//PUT_NUMBER(Rotary_InverseMaxDecel, val.InverseMaxDecel);
			//PUT_NUMBER(Rotary_Positive_DeadZone, val.Positive_DeadZone);
			//PUT_NUMBER(Rotary_Negative_DeadZone, val.Negative_DeadZone);
			//PUT_NUMBER(Rotary_MinLimitRange, val.MinLimitRange);
			//PUT_NUMBER(Rotary_MaxLimitRange, val.MaxLimitRange);
			//PUT_NUMBER(Rotary_Feedback_DiplayRow, Rotary_.Feedback_DiplayRow);
			//The logic of loop states. From LUA point of view, was if I didn't have it... it was none = 0, if I do and it is false
			//it's open which = 1, if "is_closed" is true which = 2 and if "is_closed2" is true (for position only) = 3
			//Note: Not having it means I can't even try to read it
			PUT_NUMBER(Rotary_LoopState, val.is_closed ? 2.0 : 1.0); //enum LoopStates... put as double, get as int
			PUT_BOOL(Rotary_PID_Console_Dump, val.show_pid_dump); //bool
			PUT_BOOL(Rotary_UseAggressiveStop, val.use_aggressive_stop); //bool
			//PUT_BOOL(Rotary_EncoderReversed_Wheel, val.encoder_reversed_wheel); //bool
			//Rotary Pot---------------------------------------------------------------
			PUT_NUMBER(Rotary_Pot_offset, val.pot_offset);
		}
		#pragma endregion

		//Test PID
		if (false)
		{
			prefix = csz_sFL_; //pick the encoder of the left front wheel
			//prefix = csz_aFL_; //pick the potentiometer of the left front wheel
			PUT_BOOL(Rotary_PID_Console_Dump, true); //bool
		}

		//finished with macros
		#undef PUT_NUMBER
		#undef PUT_BOOL
	}
	void TestAndromeda()
	{
		#pragma region _LUA conversion_
		//Not the fastest robot, but is the most tested and great base line
		#pragma region _units conversions_
		static const double Inches2Meters = 0.0254;
		static const double Feet2Meters = 0.3048;
		static const double Meters2Feet = 3.2808399;
		static const double Meters2Inches = 39.3700787;
		static const double OunceInchToNewton = 0.00706155183333;
		static const double Pounds2Kilograms = 0.453592;
		static const double Deg2Rad = (1.0 / 180.0) * Pi;
		#pragma endregion

		const double wheel_diameter_Andromeda_in = 4.0;
		const double g_wheel_diameter_in = wheel_diameter_Andromeda_in;   //This will determine the correct distance try to make accurate too
		const double WheelBase_Width_Andromeda_In = 21.58;
		const double WheelBase_Length_Andromeda_In = 21.58;
		static const double WheelBase_Length_In = WheelBase_Length_Andromeda_In;
		static const double WheelBase_Width_In = WheelBase_Width_Andromeda_In;
		//Should be 30.52
		const double WheelTurningDiameter_In = sqrt((WheelBase_Width_In * WheelBase_Width_In) + (WheelBase_Length_In * WheelBase_Length_In));
		const double DriveGearSpeed_Andromeda = (653.33 / 60.0) * Pi * g_wheel_diameter_in * Inches2Meters * 0.9;
		static const double DriveGearSpeed = DriveGearSpeed_Andromeda;
		static const double Drive_MaxAccel = 5.0;
		//Swerve wheels means no skid
		const double skid = 1.0;
		const double gMaxTorqueYaw = (2 * Drive_MaxAccel * Meters2Inches / WheelTurningDiameter_In) * skid;
		const int EncoderLoop = 1;

		struct averaged_motors
		{
			const double wheel_mass = 1.5;
			const double cof_efficiency = 0.9;
			const double gear_reduction = 9.0;
			const double torque_on_wheel_radius = Inches2Meters * 1.64;
			const double drive_wheel_radius = Inches2Meters * 2;
			const double number_of_motors = 4;
			const double payload_mass = 148 * Pounds2Kilograms;
			const double speed_loss_constant = 0.81;
			const double drive_train_effciency = 0.9;
			//NEO
			//TODO suppose to be 5880, but the top computed speed in drive train of encoder sim 2
			//is too high, need to work out why this is.  Note: this is only a simulation issue
			const double free_speed_rpm = 5280.0; 
			const double stall_torque = 3.36;
			const double stall_current_amp = 166.0;
			const double free_current_amp = 1.8;
		};

		#pragma region _ship 2d props_
		//Ship props:
		const double Mass = 25; //Weight kg
		const double MaxAccelLeft = 20; 
		const double MaxAccelRight = 20;
		const double MaxAccelForward = Drive_MaxAccel; 
		const double MaxAccelReverse = Drive_MaxAccel;
		const double MaxAccelForward_High = Drive_MaxAccel * 2; 
		const double MaxAccelReverse_High = Drive_MaxAccel * 2;
		const double MaxTorqueYaw = gMaxTorqueYaw; //Note Bradley had 0.78 reduction to get the feel
		const double MaxTorqueYaw_High = gMaxTorqueYaw * 5;
		const double MaxTorqueYaw_SetPoint = gMaxTorqueYaw * 2;
		const double MaxTorqueYaw_SetPoint_High = gMaxTorqueYaw * 10;
		const double rotation_tolerance = Deg2Rad * 2;
		const double rotation_distance_scalar = 1.0;

		const double MAX_SPEED = DriveGearSpeed;
		const double ACCEL = 10; //Thruster Acceleration m / s2(1g = 9.8)
		const double BRAKE = ACCEL;
		//Turn Rates(radians / sec) This is always correct do not change
		const double heading_rad = (2 * DriveGearSpeed * Meters2Inches / WheelTurningDiameter_In) * skid;
		#pragma endregion

		struct	Dimensions 
		{
			double Length = 0.9525; double Width = 0.6477;
		}; //These are 37.5 x 25.5 inches(This is not used except for UI ignore)

		//Rotary System
		const bool is_closed = 1;
		const bool is_closed_swivel = 1;

		//show_pid_dump_wheel = { fl = 0, fr = 0, rl = 0, rr = 0 },
		//show_pid_dump_swivel = { fl = 0, fr = 0, rl = 0, rr = 0 },

		//ds_display_row = -1,
		//where length is in 5 inches in, and width is 3 on each side(can only go 390 degrees a second)
		struct wheel_base_dimensions { double length_in = WheelBase_Length_In; double width_in = WheelBase_Width_In; };

		//This encoders / PID will only be used in autonomous if we decide to go steal balls
		const double wheel_diameter_in = g_wheel_diameter_in;
		struct wheel_pid { double p = 200; double i = 0; double d = 50; };
		struct swivel_pid { double p = 100; double i = 0; double d = 50; };
		const double latency = 0.0;
		const double heading_latency = 0.0;
		const double drive_to_scale = 0.50; //For 4 to 10 50 % gives a 5 inch tolerance
		//strafe_to_scale = 4 / 20, --In autonomous we need the max to match the max forward and reverse
		//This is obtainer from encoder RPM's of 1069.2 and Wheel RPM's 427.68 (both high and low have same ratio)
		struct force_voltage
		{
			double t4 = 0; double t3 = 0; double t2 = 0; double t1 = 0; double c = 1;
		};
		bool reverse_steering = false;
		const double inv_max_accel = 1 / 15.0; //solved empirically
		//motor_specs = averaged_motors;
		averaged_motors motor_specs;
		
		struct wheel_common 
		{
			int is_closed = EncoderLoop;
			bool show_pid_dump = false;
			//ds_display_row = -1;
			struct pid { double p = 200; double i = 0; double d = 25; } _pid;
			//Note: removed in Encoder Simulator v3
			//curve_voltage =
			//{t4 = 3.1199; t3 = -4.4664; t2 = 2.2378; t1 = 0.1222; c = 0};
			const double encoder_pulses_per_revolution = 560 / 4;
			const double encoder_to_wheel_ratio = 1.0;
			const bool encoder_reversed_wheel = false;
			const double max_speed = DriveGearSpeed;
			const double accel = 10.0;						//We may indeed have a two button solution(match with max accel)
			const double brake = 10.0;
			const double max_accel_forward = Drive_MaxAccel;			//These are in radians, just go with what feels right
			const double max_accel_reverse = Drive_MaxAccel;
			const bool using_range = false;	//Warning Only use range if we have a potentiometer!
			const double inv_max_accel = 1 / 15.0;  //solved empirically
			const bool use_aggressive_stop = true;
		};

		struct swivel_common
		{
			bool is_closed = 0;
			bool show_pid_dump = false;
			//ds_display_row = -1;
			const bool use_pid_up_only = true;
			struct pid_up { double p = 100; double i = 0; double d = 25; } _pid_swivel;
			//pid_down = {p = 100; i = 0; d = 25};
			const double tolerance = 0.03;
			const int tolerance_count = 1;
			const double voltage_multiply = 1.0;			//May be reversed
			const double encoder_to_wheel_ratio = 1.0;
			//In radians using 1.5 rpm
			const double max_speed = 9.4;
			const double accel = 10.0;						//We may indeed have a two button solution(match with max accel)
			const double brake = 10.0;
			const bool using_range = 0;	//Warning Only use range if we have a potentiometer!
			const bool use_aggressive_stop = true;
		};
		#pragma endregion

		#pragma region _setup put vars_
		Framework::Base::asset_manager& assets = *m_assets;
		using namespace properties::registry_v1;
		std::string constructed_name;
		const char* prefix = nullptr;

		#define PUT_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_number(constructed_name.c_str(), y);
		#define PUT_NUMBER_suffix(x,y,z) \
			constructed_name = prefix, constructed_name += csz_##x; \
			constructed_name += #z;\
			assets.put_number(constructed_name.c_str(), y);

		#define PUT_BOOL(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_bool(constructed_name.c_str(), y);
		#pragma endregion
		#pragma region _Motion 2D_
		{
			prefix = "";
			PUT_NUMBER(Drive_WheelBase_in, WheelBase_Length_In);
			PUT_NUMBER(Drive_TrackWidth_in, WheelBase_Width_In);

			PUT_NUMBER(Motion2D_max_speed_linear,MAX_SPEED);
			PUT_NUMBER(Motion2D_max_speed_angular, heading_rad);
			PUT_NUMBER(Motion2D_max_acceleration_linear, Drive_MaxAccel);
			PUT_NUMBER(Motion2D_max_acceleration_angular, gMaxTorqueYaw);
		}
		#pragma endregion
		#pragma region _Ship 2D_
		{
			#define PN_(x) \
				assets.put_number(csz_Ship2D_##x, x);
			#define PNy(x,y) \
				assets.put_number(csz_Ship2D_##x, y);

			PNy(dHeading,heading_rad);
			//PN_(EngineRampForward); PN_(EngineRampReverse); PN_(EngineRampAfterBurner);
			//PN_(EngineDeceleration); PN_(EngineRampStrafe);
			PN_(MAX_SPEED); PNy(ENGAGED_MAX_SPEED,MAX_SPEED);
			PN_(ACCEL); PN_(BRAKE); //PN_(STRAFE); PN_(AFTERBURNER_ACCEL); PN_(AFTERBURNER_BRAKE);

			PN_(MaxAccelLeft); PN_(MaxAccelRight); PN_(MaxAccelForward); PN_(MaxAccelReverse);
			PN_(MaxAccelForward_High); PN_(MaxAccelReverse_High);
			PN_(MaxTorqueYaw); PN_(MaxTorqueYaw_High);
			PN_(MaxTorqueYaw_SetPoint); PN_(MaxTorqueYaw_SetPoint_High);
			PNy(Rotation_Tolerance,rotation_tolerance);
			//PNy(Rotation_ToleranceConsecutiveCount,rotation_tolerance_count);
			PNy(Rotation_TargetDistanceScalar,rotation_distance_scalar);

			#undef PN_
			#undef PNy
		}
		#pragma endregion
		#pragma region _Put wheel_common_
		{
			prefix = csz_CommonDrive_;
			wheel_common val;
			//There is no Entity 1D set in the LUA--
			//Ship1D------------------------
			//The ones commented out are not in the LUA (client will retain default values)
			PUT_NUMBER(Ship_1D_MAX_SPEED, val.max_speed);
			//PUT_NUMBER(Ship_1D_MaxSpeed_Forward)
			//PUT_NUMBER(Ship_1D_MaxSpeed_Reverse)
			PUT_NUMBER(Ship_1D_ACCEL, val.accel);
			PUT_NUMBER(Ship_1D_BRAKE, val.brake);
			PUT_NUMBER(Ship_1D_MaxAccelForward, val.max_accel_forward);
			PUT_NUMBER(Ship_1D_MaxAccelReverse, val.max_accel_reverse);
			//PUT_NUMBER(Ship_1D_MinRange)
			//PUT_NUMBER(Ship_1D_MaxRange)
			//PUT_NUMBER(Ship_1D_DistanceDegradeScalar)
			PUT_BOOL(Ship_1D_UsingRange,val.using_range) //bool
			//Rotary--------------------------
			//PUT_NUMBER(Rotary_VoltageScalar, val.VoltageScalar);
			PUT_NUMBER(Rotary_EncoderToRS_Ratio, val.encoder_to_wheel_ratio);
			PUT_NUMBER(Rotary_EncoderPulsesPerRevolution, val.encoder_pulses_per_revolution);
			//PUT_NUMBER(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
			PUT_NUMBER_suffix(Rotary_PID, val._pid.p, _p);
			PUT_NUMBER_suffix(Rotary_PID, val._pid.i, _i);
			PUT_NUMBER_suffix(Rotary_PID, val._pid.d, _d);
			//PUT_NUMBER(Rotary_PrecisionTolerance, 0.03);
			//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
			//PUT_NUMBER(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
			PUT_NUMBER(Rotary_InverseMaxAccel, val.inv_max_accel);
			//PUT_NUMBER(Rotary_InverseMaxDecel, val.InverseMaxDecel);
			//PUT_NUMBER(Rotary_Positive_DeadZone, val.Positive_DeadZone);
			//PUT_NUMBER(Rotary_Negative_DeadZone, val.Negative_DeadZone);
			//PUT_NUMBER(Rotary_MinLimitRange, val.MinLimitRange);
			//PUT_NUMBER(Rotary_MaxLimitRange, val.MaxLimitRange);
			//PUT_NUMBER(Rotary_Feedback_DiplayRow, Rotary_.Feedback_DiplayRow);
			//The logic of loop states. From LUA point of view, was if I didn't have it... it was none = 0, if I do and it is false
			//it's open which = 1, if "is_closed" is true which = 2 and if "is_closed2" is true (for position only) = 3
			//Note: Not having it means I can't even try to read it
			PUT_NUMBER(Rotary_LoopState, val.is_closed ? 2.0 : 1.0); //enum LoopStates... put as double, get as int
			PUT_BOOL(Rotary_PID_Console_Dump, val.show_pid_dump); //bool
			PUT_BOOL(Rotary_UseAggressiveStop, val.use_aggressive_stop); //bool
			PUT_BOOL(Rotary_EncoderReversed_Wheel, val.encoder_reversed_wheel); //bool
		}
		#pragma endregion
		#pragma region _Average Drive Motor_
		{
			prefix = "";
			//TODO Simulator 3 does not bleed off properly... must fix
			PUT_BOOL(EncoderSimulation_UseEncoder2,true); //use encoder 3 simulation if false
			averaged_motors val;

			PUT_NUMBER(EncoderSimulation_Wheel_Mass, val.wheel_mass);
			PUT_NUMBER(EncoderSimulation_COF_Efficiency, val.cof_efficiency);
			PUT_NUMBER(EncoderSimulation_GearReduction, val.gear_reduction);
			PUT_NUMBER(EncoderSimulation_TorqueAccelerationDampener, val.torque_on_wheel_radius);
			PUT_NUMBER(EncoderSimulation_DriveWheelRadius, val.drive_wheel_radius);
			PUT_NUMBER(EncoderSimulation_NoMotors, val.number_of_motors);
			PUT_NUMBER(EncoderSimulation_PayloadMass, val.payload_mass);
			PUT_NUMBER(EncoderSimulation_SpeedLossConstant, val.speed_loss_constant);
			PUT_NUMBER(EncoderSimulation_DriveTrainEfficiency, val.drive_train_effciency);
			//	struct Motor_Specs
			PUT_NUMBER(EncoderSimulation_FreeSpeed_RPM, val.free_speed_rpm);
			PUT_NUMBER(EncoderSimulation_Stall_Torque_NM, val.stall_torque);
			PUT_NUMBER(EncoderSimulation_Stall_Current_Amp, val.stall_current_amp);
			PUT_NUMBER(EncoderSimulation_Free_Current_Amp, val.free_current_amp);
		}
		#pragma endregion
		#pragma region _Put swivel_common_
		{
			prefix = csz_CommonSwivel_;
			swivel_common val;
			//Entity1D----------------------
			//PUT_NUMBER(Entity1D_StartingPosition, val.starting_position);
			//Ship1D------------------------
			//The ones commented out are not in the LUA (client will retain default values)
			PUT_NUMBER(Ship_1D_MAX_SPEED, val.max_speed);
			//PUT_NUMBER(Ship_1D_MaxSpeed_Forward)
			//PUT_NUMBER(Ship_1D_MaxSpeed_Reverse)
			PUT_NUMBER(Ship_1D_ACCEL, val.accel);
			PUT_NUMBER(Ship_1D_BRAKE, val.brake);
			//PUT_NUMBER(Ship_1D_MaxAccelForward, val.max_accel_forward);
			//PUT_NUMBER(Ship_1D_MaxAccelReverse, val.max_accel_reverse);
			//PUT_NUMBER(Ship_1D_MinRange, Deg2Rad * val.min_range_deg);
			//PUT_NUMBER(Ship_1D_MaxRange, Deg2Rad * val.max_range_deg);
			//PUT_NUMBER(Ship_1D_DistanceDegradeScalar)
			PUT_BOOL(Ship_1D_UsingRange,val.using_range) //bool
			//Rotary--------------------------
			PUT_NUMBER(Rotary_VoltageScaler, val.voltage_multiply);
			PUT_NUMBER(Rotary_EncoderToRS_Ratio, val.encoder_to_wheel_ratio);
			//PUT_NUMBER(Rotary_EncoderPulsesPerRevolution, val.encoder_pulses_per_revolution);
			PUT_BOOL(Rotary_Arm_GainAssist_UsePID_Up_Only, val.use_pid_up_only);
			//PUT_NUMBER(Rotary_PID)  //double[3]... append _p _i _d to the name for each element
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.p, _p);
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.i, _i);
			PUT_NUMBER_suffix(Rotary_Arm_GainAssist_PID_Up, val._pid_swivel.d, _d);
			PUT_NUMBER(Rotary_PrecisionTolerance, val.tolerance);
			PUT_NUMBER(Rotary_Arm_GainAssist_ToleranceConsecutiveCount, val.tolerance_count);
			//Use _c, _t1, _t2, _t3, _t4 for array 0..5 respectively
			//PUT_NUMBER(Rotary_Voltage_Terms) //PolynomialEquation_forth_Props 
			//PUT_NUMBER(Rotary_InverseMaxAccel, val.inv_max_accel);
			//PUT_NUMBER(Rotary_InverseMaxDecel, val.InverseMaxDecel);
			//PUT_NUMBER(Rotary_Positive_DeadZone, val.Positive_DeadZone);
			//PUT_NUMBER(Rotary_Negative_DeadZone, val.Negative_DeadZone);
			//PUT_NUMBER(Rotary_MinLimitRange, val.MinLimitRange);
			//PUT_NUMBER(Rotary_MaxLimitRange, val.MaxLimitRange);
			//PUT_NUMBER(Rotary_Feedback_DiplayRow, Rotary_.Feedback_DiplayRow);
			//The logic of loop states. From LUA point of view, was if I didn't have it... it was none = 0, if I do and it is false
			//it's open which = 1, if "is_closed" is true which = 2 and if "is_closed2" is true (for position only) = 3
			//Note: Not having it means I can't even try to read it
			PUT_NUMBER(Rotary_LoopState, val.is_closed ? 2.0 : 1.0); //enum LoopStates... put as double, get as int
			PUT_BOOL(Rotary_PID_Console_Dump, val.show_pid_dump); //bool
			PUT_BOOL(Rotary_UseAggressiveStop, val.use_aggressive_stop); //bool
			//PUT_BOOL(Rotary_EncoderReversed_Wheel, val.encoder_reversed_wheel); //bool
			//Rotary Pot---------------------------------------------------------------
			//PUT_NUMBER(Rotary_Pot_offset, val.pot_offset);
		}
		#pragma endregion

		//Test PID
		if (false)
		{
			prefix = csz_sFL_; //pick the encoder of the left front wheel
			//prefix = csz_aFL_; //pick the potentiometer of the left front wheel
			PUT_BOOL(Rotary_PID_Console_Dump, true); //bool
		}

		//finished with macros
		#undef PUT_NUMBER
		#undef PUT_BOOL
	}
	void Test_Controls_Airflo()
	{
		#pragma region _setup put vars_
		Framework::Base::asset_manager& assets = *m_assets;
		using namespace properties::registry_v1;
		std::string constructed_name;
		const char* prefix = nullptr;

		#define PUT_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_number(constructed_name.c_str(), y);

		#define PUT_BOOL(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			assets.put_bool(constructed_name.c_str(), y);
		#pragma endregion
		{
			prefix = csz_AxisTurn_;
			PUT_NUMBER(Control_Key, 5.0);
			PUT_NUMBER(Control_CurveIntensity, 3.0);
		}
	}
public:
	void load_script(Framework::Base::asset_manager& assets)
	{
		using namespace properties::registry_v1;
		m_assets = &assets;
		#if 1
		assets.put_bool(csz_Build_bypass_simulation, false);
		#else
		assets.put_bool(csz_Build_bypass_simulation, true);
		#endif
		//TestIndivualWheels();
		//TestCurivator();
		TestAndromeda();
		//This customizes to my AirFlo controller, add your own controller
		//especially if your axis assignments need to change
		Test_Controls_Airflo();
	}
};

script_loader::script_loader()
{
	m_script_loader = std::make_shared<script_loader_impl>();;
}
void script_loader::load_script(Framework::Base::asset_manager& assets)
{
	m_script_loader->load_script(assets);
}

}