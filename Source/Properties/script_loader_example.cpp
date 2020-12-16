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
				{"sFL_","sFR_","sRL_","sRR_"},
				{"aFL_","aFR_","aRL_","aRR_"}
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
		#pragma region _LUA converstion_
		//Not the fastest robot, but is the most tested and great base line
		static const double Inches2Meters = 0.0254;
		static const double Feet2Meters = 0.3048;
		static const double Meters2Feet = 3.2808399;
		static const double Meters2Inches = 39.3700787;
		static const double OunceInchToNewton = 0.00706155183333;
		static const double Pounds2Kilograms = 0.453592;
		static const double Deg2Rad = (1.0 / 180.0) * Pi;

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
			bool is_closed = EncoderLoop;
			bool show_pid_dump = false;
			//ds_display_row = -1;
			struct pid { double p = 200; double i = 0; double d = 25; };
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
			//ds_display_row = -1;
			const bool use_pid_up_only = true;
			struct pid_up { double p = 100; double i = 0; double d = 25; };
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
	}
public:
	void load_script(Framework::Base::asset_manager& assets)
	{
		using namespace properties::registry_v1;
		m_assets = &assets;
		#if 0
		assets.put_bool(csz_Build_bypass_simulation, false);
		#else
		assets.put_bool(csz_Build_bypass_simulation, true);
		#endif
		//TestIndivualWheels();
		TestCurivator();
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