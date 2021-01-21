#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Base/AssetManager.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include "../../../Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.h"
#include "../../../Modules/Input/JoystickConverter.h"
#include "../../../Modules/Input/AI_Input/AI_Input_Example.h"
#include "../../../Modules/Input/AI_Input/SmartDashboard_HelperFunctions.h"

#include "../../../Modules/Robot/Entity2D/Entity2D/Entity2D.h"
#include "../../../Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/SwerveRobot_UI.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/Keyboard_State.h"
#include "../../../Modules/Output/SmartDashboard_PID_Monitor.h"
#include "../../../Properties/script_loader.h"
#include "../../../Properties/RegistryV1.h"
#include "TeleAutonV2.h"

//We can specify the linking in code
//https://www.dropbox.com/s/bn7ffxb7yqinzme/OSG_Viewer_x64.zip?dl=0
//unzip this to here: .\Robot_Simulation\Source\Modules\Output\OSG_Viewer
//if done correctly an x64 folder will be in the OSG_Viewer folder, then you can link and the 
//custom build step will auto copy the dll's needed to run

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib,"../../../Modules/Output/OSG_Viewer/x64/Debug/OSG_Viewer.lib")
#else
#pragma comment(lib,"../../../Modules/Output/OSG_Viewer/x64/Release/OSG_Viewer.lib")
#endif
#endif
#pragma endregion

namespace Application
{

#pragma region _Test_Swerve_Viewer_

class Test_Swerve_Properties
{
private:
	#pragma region _member variables_
	Framework::Base::asset_manager m_properties;
	properties::script_loader m_script_loader;
	Module::Localization::Entity2D m_Entity;
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	Module::Input::dx_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Input::AI_Example m_Goal;
	Module::Robot::SwerveRobot m_robot;  //keep track of our intended velocities
	
	Module::Output::OSG_Viewer m_viewer;
	Module::Output::SwerveRobot_UI m_RobotUI;
	Module::Output::SwerveRobot_UI::uSwerveRobot_State m_current_state = {};

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	double m_dTime_s=0.016;  //cached so other callbacks can access
    Module::Input::Keyboard_State m_Keyboard;

	enum class game_mode
	{
		eAuton,
		eTele,
		eTest
	} m_game_mode= game_mode::eTele;

	struct DriveAxisAssignments
	{
		struct AxisEntry
		{
			size_t AxisIndex;
			size_t JoystickIndex;
			Module::Input::Analog_EventEntry Options = {
			0.3,   //filter dead zone
			1.0,   //additional scale
			1.0,   // curve intensity
			false  //is flipped
			};
		};
		AxisEntry Strafe = {0,0};
		AxisEntry Forward = { 1,0 };
		AxisEntry Turn = { 2,0 };
	} m_Axis;

	bool m_IsStreaming = false;
	bool m_FieldCentricDrive = false;
	#pragma endregion

	void UpdateVariables()
	{
		Module::Localization::Entity2D &entity = m_Entity;
		using namespace Module::Localization;
		const Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		SmartDashboard::PutNumber("Rotation Velocity", m_Entity.GetCurrentAngularVelocity());
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x));
		m_current_state.bits.Pos_m.x = position.x;
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y));
		m_current_state.bits.Pos_m.y = position.y;
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		//If it is angle acceleration is being setpoint driven we read this (e.g. AI controlled)
		if (m_robot.GetIsDrivenAngular())
			m_current_state.bits.IntendedOrientation = m_robot.Get_IntendedOrientation();
		else
		{
			//TeleOp controlled, point forward if we are rotating in place, otherwise, point to the direction of travel
			//Like with the kinematics if we are not moving we do not update the intended orientation (using this)
			//This is just cosmetic, but may be handy to keep for teleop
			if (!IsZero(linear_velocity.x + linear_velocity.y, 0.02))
				m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
			else if (!IsZero(entity.GetCurrentAngularVelocity()))
			{
				//point forward locally when rotating in place
				m_current_state.bits.IntendedOrientation = entity.GetCurrentHeading();
			}
		}
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		m_current_state.bits.Att_r = entity.GetCurrentHeading();
		//To make this interesting, we keep the SmartDashboard to show the intended velocities...
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
		const Module::Robot::SwerveVelocities &cv = m_robot.GetCurrentVelocities();
		const Module::Robot::SwerveVelocities &iv = m_robot.GetIntendedVelocities();
		const Module::Robot::SwerveVelocities &vo = m_robot.GetCurrentVoltages();

		SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(iv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(iv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(iv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(iv.Velocity.AsArray[3]));
		SmartDashboard::PutNumber("Wheel_fl_Voltage", vo.Velocity.AsArray[0]);
		SmartDashboard::PutNumber("Wheel_fr_Voltage", vo.Velocity.AsArray[1]);
		SmartDashboard::PutNumber("Wheel_rl_Voltage", vo.Velocity.AsArray[2]);
		SmartDashboard::PutNumber("Wheel_rr_Voltage", vo.Velocity.AsArray[3]);
		SmartDashboard::PutNumber("wheel_fl_Encoder", Meters2Feet(cv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("wheel_fr_Encoder", Meters2Feet(cv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("wheel_rl_Encoder", Meters2Feet(cv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("wheel_rr_Encoder", Meters2Feet(cv.Velocity.AsArray[3]));

		//For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
		//its not perfect, but it gives a good enough direction to tell (especially when going down)
		SmartDashboard::PutNumber("Swivel_fl_Voltage", vo.Velocity.AsArray[4]);
		SmartDashboard::PutNumber("Swivel_fr_Voltage", vo.Velocity.AsArray[5]);
		SmartDashboard::PutNumber("Swivel_rl_Voltage", vo.Velocity.AsArray[6]);
		SmartDashboard::PutNumber("Swivel_rr_Voltage", vo.Velocity.AsArray[7]);
		SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(cv.Velocity.AsArray[4]));
		SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(cv.Velocity.AsArray[5]));
		SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(cv.Velocity.AsArray[6]));
		SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(cv.Velocity.AsArray[7]));

		for (size_t i = 0; i < 8; i++)
			m_current_state.bits.SwerveVelocitiesFromIndex[i] = cv.Velocity.AsArray[i];
	}
	void GetInputSlice(double dTime_s)
	{
		using namespace Module::Input;
		using JoystickInfo = dx_Joystick::JoystickInfo;

		dx_Joystick::JoyState joyinfo[4] = {0}; //setup joy zero'd out

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		for (size_t i = 0 ; i < NoJoySticks; i++)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			m_joystick.read_joystick(i, joyinfo[i]);
		}
		if (m_game_mode==game_mode::eTele)
		{
			const DriveAxisAssignments& _ = m_Axis;
			//Check if we are being driven by some AI method, we override it if we have any angular velocity (i.e. some teleop interaction)
			const double AngularVelocity = (AnalogConversion(joyinfo[_.Turn.JoystickIndex].Axis.AsArray[_.Turn.AxisIndex], _.Turn.Options) + 
				m_Keyboard.GetState().bits.m_Z) * m_max_heading_rad;
			if (!IsZero(AngularVelocity) || (m_robot.GetIsDrivenAngular()==false))
				m_robot.SetAngularVelocity(AngularVelocity);

			//Get an input from the controllers to feed in... we'll hard code the x and y axis from both joy and keyboard
			//we simply combine them so they can work inter-changeably (e.g. keyboard for strafing, joy for turning)
			const double Forward = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo[_.Forward.JoystickIndex].Axis.AsArray[_.Forward.AxisIndex], _.Forward.Options) + m_Keyboard.GetState().bits.m_Y) * -1.0);
			const double Right = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo[_.Strafe.JoystickIndex].Axis.AsArray[_.Strafe.AxisIndex], _.Strafe.Options) + m_Keyboard.GetState().bits.m_X));

			//Check if we are being driven by some AI method, we override it if we have any linear velocity (i.e. some teleop interaction)
			//Note: this logic does not quite work right for keyboard if it uses the forward "sticky" button, but since this isn't a real
			//input I am not going to filter it out, but could if needed.  Also the fabs() ensures the forward and right do not cancel
			//each other out
			if (!IsZero(fabs(Forward) + fabs(Right)) || (m_robot.GetIsDrivenLinear() == false))
				m_FieldCentricDrive? m_robot.SetLinearVelocity_global(Forward, Right) : m_robot.SetLinearVelocity_local(Forward, Right);
		}
		else if (m_game_mode == game_mode::eAuton)
		{
			using namespace Framework::Base;
			Goal& goal = m_Goal.GetGoal();
			if (goal.GetStatus()==Goal::eActive)
				goal.Process(dTime_s);
		}
		//TODO autonomous and goals here
		//This comes in handy for testing, but could be good to stop robot if autonomous needs to stop
		if (joyinfo[0].ButtonBank[0] == 1)
			Reset();
	}

	void TimeSliceLoop(double dTime_s)
	{
		m_dTime_s = dTime_s;
		//Grab kinematic velocities from controller
		GetInputSlice(dTime_s);
		//Update the predicted motion for this time slice
		m_robot.SimulatorTimeSlice(dTime_s);
		m_robot.TimeSlice(dTime_s);
		m_Entity.TimeSlice(dTime_s);
		UpdateVariables();
	}

	void SetUpHooks(bool enable)
	{
		using namespace Module::Output;
		if (enable)
		{
			#pragma region _UI Callbacks_
			OSG_Viewer &viewer = m_viewer;
			viewer.SetSceneCallback([&](void *rootNode, void *geode) { m_RobotUI.UpdateScene(geode, true); });
			//Anytime robot needs updates link it to our current state
			m_RobotUI.SetSwerveRobot_Callback([&]() {	return m_current_state.bits; });
			//When viewer updates a frame
			viewer.SetUpdateCallback(
				[&](double dTime_s)
			{
				//Synthetic timing use while stepping through code
				//dTime_s = 0.01;
				//any updates can go here to the current state
				TimeSliceLoop(dTime_s);
				//give the robot its time slice to process them
				m_RobotUI.TimeChange(dTime_s);
			});
			viewer.SetKeyboardCallback(
				[&](int key, bool press)
			{
				//printf("key=%d press=%d\n", key, press);
				m_Keyboard.UpdateKeys(m_dTime_s, key, press);
				if (key == ' ' && press == false)
					Reset();
			});
			#pragma endregion
			#pragma region _Robot Entity Linking_
			//Now to link up the callbacks for the robot motion control:  Note we can link them up even if we are not using it
			m_robot.Set_UpdateGlobalVelocity([&](const Vec2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y(), new_velocity.x());
			});
			m_robot.Set_UpdateHeadingVelocity([&](double new_velocity)
			{	m_Entity.SetAngularVelocity(new_velocity);
			});
			m_robot.Set_GetCurrentPosition([&]() -> Vec2D
			{
				//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
				//its worth the hassle
				Vec2D ret = Vec2D(m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y);
				return ret;
			});
			m_robot.Set_GetCurrentHeading([&]() -> double
			{
				return m_Entity.GetCurrentHeading();
			});
			m_robot.SetExternal_Velocity_PID_Monitor_Callback(
				[](double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler)
				{
					Module::Output::Velocity_PID_Monitor(Voltage, CurrentVelocity, Encoder_Velocity, ErrorOffset, CalibratedScaler);
				});
			m_robot.SetExternal_Position_PID_Monitor_Callback(
				[](double Voltage, double Position, double PredictedPosition, double CurrentVelocity, double Encoder_Velocity, double ErrorOffset)
				{
					Module::Output::Position_PID_Monitor(Voltage, Position, PredictedPosition, CurrentVelocity, Encoder_Velocity, ErrorOffset);
				});

			#pragma endregion
			#pragma region _AI Input Robot linking_
			m_Goal.Set_GetCurrentPosition([&]() -> Vec2D
				{
					return m_robot.GetCurrentPosition();
				});
			m_Goal.Set_GetCurrentHeading([&]() -> double
				{
					return m_robot.GetCurrentHeading();
				});
			m_Goal.Set_DriveToLocation([&](double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
				{
					m_robot.DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
				});
			m_Goal.Set_SetIntendedOrientation([&](double intended_orientation, bool absolute)
				{
					m_robot.SetIntendedOrientation(intended_orientation, absolute);
				});
			#pragma endregion
		}
		else
		{
			m_RobotUI.SetSwerveRobot_Callback(nullptr);
			m_viewer.SetSceneCallback(nullptr);
			m_viewer.SetUpdateCallback(nullptr);
			m_viewer.SetKeyboardCallback(nullptr);

			m_robot.Set_UpdateGlobalVelocity(nullptr);
			m_robot.Set_UpdateHeadingVelocity(nullptr);
			m_robot.Set_GetCurrentPosition(nullptr);
			m_robot.Set_GetCurrentHeading(nullptr);
		}
	}
	void InitController(const char* prefix, DriveAxisAssignments::AxisEntry &val)
	{
		const Framework::Base::asset_manager *asset=&m_properties;
		using namespace ::properties::registry_v1;
		double ftest = 0.0;  //use to test if an asset exists
		std::string constructed_name;
		#define GET_NUMBER(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_number(constructed_name.c_str(), y);

		#define GET_SIZE_T(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_number_size_t(constructed_name.c_str(), y);

		#define GET_BOOL(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_bool(constructed_name.c_str(), y);
		
		GET_SIZE_T(Control_Key, val.AxisIndex);
		GET_SIZE_T(Control_Joy, val.JoystickIndex);
		GET_NUMBER(Control_FilterRange, val.Options.FilterRange);
		GET_NUMBER(Control_Multiplier, val.Options.Multiplier);
		GET_NUMBER(Control_CurveIntensity, val.Options.CurveIntensity);
		GET_BOOL(Control_IsFlipped, val.Options.IsFlipped);
	}
	void InitControllers()
	{
		using namespace ::properties::registry_v1;
		const char* prefix = csz_AxisStrafe_;
		InitController(prefix, m_Axis.Strafe);
		prefix = csz_AxisForward_;
		InitController(prefix, m_Axis.Forward);
		prefix = csz_AxisTurn_;
		InitController(prefix, m_Axis.Turn);
		//--------------Also need to redo m_maxspeed and m_max_heading_rad
		const Framework::Base::asset_manager* asset = &m_properties;
		using namespace ::properties::registry_v1;
		//assume properties will have none or both to keep code simple
		m_maxspeed = Meters2Feet(asset->get_number(csz_Motion2D_max_speed_linear,Feet2Meters(m_maxspeed)));
		m_max_heading_rad = asset->get_number(csz_Motion2D_max_speed_angular, m_max_heading_rad);
	}
public:
	Test_Swerve_Properties()
	{
		SetUpHooks(true);
	}
	void Reset()
	{
		m_Entity.Reset();
		m_robot.Reset();
		m_Keyboard.Reset();
	}

	void init()
	{
		m_joystick.Init();
		using namespace Module::Robot;
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;

		//Initialize these before calling reset (as the properties can dictate what to reset to)
		//TODO: (Maybe) we may put this loading in Start() like before, but will need to ensure code can handle this stress... that is 
		//optional to make life easier for scripting updates, but since the driver station can reset the robot code without rebooting 
		//the firmware this should be an adequate work-flow.  However, I may still do this for partial support, it depends on how
		//much tweaking is needed for PID type operations.
		//populate properties method here and invoke it, init can have properties sent as a parameter here
		m_script_loader.load_script(m_properties);
		InitControllers();
		m_robot.Init(&m_properties);
		m_viewer.init();
		m_RobotUI.Initialize();
		m_FieldCentricDrive = m_properties.get_bool(properties::registry_v1::csz_Drive_UseFieldCentric, false);

		Reset();  //for entity variables
	}

	void Start()
	{
		if (!m_IsStreaming)
		{
			m_IsStreaming = true;
			m_viewer.StartStreaming();
			//Give driver station a default testing method by invoking here if we are in test mode
			if (m_game_mode == game_mode::eTest)
			{
				int test_to_run = (int)Auton_Smart_GetSingleValue("AutonTest", 1.0);
				test(test_to_run);
			}
			if (m_game_mode == game_mode::eAuton)
				m_Goal.GetGoal().Activate();
		}
	}
	void Stop()
	{
		if (m_IsStreaming)
		{
			m_IsStreaming = false;
			if (m_game_mode == game_mode::eAuton)
				m_Goal.GetGoal().Terminate();
			m_viewer.StopStreaming();
		}
	}
	void SetGameMode(int mode)
	{
		//If we are leaving from autonomous while still streaming terminate the goal
		if ((m_IsStreaming) && (m_game_mode == game_mode::eAuton) && (game_mode::eAuton != (game_mode)mode))
			m_Goal.GetGoal().Terminate();

		printf("SetGameMode to ");
		bool IsValid = true;
		switch ((game_mode)mode)
		{
		case game_mode::eAuton:
			printf("Auton \n");
			//If we are entering into autonomous already streaming activate the goal
			if ((m_IsStreaming) && (m_game_mode != game_mode::eAuton))
				m_Goal.GetGoal().Activate();
			break;
		case game_mode::eTele:
			printf("Tele \n");
			break;
		case game_mode::eTest:
			printf("Test \n");
			break;
		default:
			printf("Unknown \n");
			IsValid = false;
			break;
		}
		if (IsValid)
			m_game_mode = (game_mode)mode;
	}
	void test(int test)
	{
		switch (test)
		{
		case 1:
			printf("Testing rotate 90\n");
			m_robot.SetIntendedOrientation(PI_2, false);
			//m_robot.SetIntendedOrientation(PI_2, true);
			break;
		case 2:
			m_robot.DriveToLocation(0.0, 0.0);  //simple drive home without managing orientation
			break;
		case 3:
			m_robot.DriveToLocation(0.0, 0.0, true, true, 0.0, false);  //drive facing robot in the direction
			break;
		case 4:
			m_robot.DriveToLocation(0.0, 0.0, true, false);  //hit home at full speed (see if it oscillates)
			break;
		case 5:
			m_robot.DriveToLocation(Feet2Meters(5.0), 0.0, false);  //move forward (whatever direction it is by 5 feet)
			break;
		default:
			printf("Test %d\n", test);
		}
	}
	~Test_Swerve_Properties()
	{
		//Make sure we are stopped
		Stop();
		m_robot.Shutdown(); //detach hooks early
		SetUpHooks(false);
	}
};
#pragma endregion

#pragma region _TeleOp Wrapper methods_
void TeleAuton_V2::Reset()
{
	m_Tele_Internal->Reset();
}
void TeleAuton_V2::init()
{
	m_Tele_Internal = std::make_shared<Test_Swerve_Properties>();
	m_Tele_Internal->init();
}
void TeleAuton_V2::Start()
{
	m_Tele_Internal->Start();
}
void TeleAuton_V2::Stop()
{
	m_Tele_Internal->Stop();
}
void TeleAuton_V2::Test(int test)
{
	m_Tele_Internal->test(test);
}
void TeleAuton_V2::SetGameMode(int mode)
{
	m_Tele_Internal->SetGameMode(mode);
}

#pragma endregion

}