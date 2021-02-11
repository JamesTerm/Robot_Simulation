#include "stdafx.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include <Windows.h>
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.h"
#include "../../../Modules/Output/OSG_Viewer/OSG_Viewer/SwerveRobot_UI.h"
//helpful for interpolation of position
#include "../../../Base/Vec2d.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_2_RAD(x)		((x)*M_PI/180.0)
#define Feet2Meters(x)		((x)*0.3048)

#pragma region _main_

void cls(HANDLE hConsole = NULL)
{
	if (!hConsole)
		hConsole = ::GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coordScreen = { 0, 0 }; /* here's where we'll home the cursor */
	BOOL bSuccess;
	DWORD cCharsWritten;
	CONSOLE_SCREEN_BUFFER_INFO csbi; /* to get buffer info */
	DWORD dwConSize; /* number of character cells in the current buffer */
	/* get the number of character cells in the current buffer */
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "GetConsoleScreenBufferInfo");   
	dwConSize = csbi.dwSize.X * csbi.dwSize.Y;   /* fill the entire screen with blanks */
	bSuccess = FillConsoleOutputCharacter(hConsole, (TCHAR) ' ',
		dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputCharacter");   /* get the current text attribute */   
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "ConsoleScreenBufferInfo");   /* now set the buffer's attributes accordingly */   
	bSuccess = FillConsoleOutputAttribute(hConsole, csbi.wAttributes, dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputAttribute");   /* put the cursor at (0, 0) */   
	bSuccess = SetConsoleCursorPosition(hConsole, coordScreen);
	//PERR(bSuccess, "SetConsoleCursorPosition");   return; 
}

static void DisplayHelp()
{
	printf(
		"cls\n"
		"Zoom <size usually 100>\n"
		"test <test name or number>\n"
		"start\n"
		"stop\n"
		"pos <feet x> <feet y> \n"
		"vel <index> <degrees>\n"
		"turn <degrees>\n"
		"heading <degrees>\n"
		"Help (displays this)\n"
		"\nType \"Quit\" at anytime to exit this application\n"
	);
}

__inline void prompt()
{
	std::cout << ">";
}

namespace Module
{
	namespace Output {


class SwerveRobotTest
{
private:
	SwerveRobot_UI m_Robot;
	SwerveRobot_UI::uSwerveRobot_State m_current_state = {};
	OSG_Viewer *m_viewer = nullptr;
	std::function<void(double dTime_s)> m_UpdateCallback=nullptr;
public:
	void SetUpdateCallback(std::function<void(double dTime_s)> callback)
	{
		m_UpdateCallback = callback;
	}
	void init(OSG_Viewer &viewer)
	{
		m_viewer = &viewer; //cache to remove hook
		//Tell viewer to look at our scene for our object
		viewer.SetSceneCallback([&](void *rootNode, void *geode) { m_Robot.UpdateScene(geode, true); });
		//Anytime robot needs updates link it to our current state
		m_Robot.SetSwerveRobot_Callback([&]() {	return m_current_state.bits; });
		//When viewer updates a frame
		viewer.SetUpdateCallback(
			[&](double dTime_s)
		{
			//any updates can go here to the current state
			if (m_UpdateCallback)
				m_UpdateCallback(dTime_s);
			//give the robot its time slice to process them
			m_Robot.TimeChange(dTime_s);
		});
		//Good to go... now initialize the robot
		m_Robot.Initialize();
	}
	~SwerveRobotTest()
	{
		//clear hooks
		if (m_viewer)
		{
			m_Robot.SetSwerveRobot_Callback(nullptr);
			m_viewer->SetSceneCallback(nullptr);
			m_viewer->SetUpdateCallback(nullptr);
			m_viewer = nullptr;  //pedantic... we are done with it
		}
	}
	//Access the current state for testing
	SwerveRobot_UI::uSwerveRobot_State &get_state_as_union()
	{
		return m_current_state;
	}
	SwerveRobot_UI::SwerveRobot_State &get_current_state_rw()
	{ 
		return m_current_state.bits; 
	}
};

__inline void Smart_GetMultiValue(size_t NoItems, const char * const SmartNames[], double * const SmartVariables)
{
	for (size_t i = 0; i < NoItems; i++)
	{
		try
		{
			SmartVariables[i] = SmartDashboard::GetNumber(SmartNames[i]);
		}
		catch (...)
		{
			//I may need to prime the pump here
			SmartDashboard::PutNumber(SmartNames[i], SmartVariables[i]);
		}
	}
}

class SmartDashboard_Control
{
	#pragma region _description_
	//It was a bit challenging but this is completely decoupled from SwerveRobotTest, this makes it possible
	//to link different kind of robot objects, we may have a different update method to match it
	#pragma endregion
public:
	enum class view_mode
	{
		eUsing_PredictionVariables, //Using advanced odometry to get accurate localization
		eUsing_EncoderVariables,  //Using only encoders (will not pick up skids and slips)
		eUsign_IntendedVelocity  //Older assemblies need this
	};
private:
	view_mode m_ViewMode = view_mode::eUsing_PredictionVariables;
	class Interpolate_Position
	{
		//Newer SmartDashboard may intensionally update slower than our framerate when this happens the position looks jerky
		//this will help interpolate the in-between moments of time
	private:
		SwerveRobot_UI::uSwerveRobot_State m_last_smart_state;
		bool m_Bypass=false;
	public:
		void SetInterpolatePosition(bool interpolate_position)
		{
			m_Bypass = !interpolate_position;
		}
		SwerveRobot_UI::uSwerveRobot_State &operator() (SwerveRobot_UI::uSwerveRobot_State smart_state,double dTime_s)
		{
			//only interpolate if our times are under 20ms
			if ((!m_Bypass)&&(dTime_s<0.2))
			{
				//To start at this point we have the last state and our time delta (around 16ms)
				//finally we need our velocities from SmartDashboard
				//The last velocity is pure magnitude which we can use to gauge tolerance
				const char* const SmartNames[] = { "linear_velocity_x","linear_velocity_y","Rotation Velocity","Velocity" };
				double velocity[4] = { 0 };
				Smart_GetMultiValue(4, SmartNames, &velocity[0]);
				//Now we have everything we need to interpolate
				//first we'll interpolate the position
				using namespace Framework::Base;
				//as x , y
				Vec2D position(m_last_smart_state.bits.Pos_m.x,m_last_smart_state.bits.Pos_m.y);
				const Vec2D global_velocity(velocity[0], velocity[1]);
				const Vec2D global_pos_delta = global_velocity * dTime_s;  //how much we increment right now
				//Now we can compute this position. Offset it with last state so we can average it with the current
				position += global_pos_delta;
				//So we have 2 positions, the last state predicted, and the current state of smart dashboard... 
				//To keep smooth only apply this when we exceed a tolerance
				const Vec2D smart_pos(smart_state.bits.Pos_m.x, smart_state.bits.Pos_m.y);
				const double error = (smart_pos - position).length();
				//While position doesn't have to be perfect, having the tolerance scaled to velocity ensure's 
				//it will not glitch on slower speeds (which would be more noticeable
				const double tolerance = fabs(velocity[3]) * 0.05; 
				//SmartDashboard::PutNumber("Test", tolerance);
				const Vec2D new_pos = error > tolerance ? smart_pos : position;
				//const Vec2D new_pos = position;  //testing
				//commit this adjustment
				smart_state.bits.Pos_m.x = new_pos.x();
				smart_state.bits.Pos_m.y = new_pos.y();
				//Now for the heading more of the same
				const double heading_delta = velocity[2] * dTime_s;
				double predicted_heading = m_last_smart_state.bits.Att_r + heading_delta;
				const double smart_heading = smart_state.bits.Att_r;
				//Unlike position, the rotation needs to be perfect when there is no angular velocity, so we scale the tolerance to it
				const double tolerance_rot = 0.2 * fabs(velocity[2] * 0.5);
				//SmartDashboard::PutNumber("Test", tolerance_rot);
				const double new_heading = fabs(smart_heading - predicted_heading) > tolerance_rot ? smart_heading : predicted_heading;
				//commit this
				smart_state.bits.Att_r = new_heading;
			}
			m_last_smart_state = smart_state;
			return m_last_smart_state;
		}
	} m_interpolate;
public:
	//Note: to do nothing in the constructor gives client code the ability to instantiate it without using it or any penalty to instantiate it
	//and not need to consider it as a pointer
	void init(const char *IPAddress = "localhost")
	{
		SmartDashboard::SetClientMode();
		SmartDashboard::SetIPAddress(IPAddress);
		SmartDashboard::init();
	}
	//This particular update will directly work with the values passed in TeleOpV1
	void UpdateState_basic(SwerveRobot_UI::uSwerveRobot_State &state,double dTime_s)
	{
		SwerveRobot_UI::uSwerveRobot_State smart_state = {};
		//PSAI position, swerve velocities array, attitude, intended orientation
		const char * const SmartNames[] = { "X_ft","Y_ft",
			"Wheel_fl_Velocity","Wheel_fr_Velocity","Wheel_rl_Velocity","Wheel_rr_Velocity",
			"swivel_fl_Raw","swivel_fr_Raw","swivel_rl_Raw","swivel_rr_Raw",
			"Heading","Travel_Heading"
		};
		const char * const SmartNames_Encoder[] = { "X_ft","Y_ft",
			"wheel_fl_Encoder","wheel_fr_Encoder","wheel_rl_Encoder","wheel_rr_Encoder",
			"swivel_fl_Raw","swivel_fr_Raw","swivel_rl_Raw","swivel_rr_Raw",
			"Heading","Travel_Heading"
		};
		const char* const SmartNames_prediction[] = { "predicted_X_ft","predicted_Y_ft",
			"wheel_fl_Encoder","wheel_fr_Encoder","wheel_rl_Encoder","wheel_rr_Encoder",
			"swivel_fl_Raw","swivel_fr_Raw","swivel_rl_Raw","swivel_rr_Raw",
			"predicted_Heading","Travel_Heading"
		};

		double * const SmartVariables = &smart_state.raw.element[0];
		switch (m_ViewMode)
		{
		case view_mode::eUsing_PredictionVariables:
			Smart_GetMultiValue(12, SmartNames_prediction, SmartVariables);
			break;
		case view_mode::eUsing_EncoderVariables:
			Smart_GetMultiValue(12, SmartNames_Encoder, SmartVariables);
			break;
		case view_mode::eUsign_IntendedVelocity:
			Smart_GetMultiValue(12, SmartNames, SmartVariables);
			break;
		}
		//The values read in are in feet and degrees (human readable, we have to translate them back)
		//This is just written out so it's easy to follow and maintain
		using vi = SwerveRobot_UI::SwerveRobot_State;
		state.raw = 
		{
			Feet2Meters(smart_state.bits.Pos_m.x),
			Feet2Meters(smart_state.bits.Pos_m.y),
			Feet2Meters(smart_state.bits.SwerveVelocitiesFromIndex[vi::eWheel_FL]),
			Feet2Meters(smart_state.bits.SwerveVelocitiesFromIndex[vi::eWheel_FR]),
			Feet2Meters(smart_state.bits.SwerveVelocitiesFromIndex[vi::eWheel_RL]),
			Feet2Meters(smart_state.bits.SwerveVelocitiesFromIndex[vi::eWheel_RR]),
			DEG_2_RAD(smart_state.bits.SwerveVelocitiesFromIndex[vi::eSwivel_FL]),
			DEG_2_RAD(smart_state.bits.SwerveVelocitiesFromIndex[vi::eSwivel_FR]),
			DEG_2_RAD(smart_state.bits.SwerveVelocitiesFromIndex[vi::eSwivel_RL]),
			DEG_2_RAD(smart_state.bits.SwerveVelocitiesFromIndex[vi::eSwivel_RR]),
			DEG_2_RAD(smart_state.bits.Att_r),
			DEG_2_RAD(smart_state.bits.IntendedOrientation)
		};
		state = m_interpolate(state,dTime_s);
	}
	//Reserved make update method for voltage and physics simulation of encoders
	//void UpdateState_physics(SwerveRobot_UI::SwerveRobot_State &state)
	~SmartDashboard_Control()
	{
		SmartDashboard::shutdown();
	}
	void SetViewMode(view_mode mode)
	{
		m_ViewMode = mode;
	}
	void SetInterpolatePosition(bool interpolate_position)
	{
		m_interpolate.SetInterpolatePosition(interpolate_position);
	}
};

//We'll handle all the object binding in here
class OutputView_Tester
{
private:
	SmartDashboard_Control m_smart_control;
	OSG_Viewer *m_viewer = nullptr;
	SwerveRobotTest *m_robot;

	void SetUpHooks(bool enable)
	{
		if (enable)
		{
			//The robot has setup most the hooks we just need the update from the robot
			m_robot->SetUpdateCallback(	[&](double dTime_s)
				{	m_smart_control.UpdateState_basic(m_robot->get_state_as_union(),dTime_s);
				});
		}
		else
		{
			if (m_robot)
				m_robot->SetUpdateCallback(nullptr);
		}
	}
public:
	void init(OSG_Viewer &viewer, SwerveRobotTest &robot, const char *IpAddress)
	{
		//use m_viewer as a valve to know if we have init
		if (!m_viewer)
		{
			m_viewer = &viewer;
			m_robot = &robot;
			//Hook up our objects before init (if possible)
			SetUpHooks(true);

			//Init our robot and smart control
			m_smart_control.init(IpAddress);
			robot.init(viewer);
			//all set start the stream
			viewer.StartStreaming();
		}
	}
	void Test(size_t index)
	{
		assert(m_viewer);
		using view_mode = SmartDashboard_Control::view_mode;
		switch (index)
		{
		case 1:
			m_smart_control.SetViewMode(view_mode::eUsing_EncoderVariables);
			break;
		case 2:
			m_smart_control.SetViewMode(view_mode::eUsign_IntendedVelocity);
			break;
		case 3:
			m_smart_control.SetInterpolatePosition(false);
			break;
		default:
			m_smart_control.SetViewMode(view_mode::eUsing_PredictionVariables);
		}
	}
	~OutputView_Tester()
	{
		SetUpHooks(false);
	}
};

}}


bool CommandLineInterface()
{
	bool ret = false;
	#pragma region _CLI_SetUp_
	DisplayHelp();

	char		command[32];
	char		str_1[MAX_PATH];
	char		str_2[MAX_PATH];
	char		str_3[MAX_PATH];
	char		str_4[MAX_PATH];
	char		input_line[128];

	using namespace std;
	cout << endl;
	cout << "Ready." << endl;
	#pragma endregion
	using namespace Module::Output;
	OSG_Viewer viewer;  //setup our viewer now
	viewer.init();
	SwerveRobotTest robot;
	OutputView_Tester test;
	const char *default_ip = "localhost";
	//const char *default_ip = "192.168.1.55";

	while (prompt(), cin.getline(input_line, 128))
	{
		//init args
		command[0] = str_1[0] = str_2[0] = str_3[0] = str_4[0] = 0;
		if (sscanf(input_line, "%s %s %s %s %s", command, str_1, str_2, str_3, str_4) >= 1)
		{
			if (!_strnicmp(input_line, "cls", 3))
			{
				cls();
			}
			else if (!_strnicmp(input_line, "zoom", 4))
			{
				viewer.Zoom(atof(str_1));
			}
			else if (!_strnicmp(input_line, "test", 4))
			{
				const char *IpAddress = (str_1[0] == 0) || (str_1[0] == '0') ? default_ip : str_1;
				test.init(viewer, robot, IpAddress);
				test.Test(atoi(str_2));
			}
			else if (!_strnicmp(input_line, "init", 4))
			{
				const char *IpAddress = str_1[0] == 0 ? default_ip : str_1;
				test.init(viewer,robot,IpAddress);
			}
			else if (!_strnicmp(input_line, "start", 5))
			{
				viewer.StartStreaming();
			}
			else if (!_strnicmp(input_line, "stop", 5))
			{
				viewer.StopStreaming();
			}
			else if (!_strnicmp(input_line, "pos", 3))
			{
				robot.get_current_state_rw().Pos_m.x = Feet2Meters(atof(str_1));
				robot.get_current_state_rw().Pos_m.y = Feet2Meters(atof(str_2));
			}
			else if (!_strnicmp(input_line, "vel", 3))
			{
				size_t index = atoi(str_1);
				robot.get_current_state_rw().SwerveVelocitiesFromIndex[index] = DEG_2_RAD(atof(str_2));
			}

			else if (!_strnicmp(input_line, "turn", 4))
			{
				robot.get_current_state_rw().Att_r = DEG_2_RAD(atof(str_1));
			}
			else if (!_strnicmp(input_line, "heading", 6))
			{
				robot.get_current_state_rw().IntendedOrientation = DEG_2_RAD(atof(str_1));
			}
			else if (!_strnicmp(input_line, "Exit", 4))
			{
				break;
			}
			else if (!_strnicmp(input_line, "Help", 4))
				DisplayHelp();
			else if (!_strnicmp(input_line, "Quit", 4))
			{
				ret = true;
				break;
			}
			else
				cout << "huh? - try \"help\"" << endl;
		}
	}
	return ret;
}


int main()
{
	//SmartDashboard::init();
	CommandLineInterface();
	//SmartDashboard::shutdown();
	return 0;
}
#pragma endregion
