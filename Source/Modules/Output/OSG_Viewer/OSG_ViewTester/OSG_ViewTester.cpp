#include "stdafx.h"
#include <Windows.h>
#include "../OSG_Viewer/OSG_Viewer.h"
#include "../OSG_Viewer/SwerveRobot_UI.h"

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

namespace Robot_Tester
{

class SwerveRobotTest
{
private:
	SwerveRobot_UI m_Robot;
	SwerveRobot_UI::SwerveRobot_State m_current_state = {};
	OSG_Viewer *m_viewer = nullptr;
public:
	void init(OSG_Viewer &viewer)
	{
		m_viewer = &viewer; //cache to remove hook
		//Tell viewer to look at our scene for our object
		viewer.SetSceneCallback([&](void *rootNode, void *geode) { m_Robot.UpdateScene(geode, true); });
		//Anytime robot needs updates link it to our current state
		m_Robot.SetSwerveRobot_Callback([&]() {	return m_current_state;	});
		//When viewer updates a frame
		viewer.SetUpdateCallback(
			[&](double dTime_s)
		{
			//any updates can go here to the current state
			//--- here  (optional)
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
	SwerveRobot_UI::SwerveRobot_State &get_current_state_rw() 
	{ 
		return m_current_state; 
	}
};
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_2_RAD(x)		((x)*M_PI/180.0)
#define Feet2Meters(x)		((x)*0.3048)

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
	using namespace Robot_Tester;
	OSG_Viewer viewer_test;  //setup our viewer now
	viewer_test.init();
	SwerveRobotTest robot;
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
				viewer_test.Zoom(atof(str_1));
			}
			else if (!_strnicmp(input_line, "test", 4))
			{
				viewer_test.Test(atoi(str_1));
			}
			else if (!_strnicmp(input_line, "init", 4))
			{
				robot.init(viewer_test);
				viewer_test.StartStreaming();
			}
			else if (!_strnicmp(input_line, "start", 5))
			{
				viewer_test.StartStreaming();
			}
			else if (!_strnicmp(input_line, "stop", 5))
			{
				viewer_test.StopStreaming();
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
