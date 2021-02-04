#include "stdafx.h"
#include <Windows.h>
#include "../OSG_Viewer/OSG_Viewer.h"
#include "../OSG_Viewer/SwerveRobot_UI.h"
#include "../OSG_Viewer/Entity_UI.h"
#include "../OSG_Viewer/Keyboard_State.h"

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
		"init <--use to display entities\n"
		"start\n"
		"stop\n"
		"pos <feet x> <feet y> \n"
		"vel <index> <degrees>\n"
		"turn <degrees>\n"
		"heading <degrees>\n"
		"select <entity to manipulate>\n"
		"keyboard <turn on test=1>\n"
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

class EntityTest
{
private:
	Entity_UI m_Robot;
	Entity_UI::Entity_State m_current_state = {};
	bool m_TestKeyboard = false;
public:
	void init()
	{
		//Anytime robot needs updates link it to our current state
		m_Robot.SetEntity_Callback([&]() {	return m_current_state;	});
		//Good to go... now initialize the robot
		m_Robot.Initialize();
	}
	~EntityTest()
	{
		//clear hooks
		m_Robot.SetEntity_Callback(nullptr);
	}
	//Access the current state for testing
	Entity_UI::Entity_State &get_current_state_rw() 
	{ 
		return m_current_state; 
	}
	Entity_UI &As_EntityUI()
	{
		return m_Robot;
	}
	bool Get_TestingKeyboard() const
	{
		return m_TestKeyboard;
	}
	void Set_TestingKeyboard(bool test)
	{
		m_TestKeyboard = test;
	}
};

class SwerveRobotTest
{
private:
	SwerveRobot_UI m_Robot;
	SwerveRobot_UI::SwerveRobot_State m_current_state = {};
	OSG_Viewer *m_viewer = nullptr;
public:
	void init()
	{
		//Anytime robot needs updates link it to our current state
		m_Robot.SetSwerveRobot_Callback([&]() {	return m_current_state;	});
		//Good to go... now initialize the robot
		m_Robot.Initialize();
	}
	~SwerveRobotTest()
	{
		//clear hooks
		m_Robot.SetSwerveRobot_Callback(nullptr);
	}
	//Access the current state for testing
	SwerveRobot_UI::SwerveRobot_State &get_current_state_rw() 
	{ 
		return m_current_state; 
	}
	SwerveRobot_UI &As_SwerveRobot_UI()
	{
		return m_Robot;
	}
};

class EntityManager
{
private:
	EntityTest m_Entity;
	SwerveRobotTest m_Robot;
	OSG_Viewer *m_viewer = nullptr;
	Input::Keyboard_State m_Keyboard;
	double m_dTime_s=0.016;
	bool m_IsInit = false;  //cache state of this for early test which do not use the robot and entity objects
	void SetHooks(bool enable)
	{
		if (enable)
		{
			//Tell viewer to look at our scene for our object
			m_viewer->SetSceneCallback([&](void *rootNode, void *geode) 
			{ 
				if (!m_IsInit) 
					return;
				m_Robot.As_SwerveRobot_UI().UpdateScene(geode, true); 
				m_Entity.As_EntityUI().UpdateScene(geode, true);
			});
			//When viewer updates a frame
			m_viewer->SetUpdateCallback(
				[&](double dTime_s)
			{
				if (!m_IsInit)
					return;
				m_dTime_s = dTime_s; //for other callbacks to access
				{
					//any updates can go here to the current state
					//--- here  (optional)
					if (m_Entity.Get_TestingKeyboard())
					{
						//we can get an idea of how the keyboard state works by adding the state multipliers to the position
						//and attitude
						m_Entity.get_current_state_rw().Att_r = m_Keyboard.GetState().bits.m_Z * M_PI;
						m_Entity.get_current_state_rw().Pos_m.x = (m_Keyboard.GetState().bits.m_X * Feet2Meters(3.0)) + Feet2Meters(5);
						m_Entity.get_current_state_rw().Pos_m.y = m_Keyboard.GetState().bits.m_Y * Feet2Meters(3.0);
					}
				}
				//give the robot its time slice to process them
				m_Robot.As_SwerveRobot_UI().TimeChange(dTime_s);
				m_Entity.As_EntityUI().TimeChange(dTime_s);
			});
			m_viewer->SetKeyboardCallback(
				[&](int key, bool press)
			{
				//printf("key=%d press=%d\n", key, press);
				m_Keyboard.UpdateKeys(m_dTime_s, key, press);
			});
		}
		else
		{
			m_viewer->SetSceneCallback(nullptr);
			m_viewer->SetUpdateCallback(nullptr);
			m_viewer->SetKeyboardCallback(nullptr);
		}
	}
public:
	EntityManager(OSG_Viewer &viewer)
	{
		m_viewer = &viewer; //cache to remove hook
		SetHooks(true);
	}
	void init()
	{
		m_IsInit = true;
		m_Robot.init();
		m_Entity.init();
		m_Entity.get_current_state_rw().Pos_m.x = Feet2Meters(5);
	}
	//Allow to unwind early
	void ShutDown()
	{
		if (m_viewer)
		{
			SetHooks(false);
			m_viewer = nullptr;
		}
	}
	~EntityManager()
	{
		ShutDown();
	}
	EntityTest &As_EntityTest()
	{
		return m_Entity;
	}
	SwerveRobotTest &As_SwerveRobotTest()
	{
		return m_Robot;
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
	OSG_Viewer viewer_test;  //setup our viewer now
	viewer_test.init();
	EntityManager em(viewer_test);
	enum class selection
	{
		eRobot,
		eEntity
	} m_selection=selection::eRobot;
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
				em.init();
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
				switch (m_selection)
				{
				case selection::eRobot:
					em.As_SwerveRobotTest().get_current_state_rw().Pos_m.x = Feet2Meters(atof(str_1));
					em.As_SwerveRobotTest().get_current_state_rw().Pos_m.y = Feet2Meters(atof(str_2));
					break;
				case selection::eEntity:
					em.As_EntityTest().get_current_state_rw().Pos_m.x = Feet2Meters(atof(str_1));
					em.As_EntityTest().get_current_state_rw().Pos_m.y = Feet2Meters(atof(str_2));
					break;
				}
			}
			else if (!_strnicmp(input_line, "vel", 3))
			{
				if (m_selection == selection::eRobot)
				{
					size_t index = atoi(str_1);
					em.As_SwerveRobotTest().get_current_state_rw().SwerveVelocitiesFromIndex[index] = DEG_2_RAD(atof(str_2));
				}
				else
					printf("Current selection not supported\n");
			}
			else if (!_strnicmp(input_line, "turn", 4))
			{
				switch (m_selection)
				{
				case selection::eRobot:
					em.As_SwerveRobotTest().get_current_state_rw().Att_r = DEG_2_RAD(atof(str_1));
					break;
				case selection::eEntity:
					em.As_EntityTest().get_current_state_rw().Att_r = DEG_2_RAD(atof(str_1));
					break;
				}
			}
			else if (!_strnicmp(input_line, "heading", 6))
			{
				switch (m_selection)
				{
				case selection::eRobot:
					em.As_SwerveRobotTest().get_current_state_rw().IntendedOrientation = DEG_2_RAD(atof(str_1));
					break;
				case selection::eEntity:
					em.As_EntityTest().get_current_state_rw().IntendedOrientation = DEG_2_RAD(atof(str_1));
					break;
				}
			}
			else if (!_strnicmp(input_line, "select", 3))
			{
				size_t index = atoi(str_1);
				m_selection = (selection)index;
				const char* entity = "unknown";
				switch (m_selection)
				{
				case selection::eRobot:
					entity = "Swerve Robot";
					break;
				case selection::eEntity:
					entity = "Entity";
					break;
				}
				printf("selection=%s\n",entity);
			}
			else if (!_strnicmp(input_line, "keyboard", 3))
			{
				size_t index = atoi(str_1);
				em.As_EntityTest().Set_TestingKeyboard(index == 0 ? false : true);
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
