// Windows Header Files:
#include <windows.h>
#include <direct.h>
#include <vfw.h>


#include <math.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

// Standard C++ support
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <bitset>
#include <map>
#include <algorithm>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>
#include <future>
#include <chrono>

#include <math.h>
#include <assert.h>

#define _CP_(x,...)

#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"
//Use line plot in smart dashboard to confirm trapezoidal motion
#include "../../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include "Entity1D.h"
#pragma region _Entity1D Tester_
namespace Module {
	namespace Localization
	{

class Entity1D_Tester
{
private:
	#pragma region _member variables_
	std::future<void> m_TaskState_TimeSliceLoop;  //Use future to monitor task status

	Entity1D m_Entity;
	bool m_Done = false;
	#pragma endregion

	void UpdateVariables()
	{
		Entity1D &entity = m_Entity;
		using namespace Module::Localization;
		SmartDashboard::PutNumber("CurrentVelocity", entity.GetCurrentVelocity());
		SmartDashboard::PutNumber("position", entity.GetCurrentPosition());
	}
	void TimeSliceLoop(DWORD SleepTime)
	{
		while (!m_Done)
		{
			//hard code the time slice delta so we can observe it slowly if needed
			m_Entity.TimeSlice(0.010);
			UpdateVariables();
			Sleep(SleepTime);
		}
	}
public:
	void Reset()
	{
		m_Entity.Reset();
	}
	void Start()
	{
		m_Done = false;
		//https://stackoverflow.com/questions/9094422/how-to-check-if-a-stdthread-is-still-running
		using namespace std::chrono_literals;
		//It's considered standard practice to wait for 0ms to obtain the current status without really waiting
		const bool in_flight = m_TaskState_TimeSliceLoop.valid() && m_TaskState_TimeSliceLoop.wait_for(0ms) != std::future_status::ready;
		DWORD time_interval = 10; //actual robot time easy to step calculations a bit slower
		//DWORD time_interval = 33;   //better timing with refresh
		//DWORD time_interval = 1000;  //debugging
		if (!in_flight)
			m_TaskState_TimeSliceLoop = std::async(std::launch::async, &Entity1D_Tester::TimeSliceLoop, this, time_interval);
		_CP_("in_flight =%d", in_flight);
	}
	void Stop()
	{
		m_Done = true;
		if (m_TaskState_TimeSliceLoop.valid())
		{
			//join... wait for task to close:
			size_t TimeOut = 0;
			while (m_TaskState_TimeSliceLoop.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready)
			{
				printf("waiting for m_TaskState_TimeSliceLoop to close %zd\n", TimeOut++);
			}
		}
	}
	~Entity1D_Tester()
	{
		//Make sure we are stopped
		Stop();
	}
	void Position(double position)
	{
		m_Entity.SetPosition(position);
	}
	void Turn(double clockwise)
	{
		m_Entity.SetVelocity(clockwise);
	}

	void tester(const char *csz_test)
	{
		Entity1D &entity = m_Entity;
		enum tests
		{
			eCurrent,
			eMoveForward,
			eRangeFlip,
			eRangeFlip2,
			eOppositeClose
		};

		const char * const csz_tests[] =
		{
			"current",
			"forward",
			"range",
			"range2",
			"close_flip"
		};

		int Test = atoi(csz_test);

		//if the first character is not a number then translate the string
		if (((csz_test[0] < '0') || (csz_test[0] > '9')) && (csz_test[0] != 0))
		{
			Test = -1;
			for (int i = 0; i < _countof(csz_tests); i++)
			{
				if (stricmp(csz_tests[i], csz_test) == 0)
				{
					Test = i;
					break;
				}
			}
			if (Test == -1)
			{
				printf("No match found.  Try:\n");
				for (size_t i = 0; i < _countof(csz_tests); i++)
					printf("%s, ", csz_tests[i]);
				printf("\n");
				return;
			}
		}

		switch (Test)
		{
		case eMoveForward:
		case eCurrent:
			//Setup the motion... for this we move forward whatever our heading is and do it locally
			entity.SetVelocity(Pi2);
			for (size_t i = 0; i < 200; i++)
			{
				entity.TimeSlice(0.010);
				UpdateVariables();
				Sleep(33);  //makes it work for the line plot
			}
			entity.SetVelocity(0.0); //stop
			for (size_t i = 0; i < 100; i++)
			{
				entity.TimeSlice(0.010);
				UpdateVariables();
				Sleep(33);
			}
			break;
		case eRangeFlip:
			//So here's a good example of why I have a tester, this allows me to unit test a stress that my code failed initially
			//now I can tag the actual stress here and step through the code with ease to ensure it works properly
			entity.Reset();
			entity.SetVelocity(-Pi);
			for (size_t i = 0; i < 200; i++)
				entity.TimeSlice(0.010);
			UpdateVariables();
			entity.SetVelocity(Pi);
			entity.TimeSlice(0.010);
			break;
		case eRangeFlip2:
			entity.Reset();
			entity.SetVelocity(Pi);
			for (size_t i = 0; i < 200; i++)
				entity.TimeSlice(0.010);
			UpdateVariables();
			entity.SetVelocity(-Pi);
			entity.TimeSlice(0.010);
			break;
		case eOppositeClose:
			entity.Reset();
			entity.SetVelocity(-0.01);
			for (size_t i = 0; i < 200; i++)
				entity.TimeSlice(0.010);
			UpdateVariables();
			entity.SetVelocity(0.01);
			entity.TimeSlice(0.010);
			break;
		}
	}
};
	}
}


#pragma endregion
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
		"reset\n"
		"test <test name or number>\n"
		"start\n"
		"stop\n"
		"heading <degrees>\n"
		"turn <degrees per second>\n"
		"Help (displays this)\n"
		"\nType \"Quit\" at anytime to exit this application\n"
	);
}

__inline void prompt()
{
	std::cout << ">";
}

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

	Module::Localization::Entity1D_Tester entity_test;  //setup our entity now

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
			else if (!_strnicmp(input_line, "reset", 5))
			{
				entity_test.Reset();
			}
			else if (!_strnicmp(input_line, "test", 4))
			{
				entity_test.tester(str_1);
			}
			else if (!_strnicmp(input_line, "start", 5))
			{
				entity_test.Start();
			}
			else if (!_strnicmp(input_line, "stop", 5))
			{
				entity_test.Stop();
			}
			else if (!_strnicmp(input_line, "heading", 5))
			{
				entity_test.Position(DEG_2_RAD(atof(str_1)));
			}
			else if (!_strnicmp(input_line, "turn", 4))
			{
				entity_test.Turn(DEG_2_RAD(atof(str_1)));
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
	SmartDashboard::init();
	CommandLineInterface();
	SmartDashboard::shutdown();
	return 0;
}
#pragma endregion
