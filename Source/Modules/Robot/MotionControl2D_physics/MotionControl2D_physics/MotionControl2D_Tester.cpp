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

#include "MotionControl2D.h"
#pragma region _MotionControl2D Tester_
namespace Module {
	namespace Robot	{
		namespace Physics {
class MotionControl2D_Tester
{
private:
	#pragma region _member variables_
	std::future<void> m_TaskState_TimeSliceLoop;  //Use future to monitor task status

	MotionControl2D m_Entity;
	bool m_Done = false;
	#pragma endregion

	void UpdateVariables()
	{
		MotionControl2D &entity = m_Entity;
		using namespace Module::Robot::Physics;
		MotionControl2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		SmartDashboard::PutNumber("CurrentVelocity", Meters2Feet(magnitude));
		MotionControl2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("CurrentVelocity_yaw", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		SmartDashboard::PutNumber("yaw", RAD_2_DEG(entity.GetCurrentHeading()));
		SmartDashboard::PutNumber("heading", RAD_2_DEG(entity.Get_IntendedOrientation()));
		SmartDashboard::PutNumber("yaw_rate", RAD_2_DEG(entity.GetCurrentAngularVelocity()));
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
			m_TaskState_TimeSliceLoop = std::async(std::launch::async, &MotionControl2D_Tester::TimeSliceLoop, this, time_interval);
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
	~MotionControl2D_Tester()
	{
		//Make sure we are stopped
		Stop();
	}
	void Heading(double heading)
	{
		m_Entity.SetIntendedOrientation(heading);
	}
	void Turn(double clockwise)
	{
		m_Entity.SetAngularVelocity(clockwise);
	}

	void tester(const char *csz_test)
	{
		MotionControl2D &entity = m_Entity;
		enum tests
		{
			eCurrent,
			eMoveForward,
			eFlip180,
			eTurn90,
			eGlobalHeading
		};

		const char * const csz_tests[] =
		{
			"current",
			"forward",
			"flip180",
			"Turn90",
			"GlobalHeading"
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
			//Setup the motion... for this we move forward whatever our heading is and do it locally
			entity.SetLinearVelocity_local(Feet2Meters(12.0), 0.0);
			for (size_t i = 0; i < 200; i++)
			{
				entity.TimeSlice(0.010);
				UpdateVariables();
				Sleep(33);  //makes it work for the line plot
			}
			entity.SetLinearVelocity_local(0.0, 0.0); //stop
			for (size_t i = 0; i < 100; i++)
			{
				entity.TimeSlice(0.010);
				UpdateVariables();
				Sleep(33);
			}
			break;

		//The follow stress tests show why physics are needed, because it is not so easy to compute the resulting velocity
		//Added that there is a change in direction.  It may be possible to solve without physics which can be documented
		//from within the entity implementation, we can swap different techniques for these same tests here
		case eFlip180:
			entity.Reset();
			entity.SetLinearVelocity_local(Feet2Meters(12.0), 0.0);
			for (size_t i = 0; i < 200; i++)
				entity.TimeSlice(0.010);
			UpdateVariables();
			entity.SetLinearVelocity_local(Feet2Meters(-12.0), 0.0);
			entity.TimeSlice(0.010);
			break;
		case eTurn90:
			entity.Reset();
			entity.SetLinearVelocity_local(Feet2Meters(12.0), 0.0);
			for (size_t i = 0; i < 200; i++)
				entity.TimeSlice(0.010);
			UpdateVariables();
			entity.SetLinearVelocity_local(0.0,Feet2Meters(12.0));
			entity.TimeSlice(0.010);
			break;
		case eGlobalHeading:
		case eCurrent:
			for (size_t i = 0; i < 8; i++)
			{
				entity.SetAngularVelocity(Pi2);
				entity.TimeSlice(0.033); //update a time slice
				printf("heading=%.2f\n", RAD_2_DEG(entity.GetCurrentHeading()));
			}
			entity.SetAngularVelocity(0.0);
			for (size_t i = 0; i < 16; i++)
			{
				entity.SetLinearVelocity_local(1.0, 0.0);  //simple move forward
				entity.TimeSlice(0.010); //update a time slice
				printf("position=%.2f, x=%.2f\n", Meters2Feet(entity.GetCurrentPosition().y), Meters2Feet(entity.GetCurrentPosition().x));
			}
			entity.Reset();
			break;
		}
	}
};

}}}


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

	Module::Robot::Physics::MotionControl2D_Tester entity_test;  //setup our entity now

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
				entity_test.Heading(DEG_2_RAD(atof(str_1)));
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
