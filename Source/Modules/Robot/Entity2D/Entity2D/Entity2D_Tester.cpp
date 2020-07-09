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

#include <iostream>
#include <math.h>
#include <assert.h>
#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"
//Use line plot in smart dashboard to confirm trapezoidal motion
#include "../../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include "Entity2D.h"
#pragma region _main_

void UpdateVariables(Module::Localization::Entity2D &entity)
{
	using namespace Module::Localization;
	Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
	Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
	double magnitude = velocity_normalized.normalize();
	SmartDashboard::PutNumber("CurrentVelocity",Meters2Feet(magnitude));
	Entity2D::Vector2D position = entity.GetCurrentPosition();
	SmartDashboard::PutNumber("X", Meters2Feet(position.x));
	SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
}

void tester(const char *csz_test, Module::Localization::Entity2D &entity)
{
	enum tests
	{
		eCurrent,
		eMoveForward,

	};

	const char * const csz_tests[] =
	{
		"current",
		"forward"
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
		entity.SetLinearVelocity_local(Feet2Meters(12.0), 0.0);
		for (size_t i = 0; i < 200; i++)
		{
			entity.TimeSlice(0.010);
			UpdateVariables(entity);
			Sleep(33);  //makes it work for the line plot
		}
		entity.SetLinearVelocity_local(0.0, 0.0); //stop
		for (size_t i = 0; i < 100; i++)
		{
			entity.TimeSlice(0.010);
			UpdateVariables(entity);
			Sleep(33);
		}
		break;
	}
}


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
		"test <test name or number>"
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

	Module::Localization::Entity2D entity;  //setup our entity now

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
			else if (!_strnicmp(input_line, "test", 4))
			{
				tester(str_1,entity);
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
