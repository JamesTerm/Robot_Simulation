#include "stdafx.h"
#include "dx_Joystick.h"
// Converts a GUID to a string
inline void GUIDtoa(GUID id, wchar_t *string,size_t buffer_size) {
	swprintf_s(string, buffer_size, L"%x-%x-%x-%x%x-%x%x%x%x%x%x", id.Data1, id.Data2, id.Data3,
		id.Data4[0], id.Data4[1], id.Data4[2], id.Data4[3], id.Data4[4], id.Data4[5], id.Data4[6], id.Data4[7]);
}

HWND FindConsoleHandle()
{
	GUID ID;
	const size_t buffer_size = 512;
	TCHAR title[buffer_size];
	TCHAR tempGUIDtitle[buffer_size];
	LPCTSTR temptitle;
	if (FAILED(CoCreateGuid(&ID)))
		return NULL;
	GUIDtoa(ID, tempGUIDtitle, buffer_size);
	temptitle = tempGUIDtitle;
	if (GetConsoleTitle(title, sizeof(title) / sizeof(TCHAR)) == 0)
		return NULL; //doh
	SetConsoleTitle(temptitle);
	HWND wnd = FindWindow(NULL, temptitle);
	SetConsoleTitle(title);
	return wnd;
}


int main(int argc, char *argv[])
{
	using namespace Module::Input;
	using JoystickInfo = dx_Joystick::JoystickInfo;
	size_t JoyNum = 0;
	if (argc == 2)
		JoyNum = atoi(argv[1]);
	dx_Joystick JoyStick;
	//I didn't need the HWND but added for completion
	JoyStick.Init(FindConsoleHandle());

	size_t NoJoySticks = JoyStick.GetNoJoysticksFound();
	if (NoJoySticks)
	{
		printf("Buttons: 1=next joystick, 2=exit, 3=enum axis\n");
		for (size_t i = 0; i < NoJoySticks; i++)
		{
			dx_Joystick::JoystickInfo info = JoyStick.GetJoyInfo(i);
			assert(info.bPresent);
			printf("----------------------------------------\n");
			printf("Joystick %ls Caps:\n", info.ProductName.c_str());
			printf("Sliders=%d Povs=%d Buttons=%d\n", info.nSliderCount, info.nPOVCount, info.nButtonCount);
			if (info.JoyCapFlags & JoystickInfo::fX_Axis)
				printf("X Axis, ");
			if (info.JoyCapFlags & JoystickInfo::fY_Axis)
				printf("Y Axis, ");
			if (info.JoyCapFlags & JoystickInfo::fZ_Axis)
				printf("Z Axis, ");
			if (info.JoyCapFlags & JoystickInfo::fX_Rot)
				printf("X Rot, ");
			if (info.JoyCapFlags & JoystickInfo::fY_Rot)
				printf("Y Rot, ");
			if (info.JoyCapFlags & JoystickInfo::fZ_Rot)
				printf("Z Rot, ");
			printf("\n----------------------------------------\n");
		}
		dx_Joystick::JoyState joyinfo;
		memset(&joyinfo, 0, sizeof(dx_Joystick::JoyState));
		bool done = false;
		size_t Display = 0;
		while (!done)
		{
			if (JoyStick.read_joystick(JoyNum, joyinfo))
			{
				switch (Display)
				{
				case 0:
					printf("\r xaxis=%f yaxis=%f zaxis=%f button=0x%x             ", joyinfo.Axis.Named.lX, joyinfo.Axis.Named.lY, joyinfo.Axis.Named.lZ, joyinfo.ButtonBank[0]);
					break;
				case 1:
					printf("\r xRot=%f yRot=%f zRot=%f POV=%f               ", joyinfo.Axis.Named.lRx, joyinfo.Axis.Named.lRy, joyinfo.Axis.Named.lRz, joyinfo.Axis.Named.rgPOV[0]);
					break;
				case 2:
					printf("\r slider0=%f slider1=%f                              ", joyinfo.Axis.Named.rgSlider[0], joyinfo.Axis.Named.rgSlider[1]);
					break;
				case 3:
					break;
				}

				if (joyinfo.ButtonBank[0] == 1)
				{
					JoyNum++;
					if (JoyNum >= NoJoySticks)
						JoyNum = 0;
				}
				if (joyinfo.ButtonBank[0] == 2)
					done = true;
				if (joyinfo.ButtonBank[0] == 4)
				{
					Display++;
					if (Display > 2)
						Display = 0;
					Sleep(200); //hack a way to toggle easy
				}
			}
			Sleep(22); //about 30 times a sec
		}
	}
	else
	{
		printf("None found\n");
		Sleep(2000);
	}
	return 0;
}

#pragma region _original auto code_

// dx_Joystick_Controller.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include <iostream>

//int main()
//{
//    std::cout << "Hello World!\n";
//}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
#pragma endregion