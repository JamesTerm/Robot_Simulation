#pragma region _includes macros_
#include "stdafx.h"
#include "DriverStation.h"
#include <algorithm>
#include <functional>

#pragma comment (lib,"winmm")
#pragma comment (lib,"Shlwapi.lib")
#include <timeapi.h>
#include <shlwapi.h>
//#include "../Robot_Tester.h"
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Base/Event.h"
#include "../../../Base/EventMap.h"
#include "../../../Base/Script.h"
//#include "../../../Base/Joystick.h"
//#include "../../../Base/JoystickBinder.h"
#include "Keyboard.h"
#include "Robot_Tester.h"
#pragma endregion

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

//Since the lambda cannot capture, we must give it access to the robot here
RobotTester *s_pRobotTester = nullptr;  
//void BindRobot(RobotTester &_robot_tester);  //forward declare
void SetupPreferences();
Keyboard *s_Keyboard = nullptr;

__inline void GetParentDirectory(std::wstring &path)
{
	path = (path.substr(0, path.find_last_of(L"/\\")).c_str());
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	{ // Three magic lines of code
		TIMECAPS TimeCaps_;
		timeGetDevCaps(&TimeCaps_, sizeof(TimeCaps_));
		timeBeginPeriod(TimeCaps_.wPeriodMin);
	}

	//Well this is a bit painful, but we must ensure our path is aligned to obtain the LUA file
	{
		std::wstring path;
		wchar_t Buffer[MAX_PATH];
		GetModuleFileName(nullptr, Buffer, MAX_PATH);
		PathRemoveFileSpecW(Buffer);
		path = Buffer;
		GetParentDirectory(path);
		GetParentDirectory(path);
		SetCurrentDirectoryW(path.c_str());
	}
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// TODO: Place code here.

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_DRIVERSTATION, szWindowClass, MAX_LOADSTRING);
	//MyRegisterClass(hInstance);

	const HWND pParent = nullptr;  //just to show what the parameter is

	// Display the main dialog box.  I've always wanted to put the callback in the same place!
	HWND m_hDlg = CreateDialogW(hInstance, MAKEINTRESOURCE(IDD_DriveStation1), pParent,
		[](HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		INT_PTR ret = FALSE;
		UNREFERENCED_PARAMETER(lParam);
		switch (message)
		{
		case WM_INITDIALOG:
			//TO avoid needing to read from the tester... match the current state against the current default
			CheckDlgButton(hWnd, IDC_Tele, BM_SETCHECK);
			CheckDlgButton(hWnd, IDC_Stop, BM_SETCHECK);
			//Button_SetState(hWnd, IDStop, BM_CLICK, true, 0);
			return (INT_PTR)TRUE;
		case WM_COMMAND:
		{
			int wmId = LOWORD(wParam);
			// Parse the menu selections:
			switch (wmId)
			{
			case IDC_Start:
				//DebugBreak();
				OutputDebugStringW(L"Start\n");
				//printf("start");
				//SetupPreferences();
				s_pRobotTester->StartStreaming();
				CheckDlgButton(hWnd, IDC_Stop, BST_UNCHECKED);
				CheckDlgButton(hWnd, IDC_Start, BM_SETCHECK);
				break;
			case IDC_Stop:
				OutputDebugStringW(L"Stop\n");
				//printf("stop");
				s_pRobotTester->StopStreaming();
				CheckDlgButton(hWnd, IDC_Stop, BM_SETCHECK);
				CheckDlgButton(hWnd, IDC_Start, BST_UNCHECKED);
				break;
			case IDC_Tele:
			case IDC_Auton:
			case IDC_Test:
			{
				CheckDlgButton(hWnd, IDC_Tele, BST_UNCHECKED);
				CheckDlgButton(hWnd, IDC_Auton, BST_UNCHECKED);
				CheckDlgButton(hWnd, IDC_Test, BST_UNCHECKED);
				int game_mode = 0;
				switch (wmId)
				{
				case IDC_Auton: 
					CheckDlgButton(hWnd, IDC_Auton, BM_SETCHECK);
					game_mode = 0;
					break;
				case IDC_Tele: 
					CheckDlgButton(hWnd, IDC_Tele, BM_SETCHECK);
					game_mode = 1;
					break;
				case IDC_Test: 
					CheckDlgButton(hWnd, IDC_Test, BM_SETCHECK);
					game_mode = 2; 
					break;
				}
				s_pRobotTester->SetGameMode(game_mode);
				break;
			}
			case IDM_EXIT:
			case IDCANCEL:  //Not sure why this 2 is the same as the close button
				printf("Shutting down\n");
				DestroyWindow(hWnd);
				break;
			default:
				ret=DefWindowProc(hWnd, message, wParam, lParam);
			}
		}
		break;
		//May want to call a destructor here
		//case WM_CLOSE:
		//	DestroyWindow(hWnd);
		//	break;
		//Gah it never makes it this far... but keeping here for testing why
		case WM_KEYUP:
			OutputDebugStringW(L"KeyPressedUp\n");
			if (s_Keyboard)
				s_Keyboard->KeyPressRelease((int)wParam, false);
			break;
		case WM_KEYDOWN:
			OutputDebugStringW(L"KeyPressedDN\n");
			if (s_Keyboard)
				s_Keyboard->KeyPressRelease((int)wParam, true);
			break;
		case WM_PAINT:
		{
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hWnd, &ps);
			// TODO: Add any drawing code that uses hdc here...
			EndPaint(hWnd, &ps);
		}
		break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
			//default:
			//    return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return ret;
	}
	);

	if (!m_hDlg)
	{
		return FALSE;
	}

	//We made it this far... start up the robot
	RobotTester _robot_tester;
	s_pRobotTester = &_robot_tester;
	_robot_tester.RobotTester_create();
	//Bind robot for Keyboard binding
	//BindRobot(_robot_tester);
	_robot_tester.RobotTester_init();
	ShowWindow(m_hDlg, nCmdShow);
	UpdateWindow(m_hDlg);

	//Note dealing with accelerator tables :P
	//HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_DRIVERSTATION));

	// Main message loop:
	// Process any pending queue items
	MSG msg;
	BOOL bRet;
	while ((bRet = ::GetMessage(&msg, nullptr, 0, 0)) != 0)
	{
		if ((msg.message == WM_KEYDOWN)|| (msg.message == WM_KEYUP))
		{
			bool ProcessKey = true;
			WORD ascii = 0;
			//Translate WM_Key messages to ascii
			//https://stackoverflow.com/questions/44660035/how-to-extract-the-character-from-wm-keydown-in-pretranslatemessagemsgpmsg
			{
				const int wParam =(int) msg.wParam;
				const int lParam = (int)msg.lParam;
				const int keyboardScanCode = (lParam >> 16) & 0x00ff;
				const int virtualKey = wParam;

				BYTE keyboardState[256];
				GetKeyboardState(keyboardState);

				const int len = ToAscii(virtualKey, keyboardScanCode, keyboardState, &ascii, 0);
				ProcessKey = (len == 1);  //only simple keys
			}
			//OutputDebugStringW(L"KeyPressed\n");
			if ((ProcessKey)&&(s_Keyboard))
				s_Keyboard->KeyPressRelease((int)ascii, msg.message == WM_KEYDOWN);
		}
		else if (bRet == -1)
		{	// Finished
			assert(false);
			break;
		}
		else
		{	// Process the message
			::TranslateMessage(&msg);
			::DispatchMessage(&msg);
		}
	}


	_robot_tester.Shutdown();

	return (int)msg.wParam;
}

#if 0
//To bind the keyboard we'll need to access RobotAssem and Eventmap
//#include "../../main/cpp/Common/Entity_Properties.h"
//#include "../../main/cpp/Common/Physics_1D.h"
//#include "../../main/cpp/Common/Physics_2D.h"
//#include "../../main/cpp/Common/Entity2D.h"
//#include "../../main/cpp/Common/Goal.h"
//#include "../../main/cpp/Common/Ship_1D.h"
//#include "../../main/cpp/Common/Ship.h"
//
//#include "../../main/cpp/Config/ActiveCollection.h"
//#include "../../main/cpp/RobotAssem.h"
//#include "../../main/cpp/Common/SmartDashboard.h"
//RobotAssem *s_RobotContainer=nullptr;
Keyboard *s_StagedKeyboard = nullptr;


//TODO find a place for this class
class RobotCommon
{
private:
	Entity2D_Kind::EventMap* m_eventMap;
protected:
	virtual void Initialize(Entity2D_Kind::EventMap& em, const Entity_Properties *props = NULL);
public:
	Entity2D_Kind::EventMap* GetEventMap();
};


void BindKeyboard()
{
	if (s_RobotContainer)
	{
		if (s_Keyboard)
		{
			assert(false);  //recoverable but need to see use-case
			delete s_Keyboard;
		}
		RobotCommon *_pRobot = s_RobotContainer->GetRobot();
		s_StagedKeyboard->SetEventMap(_pRobot->GetEventMap());
		s_Keyboard = s_StagedKeyboard;
		s_StagedKeyboard = nullptr;
	}
	else
	{
		delete s_Keyboard;
		s_Keyboard = nullptr;
	}

}

void SetupPreferences()
{
	bool DevMode = SmartDashboard::GetBoolean("DevMode", false);
	if (DevMode)
	{
		//assert control defaults
		SmartDashboard::SetDefaultBoolean("Show_Controls", false);
		SmartDashboard::SetDefaultBoolean("Test_Samples", false);
	}
	const bool Test_Samples = SmartDashboard::GetBoolean("Test_Samples", false);
	const bool Show_Controls = SmartDashboard::GetBoolean("Show_Controls", false);
	assert(s_pRobotTester);
	s_pRobotTester->ShowControls(Show_Controls);
	if (s_RobotContainer) 
		s_pRobotTester->HookSampleGoals(Test_Samples);
}

void BindRobot(RobotTester &_robot_tester)
{
	_robot_tester.RobotTester_SetParentBindCallback(
		[](RobotAssem *instance, bool PropertiesBound)
	{
		s_RobotContainer = instance;
		if (!PropertiesBound)
		{
			SetupPreferences();
			s_StagedKeyboard = new Keyboard;
			s_StagedKeyboard->Keyboard_init(nullptr);
			using namespace Framework::UI;
			//Now to bind the lua calls
			KeyboardMouse_CB::SetKeyboardSupport_Add(
				[](Framework::Base::Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey)
			{
				return s_StagedKeyboard->AddKeyBinding(key, eventName, useOnOff, ForceBindThisKey);
			});
			KeyboardMouse_CB::SetKeyboardSupport_Remove(
				[](Framework::Base::Key key, const std::string eventName, bool useOnOff)
			{
				s_StagedKeyboard->RemoveKeyBinding(key, eventName, useOnOff);
			});
		}
		else
		{
			BindKeyboard();
		}
	}
	);
}
#endif