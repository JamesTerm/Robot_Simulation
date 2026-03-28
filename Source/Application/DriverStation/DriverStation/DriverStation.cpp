#pragma region _includes macros_
#include "stdafx.h"
#include "DriverStation.h"
#include <algorithm>
#include <functional>


#include <timeapi.h>
#include <shlwapi.h>
//#include "../Robot_Tester.h"
//#include "../../../Base/Base_Includes.h"
//#include "../../../Base/Vec2d.h"
//#include "../../../Base/Misc.h"
//#include "../../../Base/Event.h"
//#include "../../../Base/EventMap.h"
//#include "../../../Base/Script.h"
//#include "../../../Base/Joystick.h"
//#include "../../../Base/JoystickBinder.h"
//#include "Keyboard.h"
#include "NativeLink.h"
#include "Robot_Tester.h"
#pragma endregion

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name
HWND g_hDlg = nullptr;
HWND g_hNativeLinkCarrierLabel = nullptr;
HWND g_hNativeLinkCarrierCombo = nullptr;
HWND g_hVideoSourceLabel = nullptr;
HWND g_hVideoSourceCombo = nullptr;

//Since the lambda cannot capture, we must give it access to the robot here
RobotTester *s_pRobotTester = nullptr;  
//void BindRobot(RobotTester &_robot_tester);  //forward declare
void SetupPreferences();
ConnectionMode s_InitialConnectionMode = ConnectionMode::eDirectConnect;
// Guard flag: set true while PopulateConnectionModeCombo is repopulating the
// combo box so that the CBN_SELCHANGE notification it fires is ignored and
// does not re-enter ApplyConnectionMode.
bool s_populatingCombo = false;

namespace
{
	const wchar_t* c_ConnectionSettingsDir = L"RobotSimulation";
	const wchar_t* c_ConnectionSettingsFile = L"DriverStation.ini";
	const wchar_t* c_ConnectionSettingsSection = L"Connection";
	const wchar_t* c_ConnectionSettingsKey = L"Mode";
	const wchar_t* c_NativeLinkCarrierKey = L"NativeLinkCarrier";
	const wchar_t* c_WindowSettingsSection = L"Window";
	const wchar_t* c_WindowPosXKey = L"PosX";
	const wchar_t* c_WindowPosYKey = L"PosY";
	const int c_NativeLinkCarrierLabelId = 1010;
	const int c_NativeLinkCarrierComboId = 1011;
	const int c_VideoSourceLabelId = 1012;
	const int c_VideoSourceComboId = 1013;
	const wchar_t* c_VideoSettingsSection = L"Video";
	const wchar_t* c_VideoSourceKey = L"Source";

	// Ian: Guard flag for video source combo — same pattern as s_populatingCombo
	// for the connection mode combo.
	bool s_populatingVideoCombo = false;

	ConnectionMode GetDefaultConnectionMode()
	{
		return ConnectionMode::eDirectConnect;
	}

	NativeLink::CarrierKind GetDefaultNativeLinkCarrier()
	{
		// Ian: TCP is the intended normal runtime carrier. SharedMemory is only
		// used in Debug when the developer explicitly selects it via the carrier
		// combo (which is #ifdef _DEBUG only). In Release there is no UI to change
		// this, so always default to TCP so a clean release build just works
		// without requiring a DriverStation.ini on the target machine.
#ifdef _DEBUG
		return NativeLink::CarrierKind::SharedMemory;
#else
		return NativeLink::CarrierKind::Tcp;
#endif
	}

	bool TryGetConnectionSettingsPath(std::wstring& settingsPath)
	{
		DWORD required = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
		if (!required)
			return false;

		std::wstring basePath(required - 1, L'\0');
		if (GetEnvironmentVariableW(L"LOCALAPPDATA", &basePath[0], required) != (required - 1))
			return false;

		wchar_t settingsDir[MAX_PATH];
		wcscpy_s(settingsDir, basePath.c_str());
		PathAppendW(settingsDir, c_ConnectionSettingsDir);
		CreateDirectoryW(settingsDir, nullptr);

		wchar_t settingsFile[MAX_PATH];
		wcscpy_s(settingsFile, settingsDir);
		PathAppendW(settingsFile, c_ConnectionSettingsFile);
		settingsPath = settingsFile;
		return true;
	}

	ConnectionMode NormalizeConnectionMode(int rawMode)
	{
		switch (rawMode)
		{
		case static_cast<int>(ConnectionMode::eLegacySmartDashboard):
			return ConnectionMode::eLegacySmartDashboard;
		case static_cast<int>(ConnectionMode::eDirectConnect):
			return ConnectionMode::eDirectConnect;
		case static_cast<int>(ConnectionMode::eNetworkTablesV4):
			return ConnectionMode::eNetworkTablesV4;
		case static_cast<int>(ConnectionMode::eNativeLink):
			return ConnectionMode::eNativeLink;
		default:
			return GetDefaultConnectionMode();
		}
	}

	ConnectionMode LoadPersistedConnectionMode()
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return GetDefaultConnectionMode();

		const UINT rawMode = GetPrivateProfileIntW(
			c_ConnectionSettingsSection,
			c_ConnectionSettingsKey,
			static_cast<UINT>(GetDefaultConnectionMode()),
			settingsPath.c_str());
		return NormalizeConnectionMode(static_cast<int>(rawMode));
	}

	void PersistConnectionMode(ConnectionMode mode)
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return;

		wchar_t buffer[16];
		_swprintf_p(buffer, _countof(buffer), L"%d", static_cast<int>(mode));
		WritePrivateProfileStringW(
			c_ConnectionSettingsSection,
			c_ConnectionSettingsKey,
			buffer,
			settingsPath.c_str());
	}

	NativeLink::CarrierKind LoadPersistedNativeLinkCarrier()
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return GetDefaultNativeLinkCarrier();

		// Ian: use GetDefaultNativeLinkCarrier() as the INI fallback so a clean
		// machine (no DriverStation.ini) gets the same default as a fresh launch.
		// In Release that is Tcp; in Debug it is SharedMemory.
		const wchar_t* defaultCarrierStr =
			GetDefaultNativeLinkCarrier() == NativeLink::CarrierKind::Tcp ? L"tcp" : L"shm";

		wchar_t buffer[16] = {};
		GetPrivateProfileStringW(
			c_ConnectionSettingsSection,
			c_NativeLinkCarrierKey,
			defaultCarrierStr,
			buffer,
			static_cast<DWORD>(_countof(buffer)),
			settingsPath.c_str());

		std::wstring value(buffer);
		std::transform(value.begin(), value.end(), value.begin(), towlower);
		return value == L"tcp" ? NativeLink::CarrierKind::Tcp : NativeLink::CarrierKind::SharedMemory;
	}

	void PersistNativeLinkCarrier(NativeLink::CarrierKind carrier)
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return;

		WritePrivateProfileStringW(
			c_ConnectionSettingsSection,
			c_NativeLinkCarrierKey,
			carrier == NativeLink::CarrierKind::Tcp ? L"tcp" : L"shm",
			settingsPath.c_str());
	}

	void ApplyNativeLinkEnvironment(NativeLink::CarrierKind carrier)
	{
		SetEnvironmentVariableA("NATIVE_LINK_CARRIER", NativeLink::ToString(carrier));
		SetEnvironmentVariableA("NATIVE_LINK_CHANNEL_ID", "native-link-default");
		// Ian: Bind to 0.0.0.0 (all interfaces) so a remote SmartDashboard on another
		// machine can reach port 5810 across the LAN. Legacy NT uses INADDR_ANY for the
		// same reason. 127.0.0.1 (loopback) would silently reject every cross-machine
		// connection at the kernel level before the process even sees it, and Windows
		// Firewall would never prompt because no external packet would reach the socket.
		SetEnvironmentVariableA("NATIVE_LINK_HOST", "0.0.0.0");
		SetEnvironmentVariableA("NATIVE_LINK_PORT", "5810");
	}

	NativeLink::CarrierKind GetSelectedNativeLinkCarrier()
	{
		if (!g_hNativeLinkCarrierCombo)
			return LoadPersistedNativeLinkCarrier();

		const LRESULT selectedIndex = SendMessageW(g_hNativeLinkCarrierCombo, CB_GETCURSEL, 0, 0);
		if (selectedIndex == CB_ERR)
			return LoadPersistedNativeLinkCarrier();

		const LRESULT itemData = SendMessageW(g_hNativeLinkCarrierCombo, CB_GETITEMDATA, static_cast<WPARAM>(selectedIndex), 0);
		return itemData == static_cast<LRESULT>(NativeLink::CarrierKind::Tcp)
			? NativeLink::CarrierKind::Tcp
			: NativeLink::CarrierKind::SharedMemory;
	}

	void SyncNativeLinkCarrierUiEnabled(HWND hWnd)
	{
	#ifdef _DEBUG
		const bool enabled = s_pRobotTester && s_pRobotTester->GetConnectionMode() == ConnectionMode::eNativeLink;
		if (g_hNativeLinkCarrierLabel)
			EnableWindow(g_hNativeLinkCarrierLabel, enabled ? TRUE : FALSE);
		if (g_hNativeLinkCarrierCombo)
			EnableWindow(g_hNativeLinkCarrierCombo, enabled ? TRUE : FALSE);
	#else
		UNREFERENCED_PARAMETER(hWnd);
	#endif
	}

	void CreateNativeLinkCarrierControls(HWND hWnd)
	{
	#ifdef _DEBUG
		// Ian: In Debug builds, the carrier combo goes on the SAME row as the
		// Connection combo (to the right of it).  This keeps it out of the way
		// of the Video Source combo's dropdown and doesn't add vertical space.
		//
		// Layout:  [Connection] [conn combo] [Carrier] [carrier combo]
		// We query the Connection combo's actual position at runtime to stay
		// DPI-independent.

		// Ian: Get the Connection combo's position and size to place the Carrier
		// to its right on the same row.
		int carrierY = 0;
		int carrierLabelY = 0;
		int carrierLabelX = 200;   // fallback
		int carrierComboX = 250;   // fallback

		HWND hConnCombo = GetDlgItem(hWnd, IDC_ConnectionMode);
		if (hConnCombo)
		{
			RECT rcConn = {};
			GetWindowRect(hConnCombo, &rcConn);
			POINT ptConnTL = { rcConn.left, rcConn.top };
			POINT ptConnBR = { rcConn.right, rcConn.bottom };
			ScreenToClient(hWnd, &ptConnTL);
			ScreenToClient(hWnd, &ptConnBR);
			carrierY = ptConnTL.y;
			carrierLabelY = carrierY + 4;  // vertically center label with combo
			// Ian: Place the carrier label 10px to the right of the Connection
			// combo's right edge — enough gap to clear the Video dropdown below.
			carrierLabelX = ptConnBR.x + 10;
			carrierComboX = carrierLabelX + 46;  // "Carrier" label ~40px + 6px gap
		}
		else
		{
			carrierY = 90;
			carrierLabelY = 94;
		}

		// Ian: Widen the dialog to fit the carrier controls.
		RECT windowRect = {};
		if (GetWindowRect(hWnd, &windowRect))
		{
			const int currentWidth = windowRect.right - windowRect.left;
			const int neededWidth = carrierComboX + 120 + 30;  // combo width + margin
			if (currentWidth < neededWidth)
			{
				SetWindowPos(hWnd, nullptr, 0, 0,
					neededWidth,
					windowRect.bottom - windowRect.top,
					SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
			}
		}

		const HFONT dialogFont = reinterpret_cast<HFONT>(SendMessageW(hWnd, WM_GETFONT, 0, 0));
		g_hNativeLinkCarrierLabel = CreateWindowW(
			L"STATIC",
			L"Carrier",
			WS_CHILD | WS_VISIBLE,
			carrierLabelX,
			carrierLabelY,
			40,
			12,
			hWnd,
			reinterpret_cast<HMENU>(static_cast<INT_PTR>(c_NativeLinkCarrierLabelId)),
			hInst,
			nullptr);
		g_hNativeLinkCarrierCombo = CreateWindowW(
			L"COMBOBOX",
			L"",
			WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
			carrierComboX,
			carrierY,
			120,
			100,
			hWnd,
			reinterpret_cast<HMENU>(static_cast<INT_PTR>(c_NativeLinkCarrierComboId)),
			hInst,
			nullptr);
		if (dialogFont)
		{
			if (g_hNativeLinkCarrierLabel)
				SendMessageW(g_hNativeLinkCarrierLabel, WM_SETFONT, reinterpret_cast<WPARAM>(dialogFont), TRUE);
			if (g_hNativeLinkCarrierCombo)
				SendMessageW(g_hNativeLinkCarrierCombo, WM_SETFONT, reinterpret_cast<WPARAM>(dialogFont), TRUE);
		}

		if (g_hNativeLinkCarrierCombo)
		{
			const LRESULT shmIndex = SendMessageW(g_hNativeLinkCarrierCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Shared Memory (SHM)"));
			SendMessageW(g_hNativeLinkCarrierCombo, CB_SETITEMDATA, static_cast<WPARAM>(shmIndex), static_cast<LPARAM>(NativeLink::CarrierKind::SharedMemory));
			const LRESULT tcpIndex = SendMessageW(g_hNativeLinkCarrierCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"TCP/IP"));
			SendMessageW(g_hNativeLinkCarrierCombo, CB_SETITEMDATA, static_cast<WPARAM>(tcpIndex), static_cast<LPARAM>(NativeLink::CarrierKind::Tcp));

			const NativeLink::CarrierKind persistedCarrier = LoadPersistedNativeLinkCarrier();
			SendMessageW(g_hNativeLinkCarrierCombo, CB_SETCURSEL,
				persistedCarrier == NativeLink::CarrierKind::Tcp ? static_cast<WPARAM>(tcpIndex) : static_cast<WPARAM>(shmIndex), 0);
		}

		SyncNativeLinkCarrierUiEnabled(hWnd);
	#else
		UNREFERENCED_PARAMETER(hWnd);
	#endif
	}

	// --- Video Source persistence and UI ---

	VideoSourceMode GetDefaultVideoSource()
	{
		return VideoSourceMode::eSyntheticRadar;
	}

	VideoSourceMode NormalizeVideoSourceMode(int rawMode)
	{
		switch (rawMode)
		{
		case static_cast<int>(VideoSourceMode::eOff):
			return VideoSourceMode::eOff;
		case static_cast<int>(VideoSourceMode::eCamera):
			return VideoSourceMode::eCamera;
		case static_cast<int>(VideoSourceMode::eSyntheticRadar):
			return VideoSourceMode::eSyntheticRadar;
		case static_cast<int>(VideoSourceMode::eVirtualField):
			return VideoSourceMode::eVirtualField;
		default:
			return GetDefaultVideoSource();
		}
	}

	VideoSourceMode LoadPersistedVideoSource()
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return GetDefaultVideoSource();

		const UINT rawMode = GetPrivateProfileIntW(
			c_VideoSettingsSection,
			c_VideoSourceKey,
			static_cast<UINT>(GetDefaultVideoSource()),
			settingsPath.c_str());
		return NormalizeVideoSourceMode(static_cast<int>(rawMode));
	}

	void PersistVideoSource(VideoSourceMode mode)
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return;

		wchar_t buffer[16];
		_swprintf_p(buffer, _countof(buffer), L"%d", static_cast<int>(mode));
		WritePrivateProfileStringW(
			c_VideoSettingsSection,
			c_VideoSourceKey,
			buffer,
			settingsPath.c_str());
	}

	void PopulateVideoSourceCombo(VideoSourceMode selectedMode)
	{
		if (!g_hVideoSourceCombo)
			return;

		s_populatingVideoCombo = true;
		SendMessageW(g_hVideoSourceCombo, CB_RESETCONTENT, 0, 0);

		// Ian: All four video source modes.
		const VideoSourceMode modes[] =
		{
			VideoSourceMode::eOff,
			VideoSourceMode::eCamera,
			VideoSourceMode::eSyntheticRadar,
			VideoSourceMode::eVirtualField,
		};

		int selectedIndex = 0;
		for (int i = 0; i < static_cast<int>(_countof(modes)); ++i)
		{
			const wchar_t* label = GetVideoSourceModeName(modes[i]);
			const LRESULT index = SendMessageW(g_hVideoSourceCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
			SendMessageW(g_hVideoSourceCombo, CB_SETITEMDATA, static_cast<WPARAM>(index), static_cast<LPARAM>(modes[i]));
			if (modes[i] == selectedMode)
				selectedIndex = static_cast<int>(index);
		}

		SendMessageW(g_hVideoSourceCombo, CB_SETCURSEL, static_cast<WPARAM>(selectedIndex), 0);
		s_populatingVideoCombo = false;
	}

	// Ian: Create the Video Source label + combo box dynamically.
	// Always visible (not debug-only like the NativeLink carrier combo).
	// Placed below the Connection combo, extending the dialog height.
	void CreateVideoSourceControls(HWND hWnd)
	{
		// Extend the dialog height to fit the new row
		RECT windowRect = {};
		if (GetWindowRect(hWnd, &windowRect))
		{
			SetWindowPos(hWnd, nullptr, 0, 0,
				windowRect.right - windowRect.left,
				(windowRect.bottom - windowRect.top) + 26,
				SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
		}

		const HFONT dialogFont = reinterpret_cast<HFONT>(SendMessageW(hWnd, WM_GETFONT, 0, 0));

		// Ian: Position the Video controls directly below the Connection combo
		// by querying its actual pixel position at runtime.  This avoids
		// hardcoded pixel offsets that break if the dialog DLU→pixel mapping
		// changes (e.g. different DPI or font).  The Connection combo and label
		// are defined in the .rc template (DLU 78,70 and 26,74 respectively);
		// we mirror their X positions and derive Y from the combo's bottom edge.
		int xLabel = 26;   // fallback if GetDlgItem fails
		int xCombo = 78;
		int comboWidth = 112;
		int yCombo = 90;
		int labelHeight = 12;
		constexpr int kRowGap = 4;  // pixels between Connection bottom and Video top

		HWND hConnCombo = GetDlgItem(hWnd, IDC_ConnectionMode);
		HWND hConnLabel = GetDlgItem(hWnd, IDC_ConnectionModeLabel);
		if (hConnCombo)
		{
			RECT rcCombo = {};
			GetWindowRect(hConnCombo, &rcCombo);
			POINT ptComboTL = { rcCombo.left, rcCombo.top };
			POINT ptComboBR = { rcCombo.right, rcCombo.bottom };
			ScreenToClient(hWnd, &ptComboTL);
			ScreenToClient(hWnd, &ptComboBR);
			xCombo = ptComboTL.x;
			comboWidth = ptComboBR.x - ptComboTL.x;
			yCombo = ptComboBR.y + kRowGap;  // just below Connection combo
		}
		if (hConnLabel)
		{
			RECT rcLabel = {};
			GetWindowRect(hConnLabel, &rcLabel);
			POINT ptLabelTL = { rcLabel.left, rcLabel.top };
			POINT ptLabelBR = { rcLabel.left, rcLabel.bottom };
			ScreenToClient(hWnd, &ptLabelTL);
			ScreenToClient(hWnd, &ptLabelBR);
			xLabel = ptLabelTL.x;
			labelHeight = ptLabelBR.y - ptLabelTL.y;
		}
		// Ian: Label is vertically centered relative to the combo.  The combo
		// is taller than the label, so offset the label down by ~4px.
		const int yLabel = yCombo + 4;

		g_hVideoSourceLabel = CreateWindowW(
			L"STATIC",
			L"Video",
			WS_CHILD | WS_VISIBLE,
			xLabel,
			yLabel,
			46,
			labelHeight,
			hWnd,
			reinterpret_cast<HMENU>(static_cast<INT_PTR>(c_VideoSourceLabelId)),
			hInst,
			nullptr);
		g_hVideoSourceCombo = CreateWindowW(
			L"COMBOBOX",
			L"",
			WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
			xCombo,
			yCombo,
			comboWidth,
			100,
			hWnd,
			reinterpret_cast<HMENU>(static_cast<INT_PTR>(c_VideoSourceComboId)),
			hInst,
			nullptr);

		if (dialogFont)
		{
			if (g_hVideoSourceLabel)
				SendMessageW(g_hVideoSourceLabel, WM_SETFONT, reinterpret_cast<WPARAM>(dialogFont), TRUE);
			if (g_hVideoSourceCombo)
				SendMessageW(g_hVideoSourceCombo, WM_SETFONT, reinterpret_cast<WPARAM>(dialogFont), TRUE);
		}

		PopulateVideoSourceCombo(LoadPersistedVideoSource());
	}

	bool LoadPersistedWindowPosition(POINT& windowPosition)
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return false;

		const int sentinel = 0x7fffffff;
		const int posX = GetPrivateProfileIntW(c_WindowSettingsSection, c_WindowPosXKey, sentinel, settingsPath.c_str());
		const int posY = GetPrivateProfileIntW(c_WindowSettingsSection, c_WindowPosYKey, sentinel, settingsPath.c_str());
		if (posX == sentinel || posY == sentinel)
			return false;

		windowPosition.x = posX;
		windowPosition.y = posY;
		return true;
	}

	void PersistWindowPosition(HWND hWnd)
	{
		if (!hWnd)
			return;

		RECT windowRect = {};
		if (!GetWindowRect(hWnd, &windowRect))
			return;

		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return;

		wchar_t buffer[32];
		_swprintf_p(buffer, _countof(buffer), L"%ld", windowRect.left);
		WritePrivateProfileStringW(c_WindowSettingsSection, c_WindowPosXKey, buffer, settingsPath.c_str());
		_swprintf_p(buffer, _countof(buffer), L"%ld", windowRect.top);
		WritePrivateProfileStringW(c_WindowSettingsSection, c_WindowPosYKey, buffer, settingsPath.c_str());
	}

	void RestorePersistedWindowPosition(HWND hWnd)
	{
		if (!hWnd)
			return;

		POINT windowPosition = {};
		if (!LoadPersistedWindowPosition(windowPosition))
			return;

		SetWindowPos(hWnd, nullptr, windowPosition.x, windowPosition.y, 0, 0, SWP_NOACTIVATE | SWP_NOSIZE | SWP_NOZORDER);
	}

	void PopulateConnectionModeCombo(HWND hWnd, ConnectionMode selectedMode)
	{
		HWND combo = GetDlgItem(hWnd, IDC_ConnectionMode);
		if (!combo)
			return;

		s_populatingCombo = true;
		SendMessageW(combo, CB_RESETCONTENT, 0, 0);
		const ConnectionMode modes[] =
		{
			ConnectionMode::eLegacySmartDashboard,
			ConnectionMode::eDirectConnect,
			ConnectionMode::eNetworkTablesV4,
			ConnectionMode::eNativeLink
		};

		int selectedIndex = 0;
		for (int i = 0; i < static_cast<int>(_countof(modes)); ++i)
		{
			const wchar_t* label = GetConnectionModeName(modes[i]);
			const LRESULT index = SendMessageW(combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
			SendMessageW(combo, CB_SETITEMDATA, static_cast<WPARAM>(index), static_cast<LPARAM>(modes[i]));
			if (modes[i] == selectedMode)
				selectedIndex = static_cast<int>(index);
		}

		SendMessageW(combo, CB_SETCURSEL, static_cast<WPARAM>(selectedIndex), 0);
		s_populatingCombo = false;
	}

	void SyncConnectionModeCombo(HWND hWnd)
	{
		if (!s_pRobotTester)
			return;
		PopulateConnectionModeCombo(hWnd, s_pRobotTester->GetConnectionMode());
	}
}

bool TryParseConnectionModeFromCmdLine(LPWSTR cmd_line, ConnectionMode& mode)
{
	if (!cmd_line)
		return false;

	std::wstring cmd = cmd_line;
	std::transform(cmd.begin(), cmd.end(), cmd.begin(), towlower);

	if ((cmd.find(L"direct") != std::wstring::npos) || (cmd.find(L"conn=direct") != std::wstring::npos))
	{
		mode = ConnectionMode::eDirectConnect;
		return true;
	}
	// Ian: Accept "nt4", "shuffle" (backward compat), and "conn=nt4" / "conn=shuffle"
	if ((cmd.find(L"nt4") != std::wstring::npos) || (cmd.find(L"conn=nt4") != std::wstring::npos) ||
		(cmd.find(L"shuffle") != std::wstring::npos) || (cmd.find(L"conn=shuffle") != std::wstring::npos))
	{
		mode = ConnectionMode::eNetworkTablesV4;
		return true;
	}
	if ((cmd.find(L"native") != std::wstring::npos) || (cmd.find(L"conn=native") != std::wstring::npos))
	{
		mode = ConnectionMode::eNativeLink;
		return true;
	}
	if ((cmd.find(L"legacy") != std::wstring::npos) || (cmd.find(L"conn=legacy") != std::wstring::npos))
	{
		mode = ConnectionMode::eLegacySmartDashboard;
		return true;
	}

	return false;
}

void ApplyConnectionMode(ConnectionMode mode)
{
	if (mode == ConnectionMode::eNativeLink)
	{
		ApplyNativeLinkEnvironment(GetSelectedNativeLinkCarrier());
	}

	if (s_pRobotTester)
	{
		s_pRobotTester->SetConnectionMode(mode);
	}
	if (g_hDlg)
	{
		PopulateConnectionModeCombo(g_hDlg, mode);
		SyncNativeLinkCarrierUiEnabled(g_hDlg);
	}
	PersistConnectionMode(mode);

	std::wstring message = L"Connection Mode: ";
	message += GetConnectionModeName(mode);
	message += L"\n";
	OutputDebugStringW(message.c_str());
}

void ApplyVideoSource(VideoSourceMode mode)
{
	if (s_pRobotTester)
	{
		s_pRobotTester->SetVideoSource(mode);
		// Ian: The backend may fall back to Off if the requested source fails
		// (e.g. Camera selected but no camera connected).  Read back the actual
		// mode so the UI combo and INI stay in sync with reality.
		const VideoSourceMode actualMode = s_pRobotTester->GetVideoSource();
		if (actualMode != mode)
		{
			OutputDebugStringW(L"Video source fell back from requested mode — updating UI\n");
			mode = actualMode;
		}
	}
	if (g_hVideoSourceCombo)
	{
		PopulateVideoSourceCombo(mode);
	}
	PersistVideoSource(mode);

	std::wstring vmsg = L"Video Source: ";
	vmsg += GetVideoSourceModeName(mode);
	vmsg += L"\n";
	OutputDebugStringW(vmsg.c_str());
}

//Keyboard *s_Keyboard = nullptr;

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
		GetModuleFileNameW(nullptr, Buffer, MAX_PATH);
		PathRemoveFileSpecW(Buffer);
		path = Buffer;
		GetParentDirectory(path);
		GetParentDirectory(path);
		SetCurrentDirectoryW(path.c_str());
	}
	UNREFERENCED_PARAMETER(hPrevInstance);
	s_InitialConnectionMode = LoadPersistedConnectionMode();
	ApplyNativeLinkEnvironment(LoadPersistedNativeLinkCarrier());
	ConnectionMode commandLineMode = s_InitialConnectionMode;
	if (TryParseConnectionModeFromCmdLine(lpCmdLine, commandLineMode))
	{
		s_InitialConnectionMode = commandLineMode;
	}

	// TODO: Place code here.

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_DRIVERSTATION, szWindowClass, MAX_LOADSTRING);
	//MyRegisterClass(hInstance);

	const HWND pParent = nullptr;  //just to show what the parameter is

	// Display the main dialog box.  I've always wanted to put the callback in the same place!
	HWND m_hDlg = CreateDialogW(hInstance, MAKEINTRESOURCEW(IDD_DriveStation1), pParent,
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
			PopulateConnectionModeCombo(hWnd, s_InitialConnectionMode);
			CreateVideoSourceControls(hWnd);
			CreateNativeLinkCarrierControls(hWnd);
			RestorePersistedWindowPosition(hWnd);
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
		case IDC_ConnectionMode:
			if (HIWORD(wParam) == CBN_SELCHANGE && !s_populatingCombo)
			{
				const HWND combo = GetDlgItem(hWnd, IDC_ConnectionMode);
				const LRESULT selectedIndex = SendMessageW(combo, CB_GETCURSEL, 0, 0);
				if (selectedIndex != CB_ERR)
				{
					const LRESULT itemData = SendMessageW(combo, CB_GETITEMDATA, static_cast<WPARAM>(selectedIndex), 0);
					ApplyConnectionMode(static_cast<ConnectionMode>(itemData));
				}
			}
			break;
		case c_NativeLinkCarrierComboId:
			if (HIWORD(wParam) == CBN_SELCHANGE)
			{
				const NativeLink::CarrierKind carrier = GetSelectedNativeLinkCarrier();
				PersistNativeLinkCarrier(carrier);
				ApplyNativeLinkEnvironment(carrier);
				// Ian: do NOT call ApplyConnectionMode(eDirectConnect) followed by
				// ApplyConnectionMode(eNativeLink) here.  That two-step bounce was
				// intended to restart the NativeLink server with the new carrier env,
				// but ApplyConnectionMode calls SetConnectionMode which does a full
				// teardown/restart; the intermediate eDirectConnect transition is
				// visible to the user, flips the connection mode combo away from
				// Native Link, and (in Debug) causes the DS window to become unusable.
				// Instead, only restart if we are already in Native Link mode, and do
				// so by calling SetConnectionMode(eNativeLink) directly on the robot
				// tester — this re-applies the mode with the new carrier env that was
				// just written above, without ever surfacing a DirectConnect state.
				if (s_pRobotTester && s_pRobotTester->GetConnectionMode() == ConnectionMode::eNativeLink)
					s_pRobotTester->SetConnectionMode(ConnectionMode::eNativeLink);
			}
			break;
		// Ian: Video source combo — same pattern as connection mode combo.
		// The guard flag s_populatingVideoCombo prevents re-entrancy when
		// PopulateVideoSourceCombo fires CBN_SELCHANGE during repopulation.
		case c_VideoSourceComboId:
			if (HIWORD(wParam) == CBN_SELCHANGE && !s_populatingVideoCombo)
			{
				const LRESULT selectedIndex = SendMessageW(g_hVideoSourceCombo, CB_GETCURSEL, 0, 0);
				if (selectedIndex != CB_ERR)
				{
					const LRESULT itemData = SendMessageW(g_hVideoSourceCombo, CB_GETITEMDATA, static_cast<WPARAM>(selectedIndex), 0);
					ApplyVideoSource(static_cast<VideoSourceMode>(itemData));
				}
			}
			break;
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

		//Note: for whatever reason I cannot macro out these lines, probably because I'm in a switch statement
		//case WM_KEYUP:
		//	OutputDebugStringW(L"KeyPressedUp\n");
		//	if (s_Keyboard)
		//		s_Keyboard->KeyPressRelease((int)wParam, false);
		//	break;
		//case WM_KEYDOWN:
		//	OutputDebugStringW(L"KeyPressedDN\n");
		//	if (s_Keyboard)
		//		s_Keyboard->KeyPressRelease((int)wParam, true);
		//	break;

		case WM_PAINT:
		{
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hWnd, &ps);
			// TODO: Add any drawing code that uses hdc here...
			EndPaint(hWnd, &ps);
		}
		break;
		case WM_MOVE:
			PersistWindowPosition(hWnd);
			break;
		case WM_DESTROY:
			PersistWindowPosition(hWnd);
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
	g_hDlg = m_hDlg;

	try
	{
	//We made it this far... start up the robot
	RobotTester _robot_tester;
	s_pRobotTester = &_robot_tester;
	_robot_tester.RobotTester_create();
	ApplyConnectionMode(s_InitialConnectionMode);
	// Ian: Apply persisted video source after connection mode is established,
	// so the backend exists and can receive the SetVideoSource call.
	ApplyVideoSource(LoadPersistedVideoSource());
	//Bind robot for Keyboard binding
	//BindRobot(_robot_tester);
	_robot_tester.RobotTester_init();
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
			if (msg.message == WM_KEYDOWN)
			{
				switch (msg.wParam)
				{
				case VK_F5:
					ApplyConnectionMode(ConnectionMode::eLegacySmartDashboard);
					break;
				case VK_F6:
					ApplyConnectionMode(ConnectionMode::eDirectConnect);
					break;
				case VK_F7:
					ApplyConnectionMode(ConnectionMode::eNetworkTablesV4);
					break;
				case VK_F8:
					ApplyConnectionMode(ConnectionMode::eNativeLink);
					break;
				}
			}

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
			#if 0
			//OutputDebugStringW(L"KeyPressed\n");
			if ((ProcessKey)&&(s_Keyboard))
				s_Keyboard->KeyPressRelease((int)ascii, msg.message == WM_KEYDOWN);
			#endif
		}
		else if (bRet == -1)
		{	// GetMessage returned -1: invalid HWND or critical message-queue error.
			OutputDebugStringW(L"[DS] GetMessage returned -1 — message queue error, exiting message loop\n");
			assert(false);
			break;
		}
		else
		{	// Process the message
			::TranslateMessage(&msg);
			::DispatchMessage(&msg);
		}
	}
	OutputDebugStringA("[DS] message loop exited (GetMessage returned 0 -- WM_QUIT received)\n");

	_robot_tester.Shutdown();

	return (int)msg.wParam;
	}
	catch (const std::exception& e)
	{
		char buf[512];
		sprintf_s(buf, "[wWinMain] Unhandled exception: %s\n", e.what());
		OutputDebugStringA(buf);
		MessageBoxA(nullptr, buf, "DriverStation Fatal Error", MB_OK | MB_ICONERROR);
		return -1;
	}
	catch (...)
	{
		OutputDebugStringA("[wWinMain] Unhandled unknown exception\n");
		MessageBoxA(nullptr, "An unknown fatal error occurred.", "DriverStation Fatal Error", MB_OK | MB_ICONERROR);
		return -1;
	}
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
