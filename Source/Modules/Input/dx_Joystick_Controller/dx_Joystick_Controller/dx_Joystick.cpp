#include "stdafx.h"
#ifndef _MSC_VER
#define INITGUID
#endif

#define DIRECTINPUT_VERSION         0x0800
#include <dinput.h>
#include "dx_Joystick.h"

#pragma comment(lib, "dinput8")
#pragma comment(lib, "dxGUID")

namespace Module {
	namespace Input {

void Trim2(std::wstring &str)
{
	//http://www.codeproject.com/Articles/10880/A-trim-implementation-for-std-string
	using namespace std;
	wstring::size_type pos = str.find_last_not_of(' ');
	if (pos != wstring::npos)
	{
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if (pos != wstring::npos) str.erase(0, pos);
	}
	else str.erase(str.begin(), str.end());
}

//Forward declare
BOOL CALLBACK EnumObjectsCallback(const DIDEVICEOBJECTINSTANCE* pdidoi, VOID* pContext);
BOOL CALLBACK DIEnumDevicesCallback(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef);

class DirectInput_Joystick
{
private:
	LPDIRECTINPUT8 m_lpDInput;
	struct DIJoyInfo
	{
		dx_Joystick::JoystickInfo Common;
		LPDIRECTINPUTDEVICE8 lpDInputDevice;
	};
	std::vector<DIJoyInfo> m_JoyInfo;
	HWND const m_HWnd;
public:
	/// \param hwnd If you can easily provide the hwnd then it can handle the case like not listen if the window is minimized. (it is optional)
	DirectInput_Joystick(HWND hwnd) :	m_HWnd(hwnd)
	{
		// Modified to handle DirectInput joysticks (such as USB devices)
		{
			if (SUCCEEDED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&m_lpDInput, NULL)))
			{
				HRESULT hr;
				hr = IDirectInput_EnumDevices(m_lpDInput, DI8DEVCLASS_GAMECTRL, Input::DIEnumDevicesCallback, (void *)this, DIEDFL_ATTACHEDONLY);
				assert(hr == S_OK);
			}
		}

		//nr_joysticks = found;
		_CP_("%d joystick(s) found.\n", m_JoyInfo.size());
	}
	~DirectInput_Joystick(void)
		{
			size_t i;

			if (m_lpDInput)
			{
				for (i = 0; i < m_JoyInfo.size(); i++)
				{
					if (m_JoyInfo[i].lpDInputDevice)
					{
						m_JoyInfo[i].lpDInputDevice->Unacquire();
						m_JoyInfo[i].lpDInputDevice->Release();
						m_JoyInfo[i].lpDInputDevice = NULL;
					}
				}
				m_lpDInput->Release();
				m_lpDInput = NULL;
			}
		}
public:	//These are only public for the OS callbacks
	BOOL DIEnumDevicesCallback(LPCDIDEVICEINSTANCE lpddi)
	{

		_CP_("DirectInput: Found %s %s\n", lpddi->tszProductName, lpddi->tszInstanceName);
		DIJoyInfo NewEntry;
		NewEntry.Common.ProductName = lpddi->tszProductName;
		NewEntry.Common.InstanceName = lpddi->tszInstanceName;
		//I can't explain this but on some devices... they will change from upper to lower case depending on the whim of a cold-boot... furthermore
		//some devices will change from an all uppercase with a trailing space to a CamelCase with a non-trailing space.  We'll fix both problems here
		//by forcing all to lowercase, and to trim the spacing
		std::transform(NewEntry.Common.ProductName.begin(), NewEntry.Common.ProductName.end(), NewEntry.Common.ProductName.begin(), tolower);
		std::transform(NewEntry.Common.InstanceName.begin(), NewEntry.Common.InstanceName.end(), NewEntry.Common.InstanceName.begin(), tolower);
		Trim2(NewEntry.Common.ProductName);
		Trim2(NewEntry.Common.InstanceName);

		try
		{
			if (FAILED(m_lpDInput->CreateDevice(lpddi->guidInstance, &NewEntry.lpDInputDevice, NULL)))
				return DIENUM_STOP;

			LPDIRECTINPUTDEVICE8 device = NewEntry.lpDInputDevice;

			NewEntry.Common.bPresent = FALSE;
			if (FAILED(device->SetDataFormat(&c_dfDIJoystick2))) throw 0;

			if (m_HWnd)
			{
				//let DInput know how this device should interact with the system and with other DInput applications
				if (FAILED(NewEntry.lpDInputDevice->SetCooperativeLevel(m_HWnd, DISCL_EXCLUSIVE | DISCL_FOREGROUND))) throw 1;
			}

			NewEntry.Common.bPresent = TRUE;
			NewEntry.Common.JoyCapFlags = 0;
			NewEntry.Common.nSliderCount = 0;  // Number of returned slider controls
			NewEntry.Common.nPOVCount = 0;     // Number of returned POV controls
			NewEntry.Common.nButtonCount = 0;  // Number of returned Buttons
			m_JoyInfo.push_back(NewEntry);
			// Enumerate the joystick objects. The callback function enabled user
			// interface elements for objects that are found, and sets the min/max
			// values property for discovered axes.
			if (FAILED(device->EnumObjects(Input::EnumObjectsCallback, (VOID*)this, DIDFT_ALL)))	throw 2;
			return DIENUM_CONTINUE;
		}
		catch (int ErrorCode)
		{
			_CP_("DirectInput_Joystick::DIEnumDevicesCallback failed %d\n", ErrorCode);
			_CP_("ErrorCode %x\n", GetLastError());

			IDirectInputDevice_Release(NewEntry.lpDInputDevice);
			NewEntry.lpDInputDevice = NULL;
			//Be sure to flag as failed
			if (NewEntry.Common.bPresent == TRUE)
				m_JoyInfo[m_JoyInfo.size() - 1].Common.bPresent = FALSE;
			return DIENUM_CONTINUE;  //this one failed but others may succeed
		}
	}
	BOOL EnumObjectsCallback(const DIDEVICEOBJECTINSTANCE* pdidoi)
	{
		using JoystickInfo = dx_Joystick::JoystickInfo;
		//-----------------------------------------------------------------------------
		// Name: EnumObjectsCallback()
		// Desc: Callback function for enumerating objects (axes, buttons, POVs) on a 
		//       joystick. This function enables user interface elements for objects
		//       that are found to exist, and scales axes min/max values.
		//-----------------------------------------------------------------------------

		//TODO I'm assuming that the instance of this object corresponds to the last added joystick (it should since it will be set in the enum)
		assert(m_JoyInfo.size() > 0);
		JoystickInfo &info = m_JoyInfo[m_JoyInfo.size() - 1].Common;

		// For axes that are returned, set the DIPROP_RANGE property for the
		// enumerated axis in order to scale min/max values.
		if (pdidoi->dwType & DIDFT_AXIS)
		{
			DIPROPRANGE diprg;
			diprg.diph.dwSize = sizeof(DIPROPRANGE);
			diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER);
			diprg.diph.dwHow = DIPH_BYID;
			diprg.diph.dwObj = pdidoi->dwType; // Specify the enumerated axis
			diprg.lMin = -1000;
			diprg.lMax = +1000;

			// Set the range for the axis
			if (FAILED(m_JoyInfo[m_JoyInfo.size() - 1].lpDInputDevice->SetProperty(DIPROP_RANGE, &diprg.diph)))
				return DIENUM_STOP;

		}

		if (pdidoi->dwType & DIDFT_BUTTON)
			info.nButtonCount++;

		// Set the UI to reflect what objects the joystick supports
		if (pdidoi->guidType == GUID_XAxis)
			info.JoyCapFlags |= JoystickInfo::fX_Axis;
		else if (pdidoi->guidType == GUID_YAxis)
			info.JoyCapFlags |= JoystickInfo::fY_Axis;
		else if (pdidoi->guidType == GUID_ZAxis)
			info.JoyCapFlags |= JoystickInfo::fZ_Axis;
		else if (pdidoi->guidType == GUID_RxAxis)
			info.JoyCapFlags |= JoystickInfo::fX_Rot;
		else if (pdidoi->guidType == GUID_RyAxis)
			info.JoyCapFlags |= JoystickInfo::fY_Rot;
		else if (pdidoi->guidType == GUID_RzAxis)
			info.JoyCapFlags |= JoystickInfo::fZ_Rot;
		else if (pdidoi->guidType == GUID_Slider)
		{
			switch (info.nSliderCount++)
			{
			case 0:
				info.JoyCapFlags |= JoystickInfo::fSlider0;
				break;

			case 1:
				info.JoyCapFlags |= JoystickInfo::fSlider1;
				break;
			default:
				assert(false);  //I'd be surprised... we can still work fine
			}
		}
		else if (pdidoi->guidType == GUID_POV)
		{
			switch (info.nPOVCount++)
			{
			case 0:
				info.JoyCapFlags |= JoystickInfo::fPOV_0;
				break;

			case 1:
				info.JoyCapFlags |= JoystickInfo::fPOV_1;
				break;

			case 2:
				info.JoyCapFlags |= JoystickInfo::fPOV_2;
				break;

			case 3:
				info.JoyCapFlags |= JoystickInfo::fPOV_3;
				break;
			}
		}

		return DIENUM_CONTINUE;
	}

	bool read_joystick(size_t nr, dx_Joystick::JoyState &Info)
	{
		using JoystickInfo = dx_Joystick::JoystickInfo;
		using JoyState = dx_Joystick::JoyState;
		LPDIRECTINPUTDEVICE8 pDev = m_JoyInfo[nr].lpDInputDevice;
		JoystickInfo &caps = m_JoyInfo[nr].Common;
		HRESULT hr;

		DIJOYSTATE2 js;
		memset(&js, 0, sizeof(DIJOYSTATE2));
		memset(&Info, 0, sizeof(JoyState));

		// poll the joystick to read the current state
		hr = pDev->Poll();
		//Note: S_FALSE is DI_NOEFFECT which is successful
		if (FAILED(hr))
		{
			// did the read fail because we lost input for some reason? 
			// if so, then attempt to reacquire. If the second acquire 
			// fails, then the error from GetDeviceData will be 
			// DIERR_NOTACQUIRED, so we won't get stuck an infinite loop. 
			hr = pDev->Acquire();
			while (hr == DIERR_INPUTLOST)
				hr = pDev->Acquire();

			// hr may be DIERR_OTHERAPPHASPRIO or other errors.  This
			// may occur when the app is minimized or in the process of 
			// switching, so just try again later 
			return false;
		}

		// get data from the joystick 
		hr = pDev->GetDeviceState(sizeof(DIJOYSTATE2), &js);
		//Now to translate
		Info.JoystickNumber = nr;
		Info.Axis.Named.lX = (float)js.lX / 1000.0f;
		Info.Axis.Named.lY = (float)js.lY / 1000.0f;
		Info.Axis.Named.lZ = (float)js.lZ / 1000.0f;
		Info.Axis.Named.lRx = (float)js.lRx / 1000.0f;
		Info.Axis.Named.lRy = (float)js.lRy / 1000.0f;
		Info.Axis.Named.lRz = (float)js.lRz / 1000.0f;
		for (int i = 0; i < caps.nSliderCount; i++)
			Info.Axis.Named.rgSlider[i] = (float)js.rglSlider[i] / 1000.0f;
		for (int i = 0; i < caps.nPOVCount; i++)
		{
			int result = (int)js.rgdwPOV[i];
			Info.Axis.Named.rgPOV[i] = result == -1 ? -1.0f : (float)result / 100.0f;
		}

		//Now for the buttons (not really optimized but should be fine)
		Info.ButtonBank[0] = Info.ButtonBank[1] = Info.ButtonBank[2] = Info.ButtonBank[3] = 0;
		for (int i = 0; i < 32; i++)
		{
			size_t NewBit = js.rgbButtons[i] >> 7; //bring bit 7 to bit 0
			NewBit <<= i;
			Info.ButtonBank[0] |= NewBit;

			NewBit = js.rgbButtons[i + 32] >> 7; //bring bit 7 to bit 0
			NewBit <<= i;
			Info.ButtonBank[1] |= NewBit;

			NewBit = js.rgbButtons[i + 64] >> 7; //bring bit 7 to bit 0
			NewBit <<= i;
			Info.ButtonBank[2] |= NewBit;

			NewBit = js.rgbButtons[i + 128] >> 7; //bring bit 7 to bit 0
			NewBit <<= i;
			Info.ButtonBank[3] |= NewBit;
		}
		return hr == S_OK;
	}
	virtual size_t GetNoJoysticksFound() {return m_JoyInfo.size();}
	virtual const dx_Joystick::JoystickInfo &GetJoyInfo(size_t nr) const {return m_JoyInfo[nr].Common;}
};

BOOL CALLBACK EnumObjectsCallback( const DIDEVICEOBJECTINSTANCE* pdidoi,VOID* pContext )
{
	DirectInput_Joystick *Instance=(DirectInput_Joystick *)pContext;
	assert(pContext);
	return Instance->EnumObjectsCallback(pdidoi);
}

BOOL CALLBACK DIEnumDevicesCallback( LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef) 
{
	DirectInput_Joystick *Instance=(DirectInput_Joystick *)pvRef;
	assert(pvRef);
	return Instance->DIEnumDevicesCallback(lpddi);
}

#pragma region _wrapper methods_
void dx_Joystick::Init(void *hwnd)
{
	//Note: can provide window handle if necessary here
	m_joystick_internal = std::make_shared<DirectInput_Joystick>((HWND)hwnd);
}
size_t dx_Joystick::GetNoJoysticksFound()
{
	return m_joystick_internal->GetNoJoysticksFound();
}
const dx_Joystick::JoystickInfo &dx_Joystick::GetJoyInfo(size_t nr) const
{
	return m_joystick_internal->GetJoyInfo(nr);
}
bool dx_Joystick::read_joystick(size_t nr, JoyState &Info)
{
	return m_joystick_internal->read_joystick(nr, Info);
}

#pragma endregion

}}