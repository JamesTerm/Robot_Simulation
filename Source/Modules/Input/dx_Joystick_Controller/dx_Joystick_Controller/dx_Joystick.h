#pragma once

namespace Module
{
	namespace Input
	{

//Forward declare
class DirectInput_Joystick;

class dx_Joystick
{
	public:
		struct JoyState 
		{
			size_t JoystickNumber; //Which joystick does this state apply to
			//All floats range from -1.0 to 1.0
			float    lX;                    ///< x-axis position
			float    lY;                    ///< y-axis position
			float    lZ;                    ///< z-axis position
			float    lRx;                   ///< x-axis rotation
			float    lRy;                   ///< y-axis rotation
			float    lRz;                   ///< z-axis rotation
			float    rgSlider[2];          ///< extra axes positions
			float    rgPOV[4];             ///< POV directions in degrees- should be positive integer, if POV is not used it will be -1.0
			///128 buttons, each bank is 32 bits where each bit is the flagged value
			///Typically you will only need the first bank for 32 buttons
			unsigned long  ButtonBank[4];          
		};

		///This provides some generic capabilities of the Joystick.  You can test flags to determine what is available
		struct JoystickInfo
		{
			//These could be handy when displaying which joystick to set for a control
			std::wstring ProductName;
			std::wstring InstanceName;
			enum JoyCapFlag_enum
			{
				fX_Axis	=0x00000001,
				fY_Axis	=0x00000002,
				fZ_Axis	=0x00000004,
				fX_Rot	=0x00000008,
				fY_Rot	=0x00000010,
				fZ_Rot	=0x00000020,
				fSlider0 =0x00000040,
				fSlider1 =0x00000080,
				fPOV_0	=0x00000100,
				fPOV_1	=0x00000200,
				fPOV_2	=0x00000400,
				fPOV_3	=0x00000800
			};
			unsigned long JoyCapFlags;
			int nSliderCount;  // Number of returned slider controls
			int nPOVCount;     // Number of returned POV controls
			int nButtonCount;  // Number of returned buttons
			//Check this to ensure the JoyStick initialized properly (we may just do an assert on it)
			bool bPresent;
		};
		/// \param hwnd Optional windows pointer to handle the case like not listen if the window is minimized
		void Init(void *hwnd=nullptr);
		/// Use this in conjunction with GetJoyInfo to gather all preliminary Joystick information
		size_t GetNoJoysticksFound();
		/// Take care to submit index entry < the number of joysticks available.  This will let you know what axis will be used
		/// As well as the number of sliders, Pov's and buttons
		const JoystickInfo &GetJoyInfo(size_t nr) const;
		///Here is the main polling function which typically gets called during a time change.  Simply call this with the desired enumerated joystick
		///index and retrieve its current state
		/// \return true if successful, so do not read values if it returns false (it may typically return false on initial setup)
		bool read_joystick (size_t nr, JoyState &Info);
private:
	std::shared_ptr< DirectInput_Joystick> m_joystick_internal; //implementation
};

	}
}
