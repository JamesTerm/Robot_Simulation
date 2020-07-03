// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#define _CRT_SECURE_NO_WARNINGS
//end---adding our environment here------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <climits>
#include <stdio.h>
#include <cassert>
#include <math.h>
#include <windows.h>
#include <winbase.h>
#include <tchar.h>

#include <algorithm>
#include <functional>
//#include <xfunctional>
#include <memory>
#include <map>
#include <set>
#include <iostream>
#include <utility>
#include <array>
#include <deque>
#include <future>
#include <thread>
#include <chrono>



//I'll see what I need from here
#if 0
// reference additional headers your program requires here
#include "../../main/cpp/Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../../main/cpp/Base/Vec2d.h"
#include "../../main/cpp/Base/Misc.h"
#include "../../main/cpp/Base/Event.h"
#include "../../main/cpp/Base/EventMap.h"
#include "../../main/cpp/Base/Script.h"
#include "../../main/cpp/Common/Entity_Properties.h"
#include "../../main/cpp/Common/Physics_1D.h"
#include "../../main/cpp/Common/Physics_2D.h"
#include "../../main/cpp/Common/Entity2D.h"
#include "../../main/cpp/Common/Goal.h"
#include "../../main/cpp/Common/Ship_1D.h"
#include "../../main/cpp/Common/Ship.h"
#include "../../main/cpp/Common/AI_Base_Controller.h"
#include "../../main/cpp/Common/Vehicle_Drive.h"
#include "../../main/cpp/Common/PIDController.h"
#include "../../main/cpp/Common/Poly.h"
#include "../../main/cpp/Common/Robot_Control_Interface.h"
#include "../../main/cpp/Common/Calibration_Testing.h"
#include "../../main/cpp/Common/Rotary_System.h"
#include "../../main/cpp/Common/Servo_System.h"
#include "../../main/cpp/Base/Joystick.h"
#include "../../main/cpp/Base/JoystickBinder.h"
#include "../../main/cpp/Common/UI_Controller.h"
#include "../../main/cpp/Common/PIDController.h"
//#include "frc/WPILib.h"
#include "../../main/cpp/Base/Joystick.h"
#include "../../main/cpp/Base/JoystickBinder.h"
//#include "Common/InOut_Interface.h"
#include "../../main/cpp/Common/Debug.h"
//TODO enable robot control
#include "../../main/cpp/Common/Robot_Control_Common.h"
#include "../../main/cpp/TankDrive/Tank_Robot.h"
//This was depreciated and integrated ... stubbed out
// #include "TankDrive/src/Tank_Robot_Control.h"
//This isn't needed
// #include "TankDrive/src/Servo_Robot_Control.h"

#include "../../main/cpp/FRC2019_Robot.h"
#include "../../main/cpp/Common/SmartDashboard.h"
#endif