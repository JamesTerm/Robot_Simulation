#include "Base_Includes.h"
#include "misc.h"
#include "Event.h"
#include "EventMap.h"

using namespace Framework::Base;
using namespace std;

Framework::Base::FrameLogger::FrameLogger(string logFileName) : m_lastTime(0.0),m_logFileName(logFileName), m_active(false), m_currRecordedTimeSet(NULL)
{
}
//////////////////////////////////////////////////////////////////////////

void Framework::Base::FrameLogger::ListenForToggleEvent(Event0& ev)
{
	ev.Subscribe(ehl, *this, &Framework::Base::FrameLogger::ToggleActive);
}
//////////////////////////////////////////////////////////////////////////

void Framework::Base::FrameLogger::ListenForTimerUpdate(Event1<double>& timerEv)
{
	timerEv.Subscribe(ehl, *this, &Framework::Base::FrameLogger::TimeChanged);
}
//////////////////////////////////////////////////////////////////////////

Framework::Base::FrameLogger::~FrameLogger()
{
	// Does nothing if already written
	WriteLog();
}

void Framework::Base::FrameLogger::WriteLog()
{
	// If we are recording now, place the list in the time set
	if (m_currRecordedTimeSet)
	{
		if (m_currRecordedTimeSet->size() < 2)
			delete m_currRecordedTimeSet;
		else
			m_allRecordedTimes.push_back(m_currRecordedTimeSet);
		m_currRecordedTimeSet = NULL;
	}

	if (!m_allRecordedTimes.empty())
	{
		// Create a FILE to write to
		FILE* logFile = m_logFileName.empty() ? NULL : fopen(m_logFileName.c_str(), "w");
		if (logFile)
		{
			printf("Writing to logfile %s\n", m_logFileName.c_str());
			fprintf(logFile, "Prev Time,Curr Time,Frame Dur,FrameRate,");
			WriteTitles.Fire(logFile);
			fprintf(logFile, "\n");
		}
		else
			printf("ERROR: Could not write to logfile %s\n", m_logFileName.c_str());

		for (unsigned set = 0; set < m_allRecordedTimes.size(); ++set)
		{
			std::vector<double>* thisSet = m_allRecordedTimes[set];
			// assert that there are at least 2 elements
			if (logFile)
			{
				double lastTime = (*thisSet)[0];
				for (unsigned i = 1; i < thisSet->size(); ++i)
				{
					double currTime = (*thisSet)[i];
					double dTime = currTime-lastTime;	// ASSERT: dTime > 0.0
					fprintf(logFile, "%f,%f,%0.5f,%f,",lastTime,currTime,dTime,1.0/dTime);
					WriteValues.Fire(logFile,lastTime,currTime);
					fprintf(logFile, "\n");
					lastTime = currTime;
				}
				fprintf(logFile, "\n");
			}
			delete thisSet;
		}
		if (logFile)
			fclose(logFile);
		m_allRecordedTimes.clear();
	}
}
//////////////////////////////////////////////////////////////////////////

void Framework::Base::FrameLogger::TimeChanged(double t)
{
	// Each time must be greater than the last
	if (t <= m_lastTime) return;
	if (m_active && !m_currRecordedTimeSet)
	{
		// Just starting, do not fire the event
		m_currRecordedTimeSet = new std::vector<double>;
		m_currRecordedTimeSet->push_back(t);
		m_lastTime = t;
		return;
	}
	else if (!m_active && m_currRecordedTimeSet)
	{
		if (m_currRecordedTimeSet->size() < 2)
			delete m_currRecordedTimeSet;
		else
			m_allRecordedTimes.push_back(m_currRecordedTimeSet);
		m_currRecordedTimeSet = NULL;
		return;
	}

	// Fire the event now
	if (m_currRecordedTimeSet)
	{
		m_currRecordedTimeSet->push_back(t);
		TimerEvent.Fire(m_lastTime, t);
	}
	m_lastTime = t;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void Framework::Base::ValueLogger::SetLogger(FrameLogger& fl)
{
	fl.TimerEvent.Subscribe(ehl, *this, &Framework::Base::ValueLogger::TimerEvent);
	fl.WriteTitles.Subscribe(ehl, *this, &Framework::Base::ValueLogger::WriteTitles);
	fl.WriteValues.Subscribe(ehl, *this, &Framework::Base::ValueLogger::WriteValues);
}
//////////////////////////////////////////////////////////////////////////

void Framework::Base::ValueLogger::WriteTitles(FILE* logFile)
{
	// Write our name to the log file
	fprintf(logFile,",%s", m_itemName.c_str());
}

void Framework::Base::ValueLogger::TimerEvent(double lastTime, double currTime)
{
	// Remember our current value
	m_valSet.push_back(V);
}
void Framework::Base::ValueLogger::WriteValues(FILE* logFile, double lastTime, double currTime)
{
	fprintf(logFile,",%0.5f", m_valSet[m_writeIndex]);
	++m_writeIndex;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

Framework::Base::ProfilingLogger::ProfilingLogger(string itemName) : 
	Framework::Base::ValueLogger(itemName), begC(0), endC(0)
{
	assert(false);  
	//QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
}
//////////////////////////////////////////////////////////////////////////

// Call these to write on the next frame
void Framework::Base::ProfilingLogger::Start()
{
	assert(false);  
	//QueryPerformanceCounter((LARGE_INTEGER *)&begC);
}
void Framework::Base::ProfilingLogger::End()
{
	assert(false);
	//QueryPerformanceCounter((LARGE_INTEGER *)&endC);
	V = ((double)(endC-begC)/(double)freq);
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


Framework::Base::Timer::Timer(string logFileName) : Logger(logFileName),_currentTime_s(0.0) 
{
	Logger.ListenForTimerUpdate(CurrTimeChanged);
}
//////////////////////////////////////////////////////////////////////////

Framework::Base::Timer::~Timer()
{
}
//////////////////////////////////////////////////////////////////////////

double Framework::Base::Timer::SetCurrTime_s(double t)
{
	if (!Equals(t,_currentTime_s))
	{
		_currentTime_s = t;
		CurrTimeChanged.Fire(_currentTime_s);
	}
	return _currentTime_s;
}
//////////////////////////////////////////////////////////////////////////

double Framework::Base::Timer::IncTime_s(double i_s)
{
	if (i_s)
	{
		_currentTime_s += i_s;
		CurrTimeChanged.Fire(_currentTime_s);
	}
	return _currentTime_s;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void EventMap::SetKB_Controlled(bool controlled)
{
	if (controlled != m_KB_Controlled)
	{
		m_KB_Controlled = controlled;
		KB_Controlled.Fire(this, m_KB_Controlled);
	}
}
//////////////////////////////////////////////////////////////////////////

// initializing the key string and flag string maps
//std::map<std::string, int> Key::KEY_STRING_TO_INDEX_MAP;
Key::KeyStringMaps Key::KEY_STRING_MAPS;

Key::KeyStringMaps::KeyStringMaps()
{
#if 0
   keyStringMap[osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON]   = "MOUSE_LEFT";
   keyStringMap[osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON] = "MOUSE_MIDDLE";
   keyStringMap[osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON]  = "MOUSE_RIGHT";

   keyStringMap[osgGA::GUIEventAdapter::KEY_Space]       = "SPACE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_BackSpace]   = "BACKSPACE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Tab]         = "TAB";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Linefeed]    = "LINEFEED";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Clear]       = "CLEAR";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Return]      = "ENTER";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Pause]       = "PAUSE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Scroll_Lock] = "SCROLL_LOCK";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Sys_Req]     = "SYS_REQ";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Escape]      = "ESCAPE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Delete]      = "DELETE";


   // Cursor control & motion
   keyStringMap[osgGA::GUIEventAdapter::KEY_Home]      = "HOME";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Left]      = "LEFT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Up]        = "UP";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Right]     = "RIGHT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Down]      = "DOWN";
// keyStringMap[osgGA::GUIEventAdapter::KEY_Prior]     = "PRIOR"; duplicate of "PAGE_UP"
   keyStringMap[osgGA::GUIEventAdapter::KEY_Page_Up]   = "PAGE_UP";
// keyStringMap[osgGA::GUIEventAdapter::KEY_Next]      = "NEXT"; duplicate of "PAGE_DOWN"
   keyStringMap[osgGA::GUIEventAdapter::KEY_Page_Down] = "PAGE_DOWN";
   keyStringMap[osgGA::GUIEventAdapter::KEY_End]       = "END";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Begin]     = "BEGIN";

   // Misc Functions
   keyStringMap[osgGA::GUIEventAdapter::KEY_Select]        = "SELECT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Print]         = "PRINT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Execute]       = "EXECUTE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Insert]        = "INSERT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Undo]          = "UNDO";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Redo]          = "REDO";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Menu]          = "MENU";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Find]          = "FIND";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Cancel]        = "CANCEL";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Help]          = "HELP";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Break]         = "BREAK";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Mode_switch]   = "MODE_SWITCH";
// keyStringMap[osgGA::GUIEventAdapter::KEY_Script_switch] = "SCRIPT_SWITCH"; duplicate of "MODE_SWITCH"
   keyStringMap[osgGA::GUIEventAdapter::KEY_Num_Lock]      = "NUM_LOCK";

   // Keypad Functions & keypad numbers
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Space]     = "KP_SPACE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Tab]       = "KP_TAB";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Enter]     = "KP_ENTER";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_F1]        = "KP_F1";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_F2]        = "KP_F2";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_F3]        = "KP_F3";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_F4]        = "KP_F4";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Home]      = "KP_HOME";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Left]      = "KP_LEFT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Up]        = "KP_UP";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Right]     = "KP_RIGHT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Down]      = "KP_DOWN";
// keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Prior]     = "KP_PRIOR"; duplicate of "KP_PAGE_UP"
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Page_Up]   = "KP_PAGE_UP";
// keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Next]      = "KP_NEXT"; duplicate of "KP_PAGE_DOWN"
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Page_Down] = "KP_PAGE_DOWN";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_End]       = "KP_END";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Begin]     = "KP_BEGIN";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Insert]    = "KP_INSERT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Delete]    = "KP_DELETE";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Equal]     = "KP_EQUAL";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Multiply]  = "KP_MULTIPLY";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Add]       = "KP_ADD";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Separator] = "KP_SEPERATOR";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Subtract]  = "KP_SUBTRACT";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Decimal]   = "KP_DECIMAL";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_Divide]    = "KP_DIVIDE";

   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_0] = "KP_0";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_1] = "KP_1";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_2] = "KP_2";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_3] = "KP_3";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_4] = "KP_4";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_5] = "KP_5";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_6] = "KP_6";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_7] = "KP_7";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_8] = "KP_8";
   keyStringMap[osgGA::GUIEventAdapter::KEY_KP_9] = "KP_9";
   
   // Auxiliary Functions
   keyStringMap[osgGA::GUIEventAdapter::KEY_F1]  = "F1";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F2]  = "F2";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F3]  = "F3";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F4]  = "F4";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F5]  = "F5";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F6]  = "F6";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F7]  = "F7";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F8]  = "F8";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F9]  = "F9";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F10] = "F10";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F11] = "F11";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F12] = "F12";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F13] = "F13";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F14] = "F14";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F15] = "F15";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F16] = "F16";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F17] = "F17";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F18] = "F18";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F19] = "F19";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F20] = "F20";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F21] = "F21";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F22] = "F22";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F23] = "F23";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F24] = "F24";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F25] = "F25";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F26] = "F26";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F27] = "F27";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F28] = "F28";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F29] = "F29";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F30] = "F30";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F31] = "F31";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F32] = "F32";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F33] = "F33";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F34] = "F34";
   keyStringMap[osgGA::GUIEventAdapter::KEY_F35] = "F35";

   // Modifiers
   keyStringMap[osgGA::GUIEventAdapter::KEY_Shift_L]    = "SHIFT_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Shift_R]    = "SHIFT_R";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Control_L]  = "CTRL_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Control_R]  = "CTRL_R";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Caps_Lock]  = "CAPS_LOCK";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Shift_Lock] = "SHIFT_LOCK";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Meta_L]     = "META_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Meta_R]     = "META_R";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Alt_L]      = "ALT_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Alt_R]      = "ALT_R";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Super_L]    = "SUPER_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Super_R]    = "SUPER_R";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Hyper_L]    = "HYPER_L";
   keyStringMap[osgGA::GUIEventAdapter::KEY_Hyper_R]    = "HYPER_R";
#endif
   
   // ascii
   std::string ascii = "";
   for (char c = ' '; c <= '~'; c++)
   {
      ascii = c;
      keyStringMap[c] = ascii.c_str();
   }

   std::map<int, std::string>::iterator keyStrMapItr;
   for (keyStrMapItr = keyStringMap.begin(); keyStrMapItr != keyStringMap.end(); keyStrMapItr++)
      stringKeyMap[(*keyStrMapItr).second] = (*keyStrMapItr).first;
}

bool Key::isShiftChar (char c)
{
   return
   (
      (c >= 'A'  && c <= 'Z') || c == '<'  || c == '>' ||
       c == '?'  || c == ':'  || c == '\"' || c == '{' ||
       c == '}'  || c == '|'  || c == '+'  || c == '_' ||
       c == '('  || c == ')'  || c == '*'  || c == '&' ||
       c == '^'  || c == '%'  || c == '$'  || c == '#' ||
       c == '@'  || c == '!'  || c == '~'
   );
}

char Key::unShiftChar (char c)
{
   if (c >= 'A' && c <= 'Z')
      return (c + 32);
   else
   {
      switch (c)
      {
         case '<' : return ',' ;
         case '>' : return '.' ;
         case '?' : return '/' ;
         case ':' : return ';' ;
         case '\"': return '\'';
         case '{' : return '[' ;
         case '}' : return ']' ;
         case '|' : return '\\';
         case '+' : return '=' ;
         case '_' : return '-' ;
         case ')' : return '0' ;
         case '(' : return '9' ;
         case '*' : return '8' ;
         case '&' : return '7' ;
         case '^' : return '6' ;
         case '%' : return '5' ;
         case '$' : return '4' ;
         case '#' : return '3' ;
         case '@' : return '2' ;
         case '!' : return '1' ;
         case '~' : return '`' ;
      }
   }
   return c;
}
