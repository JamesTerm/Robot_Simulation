#pragma region _includes_
#include "stdafx.h"
#include <list>
#include <conio.h>
#include "OSG_Viewer.h"
#include "SwerveRobot_UI.h"

#include <osgUtil/Optimizer>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osg/Geode>
#include <osg/Camera>
#include <osg/ShapeDrawable>
#include <osg/Sequence>
#include <osg/PolygonMode>
#include <osg/io_utils>

#include <osgText/Font>
#include <osgText/Text>

#include <osg\Vec3>
#include <osg\Quat>
#include <osg/NodeCallback>
#include <OpenThreads/Mutex>
#include <OpenThreads/Thread>

//TODO find out why I can't use release OSG for debug builds, then I can make my own macro here

#ifndef _DEBUG
#pragma comment (lib,"OpenThreads.lib")
#pragma comment (lib,"osg.lib")
#pragma comment (lib,"osgUtil.lib")
#pragma comment (lib,"osgDB.lib")
#pragma comment (lib,"osgText.lib")
#pragma comment (lib,"osgGA.lib")
#pragma comment (lib,"osgViewer.lib")
#else
#pragma comment (lib,"OpenThreadsd.lib")
#pragma comment (lib,"osgd.lib")
#pragma comment (lib,"osgUtild.lib")
#pragma comment (lib,"osgDBd.lib")
#pragma comment (lib,"osgTextd.lib")
#pragma comment (lib,"osgGAd.lib")
#pragma comment (lib,"osgViewerd.lib")
#endif

#pragma comment (lib,"OpenGL32.lib")

#pragma endregion

#pragma region _Robot Tester (framework)_
//Robot Tester.h
#pragma region _GG_FrameWork_UI_
#pragma region _Useful Constants_
#define __UseSingleThreadMainLoop__
///////////////////
// Useful Constants
const double FRAMES_PER_SEC = 30.0;
#define TIME_2_FRAME(time) ((int)(((time)*FRAMES_PER_SEC)+0.5))
#define FRAME_2_TIME(frame) ((double)((double)(frame)/FRAMES_PER_SEC))
#pragma endregion

#pragma region _GG Framework Base_
//#include "..\Base\GG_Framework.Base.h"
#define FRAMEWORK_BASE_API
#define FRAMEWORK_UI_API
#define FRAMEWORK_UI_OSG_API
//#include "Thread.h"
#pragma region _Thread_

namespace GG_Framework
{
	namespace Base
	{
		FRAMEWORK_BASE_API void ThreadSleep(unsigned sleepMS);  //defined at bottom since its a win32 call

		class FRAMEWORK_BASE_API RefMutexWrapper
		{
		public:
			RefMutexWrapper(OpenThreads::Mutex& mutex) : m_mutex(mutex) { m_mutex.lock(); }
			~RefMutexWrapper() { m_mutex.unlock(); }

		private:
			OpenThreads::Mutex& m_mutex;
		};

		class FRAMEWORK_BASE_API MutexWrapper : RefMutexWrapper
		{
		public:
			MutexWrapper() : RefMutexWrapper(m_ownMutex) {}
		private:
			OpenThreads::Mutex m_ownMutex;
		};

		class FRAMEWORK_BASE_API ThreadedClass : public OpenThreads::Thread
		{
		public:
			virtual void run()
			{
				try
				{
					tryRun();
				}
				catch (std::exception & exc)
				{
					std::cout << "*** UNRECOVERABLE ERROR: " << exc.what() << std::endl;
					ERROR_STATE = true;
				}
				catch (const char* msg)
				{
					std::cout << "*** UNRECOVERABLE ERROR: " << msg << std::endl;
					ERROR_STATE = true;
				}
				catch (const std::string& msg)
				{
					std::cout << "*** UNRECOVERABLE ERROR: " << msg << std::endl;
					ERROR_STATE = true;
				}
				catch (...)
				{
					std::cout << "*** UNRECOVERABLE ERROR: Unknown Error Type" << std::endl;
					ERROR_STATE = true;
				}
			}
			static bool ERROR_STATE;

		protected:
			virtual void tryRun() = 0;
		};

		bool GG_Framework::Base::ThreadedClass::ERROR_STATE = false;
	}
}
#pragma endregion
//#include "ClassFactory.h"
//#include "FileOpenSave.h"
//#include "Misc.h"
#pragma region _Misc_
typedef std::map<std::string, std::string, std::greater<std::string> > StringMap;

namespace GG_Framework
{
	namespace Base
	{

		FRAMEWORK_BASE_API std::string BuildString(const char *format, ...);  //win32 implemented below
		FRAMEWORK_BASE_API void DebugOutput(const char *format, ...);
		FRAMEWORK_BASE_API char* GetLastSlash(char* fn, char* before = NULL);
		FRAMEWORK_BASE_API std::string GetContentDir_FromFile(const char* fn);

		// Parses name=value pairs, stripping whitespace and comments starting with '#'
		// Returns true if file was opened properly
		FRAMEWORK_BASE_API bool ReadStringMapFromIniFile(std::string filename, StringMap& resMap);
		FRAMEWORK_BASE_API void StripCommentsAndTrailingWhiteSpace(char* line);
		FRAMEWORK_BASE_API std::string TrimString(const std::string& StrToTrim);

		//! Returns false iff c == [ 'f', 'F', 'n', 'N', '0', 0 ]
		FRAMEWORK_BASE_API bool ParseBooleanFromChar(char c);

		//! Trying to keep the Win32 FindFirstFile stuff wrapped
		//! -4 for neither file exists
		//! -3 the first file does not exist but the second does
		//! -2 the second file does not exist but the first does
		//! -1 First file time is earlier than second file time.
		//! 0 The times are the same
		//! 1 the second file time is earlier than the first
		FRAMEWORK_BASE_API int CompareFileLastWriteTimes(const char* f1, const char* f2);

		// Internal methods
		struct FRAMEWORK_BASE_API track_memory_impl
		{
			static void add(void* p_data, const char* p_name);
			static void remove(void* p_data, const char* p_name);
		};

#ifdef _DEBUG

		// In Debug mode we track memory
		template<  typename class_name >
		struct track_memory
		{			// Constructor
			track_memory(void) { track_memory_impl::add(this, typeid(class_name).name()); }

			// Destructor
			~track_memory(void) { track_memory_impl::remove(this, typeid(class_name).name()); }
		};

#else _DEBUG

		// In release mode we compile out to nothing
		template< const typename class_name >
		struct track_memory {};

#endif _DEBUG

	};
};

// A templated averager, make sure the type being averaged can handle the +, -, and / functions
template<class T, unsigned NUMELEMENTS>
class Averager
{
public:
	Averager() : m_array(NULL), m_currIndex((unsigned)-1)
	{
		if (NUMELEMENTS > 1)
			m_array = new T[NUMELEMENTS];
	}
	virtual ~Averager() { if (m_array) delete[] m_array; }
	T GetAverage(T newItem)
	{
		if (!m_array)	// We are not really using the Averager
			return newItem;

		// If the first time called, set up the array and use this value
		if (m_currIndex == -1)
		{
			m_array[0] = newItem;
			m_currIndex = -2;
			m_sum = newItem;
			return newItem;
		}
		else if (m_currIndex < -1)
		{
			// We have not populated the array for the first time yet, still populating
			m_sum += newItem;
			int arrayIndex = (m_currIndex*-1) - 1;
			m_array[arrayIndex] = newItem;

			// Still counting backwards unless we have filled all of the array
			if (arrayIndex == (NUMELEMENTS - 1))	// This means we have filled the array
				m_currIndex = 0;				// Start taking from the array next time
			else
				--m_currIndex;

			// Return the average based on what we have counted so far
			return (m_sum / (arrayIndex + 1));
		}
		else // 0 or greater, we have filled the array
		{
			m_sum += newItem;
			m_sum -= m_array[m_currIndex];
			m_array[m_currIndex] = newItem;
			++m_currIndex;
			if (m_currIndex == NUMELEMENTS)
				m_currIndex = 0;
			return (m_sum / NUMELEMENTS);
		}
	}

	void Reset() { m_currIndex = -1; }

private:
	T* m_array;
	int m_currIndex;
	T m_sum;
};

//Threshold average only changes to new value when that value has been requested THRESHOLD times
template<class T, unsigned THRESHOLD>
class Threshold_Averager
{
public:
	Threshold_Averager() : m_Count(-1) {}
	T GetValue(T newItem)
	{
		//first time init with newItem
		if (m_Count == (size_t)-1)
		{
			m_CurrentItem = m_LastRequestedItem = newItem;
			m_Count = 0;
		}
		//T ret=m_CurrentItem;   hmmm not used
		if (newItem != m_CurrentItem)
		{
			if (newItem == m_LastRequestedItem)
			{
				m_Count++;
				if (m_Count > THRESHOLD)
					m_CurrentItem = newItem, m_Count = 0;
			}
			else
				m_Count = 0;
			m_LastRequestedItem = newItem;
		}
		return m_CurrentItem;
	}
private:
	size_t m_Count;
	T m_CurrentItem, m_LastRequestedItem;
};

//Here is a very light weight averager that uses the blend technique to obtain an average.  Designed to easily replace Averager.
template<class T>
class Blend_Averager
{
public:
	/// \param SmoothingValue a range from 0.0 - 1.0 representing percentage of new item to factor in  
	Blend_Averager(double SmoothingValue = 0.5) : m_SmoothingValue(-1.0), m_DefaultSmoothingValue(SmoothingValue) {}
	//get and set smoothing value  (optional)
	//exposing this allows for dynamic smoothing
	double &GetSmoothingValue() { return m_SmoothingValue; }
	T operator()(T newItem)
	{
		if (m_SmoothingValue != -1.0)
			m_CurrentValue = ((newItem * m_SmoothingValue) + (m_CurrentValue  * (1.0 - m_SmoothingValue)));
		else
		{
			m_SmoothingValue = m_DefaultSmoothingValue;
			m_CurrentValue = newItem;
		}
		return m_CurrentValue;
	}
	void Reset() { m_SmoothingValue = -1; }
private:
	double m_SmoothingValue, m_DefaultSmoothingValue;
	T m_CurrentValue;
};


class Priority_Averager
{
private:
	std::priority_queue<double> m_queue;
	const size_t m_SampleSize;
	const double m_PurgePercent;

	double m_CurrentBadApple_Percentage;
	size_t m_Iteration_Counter;
	void flush()
	{
		while (!m_queue.empty())
			m_queue.pop();
	}
public:
	Priority_Averager(size_t SampleSize, double PurgePercent) : m_SampleSize(SampleSize), m_PurgePercent(PurgePercent),
		m_CurrentBadApple_Percentage(0.0), m_Iteration_Counter(0)
	{
	}
	double operator()(double newItem)
	{
		m_queue.push(newItem);
		double ret = m_queue.top();
		if (m_queue.size() > m_SampleSize)
			m_queue.pop();
		//Now to manage when to purge the bad apples
		m_Iteration_Counter++;
		if ((m_Iteration_Counter % m_SampleSize) == 0)
		{
			m_CurrentBadApple_Percentage += m_PurgePercent;
			//printf(" p=%.2f ",m_CurrentBadApple_Percentage);
			if (m_CurrentBadApple_Percentage >= 1.0)
			{
				//Time to purge all the bad apples
				flush();
				m_queue.push(ret);  //put one good apple back in to start the cycle over
				m_CurrentBadApple_Percentage -= 1.0;
				//printf(" p=%.2f ",m_CurrentBadApple_Percentage);
			}
		}
		return ret;
	}
};

template<class T>
__inline T Enum_GetValue(const char *value, const char * const Table[], size_t NoItems)
{
	assert(value);  //If this fails... somebody forgot to enter a value 
	T ret = (T)-1;
	for (size_t i = 0; i < NoItems; i++)
	{
		if (strcmp(value, Table[i]) == 0)
			ret = (T)i;
	}
	return ret;
}


#define SCRIPT_TEST_BOOL_YES(x,y)  			err = script.GetField(y,&sTest,NULL,NULL);\
	x=false;\
	if (!err)\
{\
	if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))\
	x=true;\
}

#define SCRIPT_TEST_BOOL_YES_NoDefault(x,y)  			err = script.GetField(y,&sTest,NULL,NULL);\
	if (!err)\
{\
	if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))\
	x=true;\
}


#define SCRIPT_TEST_BOOL_NO(x,y)  			err = script.GetField(y,&sTest,NULL,NULL);\
	x=true;\
	if (!err)\
{\
	if ((sTest.c_str()[0]=='n')||(sTest.c_str()[0]=='N')||(sTest.c_str()[0]=='0'))\
	x=false;\
}


#define SCRIPT_INIT_DOUBLE(x,y) err=script.GetField(y, NULL, NULL, &fValue); \
	if (!err) x=fValue;

#define SCRIPT_INIT_DOUBLE_NoDefault(x,y) err=script.GetField(y, NULL, NULL, &fValue); \
	if (!err) x=fValue; \
	else if (!NoDefaults) x=0.0;

#define SCRIPT_INIT_DOUBLE2_NoDefault(x,y,z) err=script.GetField(y, NULL, NULL, &fValue); \
	if (!err) x=fValue; \
	else if (!NoDefaults) x=z;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PIF
#define M_PIF 3.141592654f
#endif
#define M_PID 3.14159265358979323846

#define DEG_2_RAD(x)		((x)*M_PI/180.0)
#define RAD_2_DEG(x)		((x)*180.0/M_PI)
#define ARRAY_SIZE(things)	((sizeof(things)/sizeof(*(things))))

#define Inches2Meters(x)	((x)*0.0254)
#define Feet2Meters(x)		((x)*0.3048)
#define Meters2Feet(x)		((x)*3.2808399)
#define Meters2Inches(x)	((x)*39.3700787)
#define MIN(a,b)			((a)<(b)?(a):(b))
#define MAX(a,b)			((a)>(b)?(a):(b))

#ifndef NULL
#define NULL 0
#endif

#ifndef null
#define null 0
#endif

//! Easily compare doubles with a delta
inline bool Equals(double d1, double d2)
{
	double tol = (d2 > 0.0) ? d2 * 0.000001 : -d2 * 0.000001;
	double delta = (d1 > d2) ? d1 - d2 : d2 - d1;
	return (delta < tol);
}
inline bool Equals(float d1, float d2)
{
	double tol = (d2 > 0.0f) ? d2 * 0.000001f : -d2 * 0.000001f;
	double delta = (d1 > d2) ? d1 - d2 : d2 - d1;
	return (delta < tol);
}

//Use this to check for zero if your value is going to be used as a denominator in division
inline bool IsZero(double value, double tolerance = 1e-5)
{
	return fabs(value) < tolerance;
}

inline double RAND_GEN(double minR = 0.0, double maxR = 1.0)
{
	return minR + (maxR - minR) * ((double)(rand()) / (double)(RAND_MAX));
}

#define wchar2char(wchar2char_pwchar_source) \
	const size_t wchar2char_Length=wcstombs(NULL,wchar2char_pwchar_source,0)+1; \
	char *wchar2char_pchar = (char *)_alloca(wchar2char_Length);; /* ";;" is needed to fix a compiler bug */ \
	wcstombs(wchar2char_pchar,wchar2char_pwchar_source,wchar2char_Length);

#define char2wchar(char2wchar_pchar_source) \
	const size_t char2wchar_Length=((strlen(char2wchar_pchar_source)+1)*sizeof(wchar_t)); \
	wchar_t *char2wchar_pwchar = (wchar_t *)_alloca(char2wchar_Length);; /* ";;" is needed to fix a compiler bug */ \
	mbstowcs(char2wchar_pwchar,char2wchar_pchar_source,char2wchar_Length);

#define _aligned_alloca( size, alignement ) ( (void*)( ((size_t)((alignement)-1)+(size_t)_alloca( (size)+(alignement) ))&(~((alignement)-1)) ) )
#pragma endregion
//#include "ASSERT.h"
#pragma region _ASSERT_
#ifdef NDEBUG
#include <stdexcept>
#define ASSERT(cond) if (!(cond)) throw std::exception((#cond));
#define ASSERT_MSG(cond, msg) if (!(cond)) throw std::exception((msg));
#else  // #ifndef NDEBUG
#include <assert.h>
#define ASSERT(cond) assert(cond);
#define ASSERT_MSG(cond, msg) if (!(cond)){printf((msg)); assert(cond);}
#endif // #ifdef NDEBUG
#pragma endregion
//#include "Event.h"
#pragma region _Event.h_

class IEvent
{
protected:
	class IEventHandler;
public:
	virtual ~IEvent() {}

	class HandlerList
	{
		friend IEvent::IEventHandler;
	public:
		~HandlerList()
		{
			std::list<IEvent::IEventHandler*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
			{
				IEventHandler* eh = *pos;
				eh->_ehl = NULL;		// Set to NULL to avoid circular problems
				IEvent& event = eh->_event;
				event.RemoveEventHandler(eh);	// Note that this will DELETE the eh
			}
			_handlerList.clear();
		}
	private:
		std::list<IEvent::IEventHandler*>	_handlerList;
		void AddEventHandler(IEvent::IEventHandler* eh) { _handlerList.push_back(eh); }
		void RemoveEventHandler(IEvent::IEventHandler* eh) { _handlerList.remove(eh); }
	};

protected:

	class IEventHandler
	{
	public:
		IEventHandler(IEvent& event, IEvent::HandlerList& ehl) :
			_event(event), _ehl(&ehl) {
			_ehl->AddEventHandler(this);
		}
		virtual ~IEventHandler() { if (_ehl) _ehl->RemoveEventHandler(this); }
	private:
		IEvent::HandlerList* _ehl;
		IEvent& _event;

		friend HandlerList;
	};

	virtual void RemoveEventHandler(IEventHandler* eh) = 0;
	friend HandlerList;
};

class Event0 : public IEvent
{
public:

	virtual ~Event0() { ClearAllHandlers(); }

	void ClearAllHandlers()
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler0* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire()
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler0* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire();
		}
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)())
	{
#ifdef _DEBUG
		// Make sure there is only one event handler from this client with this delegate
		{
			std::list<IEventHandler0*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end();)
			{
				EventHandler0<T>* posPtr = dynamic_cast<EventHandler0<T>*>(*pos);
				if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
				{
					ASSERT_MSG(false, "Event0::Subscribe() double entry");
				}
				else
					pos++;
			}
		}
#endif
		_handlerList.push_back(new EventHandler0<T>(*this, ehl, client, delegate));
	}

	template<class T>
	void Remove(T& client, void (T::*delegate)())
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler0<T>* posPtr = dynamic_cast<EventHandler0<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		ASSERT_MSG(false, "Event0::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler0* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

private:
	class IEventHandler0 : public IEventHandler
	{
	public:
		IEventHandler0(Event0& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire() = 0;
	};

	template<class T>
	class EventHandler0 : public IEventHandler0
	{
	public:
		EventHandler0(Event0& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)()) :
			IEventHandler0(event, ehl), _client(client), _delegate(delegate) {}
		virtual void Fire() { (_client.*_delegate)(); }

		T& _client;
		void (T::*_delegate)();
	};

	std::list<IEventHandler0*>	_handlerList;
};




template<class P1>
class Event1 : public IEvent
{
private:
	class IEventHandler1 : public IEventHandler
	{
	public:
		IEventHandler1(Event1& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1) = 0;
	};

	template<class T>
	class EventHandler1 : public IEventHandler1
	{
	public:
		EventHandler1(Event1& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1)) :
			IEventHandler1(event, ehl), _client(client), _delegate(delegate) {}
		virtual void Fire(P1 p1) { (_client.*_delegate)(p1); }

		T& _client;
		void (T::*_delegate)(P1 p1);
	};

	std::list<IEventHandler1*>	_handlerList;
protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler1* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}
public:

	virtual ~Event1() { ClearAllHandlers(); }

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler1* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1)
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler1* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1);
		}
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1))
	{
#ifdef _DEBUG
		// Make sure there is only one event handler from this client with this delegate
		{
			typename std::list<IEventHandler1*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end();)
			{
				EventHandler1<T>* posPtr = dynamic_cast<EventHandler1<T>*>(*pos);
				if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
				{
					ASSERT_MSG(false, "Event1::Subscribe() double entry");
				}
				else
					pos++;
			}
		}
#endif
		_handlerList.push_back(new EventHandler1<T>(*this, ehl, client, delegate));
	}

	template<class T>
	void Remove(T& client, void (T::*delegate)(P1 p1))
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler1<T>* posPtr = dynamic_cast<EventHandler1<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		ASSERT_MSG(false, "Event1::Remove() failed to find the handler");
	}
};





template<class P1, class P2>
class Event2 : public IEvent
{
public:

	virtual ~Event2() { ClearAllHandlers(); }

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler2* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2)
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler2* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2);
		}
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2))
	{
#ifdef _DEBUG
		// Make sure there is only one event handler from this client with this delegate
		{
			typename std::list<IEventHandler2*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end();)
			{
				EventHandler2<T>* posPtr = dynamic_cast<EventHandler2<T>*>(*pos);
				if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
				{
					ASSERT_MSG(false, "Event2::Subscribe() double entry");
				}
				else
					pos++;
			}
		}
#endif
		_handlerList.push_back(new EventHandler2<T>(*this, ehl, client, delegate));
	}

	template<class T>
	void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2))
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler2<T>* posPtr = dynamic_cast<EventHandler2<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		ASSERT_MSG(false, "Event2::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler2* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

private:
	class IEventHandler2 : public IEventHandler
	{
	public:
		IEventHandler2(Event2& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2) = 0;
	};

	template<class T>
	class EventHandler2 : public IEventHandler2
	{
	public:
		EventHandler2(Event2& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2)) :
			IEventHandler2(event, ehl), _client(client), _delegate(delegate) {}
		virtual void Fire(P1 p1, P2 p2) { (_client.*_delegate)(p1, p2); }

		T& _client;
		void (T::*_delegate)(P1 p1, P2 p2);
	};

	std::list<IEventHandler2*>	_handlerList;
};



template<class P1, class P2, class P3>
class Event3 : public IEvent
{
public:

	virtual ~Event3() { ClearAllHandlers(); }

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler3* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2, P3 p3)
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler3* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2, p3);
		}
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3))
	{
#ifdef _DEBUG
		// Make sure there is only one event handler from this client with this delegate
		{
			typename std::list<IEventHandler3*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end();)
			{
				EventHandler3<T>* posPtr = dynamic_cast<EventHandler3<T>*>(*pos);
				if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
				{
					ASSERT_MSG(false, "Event3::Subscribe() double entry");
				}
				else
					pos++;
			}
		}
#endif
		_handlerList.push_back(new EventHandler3<T>(*this, ehl, client, delegate));
	}

	template<class T>
	void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3))
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler3<T>* posPtr = dynamic_cast<EventHandler3<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		ASSERT_MSG(false, "Event3::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler3* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

private:
	class IEventHandler3 : public IEventHandler
	{
	public:
		IEventHandler3(Event3& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3) = 0;
	};

	template<class T>
	class EventHandler3 : public IEventHandler3
	{
	public:
		EventHandler3(Event3& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3)) :
			IEventHandler3(event, ehl), _client(client), _delegate(delegate) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3) { (_client.*_delegate)(p1, p2, p3); }

		T& _client;
		void (T::*_delegate)(P1 p1, P2 p2, P3 p3);
	};

	std::list<IEventHandler3*>	_handlerList;
};





template<class P1, class P2, class P3, class P4>
class Event4 : public IEvent
{
public:

	virtual ~Event4() { ClearAllHandlers(); }

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler4* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2, P3 p3, P4 p4)
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler4* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2, p3, p4);
		}
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4))
	{
#ifdef _DEBUG
		// Make sure there is only one event handler from this client with this delegate
		{
			typename std::list<IEventHandler4*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end();)
			{
				EventHandler4<T>* posPtr = dynamic_cast<EventHandler4<T>*>(*pos);
				if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
				{
					ASSERT_MSG(false, "Event4::Subscribe() double entry");
				}
				else
					pos++;
			}
		}
#endif
		_handlerList.push_back(new EventHandler4<T>(*this, ehl, client, delegate));
	}

	template<class T>
	void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4))
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler4<T>* posPtr = dynamic_cast<EventHandler4<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		ASSERT_MSG(false, "Event4::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler4* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

private:
	class IEventHandler4 : public IEventHandler
	{
	public:
		IEventHandler4(Event4& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3, P4 p4) = 0;
	};

	template<class T>
	class EventHandler4 : public IEventHandler4
	{
	public:
		EventHandler4(Event4& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4)) :
			IEventHandler4(event, ehl), _client(client), _delegate(delegate) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3, P4 p4) { (_client.*_delegate)(p1, p2, p3, p4); }

		T& _client;
		void (T::*_delegate)(P1 p1, P2 p2, P3 p3, P4 p4);
	};

	std::list<IEventHandler4*>	_handlerList;
};



#pragma endregion
//#include "Profiling.h"
//#include "ConfigFileIO_Base.h"
//#include "ConfigFileIO.h"
//#include "Joystick.h"
//#include "EventMap.h"
#pragma region _Event Map_
// GG_Framework.Base EventMap.h
#pragma once
#include <map>
#include <vector>
#include <stdio.h>
//#include <xmlParser.h>
#include <osgGA\GUIEventAdapter>

namespace GG_Framework
{
	namespace Base
	{
		//! Writes a File out with debug Frame Details
		//! Other things can register themselves with the information
		//! Allows other registering items to register to EACH of the events
		//! Writes a CSV (comma separated values) file that can be opened and graphed in EXCEL
		class FRAMEWORK_BASE_API FrameLogger
		{
		public:
			FrameLogger(std::string logFileName)
			{
				//TODO
			}
			~FrameLogger()
			{
				//TODO
			}
			void WriteLog()
			{
				//TODO
			}

			//! Called between time updates, with prior time and current time, ONLY when active
			Event2<double, double> TimerEvent;

			//! Write out your title, using a comma before 
			Event1<FILE*> WriteTitles;
			Event3<FILE*, double, double> WriteValues;

			void ToggleActive() { m_active = !m_active; }
			bool IsActive() { return m_active; }

			void ListenForToggleEvent(Event0& ev)
			{
			}
			void ListenForTimerUpdate(Event1<double>& timerEv)
			{
			}

		private:
			void TimeChanged(double t)
			{
			}
			double m_lastTime;
			std::vector<double>* m_currRecordedTimeSet;
			std::vector<std::vector<double>*> m_allRecordedTimes;
			bool m_active;
			IEvent::HandlerList ehl;
			std::string m_logFileName;
		};
		//////////////////////////////////////////////////////////////////////////

		//! Use this to log any value on a TimerLogger and write its value
		class FRAMEWORK_BASE_API ValueLogger
		{
		public:
			ValueLogger(std::string itemName) : m_itemName(itemName), V(0.0), m_writeIndex(0) {}
			double V; //! Set this value to write on the next frame
			void SetLogger(FrameLogger& fl);

		private:
			void WriteTitles(FILE* logFile);
			void TimerEvent(double lastTime, double currTime);
			void WriteValues(FILE* logFile, double lastTime, double currTime);

			IEvent::HandlerList ehl;
			std::string m_itemName;
			std::vector<double> m_valSet;
			unsigned m_writeIndex;
		};
		//////////////////////////////////////////////////////////////////////////

		class FRAMEWORK_BASE_API ProfilingLogger : public ValueLogger
		{
		public:
			ProfilingLogger(std::string itemName) :
				GG_Framework::Base::ValueLogger(itemName), begC(0), endC(0)
			{
				//TODO define in win32
			}

			// Call these to write on the next frame
			void Start()
			{
			}
			void End()
			{
			}

		private:
			__int64 begC, endC, freq;
		};
		//////////////////////////////////////////////////////////////////////////

		class FRAMEWORK_BASE_API Timer
		{
		public:
			Timer(std::string logFileName) : _currentTime_s(0.0), Logger(logFileName)
			{
				Logger.ListenForTimerUpdate(CurrTimeChanged);
			}
			virtual ~Timer()
			{
			}

			//! Returns the time increment since the last time Fired
			virtual double FireTimer() = 0;

			virtual double ConvertFromNetTime(unsigned int netTime_ms) = 0;
			virtual unsigned int ConvertToNetTime(double time_s) = 0;

			// All of these return the current time plus the scrub
			double GetCurrTime_s() { return _currentTime_s; }
			Event1<double> CurrTimeChanged;

			// FOr debugging only, Sync TImer may be smoothed
			virtual double GetSynchronizedActualTime() { return GetCurrTime_s(); }

			FrameLogger Logger;

		protected:
			double SetCurrTime_s(double t_s)
			{
				if (!Equals(t_s, _currentTime_s))
				{
					_currentTime_s = t_s;
					CurrTimeChanged.Fire(_currentTime_s);
				}
				return _currentTime_s;
			}
			double IncTime_s(double i_s)
			{
				if (i_s)
				{
					_currentTime_s += i_s;
					CurrTimeChanged.Fire(_currentTime_s);
				}
				return _currentTime_s;
			}

		private:
			double _currentTime_s;
		};
		//////////////////////////////////////////////////////////////////////////

		class FRAMEWORK_BASE_API Key
		{
		public:
			int key, flags;
			Key(int _key, int _flags = 0) : key(_key), flags(_flags)
			{
				if (isShiftChar(key))
				{
					key = unShiftChar(key);
					flags |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
				}
			}

			enum { DBL = 0x4000 }; // next bit up from osgGA::GUIEventAdapter::MODKEY_CAPS_LOCK = 0x2000

			class KeyStringMaps
			{
			public:
				std::map<int, std::string> keyStringMap;
				std::map<std::string, int> stringKeyMap;
				KeyStringMaps();
			};
			static KeyStringMaps KEY_STRING_MAPS;

			std::string getKeyString()
			{
				if (KEY_STRING_MAPS.keyStringMap.find(key) != KEY_STRING_MAPS.keyStringMap.end())
					return KEY_STRING_MAPS.keyStringMap[key];
				else
					return "";
			}

			bool isShiftChar(char c); // returns true if the character requires a shift
			char unShiftChar(char c); // returns the unshifted version of a shift character

			bool operator >  (const Key& rhs) const { return ((key == rhs.key) ? (flags > rhs.flags) : (key > rhs.key)); }
			bool operator == (const Key& rhs) const { return (key == rhs.key) && (flags == rhs.flags); }
		};

		struct FRAMEWORK_BASE_API UserInputEvents
		{
			Event2<Key, bool> KBCB_KeyDnUp;
			Event1<Key> KBCB_KeyDn;

			Event2<float, float> MouseMove;
			Event3<float, float, unsigned> MouseBtnPress;
			Event3<float, float, unsigned> MouseBtnRelease;
			Event1<int> MouseScroll;
		};
		//////////////////////////////////////////////////////////////////////////

		class FRAMEWORK_BASE_API EventMap
		{
		public:
			EventMap(bool listOwned = false) :
				m_listOwned(listOwned), m_KB_Controlled(false), AlternateEventTime(NULL)
			{}

			bool IsListOwned() { return m_listOwned; }

			Event2<EventMap*, bool> KB_Controlled;
			bool IsKB_Controlled() { return m_KB_Controlled; }
			void SetKB_Controlled(bool controlled);

			// These are the events that get fired by the KBMCB when we are connected
			UserInputEvents KBM_Events;

			std::map<std::string, Event0, std::greater<std::string> > Event_Map;
			std::map<std::string, Event1<bool>, std::greater<std::string> > EventOnOff_Map;
			std::map<std::string, Event1<double>, std::greater<std::string> > EventValue_Map;

			// Some things are interested in the TIME that an event is fired.  
			// Usually, this TIME is the current time on whatever timer is being used, BUT
			// In situations when events are being fired due to some network event, we want to be able to 
			// specify an event time that is earlier than the current time.
			// 
			// Rather than the huge amount of re-write that would be required to pass in the time to every event fired
			// This public member variable can hold a different event time for those that might be interested.
			// This value should be a pointer to the appropriate time, then reset back to NULL immediately after
			// firing said events.
			double* AlternateEventTime;

		private:
			// Fired for key press and key release from the KBMCB
			bool m_KB_Controlled;
			bool m_listOwned;
		};	// EventMap
		//////////////////////////////////////////////////////////////////////////

	}
}


//! This class is used to hold a collection of EventMaps (onr for each ActorParent) and 
//! Takes care of deleting them when done.
class FRAMEWORK_BASE_API EventMapList : public std::vector<GG_Framework::Base::EventMap*>
{
public:
	~EventMapList()
	{
		// Delete all of the Event Maps this list owns
		for (unsigned i = 0; i < size(); ++i)
		{
			if ((*this)[i]->IsListOwned())
				delete (*this)[i];
		}
	}
};
#pragma endregion
//#include "ArgumentParser.h"
//#include "AVI_Writer.h"
//#include "ReleaseDebugFile.h"
#pragma region _Release Debug File_
namespace GG_Framework
{
	namespace Base
	{
		class FRAMEWORK_BASE_API ReleaseDebugFile
		{
		public:
			ReleaseDebugFile();
			~ReleaseDebugFile();

			std::string MESSAGE;

			static void OutputDebugFile(const char* titlePrefix);
			static ReleaseDebugFile* GetCurrent();

			static bool FilesWritten;

		private:
			int m_threadID;
			ReleaseDebugFile* m_parent;
			static std::map<int, ReleaseDebugFile*> s_DebugFileMap;
			void OutputToFile(FILE* file);
		};

		std::map<int, ReleaseDebugFile*> ReleaseDebugFile::s_DebugFileMap;
		bool ReleaseDebugFile::FilesWritten = false;

	}
}
#pragma endregion
//#include "Time_Type.h"
#pragma endregion

#pragma region _From LightWave helpers_
static osg::Vec3 FromLW_Pos(double X, double Y, double Z) { return osg::Vec3(X, Z, Y); }
static osg::Quat FromLW_Rot(double H, double P, double R)
{
	return osg::Quat(
		DEG_2_RAD(H), osg::Vec3d(0, 0, 1),
		DEG_2_RAD(-P), osg::Vec3d(1, 0, 0),
		DEG_2_RAD(-R), osg::Vec3d(0, 1, 0));
}
static osg::Quat FromLW_Rot_Radians(double H, double P, double R)
{
	return osg::Quat(
		H, osg::Vec3d(0, 0, 1),
		-P, osg::Vec3d(1, 0, 0),
		-R, osg::Vec3d(0, 1, 0));
}
#pragma endregion
#pragma region _Other Libraries
//////////////////
// Other Libraries
//#include "..\..\CompilerSettings.h"
#pragma region _Compiler Settings_
// DLL export warning
#pragma warning ( disable : 4251 ) 

// Using "this" in the constructor init of member vars
#pragma warning ( disable : 4355 )

// Nonstandard extension used : '__restrict' keyword not supported in this product
#pragma warning ( disable : 4235 )

// warning C4799: No EMMS at end of function
#pragma warning ( disable : 4799 )

// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )

// If I cast a pointer to an int, I meant to do it
#pragma warning ( disable : 4311 )

// non dll-interface class used as base for dll-interface class
#pragma warning ( disable : 4275 )

// All the non-implicit type conversion problems
#pragma warning ( disable : 4244 )
#pragma endregion
//#include "..\Base\GG_Framework.Base.h" <--already declared
//#include "OSG\GG_Framework.UI.OSG.h"
#pragma region _Framework UI OSG_
//////////////////
// Other Libraries
//#include "..\..\..\CompilerSettings.h"  <--already declared
//#include "..\..\Base\GG_Framework.Base.h"  <--already declared (we're in it)

///////////////////////
// Fringe.UI.OSG Includes
//#include "Producer_Trackball.h"
//#include "Trackball.h"
//#include "Misc.h"
//#include "SnapImageDrawCallback.h"
//#include "Text.h"
#pragma region _Text_
namespace GG_Framework
{
	namespace UI
	{
		namespace OSG
		{
enum
{
	FONT_BEGIN = 1,
	FONT_NEXT,
	FONT_END,
	FONT_ADVANCE
};
#define MAX_STROKES 256
#define END_OF_LIST 255

#define STROKE_SCALE 1.71f
#define OUTLINE_SCALE 0.023f
#define FILLED_SCALE 0.023f

static GLint strokeFont[126+1][1 + MAX_STROKES * 3];
static GLint outlineFont[126-32+2][1 + MAX_STROKES * 3];
static GLint filledFont[126 - 32 + 2][1 + MAX_STROKES * 3];
static GLubyte bitmapFont[126 - 32 + 2][1 + 13];

class FRAMEWORK_UI_OSG_API Text 
{
public:
	enum Font
	{
		BitmapFont,
		FilledFont,
		OutlineFont,
		StrokeFont
	};
private:
	std::map<Font, GLuint> fontmap;
	GLuint loadFont(Font font)
	{
		GLuint obj = glGenLists(255);
		switch (font)
		{
		case BitmapFont:
			createBitmapFont(obj);
			break;

		case FilledFont:
			createFilledFont(obj);
			break;

		case OutlineFont:
			createOutlineFont(obj);
			break;

		case StrokeFont:
			createStrokeFont(obj);
			break;
		}
		fontmap[font] = obj;
		return obj;
	}

	bool createStrokeFont(GLuint fontBase)
	{
		GLint mode, i, j;

		for (i = 0; strokeFont[i][0] != END_OF_LIST; i++) {
			glNewList(fontBase + (unsigned int)strokeFont[i][0], GL_COMPILE);
			for (j = 1; (mode = strokeFont[i][j]); j += 3) {
				if (mode == FONT_BEGIN) {
					glBegin(GL_LINE_STRIP);
					glVertex2f((float)strokeFont[i][j + 1] * STROKE_SCALE,
						(float)strokeFont[i][j + 2] * STROKE_SCALE);
				}
				else if (mode == FONT_NEXT) {
					glVertex2f((float)strokeFont[i][j + 1] * STROKE_SCALE,
						(float)strokeFont[i][j + 2] * STROKE_SCALE);
				}
				else if (mode == FONT_END) {
					glVertex2f((float)strokeFont[i][j + 1] * STROKE_SCALE,
						(float)strokeFont[i][j + 2] * STROKE_SCALE);
					glEnd();
				}
				else if (mode == FONT_ADVANCE) {
					glTranslatef((float)strokeFont[i][j + 1] * STROKE_SCALE,
						(float)strokeFont[i][j + 2] * STROKE_SCALE, 0.0);
					break;
				}
			}
			glEndList();
		}
		return true;
	}
	bool createOutlineFont(GLuint fontBase)
	{
		GLint mode, i, j;

		for (i = 0; outlineFont[i][0] != END_OF_LIST; i++) {
			glNewList(fontBase + (unsigned int)outlineFont[i][0], GL_COMPILE);
			for (j = 1; (mode = outlineFont[i][j]); j += 3) {
				if (mode == FONT_BEGIN) {
					glBegin(GL_LINE_STRIP);
					glVertex2f((float)outlineFont[i][j + 1] * OUTLINE_SCALE,
						(float)outlineFont[i][j + 2] * OUTLINE_SCALE);
				}
				else if (mode == FONT_NEXT) {
					glVertex2f((float)outlineFont[i][j + 1] * OUTLINE_SCALE,
						(float)outlineFont[i][j + 2] * OUTLINE_SCALE);
				}
				else if (mode == FONT_END) {
					glVertex2f((float)outlineFont[i][j + 1] * OUTLINE_SCALE,
						(float)outlineFont[i][j + 2] * OUTLINE_SCALE);
					glEnd();
				}
				else if (mode == FONT_ADVANCE) {
					glTranslatef((float)outlineFont[i][j + 1] * OUTLINE_SCALE,
						(float)outlineFont[i][j + 2] * OUTLINE_SCALE, 0.0);
					break;
				}
			}
			glEndList();
		}
		return true;
	}
	bool createFilledFont(GLuint fontBase)
	{
		GLint mode, i, j;

		for (i = 0; filledFont[i][0] != END_OF_LIST; i++) {
			glNewList(fontBase + (unsigned int)filledFont[i][0], GL_COMPILE);
			for (j = 1; (mode = filledFont[i][j]); j += 3) {
				if (mode == FONT_BEGIN) {
					glBegin(GL_TRIANGLE_STRIP);
					glVertex2f((float)filledFont[i][j + 1] * FILLED_SCALE,
						(float)filledFont[i][j + 2] * FILLED_SCALE);
				}
				else if (mode == FONT_NEXT) {
					glVertex2f((float)filledFont[i][j + 1] * FILLED_SCALE,
						(float)filledFont[i][j + 2] * FILLED_SCALE);
				}
				else if (mode == FONT_END) {
					glVertex2f((float)filledFont[i][j + 1] * FILLED_SCALE,
						(float)filledFont[i][j + 2] * FILLED_SCALE);
					glEnd();
				}
				else if (mode == FONT_ADVANCE) {
					glTranslatef((float)filledFont[i][j + 1] * FILLED_SCALE,
						(float)filledFont[i][j + 2] * FILLED_SCALE, 0.0);
					break;
				}
			}
			glEndList();
		}
		return true;
	}
	bool createBitmapFont(GLuint fontBase)
	{
		GLint i;

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		for (i = 0; bitmapFont[i][0] != (unsigned char)END_OF_LIST; i++) {
			glNewList(fontBase + (unsigned int)bitmapFont[i][0], GL_COMPILE);
			glBitmap(8, 13, 0.0, 2.0, 10.0, 0.0, &bitmapFont[i][1]);
			glEndList();
		}
		return true;
	}
public:
	Text()
	{
		fontmap.insert(std::pair<Font, GLuint>(BitmapFont, 0));
		fontmap.insert(std::pair<Font, GLuint>(FilledFont, 0));
		fontmap.insert(std::pair<Font, GLuint>(OutlineFont, 0));
		fontmap.insert(std::pair<Font, GLuint>(StrokeFont, 0));
	}
	void drawString(Font font, std::string str)
	{
		GLuint base = fontmap[font];
		if (base == 0)
			base = loadFont(font);
		glPushAttrib(GL_LIST_BIT);
		glListBase(base);
		glCallLists(static_cast<GLsizei>(str.length()), GL_UNSIGNED_BYTE, (unsigned char *)str.c_str());
		glPopAttrib();
	}
};
		}
	}
}
#pragma endregion
//#include "ICamera.h"
#pragma region _ICamera_
namespace GG_Framework
{
	namespace UI
	{
		namespace OSG
		{
			class ICamera;
			class ICameraManipulator
			{
			public:
				//! This is called once per frame when the camera is active
				//! use it to update the matrix of the camera (or anything else)
				//! you can also bounce out and do nothing if there are no changes.
				virtual void UpdateCamera(ICamera* activeCamera, double dTime_s) = 0;
			};

			class FRAMEWORK_UI_OSG_API ICamera
			{
			protected:
				ICameraManipulator* m_camManip;
			public:
				ICamera() : m_camManip(NULL) {}
				virtual void setClearColor(const osg::Vec4& c) = 0;
				virtual void getClearColor(osg::Vec4& c) = 0;

				virtual void addPostDrawCallback(osg::Camera::DrawCallback& cb) = 0;
				virtual void removePostDrawCallback(osg::Camera::DrawCallback& cb) = 0;

				virtual void setFinalDrawCallback(osg::Camera::DrawCallback* cb) = 0;

				virtual osg::Matrix GetCameraMatrix() const = 0;
				virtual void SetMatrix(const osg::Matrix& camMatrix, float distortionAmt) = 0;
				virtual osg::Node* GetSceneNode() = 0;
				virtual void SetSceneNode(osg::Node* node, float distortionPCNT) = 0;

				virtual void SetCameraManipulator(ICameraManipulator* cameraManip)
				{
					if (cameraManip != m_camManip)
					{
						ICameraManipulator* old = m_camManip;
						m_camManip = cameraManip;
						CamManipChanged.Fire(this, old, m_camManip);
					}
				}
				Event3<ICamera*, ICameraManipulator*, ICameraManipulator*> CamManipChanged; // (this, old, new)
				ICameraManipulator* GetCameraManipulator() { return m_camManip; }

				virtual void Update(double dTime_s)
				{
					if (m_camManip)
						m_camManip->UpdateCamera(this, dTime_s);
				}
				Event1<const osg::Matrix&> MatrixUpdate;
			};
		}
	}	//end namespace OSG
}	//end namespace FrameWork
#pragma endregion
//#include "Timer.h"
#pragma region _Timer_
namespace GG_Framework
{
	namespace UI
	{
		namespace OSG
		{

			class FRAMEWORK_UI_OSG_API OSG_Timer : public GG_Framework::Base::Timer
			{
			private:
				osg::Timer m_OSG_timer;
				osg::Timer_t m_lastTimerTick;
			public:
				OSG_Timer(std::string logFileName) : GG_Framework::Base::Timer(logFileName)
				{
					m_lastTimerTick = m_OSG_timer.tick();
				}
				virtual ~OSG_Timer()
				{
				}
				virtual double FireTimer()
				{
					osg::Timer_t thisTimerTick = m_OSG_timer.tick();
					double dTick_s = m_OSG_timer.delta_s(m_lastTimerTick, thisTimerTick);
					m_lastTimerTick = thisTimerTick;
					IncTime_s(dTick_s);
					return dTick_s;
				}

				virtual double ConvertFromNetTime(unsigned int netTime_ms) { return (double)netTime_ms / 1000.0; }
				virtual unsigned int ConvertToNetTime(double time_s) { return (unsigned int)(time_s * 1000.0); }
			};
		}
	}
}
#pragma endregion
//#include "PickVisitor.h"
//#include "VectorDerivativeOverTimeAverager.h"
#pragma region _vector rate over time averager_
namespace GG_Framework
{
	namespace UI
	{
		namespace OSG
		{
			/// Use this class to get an averaged out Velocity vector when the velocity due to displacement
			/// is unstable due to rapidly varying frame delta times and/or position vectors.  Use this class
			/// again to smooth out the acceleration (2nd Derivative).
			class FRAMEWORK_UI_OSG_API VectorDerivativeOverTimeAverager
			{
			private:
				osg::Vec3* m_vecArray;
				double* m_timeArray;
				unsigned m_numSamples;
				unsigned m_currIndex;
				double m_sumTime;
			public:
				VectorDerivativeOverTimeAverager(unsigned numSamples)
				{
					m_numSamples = numSamples;
					ASSERT(m_numSamples > 0);
					m_vecArray = new osg::Vec3[m_numSamples];
					m_timeArray = new double[m_numSamples];
					m_currIndex = (unsigned)-1;
					m_sumTime = 0.0;
				}
				~VectorDerivativeOverTimeAverager()
				{
					delete[] m_vecArray;
					delete[] m_timeArray;
				}
				osg::Vec3 GetVectorDerivative(osg::Vec3 vec, double dTime_s)
				{
					// If this is the very FIRST pass, initialize all of the array with this vec (will return 0)
					if (m_currIndex == (unsigned)-1)
					{
						for (unsigned i = 0; i < m_numSamples; ++i)
						{
							m_vecArray[i] = vec;
							m_timeArray[i] = 0.0;
						}
						m_currIndex = 0;
						m_sumTime = 0.0;
					}

					// Find the total time elapsed
					m_sumTime += dTime_s - m_timeArray[m_currIndex];
					m_timeArray[m_currIndex] = dTime_s;

					// Find the derivative
					osg::Vec3 vecDeriv(0, 0, 0);
					if (m_sumTime > 0.0)
						vecDeriv = (vec - m_vecArray[m_currIndex]) / m_sumTime;
					m_vecArray[m_currIndex] = vec;

					// Get ready for the next pass
					m_currIndex += 1;
					if (m_currIndex == m_numSamples)
						m_currIndex = 0;

					return vecDeriv;
				}
				void Reset() { m_currIndex = (unsigned)-1; }
			};
		}
	}
}
#pragma endregion
//#include "LoadStatus.h"
#pragma region _Load Status_
namespace GG_Framework
{
	namespace UI
	{
		namespace OSG
		{

/// A singleton of this class used to report when different loading states are complete.
/// See \ref Load_Status_txt.
class FRAMEWORK_UI_OSG_API LoadStatus
{
private:
	#pragma region _members_
	std::vector<float> m_predictedLoadTimes;
	std::vector<float> m_actualLoadTimes;
	osg::Timer_t m_initialTimer;
	osg::Timer m_OSG_timer;
	OpenThreads::Mutex m_mutex;
	bool m_complete;
	#pragma endregion
public:
	#pragma region _Load Status Update Event2 member_
	/// This event is called with elapsed time and remaining time.  Remaining time is -1 if no predicted
	/// times are available.  It is provided with 0.0 when the LoadComplete happens.
	/// It will NOT be fired after that last time when LoadComplete is called.
	/// This event is protected with a mutex so that it is thread safe, but the handlers should be
	/// also because the event may be fired from various threads
	Event2<float, float> LoadStatusUpdate;
	#pragma endregion

	static LoadStatus Instance;
	LoadStatus() : m_complete(false) { m_initialTimer = m_OSG_timer.tick(); }
	void TaskComplete()
	{
		/// This function is called several times during the load process
		using namespace GG_Framework::Base;
		// This all has to be thread safe
		RefMutexWrapper rmw(m_mutex);

		// We have already been marked as complete, do not fire more events
		if (m_complete)
			return;

		// Get the current time
		float currTime = m_OSG_timer.delta_s(m_initialTimer, m_OSG_timer.tick());
		m_actualLoadTimes.push_back(currTime);

		// Find the remaining time
		float remTime = -1.0f;	// Defaults to -1.0 s if there are no better predictions
		size_t index = m_actualLoadTimes.size() - 1;
		if (m_predictedLoadTimes.size() > (index + 1))
		{
			float compTime = m_predictedLoadTimes[m_predictedLoadTimes.size() - 1];
			float predTime = m_predictedLoadTimes[index];
			if ((predTime > 0.0f) && (compTime >= predTime))
				remTime = ((currTime / predTime)*compTime) - currTime;
		}

		// Fire the event to show the update
		LoadStatusUpdate.Fire(currTime, remTime);

	}
	void LoadComplete()
	{
		/// This function is called only one time, when the load is REALLY complete
		/// Writes the completion times to a file
		using namespace GG_Framework::Base;
		// This all has to be thread safe
		RefMutexWrapper rmw(m_mutex);

		// We have already been marked as complete, do not fire more events
		if (m_complete)
			return;

		// No longer take any messages
		m_complete = true;

		// Get the current time
		float currTime = m_OSG_timer.delta_s(m_initialTimer, m_OSG_timer.tick());
		m_actualLoadTimes.push_back(currTime);

		// Write the times out to a file
		FILE* outFile = fopen("LOAD_TIMES.txt", "w");
		if (outFile)
		{
			// Write the times in a way easy to paste to LUA
			fprintf(outFile, "LOAD_TIMES = {\n");
			for (size_t i = 0; i < m_actualLoadTimes.size(); ++i)
				fprintf(outFile, "\t%f,\n", m_actualLoadTimes[i]);
			fprintf(outFile, "}\n");
			fclose(outFile);
			outFile = NULL;
		}

		// Fire the event to say the load is complete
		LoadStatusUpdate.Fire(currTime, 0.0f);
	}
	void SetPredictedLoadTimes(std::vector<float>& predLoad)
	{
		/// Called once from the application, sets the predicted times until the load is complete
		/// Also resets the timer
		using namespace GG_Framework::Base;
			// This all has to be thread safe
		RefMutexWrapper rmw(m_mutex);

		// We have already been marked as complete, do not fire more events
		if (m_complete)
			return;

		// If there are no items, it does not help us
		if (predLoad.empty())
			return;

		// We want to keep this list of predicted times
		m_predictedLoadTimes = predLoad;

		// Fire the event to let everyone know about the remaining time
		LoadStatusUpdate.Fire(0.0f, m_predictedLoadTimes[m_predictedLoadTimes.size() - 1]);
	}
};

// The singleton instance
GG_Framework::UI::OSG::LoadStatus GG_Framework::UI::OSG::LoadStatus::Instance;

		}
	}
}
#pragma endregion
#pragma endregion
//#include "Audio\GG_Framework.UI.Audio.h"
#pragma endregion
#pragma region _Fringe UI Includes_
///////////////////////
// Fringe.UI Includes
//#include "EventMap.h"  <--Note: different version than in base
#pragma region _UI EventMap_
namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API EventMap : public GG_Framework::Base::EventMap
		{
		public:
			EventMap(bool listOwned = false) : GG_Framework::Base::EventMap(listOwned) {}

			// Include point based events
			std::map<std::string, Event1<const osg::Vec3d&>, std::greater<std::string> > EventPt_Map;
			std::map<std::string, Event2<const osg::Vec3d&, bool>, std::greater<std::string> > EventPtOnOff_Map;
			std::map<std::string, Event2<const osg::Vec3d&, double>, std::greater<std::string> > EventPtValue_Map;
		};
	}
}
#pragma endregion
//#include "Text_PDCB.h"
#pragma region _Text PDCB_
namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API Text_PDCB : public osg::Camera::DrawCallback
		{
		protected:
			bool m_enabled;
			mutable GG_Framework::UI::OSG::Text m_text;
		public:
			Text_PDCB() : m_enabled(false) {}
			void ToggleEnabled() { m_enabled = !m_enabled; }
			virtual bool IsEnabled() const { return m_enabled; }
			void Enable(bool enable) { m_enabled = enable; }

			virtual void operator () (const osg::Camera &cam) const
			{
				// If nothing is enabled, just pop out
				if (!m_enabled) return;

				double width = cam.getViewport()->width();
				double height = cam.getViewport()->height();

				glViewport(0, 0, width, height);
				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				glLoadIdentity();
				glOrtho(0.0, double(width), 0.0, double(height), -1.0, 1.0);
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glLoadIdentity();

				glPushAttrib(GL_ENABLE_BIT);
				glDisable(GL_TEXTURE_2D);
				glDisable(GL_LIGHTING);
				glDisable(GL_DEPTH_TEST);

				osg::Vec4 color = GetTextColor();
				glColor4f(color[0], color[1], color[2], color[3]);

				osg::Vec2 posn = GetPosition(width, height);
				glRasterPos2f(posn[0], posn[1]);

				std::string text = GetText();
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, text.c_str());

				glPopAttrib();
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(GL_MODELVIEW);
				glPopMatrix();
			}

			//! The text that will be displayed when enabled
			virtual std::string GetText() const = 0;

			//! The position of the lower left of the string, winHeight is top of the window, winWidth is right edge
			//! It appears that each character is about 10 units.
			virtual osg::Vec2 GetPosition(double winWidth, double winHeight) const = 0;

			//! The default text color is yellow
			virtual osg::Vec4 GetTextColor() const { return osg::Vec4(1, 1, 0, 1); }

		};
	}
}
#pragma endregion
//#include "Interfaces.h"
//#include "Effect.h"
//#include "NamedEffect.h"
//#include "Action.h"
//#include "Impulse.h"
//#include "Actor.h"
//#include "ParticleEffect.h"
//#include "ActorScene.h"
//#include "SoundEffect.h"
//#include "CameraParentedTransforms.h"
//#include "LightSource.h"
//#include "ImageNodes.h"
//#include "Camera.h"
#pragma region _Camera_
typedef std::list<osg::ref_ptr<osg::Camera::DrawCallback>* > Ref_List_impl;

class DistortionSubgraph;
namespace GG_Framework
{
	namespace UI
	{
		const unsigned int NUM_DISTORTION_LEVELS = 120;
		const unsigned int tex_width = 1024;
		const unsigned int tex_height = 1024;
		#undef DISABLE_ALL_OPTIMIZATIONS

		class Camera : public GG_Framework::UI::OSG::ICamera
		{
		private:
			osg::ref_ptr<osgViewer::Viewer> m_camGroup;
			class CompositeDrawCallback : public osg::Camera::DrawCallback
			{
			protected:
				virtual ~CompositeDrawCallback()
				{
					GG_Framework::Base::RefMutexWrapper rmw(MyMutex);
					for (Ref_List_impl::iterator i = CallbackList.begin(); i != CallbackList.end(); ++i)
					{
						osg::ref_ptr<osg::Camera::DrawCallback>* ptr = (*i);
						delete ptr;
					}
				}
			public:
				Ref_List_impl CallbackList;
				mutable OpenThreads::Mutex MyMutex;
				virtual void operator () (const osg::Camera& camera) const
				{
					GG_Framework::Base::RefMutexWrapper rmw(MyMutex);
					for (Ref_List_impl::const_iterator i = CallbackList.begin(); i != CallbackList.end(); ++i)
					{
						osg::ref_ptr<osg::Camera::DrawCallback>* ptr = (*i);
						ptr->get()->operator ()(camera);
					}
				}
			};
			osg::ref_ptr<CompositeDrawCallback> m_compositePostDrawCallback;
			DistortionSubgraph* m_distortion;

		public:
			Camera(osgViewer::Viewer* camGroup) : m_camGroup(camGroup)
			{
				ASSERT(m_camGroup.valid());
				// Default to a black background
				m_camGroup->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
				m_compositePostDrawCallback = new Camera::CompositeDrawCallback;

				// Work with a larger near/far ratio
				m_camGroup->getCamera()->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
				m_camGroup->getCamera()->setNearFarRatio(0.00001f);
				m_camGroup->getCamera()->setFinalDrawCallback(m_compositePostDrawCallback.get());
			}
			virtual void setClearColor(const osg::Vec4& c)
			{
				m_camGroup->getCamera()->setClearColor(c);
			}
			virtual void getClearColor(osg::Vec4& c)
			{
				c = m_camGroup->getCamera()->getClearColor();
			}
			virtual void addPostDrawCallback(osg::Camera::DrawCallback& cb)
			{
				GG_Framework::Base::RefMutexWrapper rmw(m_compositePostDrawCallback->MyMutex);
				osg::ref_ptr<osg::Camera::DrawCallback>* rp = new osg::ref_ptr<osg::Camera::DrawCallback>(&cb);
				m_compositePostDrawCallback->CallbackList.push_back(rp);
			}
			virtual void removePostDrawCallback(osg::Camera::DrawCallback& cb)
			{
				GG_Framework::Base::RefMutexWrapper rmw(m_compositePostDrawCallback->MyMutex);
				for (Ref_List_impl::iterator i = m_compositePostDrawCallback->CallbackList.begin(); i != m_compositePostDrawCallback->CallbackList.end(); ++i)
				{
					osg::ref_ptr<osg::Camera::DrawCallback>* ptr = (*i);
					if (ptr->get() == &cb)
					{
						delete ptr;
						m_compositePostDrawCallback->CallbackList.erase(i);
						return;
					}
				}
			}

			virtual void setFinalDrawCallback(osg::Camera::DrawCallback* cb)
			{
				m_camGroup->getCamera()->setFinalDrawCallback(cb);
			}
			virtual void SetMatrix(const osg::Matrix& camMatrix, float distortionAmt)
			{
				m_camGroup->getCamera()->setViewMatrix(camMatrix);
				MatrixUpdate.Fire(camMatrix);
				//Maybe later on this
				//if (m_distortion)
				//	m_distortion->SetDistortionLevel(distortionAmt);
			}
			virtual osg::Matrix GetCameraMatrix() const
			{
				return m_camGroup->getCamera()->getViewMatrix();
			}
			virtual osg::Node* GetSceneNode()
			{
				return m_camGroup->getSceneData();
			}
			virtual void SetSceneNode(osg::Node* node, float distortionPCNT)
			{
				#ifndef DISABLE_ALL_OPTIMIZATIONS
				// optimize the scene graph
				osgUtil::Optimizer optimizer;

				//TODO determine what new optimizations are stable; otherwise the original ones should be fine
				//  [5/15/2009 JamesK]
				#if 1
				unsigned optOptions =	// We do not want all options, because it kills some of our camera point at effects
					//osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS |
					//osgUtil::Optimizer::REMOVE_REDUNDANT_NODES |
					osgUtil::Optimizer::REMOVE_LOADED_PROXY_NODES |
					osgUtil::Optimizer::COMBINE_ADJACENT_LODS |
					osgUtil::Optimizer::SHARE_DUPLICATE_STATE |
					osgUtil::Optimizer::MERGE_GEOMETRY |
					osgUtil::Optimizer::CHECK_GEOMETRY |
					osgUtil::Optimizer::OPTIMIZE_TEXTURE_SETTINGS |
					osgUtil::Optimizer::STATIC_OBJECT_DETECTION;
				#else
				unsigned optOptions = osgUtil::Optimizer::ALL_OPTIMIZATIONS
					// We do not want all options, because it kills some of our camera point at effects
					& ~(
						osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS |
						osgUtil::Optimizer::REMOVE_REDUNDANT_NODES |
						osgUtil::Optimizer::TRISTRIP_GEOMETRY
						);
				#endif
				optimizer.optimize(node, optOptions);
				#endif

				m_distortion = NULL;
				//if (distortionPCNT > 0.0)
				//{
				//	m_distortion = new DistortionSubgraph();
				//	osg::Vec4 color; getClearColor(color);
				//	m_camGroup->setSceneData(m_distortion->createDistortionSubgraph(node, color, distortionPCNT));
				//}
				//else
					m_camGroup->setSceneData(node);

				// this might be a task that could take some time, lets fire a task complete to keep the times updated
				GG_Framework::UI::OSG::LoadStatus::Instance.TaskComplete();

			}
		};
	}
}
#pragma endregion
//#include "ConfigurationManager.h"
//#include "JoystickBinder.h"
//#include "KeyboardMouse_CB.h"
//#include "MainWindow.h"
#pragma region _Main Window_

#define DEBUG_SCREEN_RESIZE // printf
//The AutomaticSelection may have changed over time to multi-threaded
//#define THREADING_MODEL osgViewer::ViewerBase::AutomaticSelection
//Life is good with a simple single thread... no miss fired artifacts, and do not need to worry about critical sections
#define THREADING_MODEL osgViewer::ViewerBase::SingleThreaded

namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API Window
		{
		public:
			#pragma region _ThreadSafeViewer_
			class FRAMEWORK_UI_API ThreadSafeViewer : public osgViewer::Viewer
			{
			public:
				Event0 PostUpdateEvent;
				/*
				virtual void updateTraversal()
				{
					// Just wrap the update traversal in a mutex
					GG_Framework::Base::RefMutexWrapper rmw(Window::UpdateMutex);
					__super::updateTraversal();
					PostUpdateEvent.Fire();
				}
				*/
			};
			#pragma endregion
		private:
			#pragma region _disabled_
			// Attach to events with this, also change where the callbacks go
			//osg::ref_ptr<KeyboardMouse_CB> refKBM;
			//KeyboardMouse_CB *m_Keyboard_Mouse;
			//JoyStick_Binder *m_Joystick;					// Scoped pointer.
			//AudioVolumeControls_PlusBLS *m_VolumeControls;	// Scoped pointer.

			//TODO see if we need this
			//ConfigurationManager m_ConfigurationManager;
			#pragma endregion
		protected:
			#pragma region _members_
			bool m_useAntiAlias;
			osg::ref_ptr<ThreadSafeViewer>	m_camGroup;
			bool m_isFullScreen;
			unsigned m_origScreenWidth, m_origScreenHeight;
			unsigned m_newScreenWidth, m_newScreenHeight;
			int m_lastX, m_lastY;
			int m_lastWidth, m_lastHeight;

			Camera m_mainCam;

			// Used for calculating frame-rate
			double m_frameRate;
			double m_frameRateAvg;
			Averager<double, 30> m_framerateAverager;
			double m_lastFrameRateCheckTime_s;
			int m_frameNum;
			int m_performanceIndex;
			double m_waitForPerformanceIndex_s;
			int m_performanceStrikes;
			GG_Framework::UI::OSG::VectorDerivativeOverTimeAverager m_velocityAvg;
			#pragma endregion
			osgViewer::GraphicsWindow* GetGraphicsWindow()
			{
				osgViewer::ViewerBase::Windows windows;
				m_camGroup->getWindows(windows);
				if (windows.size() > 0)
					return (windows[0]);
				else
					return NULL;
			}
			void UpdateFrameRate(double currTime_s, double lastDrawnFrameDur)
			{
				// Set the first time
				if (m_lastFrameRateCheckTime_s < 0.0)
				{
					m_lastFrameRateCheckTime_s = currTime_s;
					return;
				}

				// Work out the frame-rate
				double dTime_s = currTime_s - m_lastFrameRateCheckTime_s;
				if (dTime_s > 1.0)	// Checks about every second
				{
					m_frameRate = (double)m_frameNum / dTime_s;
					m_frameRateAvg = m_framerateAverager.GetAverage(m_frameRate);	//!< Keeps a 30 second average
					m_lastFrameRateCheckTime_s = currTime_s;
					m_frameNum = 0;

					// Wait a few seconds before we make a performance update
					if (m_waitForPerformanceIndex_s > 0.0)
						m_waitForPerformanceIndex_s -= dTime_s;
					else if (lastDrawnFrameDur > 0.0)
					{
						// Make the performance index adjustments based ONLY on what the drawing does
						double perfFramerate = 1.0 / lastDrawnFrameDur;

						// Update the performance index, but only after we have been running a little bit
						if ((perfFramerate < PERFORMANCE_MIN) && (m_performanceIndex > -20))
						{
							// Watch for being low because we are throttling, no need to drop index for that
							if ((GetThrottleFPS() == 0.0) || (perfFramerate < (GetThrottleFPS() - 1.0)))
							{
								// Wait until we have had three seconds in a row of bad frame rates
								if (m_performanceStrikes <= -3)
								{
									m_performanceStrikes = 0;
									--m_performanceIndex;
									PerformanceIndexChange.Fire(m_performanceIndex + 1, m_performanceIndex);

									// Set the LOD scale a little lower
									if (m_performanceIndex < 1)
										m_camGroup->getCamera()->setLODScale(m_camGroup->getCamera()->getLODScale()*1.25f);
								}
								else if (m_performanceStrikes <= 0)
									--m_performanceStrikes;
								else
									m_performanceStrikes = 0;
							}
						}
						else if ((perfFramerate > PERFORMANCE_MAX) && (perfFramerate < 10))
						{
							// Wait until we have had two seconds in a row of good frame rates
							if (m_performanceStrikes >= 2)
							{
								m_performanceStrikes = 0;
								++m_performanceIndex;
								PerformanceIndexChange.Fire(m_performanceIndex - 1, m_performanceIndex);

								// We can set the LOD scale to make things a little closer as well (only one time)
								if (m_performanceIndex < 2)
									m_camGroup->getCamera()->setLODScale(m_camGroup->getCamera()->getLODScale() / 1.25f);
							}
							else if (m_performanceStrikes >= 0)
								++m_performanceStrikes;
							else
								m_performanceStrikes = 0;
						}
						else
							m_performanceStrikes = 0;
					}
				}

				++m_frameNum;
			}
			bool UpdateCameraGroup(double currTime_s)
			{
				if (m_camGroup->done())
					return false;
				//This is how it's done for ViewerBase, but may be redundant (need to check)
				if (m_camGroup->checkNeedToDoFrame())
					m_camGroup->frame(currTime_s);
				//else
				//{
				//	static int counter = 0;
				//	printf("Test %X\n",counter++);
				//}
				return true;
			}
			void UpdateSound(double dTick_s)
			{
				//stripped out
			}
		public:
			#pragma region _members_
			static OpenThreads::Mutex UpdateMutex;   // Use this block for all threading updates
			Event2<int, int> PerformanceIndexChange; //! provides old and new values, smaller means struggling
			// Used for performance indexing
			static double PERFORMANCE_MIN;
			static double PERFORMANCE_MAX;
			static int PERFORMANCE_INIT_INDEX;
			#pragma endregion
			Window(bool useAntiAlias, unsigned screenWidth, unsigned screenHeight, bool useUserPrefs) :
				m_useAntiAlias(useAntiAlias),
				m_camGroup(new ThreadSafeViewer),
				m_mainCam(m_camGroup.get()),
				//m_ConfigurationManager(useUserPrefs),
				m_lastWidth(640), m_lastHeight(480), m_lastX(100), m_lastY(100),
				m_isFullScreen(false),
				m_newScreenWidth(screenWidth),
				m_newScreenHeight(screenHeight),
				m_origScreenWidth(0),
				m_origScreenHeight(0),
				m_velocityAvg(15),
				m_waitForPerformanceIndex_s(6.0), // Wait a few seconds before making performance updates
				m_performanceStrikes(0), // We need to have a few seconds on a row of performance changes before we switch
				m_performanceIndex(PERFORMANCE_INIT_INDEX)
			{

				osg::DisplaySettings* ds = new osg::DisplaySettings();

				// Try with multi-threading draw dispatch
				ds->setSerializeDrawDispatch(false);

				// Try it with Anti-Aliasing to make it look really pretty!
				if (m_useAntiAlias)
					ds->setNumMultiSamples(4);

				m_camGroup->setDisplaySettings(ds);

				// We do NOT want the escape key to dismiss this window
				m_camGroup->setKeyEventSetsDone(0);

				// Play with different threading models
				#ifdef THREADING_MODEL
				m_camGroup->setThreadingModel(THREADING_MODEL);
				#endif

				// We want a stats handler, but bound to the F5 key
				osg::ref_ptr<osgViewer::StatsHandler> statsHandler = new osgViewer::StatsHandler;
				statsHandler->setKeyEventPrintsOutStats(osgGA::GUIEventAdapter::KEY_F5);
				statsHandler->setKeyEventTogglesOnScreenStats(osgGA::GUIEventAdapter::KEY_F5);
				m_camGroup->addEventHandler(statsHandler.get());

				// Init variables for finding frame-rate
				m_frameRate = 0.0;
				m_frameRateAvg = 0.0;
				m_lastFrameRateCheckTime_s = -1.0;
				m_frameNum = -3; // Wait a few frames before starting to count frame rate

				// Config Manager
				//m_Joystick = new JoyStick_Binder(&m_ConfigurationManager);
				//refKBM = new KeyboardMouse_CB(&m_ConfigurationManager);
				//m_Keyboard_Mouse = refKBM.get();

				//TODO fix this to whatever keys and operations are intuitive
				//GetKeyboard_Mouse().AddKeyBindingR(false, "VolumeDown", 'v');
				//GetKeyboard_Mouse().AddKeyBindingR(false, "VolumeUp", 'b');
				//GetKeyboard_Mouse().AddKeyBindingR(false, "VolumeSelect", 'n');
				//m_VolumeControls = new AudioVolumeControls_PlusBLS(
				//	GetKeyboard_Mouse().GlobalEventMap.Event_Map["VolumeSelect"],
				//	GetKeyboard_Mouse().GlobalEventMap.Event_Map["VolumeUp"],
				//	GetKeyboard_Mouse().GlobalEventMap.Event_Map["VolumeDown"]
				//);

				// Populate list
				//m_ConfigurationManager.AddConfigLoadSaveInterface(m_Joystick);
				//m_ConfigurationManager.AddConfigLoadSaveInterface(m_Keyboard_Mouse);
				//m_ConfigurationManager.AddConfigLoadSaveInterface(m_VolumeControls);
				//Now to load em
				//m_ConfigurationManager.UpdateSettings_Load();
			}
			virtual ~Window()
			{
				//m_ConfigurationManager.UpdateSettings_Save();
				// Revert the window to no longer be full screen, which will also set the window back to the proper size
				SetFullScreen(false);
				//delete m_Joystick;
				//m_Joystick = NULL;
				//delete m_VolumeControls;
				//m_VolumeControls = NULL;
				//Note refKBM is a osg ref pointer and does not require a delete call it will unreference itself automatically
			}

			void SetWindowRectangle(int x, int y, int w, int h, bool resize)
			{
				m_camGroup->getCamera()->getViewport()->setViewport(x, y, w, h);
			}
			void GetWindowRectangle(int& x, int& y, unsigned& w, unsigned& h)
			{
				osg::Viewport* vp = m_camGroup->getCamera()->getViewport();
				//For dual setup we'll just get the first monitor (we may want to provide a sum area)
				if (!vp)
					vp = m_camGroup->getSlave(0)._camera->getViewport();
				x = vp->x();
				y = vp->y();
				w = vp->width();
				h = vp->height();
			}
			void SetFullScreen(bool fs)
			{
				DEBUG_SCREEN_RESIZE("Window::SetFullScreen(%s) - was %s\n", fs ? "true" : "false", m_isFullScreen ? "true" : "false");
				// No need to make changes
				if (m_isFullScreen == fs)
					return;

				// MAke sure we have a proper graphics window and windowing system interface
				osgViewer::GraphicsWindow *window = GetGraphicsWindow();
				if (!window)
					return;
				osg::GraphicsContext::WindowingSystemInterface    *wsi = osg::GraphicsContext::getWindowingSystemInterface();
				if (!wsi)
					return;

				//bool ignoringMouse = m_Keyboard_Mouse->IgnoreMouseMotion;
				//m_Keyboard_Mouse->IgnoreMouseMotion = true;

				m_isFullScreen = fs;

				if (m_isFullScreen)
				{
					// Remember the last position and size of the window for when it is set back
					window->getWindowRectangle(m_lastX, m_lastY, m_lastWidth, m_lastHeight);

					// We need to get the original size of the screen, in case we want to set it back later
					wsi->getScreenResolution(*(window->getTraits()), m_origScreenWidth, m_origScreenHeight);

					// When in full screen mode, remove the title bar and borders
					window->setWindowDecoration(false);

					// When setting full screen, the user may want to resize the screen based on the parameters passed to the c'tor
					if ((m_newScreenHeight > 0) && (m_newScreenWidth > 0))
					{
						// Place the window properly first
						window->setWindowRectangle(0, 0, m_newScreenWidth, m_newScreenHeight);

						// Set the new screen height and width
						wsi->setScreenResolution(*(window->getTraits()), m_newScreenWidth, m_newScreenHeight);
					}
					else
					{
						// Otherwise we just set the the original screen size
						window->setWindowRectangle(0, 0, m_origScreenWidth, m_origScreenHeight);
					}
				}
				else // Windowed mode
				{
					// Make sure the screen is set back to its previous resolution
					if ((m_newScreenHeight > 0) && (m_newScreenWidth > 0))
					{
						// Set the new screen height and width
						wsi->setScreenResolution(*(window->getTraits()), m_origScreenWidth, m_origScreenHeight);
					}

					// When in windowed mode, use the title bar and borders
					window->setWindowDecoration(true);

					// Use the previous window sizes
					window->setWindowRectangle(m_lastX, m_lastY, m_lastWidth, m_lastHeight);
				}

				// Always make sure we have focus
				window->grabFocusIfPointerInWindow();

				// Place the mouse back at 0,0
				PositionPointer(0.0f, 0.0f);

				// Draw a frame
				m_camGroup->frame(0.0);

				// Start listening to the mouse gain if we were before
				//m_Keyboard_Mouse->IgnoreMouseMotion = ignoringMouse;
			}
			bool IsFullScreen()
			{
				return m_isFullScreen;
			}
			void SetWindowText(const char* windowTitle)
			{
				ASSERT(windowTitle != NULL);
				osgViewer::GraphicsWindow *window = GetGraphicsWindow();
				if (window)
					window->setWindowName(windowTitle);
			}
			bool IsAntiAliased() { return m_useAntiAlias; }
			virtual void Realize()
			{
				//If the user has not explicitly set the OSG_SCREEN environment variable then
				//Lets explicitly work with only one monitor (TODO zero does not necessarily mean primary display, we should fix)
				if (getenv("OSG_SCREEN") == 0)
					putenv("OSG_SCREEN=0");
				// Make sure this only happens once
				if (!m_camGroup->isRealized())
				{
					//m_camGroup->addEventHandler(m_Keyboard_Mouse);
					m_camGroup->realize();

					// See if we can get a better FOV
					double hfov, aspect, nearClip, farClip;
					m_camGroup->getCamera()->getProjectionMatrixAsPerspective(hfov, aspect, nearClip, farClip);
					double adjust = 1.5;
					m_camGroup->getCamera()->setProjectionMatrixAsPerspective(hfov*adjust, aspect, nearClip, farClip);

					// Set as Windowed mode, with the default size
					osgViewer::GraphicsWindow *window = GetGraphicsWindow();
					window->setWindowDecoration(true);
					window->setWindowRectangle(m_lastX, m_lastY, m_lastWidth, m_lastHeight);
					m_isFullScreen = false;

					// Adjust the LOD scale based on the initial values
					int i = 0;
					for (i = 0; i < m_performanceIndex; ++i)
						m_camGroup->getCamera()->setLODScale(m_camGroup->getCamera()->getLODScale() / 1.25f);
					for (i = 0; i > m_performanceIndex; --i)
						m_camGroup->getCamera()->setLODScale(m_camGroup->getCamera()->getLODScale()*1.25f);

					//Make the sound listener start WAY far away, rather than the middle of the scene
					//AudioVector3f Position(1e6, 1e6, 1e6);
					//AudioVector3f Velocity(0, 0, 0);
					//AudioVector3f Forward(0, 1, 0);
					//AudioVector3f Up(0, 0, 1);
					//SOUNDSYSTEM.Set3DListenerAttributes(0, &Position, &Velocity, &Forward, &Up);

					// Call frame at time 0 to get it all initialized
					m_camGroup->frame(0.0);

					// Position the pointer at 0 to get started
					PositionPointer(0.0f, 0.0f);
				}
			}
			virtual bool Update(double currTime_s, double dTick_s)
			{
				//m_Keyboard_Mouse->IncrementTime(dTick_s);
				//m_Joystick->UpdateJoyStick(dTick_s);	//! < Update the Joystick too

				m_mainCam.Update(dTick_s);			//! < Then the Camera Matrix
				UpdateSound(dTick_s);				//! < Then the sound that uses the camera matrix

				// Turn the mouse back on with the first pass through
				//if (m_Keyboard_Mouse->IgnoreMouseMotion)
				//	EnableMouse();	// Enable the mouse the first time, do not mess with framerate

				#ifndef __UseSingleThreadMainLoop__
				// The keyboard is now updated in the frame() call, but if running multi-threaded, we need to process all the
				// events we already got from the OSG thread.
				m_Keyboard_Mouse->ProcessThreadedEvents();
				return true;
				#else
				return UpdateCameraGroup(currTime_s);
				#endif
			}
			virtual double GetThrottleFPS() { return 0.0; }

			double GetFrameRate() { return m_frameRate; }
			double GetAverageFramerate() { return m_frameRateAvg; }
			int GetPerformanceIndex() { return m_performanceIndex; }
			GG_Framework::UI::OSG::ICamera* GetMainCamera()
			{
				return &m_mainCam;
			}
			void UseCursor(bool flag)
			{
				// Work with the Cursor, we will eventually be able to manipulate the cursor itself
				//stripped out
			}
			void EnableMouse()
			{
				// Call this just as we are starting the main loop to enable the camera and get one more frame in
				// stripped out
			}
			void PositionPointer(float x, float y)
			{
				// Position the pointer explicitly
				float pixel_x, pixel_y;
				if (!ComputePixelCoords(x, y, pixel_x, pixel_y))
					return;
				m_camGroup->requestWarpPointer(pixel_x, pixel_y);
			}
			bool ComputePixelCoords(float x, float y, float& pixel_x, float& pixel_y)
			{
				/** compute, from normalized mouse coordinates (x,y) the,  for the specified
				* RenderSurface, the pixel coordinates (pixel_x,pixel_y). return true
				* if pixel_x and pixel_y have been successful computed, otherwise return
				* false with pixel_x and pixel_y left unchanged.*/
				// Copied from KeyboardMouse::computePixelCoords() when there is not input area
				if (!m_camGroup->isRealized())
					return false;
				if (x < -1.0f) return false;
				if (x > 1.0f) return false;

				if (y < -1.0f) return false;
				if (y > 1.0f) return false;

				float rx = (x + 1.0f)*0.5f;
				float ry = (y + 1.0f)*0.5f;

				int wx, wy;
				unsigned int w, h;
				GetWindowRectangle(wx, wy, w, h);

				pixel_x = (float)wx + ((float)w)* rx;
				pixel_y = (float)wy + ((float)h)* ry;

				return true;
			}
			ThreadSafeViewer* GetViewer() { return m_camGroup.get(); }
			#pragma region _disabled_
			//Note: we strip out input
			//KeyboardMouse_CB &GetKeyboard_Mouse() const { return *m_Keyboard_Mouse; }
			//JoyStick_Binder &GetJoystick() const { return *m_Joystick; }
			#pragma endregion
		};
		#pragma region _Statics Globals_
		double Window::PERFORMANCE_MIN = 25.0;
		double Window::PERFORMANCE_MAX = 45.0;
		int Window::PERFORMANCE_INIT_INDEX = 0;
		OpenThreads::Mutex Window::UpdateMutex;
		#pragma endregion
		class FRAMEWORK_UI_API MainWindow : public Window
		{
		private:
			#pragma region _members_
			static MainWindow* s_mainWindow;
			bool m_quitting;
			osg::Timer m_throttleTimer;
			double m_throttleFPS;
			#pragma endregion
			void OnEscape()
			{
				bool closeOnEsc = true;
				EscapeQuit.Fire(this, closeOnEsc);
				if (closeOnEsc)
					TryClose();
			}
			void ThrottleFrame()
			{
				// Nothing to do if there is no throttle anyway
				if (m_throttleFPS == 0.0)
					return;

				// How much time in total do we need to wait?
				double waitTime_s = ((1.0 / m_throttleFPS) - m_throttleTimer.time_s());

				// We loop here because Sleep has a small resolution, calling it on too small of a wait is just like calling sleep(0)
				while (waitTime_s > 0.0)
				{
					// Do the sleep
					GG_Framework::Base::ThreadSleep((unsigned)(waitTime_s*1000.0));

					// Did we sleep long enough?
					waitTime_s = ((1.0 / m_throttleFPS) - m_throttleTimer.time_s());
				}

				// All done, reset the timer for next time
				m_throttleTimer.setStartTick();
			}

			#pragma region _multi theaded loop_
			#ifndef __UseSingleThreadMainLoop__
			class FRAMEWORK_UI_API GUIThread : public GG_Framework::Base::ThreadedClass
			{
			public:
				GUIThread(MainWindow *Parent) : m_pParent(Parent), m_currTime_s(0.0) { start(); }
				~GUIThread() { cancel(); }
				//Set and Get the Signal Event
				OpenThreads::Condition &SignalEvent() { return m_SignalEvent; }
				//Set and Get the current time
				double &currTime_s() { return m_currTime_s; }
			protected:
				void tryRun()
				{
					//Rick, we may want to remove this code and let the scene start immediately and then we can show a progress bar
					//by using osg itself
					#if 1
						//wait until we get an initial signal before running the scene
					m_BlockSignalEvent.lock();
					//hmmm I usually do not like to wait indefinitely but it seems to be the right thing
					m_SignalEvent.wait(&m_BlockSignalEvent);
					m_BlockSignalEvent.unlock();
					#endif
					//start the initial call to set the done status
					m_pParent->UpdateCameraGroup(m_currTime_s);
					while (!m_pParent->m_camGroup->done())
					{
						//There is s reversed scope mutex inside the wait... so we have to lock unlock around it
						m_BlockSignalEvent.lock();
						m_SignalEvent.wait(&m_BlockSignalEvent, 1000);
						m_BlockSignalEvent.unlock();
						m_pParent->UpdateCameraGroup(m_currTime_s);
					}
					printf("Exiting GUI Thread\n");
				}
			private:
				MainWindow * const m_pParent;
				OpenThreads::Condition m_SignalEvent;
				OpenThreads::Mutex m_BlockSignalEvent; //used internally inside the signal event to wait
				double m_currTime_s;
			} m_GUIThread;
			#endif
			#pragma endregion
		public:
			#pragma region _members_
			Event2<MainWindow*, bool&> Closing;
			Event2<MainWindow*, bool&> EscapeQuit;
			Event1<MainWindow*> Close;
			GG_Framework::Base::ProfilingLogger UpdateTimerLogger;
			// This is public so we can tie events to it
			IEvent::HandlerList ehl;
			#pragma endregion
			MainWindow(bool useAntiAlias, double throttle_fps, unsigned screenWidth, unsigned screenHeight, bool useUserPrefs) :
				Window(useAntiAlias, screenWidth, screenHeight, useUserPrefs), m_quitting(false), UpdateTimerLogger("OSG"),
				m_throttleFPS(throttle_fps) // Do we have a user defined throttle?
				#ifndef __UseSingleThreadMainLoop__
				, m_GUIThread(this)
				#endif
			{
				ASSERT(!s_mainWindow);
				s_mainWindow = this;

				//GetKeyboard_Mouse().AddKeyBinding(osgGA::GUIEventAdapter::KEY_Escape, "ESC", false);
				//GetKeyboard_Mouse().GlobalEventMap.Event_Map["ESC"].Subscribe(ehl, *this, &MainWindow::OnEscape);
			}
			virtual bool Update(double currTime_s, double dTick_s)
			{
				if (!m_quitting)
				{
					bool ret = false;
					UpdateTimerLogger.Start();
					#ifdef __UseSingleThreadMainLoop__
					//TODO this causes jitters in test 1... if we really need this then we can revisit the use-case
					//ThrottleFrame();  // Wait a bit if we need to
					ret = Window::Update(currTime_s, dTick_s);
					#else	
					m_GUIThread.currTime_s() = currTime_s; //TODO this should be atomic but may need a critical section
					m_GUIThread.SignalEvent().signal();  //set the event to wake up the GUI thread

					ThrottleFrame();  // Wait a bit if we need to
					Window::Update(currTime_s, dTick_s);

					//This should be a safe atomic operation
					ret = (!m_camGroup->done());
					#endif __UseSingleThreadMainLoop__
					UpdateTimerLogger.End();
					UpdateFrameRate(currTime_s, UpdateTimerLogger.V);
					return ret;
				}
				else return false;
			}
			virtual ~MainWindow()
			{
				//C4297 ... sorry Rick, you can't do that
				//ASSERT(s_mainWindow == this);
				s_mainWindow = NULL;
			}
			virtual void Realize()
			{
				__super::Realize();

				// If there is no user defined throttle, we may want to throttle back for multi-threading based on video card refresh rates
				// TODO: Properly calculate refresh rates
				if (m_throttleFPS == 0.0)
					m_throttleFPS = 60.0;

				// When working with the throttle, reset the timer
				m_throttleTimer.setStartTick();
			}
			bool IsQuitting() { return m_quitting; }
			void ToggleFullScreen() { SetFullScreen(!IsFullScreen()); }
			void StartLoop()
			{
				m_quitting = false;
			}
			void StopLoop()
			{
				m_quitting = true;
			}
			void TryClose()
			{
				if (!m_quitting)
				{
					StopLoop();
					Closing.Fire(this, m_quitting);
					if (m_quitting)
					{
						// Fire the close message to let everything know
						m_camGroup->setDone(true);
						Close.Fire(this);
					}
				}
			}
			// Here is the singleton
			static MainWindow* GetMainWindow() { return s_mainWindow; }
			virtual double GetThrottleFPS() { return m_throttleFPS; }
			void SetThrottleFPS(double throttleFPS)
			{
				ASSERT(throttleFPS >= 0.0);
				m_throttleFPS = throttleFPS;
			}
		};
		#pragma region _Statics Globals_
		MainWindow* MainWindow::s_mainWindow = NULL;
		#pragma endregion
	}
}

#pragma endregion
//#include "CenteredTrackball_CamManipulator.h"
//#include "Framerate_PDCB.h"
#pragma region _Framerate PDCB_
namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API Framerate_PDCB : public Text_PDCB
		{
		private:
			bool m_enabled;
			Window& m_window;
			GG_Framework::UI::OSG::Text m_text;
		public:
			Framerate_PDCB(Window& window) : Text_PDCB(), m_window(window) {}

			// Use this handler to tie events to this callback to toggle
			IEvent::HandlerList ehl;

			virtual std::string GetText() const { return GG_Framework::Base::BuildString("%3.1f (%3.1f) fps [%i]", m_window.GetFrameRate(), m_window.GetAverageFramerate(), m_window.GetPerformanceIndex()); }
			virtual osg::Vec2 GetPosition(double winWidth, double winHeight) const { return osg::Vec2(winWidth - 230.0, winHeight - 20.0); }
		};
	}
}
#pragma endregion
//#include "DebugOut_PDCB.h"
#pragma region _DebugOut PDCB_
// These simplify the use of the static DebugOut_PDCB text values
#define DOUT1(x,...) GG_Framework::UI::DebugOut_PDCB::TEXT1 = BuildString(x,__VA_ARGS__)
#define DOUT2(x,...) GG_Framework::UI::DebugOut_PDCB::TEXT2 = BuildString(x,__VA_ARGS__)
#define DOUT3(x,...) GG_Framework::UI::DebugOut_PDCB::TEXT3 = BuildString(x,__VA_ARGS__)
#define DOUT4(x,...) GG_Framework::UI::DebugOut_PDCB::TEXT4 = BuildString(x,__VA_ARGS__)
#define DOUT5(x,...) GG_Framework::UI::DebugOut_PDCB::TEXT5 = BuildString(x,__VA_ARGS__)

namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API DebugOut_PDCB : public Text_PDCB
		{
		public:
			DebugOut_PDCB(Window& window) : Text_PDCB() {}

			// Use this handler to tie events to this callback to toggle
			IEvent::HandlerList ehl;

			virtual std::string GetText() const
			{
				return TEXT1 + "\n" + TEXT2 + "\n" + TEXT3 + "\n" + TEXT4 + "\n" + TEXT5;
			}
			virtual osg::Vec2 GetPosition(double winWidth, double winHeight) const { return osg::Vec2(10.0, winHeight - 20.0); }
			virtual void operator () (const osg::Camera &cam) const
			{
				// If nothing is enabled, just pop out
				if (!m_enabled) return;

				double width = cam.getViewport()->width();
				double height = cam.getViewport()->height();

				glViewport(0, 0, width, height);
				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				glLoadIdentity();
				glOrtho(0.0, double(width), 0.0, double(height), -1.0, 1.0);
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glLoadIdentity();

				glPushAttrib(GL_ENABLE_BIT);
				glDisable(GL_TEXTURE_2D);
				glDisable(GL_LIGHTING);
				glDisable(GL_DEPTH_TEST);

				osg::Vec4 color = GetTextColor();
				glColor4f(color[0], color[1], color[2], color[3]);

				osg::Vec2 posn = GetPosition(width, height);
				int vs = 20; // vertical spacing
				int yPos = posn[1];

				glRasterPos2f(posn[0], yPos); yPos -= vs;
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, TEXT1.c_str());
				glRasterPos2f(posn[0], yPos); yPos -= vs;
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, TEXT2.c_str());
				glRasterPos2f(posn[0], yPos); yPos -= vs;
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, TEXT3.c_str());
				glRasterPos2f(posn[0], yPos); yPos -= vs;
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, TEXT4.c_str());
				glRasterPos2f(posn[0], yPos); yPos -= vs;
				m_text.drawString(GG_Framework::UI::OSG::Text::BitmapFont, TEXT5.c_str());

				glPopAttrib();
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(GL_MODELVIEW);
				glPopMatrix();
			}
			// To populate this with a formatted string, Use
			// GG_Framework::UI::DebugOut_PDCB::TEXT1 = GG_Framework::Base::BuildString(format, ...);
			static std::string TEXT1;
			static std::string TEXT2;
			static std::string TEXT3;
			static std::string TEXT4;
			static std::string TEXT5;
		};

		std::string DebugOut_PDCB::TEXT1("DebugOut_PDCB::TEXT1");
		std::string DebugOut_PDCB::TEXT2("DebugOut_PDCB::TEXT2");
		std::string DebugOut_PDCB::TEXT3("DebugOut_PDCB::TEXT3");
		std::string DebugOut_PDCB::TEXT4("DebugOut_PDCB::TEXT4");
		std::string DebugOut_PDCB::TEXT5("DebugOut_PDCB::TEXT5");

	}
}
#pragma endregion
//#include "ArgumentParser.h"
//#include "PerformanceSwitch.h"
//#include "ScreenCaptureTool.h"
//#include "MapFramesEffect.h"
//#include "OsgFile_ParticleEffect.h"
//#include "CustomParticleEffect.h"
#pragma endregion

#pragma endregion
#pragma region _Drive_
//project dependencies
//#include "../Common/Common.h"
#pragma region _Common_
#define COMMON_API
//project dependencies
//#include "../../../GG_Framework/UI/GG_Framework.UI.h"
//#include "../../../GG_Framework/Logic/Scripting/GG_Framework.Logic.Scripting.h"

#define Robot_TesterCode  //used to branch AI test code from wind-river code

#define __DisableSmartDashboard__ //used to quickly disable the smart dashboard
#ifndef __DisableSmartDashboard__
#include "../../SmartDashboard/SmartDashboard_import.h"
#else
class SmartDashboard //: public SensorBase
{
public:
	static void init() {}
	static void shutdown() {}

	//static void PutData(std::string key, Sendable *data) {}
	//static void PutData(NamedSendable *value){}
	//static Sendable* GetData(std::string keyName);

	static void PutBoolean(std::string keyName, bool value) {}
	static bool GetBoolean(std::string keyName) { return false; }

	static void PutNumber(std::string keyName, double value) {}
	static double GetNumber(std::string keyName) { return 0.0; }

	static void PutString(std::string keyName, std::string value) {}
	static int GetString(std::string keyName, char *value, unsigned int valueLen) { return 0; }
	static std::string GetString(std::string keyName) { return ""; }

	//static void PutValue(std::string keyName, ComplexData& value) {}
	//static void RetrieveValue(std::string keyName, ComplexData& value) {}
};
#endif

typedef osg::Vec2d Vec2D;
namespace Base = GG_Framework::Base;
namespace UI = GG_Framework::UI;
//namespace Scripting = GG_Framework::Logic::Scripting;

//local includes
namespace Robot_Tester
{
	//Note like our game everything can reach the UI, but the UI knows nothing about the entity
//#include "Entity_Properties.h"
#pragma region _Entity Properties_
	class Entity1D;
	class COMMON_API Entity1D_Properties
	{
	public:
		Entity1D_Properties();
		Entity1D_Properties(const char EntityName[], double Mass, double Dimension, bool IsAngular = false);
		virtual ~Entity1D_Properties() {}
		//virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults = false);
		void Initialize(Entity1D *NewEntity) const;
		double GetMass() const { return m_Mass; }
	protected:
		std::string m_EntityName;  //derived classes can let base class know what type to read
	private:
		//Stuff needed for physics
		double m_StartingPosition;  //the position used when reset position is called
		double m_Mass;
		double m_Dimension; //Dimension- Length for linear and diameter for angular
		bool m_IsAngular;
	};

	class Entity2D;
	class COMMON_API Entity_Properties
	{
	private:
		//std::string m_NAME;  <-do not need this
		//Stuff needed for physics
		double m_Mass;
		double m_Dimensions[2]; //Dimensions- Length Width
		//Note: this is in Transmitted entity, but I don't think it belongs here as it doesn't describe what, but rather where
		//! Positions in meters, rotation in degrees
		//double m_X, m_Y, m_Heading;
	protected:
		std::string m_EntityName;  //derived classes can let base class know what type to read
	public:
		Entity_Properties()
		{
			m_EntityName = "Entity";
			//m_NAME="default";
			m_Mass = 25.0;
			m_Dimensions[0] = 0.6477;
			m_Dimensions[1] = 0.9525;
		}
		virtual ~Entity_Properties() {}
		//The prep script takes care of the outer layer global table setup
		//override to search the appropriate global table
		//virtual const char *SetUpGlobalTable(Scripting::Script& script) { return script.GetGlobalTable(m_EntityName.c_str()); }
		//virtual void LoadFromScript(Scripting::Script& script);
		//void Initialize(Entity2D *NewEntity) const;
	};

#pragma endregion
//#include "FrameWork_UI.h" <--already disabled
//#include "Physics_2D.h"
#pragma region _Physics 2D_

	class COMMON_API PhysicsEntity_2D
	{
	public:
		PhysicsEntity_2D();
		virtual ~PhysicsEntity_2D() {}

		///You must set the mass otherwise you will get the default
		void SetMass(double mass);
		double GetMass() const;
		///This will zero out all vectors
		virtual void ResetVectors();

		// An overloaded operator for matching the current physics and position
		virtual void CopyFrom(const PhysicsEntity_2D& rhs);

		///This will compute all the displacement. Call this each time slice to obtain the rotation and position displacements.  
		///You must call this to run the physics engine!
		virtual void TimeChangeUpdate(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement);

		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This is ideal to be used with GetTorqueFromVelocity
		/// \param normalize will normalize the distance between -pi - pi
		double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel, bool normalize = true);

		inline Vec2D GetVelocityFromCollision(Vec2D ThisVelocityToUse, double otherEntityMass, Vec2D otherEntityVelocity);
		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This can either work with local or global orientation that all depends on the orientation of the restraints typically this works in local
		virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel);
		virtual Vec2D GetVelocityFromDistance_Linear_v1(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel);

		///These are coefficients to use when dealing with a force of friction typically 0.8 and 0.2 respectively
		void SetFriction(double StaticFriction, ///<The amount of friction to be applied when object is not moving
			double KineticFriction ///<The amount of friction to be applied when object is moving
		);

		///This is the direct way to handle torque for various mass distributions
		///Here are some examples:
		/// - Disk rotating around its center				0.5
		/// - Hollow cylinder rotating around its center	1.0
		/// - Hollow sphere									0.66 (2/3)
		/// - Hoop rotating around its center				1.0
		/// - Point mass rotating at radius r				1.0
		/// - Solid cylinder								0.5
		/// - Solid sphere									0.4
		/// \todo Provide helper methods to compute this value
		void SetAngularInertiaCoefficient(double AngularInertiaCoefficient);
		///This sets the radius for yaw axis, pitch axis, and roll axis.  Default = 1,1,1
		void SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass);
		double GetRadiusOfConcentratedMass() const;

		void SetLinearVelocity(const Vec2D &LinearVelocity);
		Vec2D GetLinearVelocity() const;

		void SetAngularVelocity(double AngularVelocity);
		double GetAngularVelocity() const;

		//This will give the acceleration delta given the torque which is: torque / AngularInertiaCoefficient * Mass
		//Note: It is torque if the radial arm distance is already factored in (Fr) and leaving RadialArmDistance as 1.0; otherwise it is force
		inline double GetAngularAccelerationDelta(double torque, double RadialArmDistance = 1.0);

		///These will auto sum for each call made, the forces last for one second during each timer update, so you have to call them repeatedly to 
		///continue to apply force.  If you want to apply a force for a specific amount of time, this can be achieved by calling this
		///during each time slice.  
		/// \note This should offer enough precision in time, but you can get more precise by computing a fraction force if necessary
		//void ApplyForce( const Vec2D &force);
		//void ApplyTorque( double torque);

		///These work like the above except that the force applied only happens for a fraction of a second.  For accurate results FrameDuration
		///should be <= 1/framerate.  Ideally these should be used for high precision movements like moving a ship, where the FrameDuration is
		///typically the TimeDelta value
		void ApplyFractionalForce(const Vec2D &force, double FrameDuration);
		//Note: It is torque if the radial arm distance is already factored in (Fr) and leaving RadialArmDistance as 1.0; otherwise it is force
		void ApplyFractionalTorque(double torque, double FrameDuration, double RadialArmDistance = 1.0);

		///This one is ideal to use for collision detection.  It will basically evaluate the point and determine the amount of force and torque to
		///apply.  It will implicitly call ApplyFractionalForce and ApplyFractionalTorque.
		void ApplyFractionalForce(const Vec2D &force, const Vec2D &point, double FrameDuration);

		///You may prefer to set a desired speed instead of computing the forces.  These values returned are intended to be used with 
		///ApplyFractionalForce, and ApplyFractionalTorque respectively.
		/// For the force, this works with the current linear velocity; therefore the desired velocity and return must work in global orientation
		virtual Vec2D GetForceFromVelocity(
			const Vec2D &vDesiredVelocity,	///< How fast do you want to go (in a specific direction)
			double DeltaTime_s					///< How quickly do you want to get there
		) const;
		virtual double GetTorqueFromVelocity(
			double vDesiredVelocity,	///< How fast do you want to go (in a specific direction)
			double DeltaTime_s					///< How quickly do you want to get there (usually in time slices)
		) const;

		///This is a clean way to compute how much torque that can be applied given the maximum amount available (e.g. thruster capacity)
		///It should be noted that this treats roll as a separate factor, which is best suited for avionic type of context
		/// \Note all restraint parameters are positive (i.e. ForceRestraintNegative)
		double ComputeRestrainedTorque(double Torque, double TorqueRestraint, double dTime_s);
		Vec2D ComputeRestrainedForce(const Vec2D &LocalForce, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double dTime_s);
		//This returns in the form of magnitude using the proper equations
		static double GetCentripetalAcceleration(double LinearVelocity, double AngularVelocity, double DeltaTime_s);
		double GetCentripetalAcceleration(double DeltaTime_s) const;
		///This returns the global acceleration needed to maintain linear velocity
		Vec2D GetCentripetalAcceleration_2D(double DeltaTime_s) const;
		///This returns the global force needed to maintain the current linear velocity
		Vec2D GetCentripetalForce(double DeltaTime_s) const;
	protected:
		double m_EntityMass;
		double m_StaticFriction, m_KineticFriction;

		double m_AngularInertiaCoefficient;
		double m_RadiusOfConcentratedMass; //This is used to compute the moment of inertia for torque (default 1,1,1)

		Vec2D m_LinearVelocity;		///< This must represent global orientation for external forces to work properly
		double m_AngularVelocity;		///< All angle variables are in radians!

		///This variable is factored in but is managed externally 
		Vec2D m_SummedExternalForces;
		double m_lastTime_s;
	};

	///This class is a expands on some common tasks that deal more specifically with flight.  This attempts to pull common tasks needed from physics in a way
	///Where it is easy to use for ships and other objects that deal with orientation and position
	class COMMON_API FlightDynamics_2D : public PhysicsEntity_2D
	{
	public:
		//provide common area to initialize members
		void init();
		FlightDynamics_2D();
		FlightDynamics_2D(const double *HeadingToUse);
		FlightDynamics_2D(const double &HeadingToUse);
		virtual ~FlightDynamics_2D() {}

		//Allow late binding of heading to use (client code should set this once as soon as possible)
		void SetHeadingToUse(const double *HeadingToUse) { m_HeadingToUse = HeadingToUse; }
		virtual void ResetVectors();

		///This will measure the distance between this quat and the look dir quat.  With the current algorithm the most desired results occur when the
		///delta's are less than 90 degrees for yaw and pitch.  Roll is computed separately.
		/// \param lookDir This is another orientation that you are comparing against
		/// \param UpDir is usually the same quat's orientation * 0,0,1
		Vec2D ComputeAngularDistance_asLookDir(const Vec2D &lookDir);
		double ComputeAngularDistance(const Vec2D &lookDir);
		/// \param Orientation this will break down the quat into its lookDir and UpDir for you
		//Vec2D ComputeAngularDistance(double Orientation);
		double ComputeAngularDistance(double Orientation);
		const double &GetHeading() { return *m_HeadingToUse; }

		virtual void TimeChangeUpdate(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement);

		//Acceleration rate methods:

		///If these methods are being used, this must be set to true
		void SetUsingAccelerationRate(bool UseIt) { m_UsingAccelerationRate = UseIt; }
		struct LinearAccelerationRates
		{
			Vec2D AccDeltaPos;        //when increasing from a positive position
			Vec2D AccDeltaNeg;        //when -increasing from a negative position
			Vec2D DecDeltaPos;        //when decreasing from a positive position
			Vec2D DecDeltaNeg;        //when -decreasing from a negative position
		};
		///Get and set the linear acceleration rates here.
		///For now this is the parameters for the simple model, which may or may not be applied for other engine models
		///Typically this should be a one time setup, but can be dynamic (e.g. afterburner)
		LinearAccelerationRates &GetLinearAccelerationRates();
		//Set to the desired acceleration level.  This should be called first per time interval, so the other methods can work properly
		//This must use local orientation
		void SetTargetAcceleration(const Vec2D &TargetAcceleration);
		///This can be defined to just about anything, but to keep things simple it is a linear constant depending on direction
		///We could later have different models and have client choose which to use
		Vec2D GetAcceleration_Delta(double dTime_s);
		///With this version you may use to predict the capabilities of max acceleration or deceleration
		/// \param Clipping this chooses whether to extract the full power when the acceleration approaches target acceleration, or
		/// whether to compute the actual force necessary to hit target.  Typically this will be on when manipulating the force
		/// The no clip is useful for things which need know full capability to get to threshold such as in GetForceFromVelocity
		Vec2D GetAcceleration_Delta(double dTime_s, const Vec2D &TargetAcceleration, bool Clipping = true);
		///This applies the current acceleration delta (i.e. calls GetAcceleration_Delta) to acceleration.
		/// \note Special care has been taken to ensure that any reads to GetAcceleration_Delta will be the same value applied here.
		///This must be consistent per time slice so that other clients can properly predict the acceleration.
		void Acceleration_TimeChangeUpdate(double dTime_s);
		///Read-only access of the current acceleration
		const Vec2D &GetCurrentAcceleration() { return m_CurrentAcceleration; }

		/// These are overloaded to optionally factor in the acceleration period
		virtual Vec2D GetForceFromVelocity(const Vec2D &vDesiredVelocity, double DeltaTime_s);
		virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel);

		double StructuralDmgGLimit;
		double G_Dampener;

	private:
#ifndef Robot_TesterCode
		typedef PhysicsEntity_2D __super;
#endif
		double m_DefaultHeading;
		// I'll try to keep this read only, so that client who own their own Heading can use this code, without worrying about the Heading being changed
		const double *m_HeadingToUse;
		//This keeps track of the current rate of acceleration.  These are in local orientation.
		Vec2D m_CurrentAcceleration, m_TargetAcceleration;
		LinearAccelerationRates m_LinearAccelerationRates;
		//Some objects may not need to use this (by default they will not)
		bool m_UsingAccelerationRate;
	};

#pragma endregion
//#include "Physics_1D.h"
#pragma region _Physics 1D_
#pragma once

	class COMMON_API PhysicsEntity_1D
	{
	public:
		PhysicsEntity_1D();
		///You must set the mass otherwise you will get the default
		void SetMass(double mass);
		double GetMass() const;
		///This will zero out all vectors
		virtual void ResetVectors();

		// An overloaded operator for matching the current physics and position
		virtual void CopyFrom(const PhysicsEntity_1D& rhs);

		///This will compute all the displacement. Call this each time slice to obtain the rotation and position displacements.  
		///You must call this to run the physics engine!
		virtual void TimeChangeUpdate(double DeltaTime_s, double &PositionDisplacement);

		inline double GetVelocityFromCollision(double ThisVelocityToUse, double otherEntityMass, double otherEntityVelocity);

		///These distance methods simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		/// For 1D, client code needs to determine if the dimension is linear or angular which is identified in Entity 1D via boolean value

		///This can either work with local or global orientation that all depends on the orientation of the restraints typically this works in local
		virtual double GetVelocityFromDistance_Linear(double Distance, double ForceRestraintPositive, double ForceRestraintNegative, double DeltaTime_s, double matchVel);
		virtual double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel);

		///These are coefficients to use when dealing with a force of friction typically 0.8 and 0.2 respectively
		void SetFriction(double StaticFriction, ///<The amount of friction to be applied when object is not moving
			double KineticFriction ///<The amount of friction to be applied when object is moving
		);

		///This is the direct way to handle torque for various mass distributions
		///Here are some examples:
		/// - Disk rotating around its center				0.5
		/// - Hollow cylinder rotating around its center	1.0
		/// - Hollow sphere									0.66 (2/3)
		/// - Hoop rotating around its center				1.0
		/// - Point mass rotating at radius r				1.0
		/// - Solid cylinder								0.5
		/// - Solid sphere									0.4
		/// \todo Provide helper methods to compute this value
		void SetAngularInertiaCoefficient(double AngularInertiaCoefficient);
		///This sets the radius for yaw axis, pitch axis, and roll axis.  Default = 1,1,1
		///This may represent the object's bounding box when used in conjunction with the angular inertia coefficient
		void SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass);
		double GetRadiusOfConcentratedMass() const;

		void SetVelocity(double Velocity);
		double GetVelocity() const;

		///These will auto sum for each call made, the forces last for one second during each timer update, so you have to call them repeatedly to 
		///continue to apply force.  If you want to apply a force for a specific amount of time, this can be achieved by calling this
		///during each time slice.  
		/// \note This should offer enough precision in time, but you can get more precise by computing a fraction force if necessary
		//void ApplyForce( double force);

		///These work like the above except that the force applied only happens for a fraction of a second.  For accurate results FrameDuration
		///should be <= 1/framerate.  Ideally these should be used for high precision movements like moving a ship, where the FrameDuration is
		///typically the TimeDelta value
		void ApplyFractionalForce(double force, double FrameDuration);
		//This is like getting mass in the angular world so it is useful for client code that needs to reconstruct torque
		//I=sum(m*r^2) or sum(AngularCoef*m*r^2)
		__inline double GetMomentofInertia(double RadialArmDistance = 1.0);
		//Note:  (For  GetAngularAccelerationDelta and ApplyFractionalTorque)
			//It is torque if the radial arm distance is already factored in (Fr) and leaving RadialArmDistance as 1.0; otherwise it is force
		//This will give the acceleration delta given the torque which is: torque / AngularInertiaCoefficient * Mass
		__inline double GetAngularAccelerationDelta(double torque, double RadialArmDistance = 1.0);
		void ApplyFractionalTorque(double torque, double FrameDuration, double RadialArmDistance = 1.0);

		///You may prefer to set a desired speed instead of computing the forces.  These values returned are intended to be used with 
		///ApplyFractionalForce, and ApplyFractionalTorque respectively.
		/// For the force, this works with the current linear velocity; therefore the desired velocity and return must work in global orientation
		virtual double GetForceFromVelocity(
			double vDesiredVelocity,			///< How fast do you want to go (in a specific direction)
			double DeltaTime_s					///< How quickly do you want to get there
		);

		///This is a clean way to compute how much torque that can be applied given the maximum amount available (e.g. thruster capacity)
		///It should be noted that this treats roll as a separate factor, which is best suited for avionic type of context
		/// \Note all restraint parameters are positive (i.e. ForceRestraintNegative)
		double ComputeRestrainedForce(double LocalForce, double ForceRestraintPositive, double ForceRestraintNegative, double dTime_s);
		__inline double GetForceNormal(double gravity = 9.80665) const;
		/// \param Brake is a brake coast parameter where if gravity pulls it down it can apply a scalar to slow down the reversed rate
		/// where 0 is full stop and 1 is full coast (range is 0 - 1)
		double GetFrictionalForce(double DeltaTime_s, double Ground = 0.0, double gravity = 9.80665, double BrakeResistence = 0.0) const;
	protected:
		double m_EntityMass;
		double m_StaticFriction, m_KineticFriction;

		double m_AngularInertiaCoefficient;
		double m_RadiusOfConcentratedMass; //This is used to compute the moment of inertia for torque (default 1,1,1)

		double m_Velocity;

		///This variable is factored in but is managed externally 
		double m_SummedExternalForces;
		double m_lastTime_s;
	};

#pragma endregion
//#include "Entity2D.h"
#pragma region _Entity 2D_

#ifdef Robot_TesterCode
	class Actor_Text;

	class COMMON_API EntityPropertiesInterface
	{
	public:
		virtual const Vec2D &GetPos_m() const = 0;
		virtual double GetAtt_r() const = 0;
		virtual const std::string &GetName() const = 0;
		virtual const Vec2D &GetDimensions() const = 0;
		virtual const double &GetIntendedOrientation() const = 0;
		//perform an optional custom update on the ship
		virtual void UI_Init(Actor_Text *parent) {}
		virtual void custom_update(osg::NodeVisitor *nv, osg::Drawable *draw, const osg::Vec3 &parent_pos) {}
		//This is called when it is detected the size has changed
		virtual void Text_SizeToUse(double SizeToUse) {}
		//AddOrRemove if true it is add false is remove
		virtual void UpdateScene(osg::Geode *geode, bool AddOrRemove) {}
	};
#else
	class COMMON_API EntityPropertiesInterface
	{
	};

	inline void NormalizeRotation(double &Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
	}

#endif

	class COMMON_API Entity1D
	{
	private:
		friend class Entity1D_Properties;

		Base::EventMap* m_eventMap;
		double m_StartingPosition;  //the position used when reset position is called
		double m_Dimension;
		double m_Position;
		std::string m_Name;
		bool m_BypassPos_Update;  //used to preserve pos during a ResetPos() call
	public:
		Entity1D(const char EntityName[]);

		//This allows the game client to setup the ship's characteristics
		virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL);
		virtual ~Entity1D(); //Game Client will be nuking this pointer
		const std::string &GetName() const { return m_Name; }
		virtual void TimeChange(double dTime_s);
		PhysicsEntity_1D &GetPhysics() { return m_Physics; }
		const PhysicsEntity_1D &GetPhysics() const { return m_Physics; }
		virtual double GetDimension() const { return m_Dimension; }
		virtual double GetStartingPosition() const { return m_StartingPosition; }
		virtual void ResetPosition(double Position);
		virtual void ResetPos();
		// This is where both the entity and camera need to align to, by default we use the actual position
		virtual const double &GetIntendedPosition() const { return m_Position; }
		Base::EventMap* GetEventMap() { return m_eventMap; }

		virtual double GetPos_m() const { return m_Position; }
		//This is used when a sensor need to correct for the actual position
		void SetPos_m(double value) { m_Position = value; }
		//Be sure to always set this back to false!
		void SetBypassPos_Update(bool bypass) { m_BypassPos_Update = bypass; }
		bool GetBypassPos_Update() const { return m_BypassPos_Update; }
	protected:
		///This gives derived class the ability to manipulate the displacement
		/// \ret true if this is to be used and manipulated, false uses the default displacement
		virtual bool InjectDisplacement(double DeltaTime_s, double &PositionDisplacement) { return false; }

		PhysicsEntity_1D m_Physics;
		bool m_IsAngular;
	};


	class Ship_Tester;
	//This contains everything the AI needs for game play; Keeping this encapsulated will help keep a clear division
	//of what Entity3D looked like before applying AI with goals
#ifdef Robot_TesterCode
	typedef Entity2D Entity2D_Kind;
#else
	namespace Entity2D_Kind = Framework::Base;
#endif

	//Note Entity2D should not know anything about an actor
	class COMMON_API Entity2D : public EntityPropertiesInterface
	{
	public:
#ifdef Robot_TesterCode
		class EventMap : public GG_Framework::UI::EventMap
		{
		public:
			EventMap(bool listOwned = false) : GG_Framework::UI::EventMap(listOwned) {}
		};
#endif
	private:
		friend class Ship_Tester;
		friend class Entity_Properties;
		struct PosAtt
		{
			Vec2D m_pos_m;

			//2d Orientation:
			double m_att_r;  //a.k.a heading
			//measurement in radians where 0 points north, pi/2 = east,  pi=south, and -pi/2 (or pi + pi/2) = west
			//from this a normalized vector can easily be computed by
			//x=sin(heading) y=cos(heading)
			//We can keep the general dimensions of the entity
			//Note: we do not need pitch or roll axis so this keeps things much simpler
		} m_PosAtt_Buffers[2];

		//All read cases use the read pointer, all write cases use the write pointer followed by an interlocked exchange of the pointers
		OpenThreads::AtomicPtr m_PosAtt_Read, m_PosAtt_Write;
		double m_att_r;  //I need a dedicated heading for physics to use
		void UpdatePosAtt();

		Vec2D m_DefaultPos;
		double m_DefaultAtt;
		Entity2D_Kind::EventMap* m_eventMap;

		Vec2D m_Dimensions;
		std::string m_Name;
		bool m_BypassPosAtt_Update;  //used to preserve pos att during a ResetPos() call

	public:
		Entity2D(const char EntityName[]);

		//This allows the game client to setup the ship's characteristics
		virtual void Initialize(Entity2D_Kind::EventMap& em, const Entity_Properties *props = NULL);
		virtual ~Entity2D(); //Game Client will be nuking this pointer
		const std::string &GetName() const { return m_Name; }
		virtual void TimeChange(double dTime_s);
		FlightDynamics_2D &GetPhysics() { return m_Physics; }
		const PhysicsEntity_2D &GetPhysics() const { return m_Physics; }
		virtual const Vec2D &GetDimensions() const { return m_Dimensions; }
		virtual void ResetPos();
		// This is where both the vehicle entity and camera need to align to, by default we use the actual orientation
		virtual const double &GetIntendedOrientation() const { return ((PosAtt *)m_PosAtt_Read.get())->m_att_r; }
		Entity2D_Kind::EventMap* GetEventMap() { return m_eventMap; }

		//from EntityPropertiesInterface
		virtual const Vec2D &GetPos_m() const { return ((PosAtt *)m_PosAtt_Read.get())->m_pos_m; }
		virtual double GetAtt_r() const { return ((PosAtt *)m_PosAtt_Read.get())->m_att_r; }

		//This is the position used when ResetPos() is called
		void SetDefaultPosition(const Vec2D &pos) { m_DefaultPos = pos; }
		void SetDefaultAttitude(double att) { m_DefaultAtt = att; }
		//Be sure to always set this back to false!
		void SetBypassPosAtt_Update(bool bypass) { m_BypassPosAtt_Update = bypass; }
		bool GetBypassPosAtt_Update() const { return m_BypassPosAtt_Update; }
	protected:
		FlightDynamics_2D m_Physics;
		///This gives derived class the ability to manipulate the displacement
		/// \ret true if this is to be used and manipulated, false uses the default displacement
		virtual bool InjectDisplacement(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement) { return false; }
	};

	typedef Entity2D Ship;
#pragma endregion
//#include "Goal.h"
//#include "Ship_1D.h"
//#include "Ship.h"
//#include "Vehicle_Drive.h"
//#include "AI_Base_Controller.h"
//#include "UI_Controller.h"
//#include "Poly.h"
//#include "Debug.h"
//#include "UDP_Listener.h"
//#include "Robot_Control_Common.h"
}

#pragma endregion
#pragma region _common robot specifics_
namespace Robot_Tester
{
//#include "../Common/PIDController.h"
//#include "../Common/Calibration_Testing.h"
//#include "../Common/Robot_Control_Interface.h"
//#include "../Common/Rotary_System.h"
//#include "../Common/Servo_System.h"
//#include "Tank_Robot.h"
//#include "Swerve_Robot.h"
//#include "Nona_Robot.h"
}
#pragma endregion
#pragma endregion
#pragma region _FrameWork_UI_
namespace Robot_Tester
{
const double PI = 3.1415926535897;
const double Pi2 = M_PI * 2.0;
const double PI_2 = 1.57079632679489661923;

//TODO evaluate my window size instead of using these constants
#if 0
const double c_Scene_XRes_InPixels = 640.0;
const double c_Scene_YRes_InPixels = 480.0;
#endif
#if 1
const double c_Scene_XRes_InPixels = 1280.0;
const double c_Scene_YRes_InPixels = 1024.0;
#endif
#if 0
const double c_Scene_XRes_InPixels = 1680.0;
const double c_Scene_YRes_InPixels = 1050.0;
#endif
#if 0
const double c_Scene_XRes_InPixels = 1600.0;
const double c_Scene_YRes_InPixels = 1200.0;
#endif
const double c_halfxres = c_Scene_XRes_InPixels / 2.0;
const double c_halfyres = c_Scene_YRes_InPixels / 2.0;

//This is dynamic so that we can zoom in on the fly
//double g_WorldScaleFactor=0.01; //This will give us about 128,000 x 102,400 meters resolution before wrap around
//double g_WorldScaleFactor=0.1; //This will give us about 12,800 x 10,240 meters resolution before wrap around ideal size to see
//double g_WorldScaleFactor=1.0; //This only give 1280 x 1024 meters but ideal to really see everything
//double g_WorldScaleFactor = 2.0; //This only give 640 x 512 good for windowed mode
//double g_WorldScaleFactor = 100.0; //This has been the default for the robots... previous sizes were for the bigger ships
double g_WorldScaleFactor = 140.0;  //for the small window this works well, but may rework this later for full screen

bool g_TestPhysics = false;

class Actor
{
protected:
	//Gives life to this shell dynamically
	EntityPropertiesInterface *m_EntityProperties_Interface;
	//osg::Node &m_Node;
public:
	Actor() : m_EntityProperties_Interface(NULL)
	{
		 //osg::Node &node <--may not need this
	}
	virtual void SetEntityProperties_Interface(EntityPropertiesInterface *entity) { m_EntityProperties_Interface = entity; }
	EntityPropertiesInterface *GetEntityProperties_Interface() const { return m_EntityProperties_Interface; }
};

class UI_GameClient;
class Actor_Text : public Actor, public osg::Drawable::UpdateCallback
{
private:
	//Note: this really needs to be decoupled from UI_GameClient
	//UI_GameClient *m_pParent;
	std::string m_TextImage;
	std::string m_TeamName; //cache team name to avoid flooding
	osg::ref_ptr<osgText::Text> m_Text;
	osg::ref_ptr<osgText::Text> m_IntendedOrientation;
	//Here is a quick reference on the character layout, used to determine size against the real dimensions, and also to center the intended orientation
	//caret for ships
	osg::Vec2d m_CharacterDimensions;
	double m_FontSize; //cache the last size setting to avoid flooding
protected:
	virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw)
	{
		if (m_EntityProperties_Interface)
		{
			osgText::Text *Text = dynamic_cast<osgText::Text *>(draw);
			if (Text)
			{
				//Note copy it for the vec... since it is volatile there is only one read to it now
				osg::Vec2d Position = m_EntityProperties_Interface->GetPos_m();
				double Heading = m_EntityProperties_Interface->GetAtt_r();
				double IntendedOrientation = m_EntityProperties_Interface->GetIntendedOrientation();

				//Scale down position here first
				Position[0] *= g_WorldScaleFactor;
				Position[1] *= g_WorldScaleFactor;

				//Center the offset
				Position[0] += c_halfxres;
				Position[1] += c_halfyres;
				//Now to wrap around the resolution
				Position[0] -= floor(Position[0] / c_Scene_XRes_InPixels)*c_Scene_XRes_InPixels;
				Position[1] -= floor(Position[1] / c_Scene_YRes_InPixels)*c_Scene_YRes_InPixels;
				//DOUT1 ("%f %f",Position[0],Position[1]);

				osg::Vec3 pos(Position[0], Position[1], 0.0f);

				Text->setPosition(pos);
				//Using negative so that radians increment in a clockwise direction //e.g. 90 is to the right
				Text->setRotation(FromLW_Rot_Radians(-Heading, 0.0, 0.0));

				if (m_IntendedOrientation.valid())
				{
					m_IntendedOrientation->setPosition(pos);
					m_IntendedOrientation->setRotation(FromLW_Rot_Radians(-IntendedOrientation, 0.0, 0.0));
				}
				m_EntityProperties_Interface->custom_update(nv, draw, pos);

				{//Now to determine the size the setCharacterSize is roughly one pixel per meter with the default font
					double XSize = m_EntityProperties_Interface->GetDimensions()[0] / m_CharacterDimensions[0] * g_WorldScaleFactor;
					double YSize = m_EntityProperties_Interface->GetDimensions()[1] / m_CharacterDimensions[1] * g_WorldScaleFactor;
					double SizeToUse = std::max(XSize, YSize); //we want bigger always so it is easier to see
					SizeToUse = std::max(SizeToUse, 5.0);  //Anything smaller than 5 is not visible


					if (SizeToUse != m_FontSize)
					{
						//TODO this should be working... it did once... need to figure out clean way to alter it
						Text->setCharacterSize(SizeToUse);
						if (m_IntendedOrientation.valid())
							m_IntendedOrientation->setCharacterSize(SizeToUse);
						m_EntityProperties_Interface->Text_SizeToUse(SizeToUse);
						m_FontSize = SizeToUse;
					}
				}
				#if 0
				{ //Now to determine the color based on the team name
					if (m_TeamName != m_EntityProperties_Interface->GetTeamName())
					{
						m_TeamName = m_EntityProperties_Interface->GetTeamName();
						if (m_TeamName == "red")
							Text->setColor(osg::Vec4(1.0f, 0.0f, 0.5f, 1.0f));  //This is almost magenta (easier to see)
						else if (m_TeamName == "blue")
							Text->setColor(osg::Vec4(0.0f, 0.5f, 1.0f, 1.0f));  //This is almost cyan (easier to see too)
						else if (m_TeamName == "green")
							Text->setColor(osg::Vec4(0.0f, 1.0f, 0.5f, 1.0f));
					}
				}
				#endif
			}
		}
	}
public:
	~Actor_Text()
	{
		//Keeping destructor for debugging purposes (make sure stuff is getting deleted)
	}
	Actor_Text(const char TextImage[] = "X") : 
		Actor(), 
		//m_pParent(parent), 
		m_TextImage(TextImage)
	{
		m_Text = new osgText::Text;
		m_FontSize = 20.0;
		osg::Vec4 layoutColor(1.0f, 1.0f, 1.0f, 1.0f);
		//Seems like the default font is ideal for me for now
		//osgText::Font* font = osgText::readFontFile("fonts/VeraMono.ttf");
		//m_Text->setFont(font);
		m_Text->setColor(layoutColor);
		m_Text->setCharacterSize(m_FontSize);
		m_Text->setFontResolution(10, 10);

		osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);
		m_Text->setPosition(position);
		//text->setDrawMode(osgText::Text::TEXT|osgText::Text::BOUNDINGBOX);
		m_Text->setAlignment(osgText::Text::CENTER_CENTER);
		m_Text->setText(TextImage);
		m_Text->setUpdateCallback(this);
	}
	//call this if you want intended orientation graphics (after setting up the character dimensions)
	void Init_IntendedOrientation()
	{
		osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);
		m_IntendedOrientation = new osgText::Text;
		m_IntendedOrientation->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
		m_IntendedOrientation->setCharacterSize(m_FontSize);
		m_IntendedOrientation->setFontResolution(10, 10);
		m_IntendedOrientation->setPosition(position);
		m_IntendedOrientation->setAlignment(osgText::Text::CENTER_CENTER);
		{
			char IntendedImage[128];
			strcpy(IntendedImage, "^\n");
			for (size_t i = 0; i < m_CharacterDimensions.y(); i++)
				strcat(IntendedImage, " \n");
			m_IntendedOrientation->setText(IntendedImage);
		}
		m_IntendedOrientation->setUpdateCallback(this);
	}
	osg::ref_ptr<osgText::Text> GetText() { return m_Text; }
	osg::ref_ptr<osgText::Text> GetIntendedOrientation() { return m_IntendedOrientation; }
	std::string &GetTextImage() { return m_TextImage; }
	//For now just have the client code write these
	osg::Vec2d &GetCharacterDimensions() { return m_CharacterDimensions; }
	double GetFontSize() const { return m_FontSize; }
	virtual void SetEntityProperties_Interface(EntityPropertiesInterface *entity)
	{
		__super::SetEntityProperties_Interface(entity);
		assert(m_EntityProperties_Interface);
		m_EntityProperties_Interface->UI_Init(this);
	}
	virtual void UpdateScene_Additional(osg::Geode *geode, bool AddOrRemove)
	{
		if (m_EntityProperties_Interface)
			m_EntityProperties_Interface->UpdateScene(geode, AddOrRemove);
	}
	//UI_GameClient *GetParent() { return m_pParent; }
};

class Viewer_Callback_Interface
{
public:
	virtual void UpdateData(double dtime_s) = 0;
	virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode) = 0;
};

class Entity2D;

class GameClient : public Viewer_Callback_Interface
{
private:
	EventMapList m_MapList;
protected:
	virtual void UpdateData(double dtime_s);
	virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode) {}  //No UI to muck with at this level

	//Derived classes may overload AddEntity and use these instead for the same functionality
	void AddEntity(Entity2D *Entity);
	Entity2D *CreateEntity(const char EntityName[], const Entity_Properties &props);

	//Note: we may need to put a critical section around this during a scene update
	std::vector<Entity2D *> m_Entities;
	typedef std::vector<Entity2D *>::iterator EntityIterator;
public:
	~GameClient();
	//For now use Inert type to populate obstacles (will not be a ship entity)
	Entity2D *GetEntity(const char EntityName[]);
	virtual void RemoveEntity(Entity2D *Entity);
	void RemoveAllEntities();
};

class UI_GameClient : public GameClient
{
private:
	OpenThreads::Mutex m_BlockActorLists;
	//The new and old actor lists are 
	std::vector<osg::ref_ptr<Actor_Text>> m_Actors, m_NewActors, m_OldActors;
	typedef std::vector<osg::ref_ptr<Actor_Text>>::iterator ActorIterator;
	osg::ref_ptr<osg::Group> m_RootNode; //may need to share this
protected:
	//This only manages adding and removing nodes not their positions
	virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode);
	virtual void AboutTo_RemoveEntity(Entity2D *Entity) {}
public:
	Entity2D *AddEntity(const char EntityName[], const Entity_Properties &props);
	void RemoveEntity(Entity2D *Entity);
	void RemoveEntity(const char EntityName[]);
	osg::ref_ptr<osg::Group> GetRootNode() { return m_RootNode; }
};
#if 0
class UI_Controller_GameClient : public UI_GameClient
{
private:
	//The one, the only!
	UI_Controller *m_UI_Controller;  //unfortunately this is late binding once the window is setup
	Entity2D* m_controlledEntity;
public:
	UI_Controller_GameClient();
	~UI_Controller_GameClient();
	virtual void SetControlledEntity(Entity2D* newEntity, bool AddJoystickDefaults = true);
	virtual void AboutTo_RemoveEntity(Entity2D *Entity) { if (Entity == m_controlledEntity) SetControlledEntity(NULL); }
};
#endif
}
#pragma endregion
//---------------
#pragma endregion

#pragma region  _Viewer_
namespace Robot_Tester
{
class Viewer
{
private:
	#pragma region _members_
	GG_Framework::UI::MainWindow *m_MainWin;
	//Exposing our node to render text
	osg::ref_ptr<osg::Group> m_RootNode;
	osg::ref_ptr<osg::Geode> m_Geode;
	Viewer_Callback_Interface *m_Callback;
	std::function<void(double dTime_s)> m_Callback2 = nullptr;
	std::function<void(osg::Group *rootNode, osg::Geode *geode)> m_SceneCallback = nullptr;

	class ViewerCallback : public osg::NodeCallback
	{
	private:
		Viewer * const m_pParent;
	protected:
		//From NodeCallback
		void operator()(osg::Node *node, osg::NodeVisitor *nv)
		{
			if (m_pParent->m_Callback)
				m_pParent->m_Callback->UpdateScene(m_pParent->m_RootNode, m_pParent->m_Geode);
			if (m_pParent->m_SceneCallback)
				m_pParent->m_SceneCallback(m_pParent->m_RootNode, m_pParent->m_Geode);
			traverse(node, nv);
		}
	public:
		ViewerCallback(Viewer *parent) : m_pParent(parent) {}
	};
	osg::ref_ptr <ViewerCallback> m_ViewerCallback;
	//This will make all time deltas the same length (ideal for debugging)
	bool m_UseSyntheticTimeDeltas;
	bool m_UseUserPrefs;
	#pragma endregion
public:
	Viewer(bool useUserPrefs = true) :m_Callback(NULL), m_UseSyntheticTimeDeltas(false), m_UseUserPrefs(useUserPrefs) {}
	//Pick which callback you prefer
	void SetCallbackInterface(Viewer_Callback_Interface *callback) { m_Callback = callback; }
	void SetUpdateCallback(std::function<void(double dTime_s)> callback) { m_Callback2 = callback; }
	void SetSceneCallback(std::function<void(osg::Group *rootNode, osg::Geode *geode)> callback) 	{ m_SceneCallback = callback; }
	void Init()
	{
		using namespace Robot_Tester;
		using namespace GG_Framework::Base;
		using namespace GG_Framework::UI;

		#pragma region _diabled code_
		// Content Directory
		//char contentDIR[512];
		//_getcwd(contentDIR, 512);
		//printf("Content Directory: %s\n", contentDIR);

		//Audio::ISoundSystem* sound = new Audio::SoundSystem_Mock();
		#pragma  endregion
		
		// Create the singletons, These must be instantiated before the scene manager too - Rick
		m_MainWin = new MainWindow(true, 0.0, 0, 0, m_UseUserPrefs);
		MainWindow &mainWin = *m_MainWin;

		// Create the scene manager with each of the files
		osg::Group* mainGroup = NULL;

		// Here is a good example of setting up an easy text PDCB
		Framerate_PDCB* fpdcb = new Framerate_PDCB(mainWin);
		mainWin.GetMainCamera()->addPostDrawCallback(*fpdcb);
		// And a debug one
		DebugOut_PDCB* dpdcb = new DebugOut_PDCB(mainWin);
		mainWin.GetMainCamera()->addPostDrawCallback(*dpdcb);
		#pragma region _keyboard controls for viewer_
		//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ShowFR", osgGA::GUIEventAdapter::KEY_F2);
		//mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ShowFR"].Subscribe(
		//	fpdcb->ehl, (Text_PDCB&)(*fpdcb), &Text_PDCB::ToggleEnabled);
		//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ShowDebug", osgGA::GUIEventAdapter::KEY_F9);
		//mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ShowDebug"].Subscribe(
		//	dpdcb->ehl, (Text_PDCB&)(*dpdcb), &Text_PDCB::ToggleEnabled);

		// Make the 'm' key record
		//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "RecordFrames", 'm');
		//ScreenCaptureTool sct(mainWin, mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["RecordFrames"]);

		// We can tie the events to Toggle fullscreen
		//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ToggleFullScreen", osgGA::GUIEventAdapter::KEY_F3);
		//mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ToggleFullScreen"].Subscribe(mainWin.ehl, mainWin, &MainWindow::ToggleFullScreen);

		// Set the scene and realize the camera at full size
		//mainWin.GetMainCamera()->SetSceneNode(actorScene.GetScene(), 0.0f);
		//mainWin.GetMainCamera()->SetSceneNode(createHUDText(), 0.0f);
		#pragma endregion
		// make sure the root node is group so we can add extra nodes to it.
		osg::Group* group = new osg::Group;
		#pragma region _Set up camera_
		{
			// create the hud.
			osg::Camera* camera = new osg::Camera;
			camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
			camera->setProjectionMatrixAsOrtho2D(0, c_Scene_XRes_InPixels, 0, c_Scene_YRes_InPixels);
			camera->setViewMatrix(osg::Matrix::identity());
			camera->setClearMask(GL_DEPTH_BUFFER_BIT);
			m_RootNode = new osg::Group;
			m_Geode = new osg::Geode;
			m_ViewerCallback = new ViewerCallback(this);
			m_RootNode->setUpdateCallback(m_ViewerCallback);
			m_RootNode->addChild(m_Geode);
			camera->addChild(m_RootNode);
			camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

			group->addChild(camera);
		}
		#pragma endregion
		mainWin.GetMainCamera()->SetSceneNode(group, 0.0f);

		mainWin.Realize();
		mainWin.SetFullScreen(false);
		mainWin.SetWindowText("Robot Tester");

		// Let all of the scene know we are starting
		//mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["START"].Fire();

		// Have a frame stamp to run the non-Actor parts
		osg::ref_ptr<osg::FrameStamp> frameStamp = new osg::FrameStamp;
		osgUtil::UpdateVisitor update;
		update.setFrameStamp(frameStamp.get());
		frameStamp->setSimulationTime(0.0);
		frameStamp->setFrameNumber(0);
	}
	//This loop runs until StopLoop is called (or object closes)
	void RunLoop()
	{
		using namespace Robot_Tester;
		using namespace GG_Framework::Base;
		using namespace GG_Framework::UI;
		MainWindow &mainWin = *m_MainWin;

		// Create a new scope, so all auto-variables will be deleted when they fall out
		{
			// We are going to use this single timer to fire against
			OSG::OSG_Timer timer("OSGV Timer Log.csv");
			//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ToggleFrameLog", osgGA::GUIEventAdapter::KEY_F7);
			//timer.Logger.ListenForToggleEvent(mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ToggleFrameLog"]);

			// Connect the argParser to the camera, in case it wants to handle stats (I do not know that I like this here)
			//argParser.AttatchCamera(&mainWin, &timer);

			// Loop while we are waiting for the Connection to be made and all of the scenes
			double dTime_s = 0.0;
			double currTime = 0.0;
			//Note: This callback already happens in ViewerCallback prior to traverse and need not be in this loop
			//m_Callback->UpdateScene(m_RootNode, m_Geode);
			mainWin.StartLoop();
			do
			{
				if (!m_UseSyntheticTimeDeltas)
					dTime_s = timer.FireTimer();
				else
				{
					//dTime_s = 0.016;  //hard code a typical 60 fps
					dTime_s = 0.010;  //Testing robot autonomous loop
					timer.FireTimer();
				}

				currTime = timer.GetCurrTime_s();
				if (m_Callback)
					m_Callback->UpdateData(dTime_s);
				if (m_Callback2)
					m_Callback2(dTime_s);
				//Audio::ISoundSystem::Instance().SystemFrameUpdate();
				//printf("\r %f      ",dTime_s);
			} while (mainWin.Update(currTime, dTime_s));

			// Write the log file
			timer.Logger.WriteLog();

			// Sleep to make sure everything completes
			ThreadSleep(200);
		}

		// Done with the sound
		//delete sound;

		// If any debug output files were written, hold the console so we all know about them
		if (ReleaseDebugFile::FilesWritten)
		{
			std::cout << "DEBUG Files written to CONTENT DIRECTORY" << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getch();
		}
	}
	void StopLoop()
	{
		m_MainWin->StopLoop();
	}
	void SetUseSyntheticTimeDeltas(bool UseSyntheticTimeDeltas) { m_UseSyntheticTimeDeltas = UseSyntheticTimeDeltas; }
	~Viewer()
	{
		using namespace GG_Framework::Base;
		if (m_MainWin)
		{
			m_MainWin->TryClose();
			// Sleep to make sure everything completes
			ThreadSleep(500);
			delete m_MainWin;
			m_MainWin = NULL;
		}
	}
};
}
#pragma endregion

//Put all windows dependent implementation last
#pragma region _win32_
#include <Windows.h>

namespace GG_Framework
{
	namespace Base
	{
		void ThreadSleep(unsigned sleepMS)
		{
			Sleep(sleepMS);
		}

		std::string BuildString(const char *format, ...)
		{
			char Temp[2048];
			va_list marker;
			va_start(marker, format);
			vsprintf(Temp, format, marker);
			va_end(marker);
			std::string ret(Temp);
			return ret;

		}

	}
}
#pragma endregion

#pragma region _Robot Tester (interface)_
namespace Robot_Tester
{
#pragma region _GUI Thread_
class GUIThread : public GG_Framework::Base::ThreadedClass
{
private:
	Viewer *m_Viewer;
	bool m_IsBeingDestroyed;
	bool m_UseUserPrefs;
protected:
	void tryRun()
	{
		assert(!m_Viewer);
		m_Viewer = new Viewer(m_UseUserPrefs);
		m_Viewer->Init();
		m_Viewer->RunLoop();
		//printf("Exiting GUI Thread\n");
		if (!m_IsBeingDestroyed)
		{
			m_IsBeingDestroyed = true;
			delete m_Viewer;
			m_Viewer = NULL;
		}
	}
public:
	GUIThread(bool useUserPrefs = true) : m_Viewer(NULL), m_IsBeingDestroyed(false), m_UseUserPrefs(useUserPrefs)
	{
		Run();
	}
	Viewer *GetUI() { return m_Viewer; }
	bool GetUseUserPrefs() const { return m_UseUserPrefs; }
	void Init(Viewer_Callback_Interface *ViewerCallback)
	{
		using namespace GG_Framework::Base;
		//Ensure the thread is running... it will not be in the case where user closed UI window from a previous session
		if ((!this->GetUI()) && (!this->isRunning()))
			this->Run();

		//Now to let the thread kick in... we wait for it to finish
		size_t TimeOut = 0;
		//wait for the UI to get set up (doh)
		while (!this->GetUI() && TimeOut++ < 2000)
			ThreadSleep(10);

		//Finally it should be good to go... set the callback interface
		if (this->GetUI())
			this->GetUI()->SetCallbackInterface(ViewerCallback);
		else
			throw "Unable to start UI";
	}
	~GUIThread()
	{
		//ThreadSleep(600);
		if (!m_IsBeingDestroyed)
		{
			m_IsBeingDestroyed = true;
			delete m_Viewer;
			m_Viewer = NULL;
		}
		cancel();
	}
	void Run()
	{
		m_IsBeingDestroyed = false;
		start();
	}
};

class GUIThread2
{
private:
	bool m_UseUserPrefs=true;
	bool m_IsStreaming = false;
	std::shared_ptr<Viewer> m_Viewer=nullptr;
	std::future<void> m_TaskState_LaunchLoop;  //Use future to monitor task status
	void dispatch_loop()
	{
		m_Viewer->RunLoop();
	}
public:
	void init(bool useUserPrefs = true)
	{
		m_Viewer = std::make_shared<Viewer>();
		m_Viewer->Init();
	}
	void SetCallbackInterface(Viewer_Callback_Interface *callback) { m_Viewer->SetCallbackInterface(callback); }
	void SetUpdateCallback(std::function<void(double dTime_s)> callback) { m_Viewer->SetUpdateCallback(callback); }
	void SetSceneCallback(std::function<void(osg::Group *rootNode, osg::Geode *geode)> callback) { m_Viewer->SetSceneCallback(callback); }
	void SetUseSyntheticTimeDeltas(bool UseSyntheticTimeDeltas) { m_Viewer->SetUseSyntheticTimeDeltas(UseSyntheticTimeDeltas); }
	void StartStreaming()
	{
		if (!m_IsStreaming)
		{
			//https://stackoverflow.com/questions/9094422/how-to-check-if-a-stdthread-is-still-running
			using namespace std::chrono_literals;
			//It's considered standard practice to wait for 0ms to obtain the current status without really waiting
			const bool in_flight = m_TaskState_LaunchLoop.valid() && m_TaskState_LaunchLoop.wait_for(0ms) != std::future_status::ready;
			if (!in_flight)
				m_TaskState_LaunchLoop = std::async(std::launch::async, &GUIThread2::dispatch_loop, this);
			//_CP_("in_flight =%d", in_flight);
			m_IsStreaming = true;
		}
	}
	void StopStreaming()
	{
		if (m_IsStreaming)
		{
			m_Viewer->StopLoop();
			//join... wait for thread to complete:
			size_t TimeOut = 0;
			using namespace std::chrono_literals;
			while (m_TaskState_LaunchLoop.wait_for(500ms) != std::future_status::ready)
			{
				printf("GUIThread2 [%p] waiting %zd\n", this, TimeOut++);
				if (TimeOut == 10)
				{
					printf("timed out... is it locked up\n");
					break;  //nothing more we can do at this point
				}
			}
			m_IsStreaming = false;
		}
	}
	~GUIThread2()
	{
		StopStreaming();
	}
};

#pragma endregion
#pragma region _robot Actor Text objects_
#pragma region _Common UI_

#pragma region _local global 2D converters_
inline Vec2D LocalToGlobal(double Heading, const Vec2D &LocalVector)
{
	return Vec2D(sin(Heading)*LocalVector[1] + cos(-Heading)*LocalVector[0],
		cos(Heading)*LocalVector[1] + sin(-Heading)*LocalVector[0]);
}

inline Vec2D GlobalToLocal(double Heading, const Vec2D &GlobalVector)
{
	return Vec2D(sin(-Heading)*GlobalVector[1] + cos(Heading)*GlobalVector[0],
		cos(-Heading)*GlobalVector[1] + sin(Heading)*GlobalVector[0]);
}
#pragma endregion

class Side_Wheel_UI
{
public:
	struct Wheel_Properties
	{
		osg::Vec2d m_Offset;  //Placement of the wheel in reference to the parent object (default 0,0)
		osg::Vec4d m_Color;
		const wchar_t *m_TextDisplay;
	};
private:
	Actor_Text *m_UIParent;
	Wheel_Properties m_props;
	osg::ref_ptr<osgText::Text> m_Wheel;
	double m_Rotation;

	osg::Vec4d m_Color;
	const wchar_t *m_TextDisplay;
public:
	Side_Wheel_UI() : m_UIParent(NULL), m_Rotation(0.0) {}
	void UI_Init(Actor_Text *parent)
	{
		m_UIParent = parent;

		osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);

		m_Wheel = new osgText::Text;
		m_Wheel->setColor(m_props.m_Color);
		m_Wheel->setCharacterSize(m_UIParent->GetFontSize());
		m_Wheel->setFontResolution(10, 10);
		m_Wheel->setPosition(position);
		m_Wheel->setAlignment(osgText::Text::CENTER_CENTER);
		m_Wheel->setText(m_props.m_TextDisplay);
		m_Wheel->setUpdateCallback(m_UIParent);
	}
	virtual void Initialize(const Wheel_Properties *props = NULL)
	{
		//Client code can manage the properties
		if (props)
			m_props = *props;
		else
		{
			m_props.m_Offset = osg::Vec2d(0, 0);
			m_props.m_Color = osg::Vec4(1.0, 0.0, 0.5, 1.0);
			m_props.m_TextDisplay = L"|";
		}
	}
	//Keep virtual for special kind of wheels
	virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw, const osg::Vec3 &parent_pos, double Heading)
	{
		double HeadingToUse = Heading + m_Rotation;
		const double FS = m_UIParent->GetFontSize();
		const osg::Vec2d WheelOffset(m_props.m_Offset[0], m_props.m_Offset[1]);
		const osg::Vec2d WheelLocalOffset = GlobalToLocal(Heading, WheelOffset);
		const osg::Vec3 WheelPos(parent_pos[0] + (WheelLocalOffset[0] * FS), parent_pos[1] + (WheelLocalOffset[1] * FS), parent_pos[2]);

		if (m_Wheel.valid())
		{
			m_Wheel->setPosition(WheelPos);
			m_Wheel->setRotation(FromLW_Rot_Radians(HeadingToUse, 0.0, 0.0));
		}
	}
	virtual void Text_SizeToUse(double SizeToUse)
	{
		if (m_Wheel.valid()) m_Wheel->setCharacterSize(SizeToUse);
	}

	virtual void UpdateScene(osg::Geode *geode, bool AddOrRemove)
	{
		if (AddOrRemove)
			if (m_Wheel.valid()) geode->addDrawable(m_Wheel);
			else
				if (m_Wheel.valid()) geode->removeDrawable(m_Wheel);
	}
	//This will add to the existing rotation and normalize
	void AddRotation(double RadiansToAdd)
	{
		m_Rotation += RadiansToAdd;
		if (m_Rotation > Pi2)
			m_Rotation -= Pi2;
		else if (m_Rotation < -Pi2)
			m_Rotation += Pi2;
	}
	void UpdatePosition(double x, double y) { m_props.m_Offset[0] = x, m_props.m_Offset[1] = y; }
	double GetFontSize() const { return m_UIParent ? m_UIParent->GetFontSize() : 10.0; }
};

class Swivel_Wheel_UI
{
public:
	struct Wheel_Properties
	{
		Vec2D m_Offset;  //Placement of the wheel in reference to the parent object (default 0,0)
		double m_Wheel_Diameter; //in meters default 0.1524  (6 inches)
	};
private:
	Actor_Text *m_UIParent;
	Wheel_Properties m_props;
	osg::ref_ptr<osgText::Text> m_Front, m_Back, m_Tread; //Tread is really a line that helps show speed
	double m_Rotation, m_Swivel;
public:
	Swivel_Wheel_UI() : m_UIParent(NULL) {}
	virtual ~Swivel_Wheel_UI() {}

	void UI_Init(Actor_Text *parent)
	{
		m_UIParent = parent;

		osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);
		m_Front = new osgText::Text;
		m_Front->setColor(GetFrontWheelColor());
		m_Front->setCharacterSize(m_UIParent->GetFontSize());
		m_Front->setFontResolution(10, 10);
		m_Front->setPosition(position);
		m_Front->setAlignment(osgText::Text::CENTER_CENTER);
		m_Front->setText(L"U");
		m_Front->setUpdateCallback(m_UIParent);

		m_Back = new osgText::Text;
		m_Back->setColor(GetBackWheelColor());
		m_Back->setCharacterSize(m_UIParent->GetFontSize());
		m_Back->setFontResolution(10, 10);
		m_Back->setPosition(position);
		m_Back->setAlignment(osgText::Text::CENTER_CENTER);
		m_Back->setText(L"U");
		m_Back->setUpdateCallback(m_UIParent);

		m_Tread = new osgText::Text;
		m_Tread->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
		m_Tread->setCharacterSize(m_UIParent->GetFontSize());
		m_Tread->setFontResolution(10, 10);
		m_Tread->setPosition(position);
		m_Tread->setAlignment(osgText::Text::CENTER_CENTER);
		//m_Tread->setText(L"\"");
		m_Tread->setText(L"__");
		m_Tread->setUpdateCallback(m_UIParent);
	}

	virtual void Initialize(const Wheel_Properties *props = NULL)
	{
		//Client code can manage the properties
		m_props = *props;
		m_Swivel = 0.0;
		m_Rotation = 0.0;
	}
	virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw, const osg::Vec3 &parent_pos, double Heading)
	{
		using Vec2d = osg::Vec2d;
		//Keep virtual for special kind of wheels
		const double FS = m_UIParent->GetFontSize();
		Vec2d FrontSwivel(0.0, 0.5);
		Vec2d BackSwivel(0.0, -0.5);
		//Vec2d TreadRotPos(0.0,cos(m_Rotation)-0.3);
		Vec2d TreadRotPos(sin(m_Rotation)*((fabs(m_Swivel) > PI_2) ? 0.5 : -0.5), (cos(m_Rotation)*.8) + 0.5);
		FrontSwivel = GlobalToLocal(m_Swivel, FrontSwivel);
		BackSwivel = GlobalToLocal(m_Swivel, BackSwivel);
		TreadRotPos = GlobalToLocal(m_Swivel, TreadRotPos);

		const Vec2d frontOffset(m_props.m_Offset[0] + FrontSwivel[0], m_props.m_Offset[1] + FrontSwivel[1]);
		const Vec2d backOffset(m_props.m_Offset[0] + BackSwivel[0], m_props.m_Offset[1] + BackSwivel[1]);
		const Vec2d TreadOffset(m_props.m_Offset[0] + TreadRotPos[0], m_props.m_Offset[1] + TreadRotPos[1]);

		const Vec2d FrontLocalOffset = GlobalToLocal(Heading, frontOffset);
		const Vec2d BackLocalOffset = GlobalToLocal(Heading, backOffset);
		const Vec2d TreadLocalOffset = GlobalToLocal(Heading, TreadOffset);
		const osg::Vec3 frontPos(parent_pos[0] + (FrontLocalOffset[0] * FS), parent_pos[1] + (FrontLocalOffset[1] * FS), parent_pos[2]);
		const osg::Vec3 backPos(parent_pos[0] + (BackLocalOffset[0] * FS), parent_pos[1] + (BackLocalOffset[1] * FS), parent_pos[2]);
		const osg::Vec3 TreadPos(parent_pos[0] + (TreadLocalOffset[0] * FS), parent_pos[1] + (TreadLocalOffset[1] * FS), parent_pos[2]);

		const double TreadColor = ((sin(-m_Rotation) + 1.0) / 2.0) * 0.8 + 0.2;
		m_Tread->setColor(osg::Vec4(TreadColor, TreadColor, TreadColor, 1.0));

		if (m_Front.valid())
		{
			m_Front->setPosition(frontPos);
			m_Front->setRotation(FromLW_Rot_Radians(PI + Heading + m_Swivel, 0.0, 0.0));
		}
		if (m_Back.valid())
		{
			m_Back->setPosition(backPos);
			m_Back->setRotation(FromLW_Rot_Radians(Heading + m_Swivel, 0.0, 0.0));
		}
		if (m_Tread.valid())
		{
			m_Tread->setPosition(TreadPos);
			m_Tread->setRotation(FromLW_Rot_Radians(Heading + m_Swivel, 0.0, 0.0));
		}
	}
	virtual void Text_SizeToUse(double SizeToUse)
	{
		if (m_Front.valid())	m_Front->setCharacterSize(SizeToUse);
		if (m_Back.valid()) m_Back->setCharacterSize(SizeToUse);
		if (m_Tread.valid()) m_Tread->setCharacterSize(SizeToUse);
	}
	virtual void UpdateScene(osg::Geode *geode, bool AddOrRemove)
	{
		if (AddOrRemove)
		{
			if (m_Front.valid()) geode->addDrawable(m_Front);
			if (m_Back.valid()) geode->addDrawable(m_Back);
			if (m_Tread.valid()) geode->addDrawable(m_Tread);
		}
		else
		{
			if (m_Front.valid()) geode->removeDrawable(m_Front);
			if (m_Back.valid()) geode->removeDrawable(m_Back);
			if (m_Tread.valid()) geode->removeDrawable(m_Tread);
		}
	}
	//Where 0 is up and 1.57 is right and -1.57 is left
	void SetSwivel(double SwivelAngle) { m_Swivel = -SwivelAngle; }
	void AddRotation(double RadiansToAdd)
	{
		//This will add to the existing rotation and normalize
		m_Rotation += RadiansToAdd;
		if (m_Rotation > Pi2)
			m_Rotation -= Pi2;
		else if (m_Rotation < -Pi2)
			m_Rotation += Pi2;
	}
	void SetRotation(double position) { m_Rotation = -position; }
	double GetFontSize() const { return m_UIParent ? m_UIParent->GetFontSize() : 10.0; }
	enum WheelEnum
	{
		eFront, eBack, eTread
	};
	void SetWheelColor(osg::Vec4 Color, WheelEnum Wheel)
	{
		switch (Wheel)
		{
		case eFront:
			m_Front->setColor(Color);
			break;
		case eBack:
			m_Back->setColor(Color);
			break;
		case eTread:
			//all though this is in-effective its added for completion
			m_Tread->setColor(Color);
			break;
		}
	}

	virtual osg::Vec4 GetFrontWheelColor() const { return osg::Vec4(0.0, 1.0, 0.0, 1.0); }
	virtual osg::Vec4 GetBackWheelColor() const { return osg::Vec4(1.0, 0.0, 0.0, 1.0); }
};

#pragma endregion
#pragma region _Swerve Robot UI_
class Swerve_Robot_UI : public EntityPropertiesInterface
{
public:
	#pragma region _swerve robot state_
	using SwerveRobot_State = SwerveRobot_UI::SwerveRobot_State;
	static SwerveRobot_State DefaultRobotState()
	{
		static SwerveRobot_State::Vector2D Pos_m;
		static double IntendedOrientation;
		SwerveRobot_State ret =
		{ Pos_m, {}, 0,
			IntendedOrientation
		};
		return ret;
	}
	#pragma endregion
	#pragma region _swerve robot properties_
	struct SwerveRobot_Properties
	{
		double WheelDiameter;
		//EPI references these
		std::string &entity_name;     //EPIrrksa
		Vec2D &Dimensions;            //EPIrrksa   x-width, y-length in meters
		Vec2D &Character_Dimensions;  //EPIrrksa
		std::string text_image;
	};
	//Note: this makes it possible to not need a callback, and provides a good template for new client code
	static SwerveRobot_Properties DefaultRobotProps()
	{
		static std::string entity_name = "default";
		static Vec2D Dimensions = Vec2D( Inches2Meters(19.5), Inches2Meters(27.5));  //length, width (font chars not robot's (27.5, 19.5))
		static Vec2D Character_Dimensions = Vec2D(5.0, 5.0);  //length, width (font chars not robot's (27.5, 19.5))

		SwerveRobot_Properties ret =
		{ 
			Inches2Meters(6),
			entity_name,
			Dimensions,
			Character_Dimensions,
			"     \n,   ,\n(-+-)\n'   '\n     "
		};
		return ret;
	}
	#pragma endregion
private:
	#pragma region _members_
	osg::ref_ptr<Actor_Text> m_Actor;
	//Allow subclasses to change wheels look
	Swivel_Wheel_UI *m_Wheel[4];
	std::function<SwerveRobot_State()> m_SwerveRobot = DefaultRobotState;
	std::function<SwerveRobot_Properties()> m_SwerveRobot_props = DefaultRobotProps;
	mutable Vec2D m_Pos_m;  //for backward compatibility
	mutable double m_IntendedOrientation;
	#pragma endregion
	virtual void UpdateVoltage(size_t index, double Voltage) {}
	virtual void CloseSolenoid(size_t index, bool Close) {}
	virtual Swivel_Wheel_UI *Create_WheelUI() { return new Swivel_Wheel_UI; }
	virtual void Destroy_WheelUI(Swivel_Wheel_UI *wheel_ui) { delete wheel_ui; }
	#pragma region _EntityPropertiesInterface_
protected: //from EntityPropertiesInterface
	virtual void UI_Init(Actor_Text *parent)
	{
		for (size_t i = 0; i < 4; i++)
		{
			m_Wheel[i]->UI_Init(parent);
		}
	}
	virtual void custom_update(osg::NodeVisitor *nv, osg::Drawable *draw, const osg::Vec3 &parent_pos)
	{
		//just dispatch the update to the wheels (for now)
		for (size_t i = 0; i < 4; i++)
			m_Wheel[i]->update(nv, draw, parent_pos, -(m_SwerveRobot().Att_r));
	}
	virtual void Text_SizeToUse(double SizeToUse)
	{
		for (size_t i = 0; i < 4; i++)
			m_Wheel[i]->Text_SizeToUse(SizeToUse);
	}
	virtual void UpdateScene(osg::Geode *geode, bool AddOrRemove)
	{
		//Note: update scene get called repeatedly, so we only reconfigure this for new actors
		//in our case at this level we'll never change, so in essence this will be called once
		//in the past the Game client would add or remove actors, so this could be delegated to
		//work that way if we are drawing multiple actors
		if (!m_Actor)
		{
			//setup our actor
			m_Actor = new Actor_Text(m_SwerveRobot_props().text_image.c_str());
			m_Actor->GetCharacterDimensions() = m_SwerveRobot_props().Character_Dimensions;
			//This can be removed if we do not want to see this image
			m_Actor->Init_IntendedOrientation();
			//Bind the EPI with its actor
			m_Actor->SetEntityProperties_Interface(this);
			geode->addDrawable(m_Actor->GetText());
			if (m_Actor->GetIntendedOrientation().valid())
				geode->addDrawable(m_Actor->GetIntendedOrientation());
			//since we are calling directly here... do not need to call this
			//m_Actor->UpdateScene_Additional(geode, true); //Add any additional nodes
			for (size_t i = 0; i < 4; i++)
				m_Wheel[i]->UpdateScene(geode, AddOrRemove);
		}
	}
	//Implement with our callback, not going to force a coupling with entity 2D
	virtual const Vec2D &GetPos_m() const
	{
		//Note: this is backward compatible so I am not too worried about the conversion
		SwerveRobot_State::Vector2D Pos_m= m_SwerveRobot().Pos_m;
		m_Pos_m = Vec2D(Pos_m.x, Pos_m.y);
		return m_Pos_m;
	}
	virtual double GetAtt_r() const
	{
		return m_SwerveRobot().Att_r;
	}
	virtual const std::string &GetName() const
	{
		return m_SwerveRobot_props().entity_name;
	}
	virtual const Vec2D &GetDimensions() const
	{
		return m_SwerveRobot_props().Dimensions;
	}
	virtual const double &GetIntendedOrientation() const
	{
		//Note: this is backward compatible so I am not too worried about the conversion
		m_IntendedOrientation = m_SwerveRobot().IntendedOrientation;
		return m_IntendedOrientation;
	}
	#pragma endregion
public:
	Swerve_Robot_UI()
	{
		for (size_t i = 0; i < 4; i++)
			m_Wheel[i] = NULL;
	}
	~Swerve_Robot_UI()
	{
		for (size_t i = 0; i < 4; i++)
		{
			Destroy_WheelUI(m_Wheel[i]);
			m_Wheel[i] = NULL;
		}
	}
	void SetSwerveRobot_Callback(std::function<SwerveRobot_State()> callback)
	{
		m_SwerveRobot = callback;
	}
	//This one is for completion, the defaults should suffice
	void SetSwerveRobot_props_Callback(std::function<SwerveRobot_Properties()> callback)
	{
		m_SwerveRobot_props = callback;
	}
	//Be sure to set callback before initialize
	virtual void Initialize()
	{
		Vec2D Offsets[4] =
		{
			Vec2D(-1.6, 2.0),
			Vec2D(1.6, 2.0),
			Vec2D(-1.6,-2.0),
			Vec2D(1.6,-2.0),
		};
		for (size_t i = 0; i < 4; i++)
		{
			Swivel_Wheel_UI::Wheel_Properties props;
			props.m_Offset = Offsets[i];
			props.m_Wheel_Diameter = m_SwerveRobot_props().WheelDiameter;
			m_Wheel[i] = Create_WheelUI();
			m_Wheel[i]->Initialize(&props);
		}
		//Note: actor gets set up in the update scene
	}
	virtual void TimeChange(double dTime_s)
	{
		SwerveRobot_State _ = m_SwerveRobot();
		for (size_t i = 0; i < 4; i++)
		{
			//TODO GetIntendedVelocities for intended UI
			m_Wheel[i]->SetSwivel(_.SwerveVelocitiesFromIndex[i + 4]);
			//For the linear velocities we'll convert to angular velocity and then extract the delta of this slice of time
			const double LinearVelocity = _.SwerveVelocitiesFromIndex[i];
			const double PixelHackScale = m_Wheel[i]->GetFontSize() / 8.0;  //scale the wheels to be pixel aesthetic
			const double RPS = LinearVelocity / (PI * m_SwerveRobot_props().WheelDiameter * PixelHackScale);
			const double AngularVelocity = RPS * Pi2;
			m_Wheel[i]->AddRotation(AngularVelocity*dTime_s);
		}
	}
	EntityPropertiesInterface &As_EPI() { return *this; }
};

#pragma endregion
#pragma endregion

#pragma region _Test Text_
//const double PI = 3.1415926535897;

class Tester
{
private:
	#pragma region _members_
	std::shared_ptr<GUIThread2> m_UI_thread = nullptr;
	double m_dTest = 0.0;
	double m_dTest2 = 0.0;
	#pragma endregion
	#pragma region _Test 0 Simple hud text demo_
	static void createHUDText(osg::Group *rootNode, osg::Geode* geode)
	{
		osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");

		float windowHeight = 1024.0f;
		float windowWidth = 1280.0f;
		float margin = 50.0f;

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//    
		// Examples of how to set up different text layout
		//

		osg::Vec4 layoutColor(1.0f, 1.0f, 0.0f, 1.0f);
		float layoutCharacterSize = 20.0f;

		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(layoutColor);
			text->setCharacterSize(layoutCharacterSize);
			text->setPosition(osg::Vec3(margin, windowHeight - margin, 0.0f));

			// the default layout is left to right, typically used in languages
			// originating from europe such as English, French, German, Spanish etc..
			text->setLayout(osgText::Text::LEFT_TO_RIGHT);

			text->setText("text->setLayout(osgText::Text::LEFT_TO_RIGHT);");
			geode->addDrawable(text);
		}

		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(layoutColor);
			text->setCharacterSize(layoutCharacterSize);
			text->setPosition(osg::Vec3(windowWidth - margin, windowHeight - margin, 0.0f));

			// right to left layouts would be used for hebrew or arabic fonts.
			text->setLayout(osgText::Text::RIGHT_TO_LEFT);
			text->setAlignment(osgText::Text::RIGHT_BASE_LINE);

			text->setText("text->setLayout(osgText::Text::RIGHT_TO_LEFT);");
			geode->addDrawable(text);
		}

		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(layoutColor);
			text->setPosition(osg::Vec3(margin, windowHeight - margin, 0.0f));
			text->setCharacterSize(layoutCharacterSize);

			// vertical font layout would be used for asian fonts.
			text->setLayout(osgText::Text::VERTICAL);

			text->setText("text->setLayout(osgText::Text::VERTICAL);");
			geode->addDrawable(text);
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//    
		// Examples of how to set up different font resolution
		//

		osg::Vec4 fontSizeColor(0.0f, 1.0f, 1.0f, 1.0f);
		float fontSizeCharacterSize = 30;

		osg::Vec3 cursor = osg::Vec3(margin * 2, windowHeight - margin * 2, 0.0f);

		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(fontSizeColor);
			text->setCharacterSize(fontSizeCharacterSize);
			text->setPosition(cursor);

			// use text that uses 10 by 10 texels as a target resolution for fonts.
			text->setFontResolution(10, 10); // blocky but small texture memory usage

			text->setText("text->setFontResolution(10,10); // blocky but small texture memory usage");
			geode->addDrawable(text);
		}

		cursor.y() -= fontSizeCharacterSize;
		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(fontSizeColor);
			text->setCharacterSize(fontSizeCharacterSize);
			text->setPosition(cursor);

			// use text that uses 20 by 20 texels as a target resolution for fonts.
			text->setFontResolution(20, 20); // smoother but higher texture memory usage (but still quite low).

			text->setText("text->setFontResolution(20,20); // smoother but higher texture memory usage (but still quite low).");
			geode->addDrawable(text);
		}

		cursor.y() -= fontSizeCharacterSize;
		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(fontSizeColor);
			text->setCharacterSize(fontSizeCharacterSize);
			text->setPosition(cursor);

			// use text that uses 40 by 40 texels as a target resolution for fonts.
			text->setFontResolution(40, 40); // even smoother but again higher texture memory usage.

			text->setText("text->setFontResolution(40,40); // even smoother but again higher texture memory usage.");
			geode->addDrawable(text);
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//    
		// Examples of how to set up different sized text
		//

		osg::Vec4 characterSizeColor(1.0f, 0.0f, 1.0f, 1.0f);

		cursor.y() -= fontSizeCharacterSize * 2.0f;

		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(characterSizeColor);
			text->setFontResolution(20, 20);
			text->setPosition(cursor);

			// use text that is 20 units high.
			text->setCharacterSize(20); // small

			text->setText("text->setCharacterSize(20.0f); // small");
			geode->addDrawable(text);
		}

		cursor.y() -= 30.0f;
		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(characterSizeColor);
			text->setFontResolution(30, 30);
			text->setPosition(cursor);

			// use text that is 30 units high.
			text->setCharacterSize(30.0f); // medium

			text->setText("text->setCharacterSize(30.0f); // medium");
			geode->addDrawable(text);
		}

		cursor.y() -= 50.0f;
		{
			osgText::Text* text = new osgText::Text;
			text->setFont(font);
			text->setColor(characterSizeColor);
			text->setFontResolution(40, 40);
			text->setPosition(cursor);

			// use text that is 60 units high.
			text->setCharacterSize(60.0f); // large

			text->setText("text->setCharacterSize(60.0f); // large");
			geode->addDrawable(text);
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//    
		// Examples of how to set up different alignments
		//

		osg::Vec4 alignmentSizeColor(0.0f, 1.0f, 0.0f, 1.0f);
		float alignmentCharacterSize = 25.0f;
		cursor.x() = 640;
		cursor.y() = margin * 4.0f;

		typedef std::pair<osgText::Text::AlignmentType, std::string> AlignmentPair;
		typedef std::vector<AlignmentPair> AlignmentList;
		AlignmentList alignmentList;
		alignmentList.push_back(AlignmentPair(osgText::Text::LEFT_TOP, "text->setAlignment(\nosgText::Text::LEFT_TOP);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::LEFT_CENTER, "text->setAlignment(\nosgText::Text::LEFT_CENTER);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::LEFT_BOTTOM, "text->setAlignment(\nosgText::Text::LEFT_BOTTOM);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::CENTER_TOP, "text->setAlignment(\nosgText::Text::CENTER_TOP);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::CENTER_CENTER, "text->setAlignment(\nosgText::Text::CENTER_CENTER);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::CENTER_BOTTOM, "text->setAlignment(\nosgText::Text::CENTER_BOTTOM);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::RIGHT_TOP, "text->setAlignment(\nosgText::Text::RIGHT_TOP);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::RIGHT_CENTER, "text->setAlignment(\nosgText::Text::RIGHT_CENTER);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::RIGHT_BOTTOM, "text->setAlignment(\nosgText::Text::RIGHT_BOTTOM);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::LEFT_BASE_LINE, "text->setAlignment(\nosgText::Text::LEFT_BASE_LINE);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::CENTER_BASE_LINE, "text->setAlignment(\nosgText::Text::CENTER_BASE_LINE);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::RIGHT_BASE_LINE, "text->setAlignment(\nosgText::Text::RIGHT_BASE_LINE);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::LEFT_BOTTOM_BASE_LINE, "text->setAlignment(\nosgText::Text::LEFT_BOTTOM_BASE_LINE);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::CENTER_BOTTOM_BASE_LINE, "text->setAlignment(\nosgText::Text::CENTER_BOTTOM_BASE_LINE);"));
		alignmentList.push_back(AlignmentPair(osgText::Text::RIGHT_BOTTOM_BASE_LINE, "text->setAlignment(\nosgText::Text::RIGHT_BOTTOM_BASE_LINE);"));


		osg::Sequence* sequence = new osg::Sequence;
		{
			for (AlignmentList::iterator itr = alignmentList.begin();
				itr != alignmentList.end();
				++itr)
			{
				osg::Geode* alignmentGeode = new osg::Geode;
				sequence->addChild(alignmentGeode);
				sequence->setTime(sequence->getNumChildren(), 1.0f);

				osgText::Text* text = new osgText::Text;
				text->setFont(font);
				text->setColor(alignmentSizeColor);
				text->setCharacterSize(alignmentCharacterSize);
				text->setPosition(cursor);
				text->setDrawMode(osgText::Text::TEXT | osgText::Text::ALIGNMENT | osgText::Text::BOUNDINGBOX);

				text->setAlignment(itr->first);
				text->setText(itr->second);

				alignmentGeode->addDrawable(text);


			}

		}

		sequence->setMode(osg::Sequence::START);
		sequence->setInterval(osg::Sequence::LOOP, 0, -1);
		sequence->setDuration(1.0f, -1);

		rootNode->addChild(sequence);


		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//    
		// Examples of how to set up different fonts...
		//

		cursor.x() = margin * 2.0f;
		cursor.y() = margin * 2.0f;

		osg::Vec4 fontColor(1.0f, 0.5f, 0.0f, 1.0f);
		float fontCharacterSize = 20.0f;
		float spacing = 40.0f;

		{
			osgText::Text* text = new osgText::Text;
			text->setColor(fontColor);
			text->setPosition(cursor);
			text->setCharacterSize(fontCharacterSize);

			text->setFont(0);
			text->setText("text->setFont(0); // inbuilt font.");
			geode->addDrawable(text);

			cursor.x() = text->getBoundingBox().xMax() + spacing;
		}

		{
			osgText::Font* arial = osgText::readFontFile("fonts/arial.ttf");

			osgText::Text* text = new osgText::Text;
			text->setColor(fontColor);
			text->setPosition(cursor);
			text->setCharacterSize(fontCharacterSize);

			text->setFont(arial);
			text->setText(arial != 0 ?
				"text->setFont(\"fonts/arial.ttf\");" :
				"unable to load \"fonts/arial.ttf\"");
			geode->addDrawable(text);

			cursor.x() = text->getBoundingBox().xMax() + spacing;
		}

		{
			osgText::Font* times = osgText::readFontFile("fonts/times.ttf");

			osgText::Text* text = new osgText::Text;
			text->setColor(fontColor);
			text->setPosition(cursor);
			text->setCharacterSize(fontCharacterSize);

			geode->addDrawable(text);
			text->setFont(times);
			text->setText(times != 0 ?
				"text->setFont(\"fonts/times.ttf\");" :
				"unable to load \"fonts/times.ttf\"");

			cursor.x() = text->getBoundingBox().xMax() + spacing;
		}

		cursor.x() = margin * 2.0f;
		cursor.y() = margin;

		{
			osgText::Font* dirtydoz = osgText::readFontFile("fonts/dirtydoz.ttf");

			osgText::Text* text = new osgText::Text;
			text->setColor(fontColor);
			text->setPosition(cursor);
			text->setCharacterSize(fontCharacterSize);

			text->setFont(dirtydoz);
			text->setText(dirtydoz != 0 ?
				"text->setFont(\"fonts/dirtydoz.ttf\");" :
				"unable to load \"fonts/dirtydoz.ttf\"");
			geode->addDrawable(text);

			cursor.x() = text->getBoundingBox().xMax() + spacing;
		}

		{
			osgText::Font* fudd = osgText::readFontFile("fonts/fudd.ttf");

			osgText::Text* text = new osgText::Text;
			text->setColor(fontColor);
			text->setPosition(cursor);
			text->setCharacterSize(fontCharacterSize);

			text->setFont(fudd);
			text->setText(fudd != 0 ?
				"text->setFont(\"fonts/fudd.ttf\");" :
				"unable to load \"fonts/fudd.ttf\"");
			geode->addDrawable(text);

			cursor.x() = text->getBoundingBox().xMax() + spacing;
		}

	}
	class TestCallback : public Viewer_Callback_Interface
	{
	public:
		TestCallback() : m_IsSetup(false) {}
		bool GetIsSetup() const { return m_IsSetup; }
	protected:  //from UI_Callback_Interface
		virtual void UpdateData(double dtime_s) {}
		virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode)
		{
			if (!m_IsSetup)
			{
				createHUDText(rootNode, geode);
				m_IsSetup = true;
			}
		}
	private:
		bool m_IsSetup;
	};
	#pragma endregion
	#pragma region _Test 1 manual moving actors_
	//Use PI/2 to init the cycle on zero for rho
	static double SineInfluence(double &rho, double freq_hz = 0.1, double SampleRate = 30.0, double amplitude = 1.0)
	{
		double			 theta, scale, pi2;

		pi2 = PI;
		theta = freq_hz / SampleRate;
		theta *= (pi2 * 2.0);
		pi2 *= 2.0;

		scale = amplitude;

		double Sample = sin(rho) * scale;
		rho += theta;
		if (rho > pi2)
			rho -= pi2;
		return Sample;
	}

	class Test_Actor2 : public osg::Drawable::UpdateCallback
	{
	public:
		Test_Actor2() : m_Rho(0.0)
		{
			osg::Vec4 layoutColor(0.0f, 1.0f, 1.0f, 1.0f);
			m_Text = new osgText::Text;
			////m_Text->setFont(font);
			m_Text->setColor(layoutColor);
			m_Text->setCharacterSize(20.0);
			m_Text->setFontResolution(10, 10);

			osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);
			m_Text->setPosition(position);
			//m_Text->setDrawMode(osgText::Text::TEXT|osgText::Text::BOUNDINGBOX);
			m_Text->setAlignment(osgText::Text::CENTER_CENTER);
			m_Text->setText("/^\\\n|:|\nC|:|D\n/*\\");
			m_Text->setUpdateCallback(this);
		}
		osg::ref_ptr<osgText::Text> GetText() { return m_Text; }
	protected:
		virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw)
		{
			osgText::Text *Text = dynamic_cast<osgText::Text *>(draw);
			double sample = SineInfluence(m_Rho);
			if (Text)
			{
				double halfxres = c_Scene_XRes_InPixels / 2.0;
				double halfyres = c_Scene_YRes_InPixels / 2.0;
				osg::Vec3 position(sample*halfxres + halfxres, 0.5*c_Scene_YRes_InPixels, 0.0f);
				Text->setPosition(position);
				Text->setRotation(FromLW_Rot_Radians(sample*PI, 0.0, 0.0));
			}
			//printf("\r %f      ",sample);

		}
	private:
		osg::ref_ptr<osgText::Text> m_Text;
		double m_Rho;
	};

	class Test_Actor : public osg::Drawable::UpdateCallback
	{
	public:
		Test_Actor() : m_Rho(0.0) {}
	protected:
		virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw)
		{
			osgText::Text *Text = dynamic_cast<osgText::Text *>(draw);
			double sample = SineInfluence(m_Rho);
			if (Text)
			{
				double halfxres = c_Scene_XRes_InPixels / 2.0;
				double halfyres = c_Scene_YRes_InPixels / 2.0;
				osg::Vec3 position(sample*halfxres + halfxres, 0.5*c_Scene_YRes_InPixels, 0.0f);
				//osg::Vec3 position(0.5*c_Scene_XRes_InPixels,sample*halfyres + halfyres,0.0f);
				Text->setPosition(position);
				//Text->setRotation(FromLW_Rot_Radians(m_dTest,0.0,0.0));
				Text->setRotation(FromLW_Rot_Radians(sample*PI, 0.0, 0.0));
			}
			//printf("\r %f      ",sample);

		}
	private:
		double m_Rho;
	};

	class TestCallback_2 : public Viewer_Callback_Interface
	{
	private:
		//osg::ref_ptr<Test_Actor> m_Actor;
		osg::ref_ptr<Test_Actor2> m_Actor;
		bool m_IsSetup;
	protected:  //from UI_Callback_Interface
		virtual void UpdateData(double dtime_s) {}
		virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode)
		{
			if (!m_IsSetup)
			{
				#if 0
				osg::Vec4 layoutColor(0.0f, 1.0f, 1.0f, 1.0f);
				osgText::Text *text = new osgText::Text;
				////text->setFont(font);
				text->setColor(layoutColor);
				text->setCharacterSize(20.0);
				text->setFontResolution(10, 10);

				osg::Vec3 position(0.5*c_Scene_XRes_InPixels, 0.5*c_Scene_YRes_InPixels, 0.0f);
				text->setPosition(position);
				//text->setDrawMode(osgText::Text::TEXT|osgText::Text::BOUNDINGBOX);
				text->setAlignment(osgText::Text::CENTER_CENTER);
				text->setText("/^\\\n|:|\nC|:|D\n/*\\");
				m_Actor = new Test_Actor;
				text->setUpdateCallback(m_Actor);
				geode->addDrawable(text);
				#else
				if (!m_Actor)
				{
					//Note: this is only setup one time
					m_Actor = new Test_Actor2;
					geode->addDrawable(m_Actor->GetText());
				}
				#endif
				//not sure if this is really necessary, but it allows an interesting local scope of the callback
				m_IsSetup = true;  
			}
		}
	public:
		TestCallback_2() : m_IsSetup(false) {}
		bool GetIsSetup() const { return m_IsSetup; }
	};
	#pragma endregion
	#pragma region _Test 2 swerve_
	inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}
	class TestCallback_3 : public Viewer_Callback_Interface
	{
	private:
		Swerve_Robot_UI m_robot;
		std::function<void(double dTime_s)> m_UpdateCallback=nullptr;
	protected:  //from UI_Callback_Interface
		virtual void UpdateData(double dtime_s) 
		{
			if (m_UpdateCallback)
				m_UpdateCallback(dtime_s);
			m_robot.TimeChange(dtime_s);
		}
		virtual void UpdateScene(osg::Group *rootNode, osg::Geode *geode)
		{
			m_robot.As_EPI().UpdateScene(geode,true);
		}
	public:
		Swerve_Robot_UI &Get_Robot() { return m_robot; }  //access to set callbacks
		void init()
		{
			m_robot.Initialize();
		}
		void SetUpdateCallback(std::function<void(double dTime_s)> callback)
		{
			m_UpdateCallback = callback;
		}
	};
	#pragma endregion
	void init(std::shared_ptr<GUIThread2> ui_thread)
	{
		if (!m_UI_thread)
			m_UI_thread = ui_thread;
	}
public:
	void Test(size_t index, std::shared_ptr<GUIThread2> ui_thread)
	{
		init(ui_thread);
		switch (index)
		{
		case 0:
		{
			using namespace GG_Framework::Base;
			TestCallback test;
			m_UI_thread->SetCallbackInterface(&test);
			m_UI_thread->StartStreaming();
			assert(m_UI_thread);
			size_t TimeOut = 0;
			while (!test.GetIsSetup() && TimeOut++ < 100)
				ThreadSleep(200);
			m_UI_thread->SetCallbackInterface(NULL);
		}
		break;
		case 1:
		{
			using namespace GG_Framework::Base;
			//m_UI_thread->GetUI()->SetUseSyntheticTimeDeltas(true);
			TestCallback_2 test;
			m_UI_thread->SetCallbackInterface(&test);
			m_UI_thread->StartStreaming();
			assert(m_UI_thread);
			size_t TimeOut = 0;
			while (!test.GetIsSetup() && TimeOut++ < 100)
				ThreadSleep(200);
			m_UI_thread->SetCallbackInterface(NULL);
		}
		break;
		case 2:
		{
			using namespace GG_Framework::Base;
			//m_UI_thread->SetUseSyntheticTimeDeltas(true);
			static TestCallback_3 test;  //this needs to remain on after this test finishes
			//set hooks here
			static double IntendedOrientation;

			static Swerve_Robot_UI::SwerveRobot_State current_state =
			{ {}, {}, 0,
				IntendedOrientation
			};

			test.Get_Robot().SetSwerveRobot_Callback([&]()	{	return current_state;	});
			test.SetUpdateCallback(
				[&](double dTime_s)
				{
					//profile check... ensure nothing is clogging the traversal and rendering
					#if 0
					using namespace std::chrono;
					using time_point = system_clock::time_point;
					static time_point then = system_clock::now();
					const time_point now = system_clock::now();
					const duration<double> delta_t = now - then;
					const double delta = delta_t.count();
					printf("test %.2f\n",delta);
					then = now;
					#endif
					static double m_Rho;
					const double sample = SineInfluence(m_Rho);
					current_state.Att_r += NormalizeRotation2(sample * 0.02);
					IntendedOrientation += NormalizeRotation2(sample * 0.25);
					current_state.SwerveVelocitiesFromIndex[0] += NormalizeRotation2(sample * 0.02);
					current_state.SwerveVelocitiesFromIndex[5] += NormalizeRotation2(sample * 0.02);
					current_state.Pos_m.y = sample * 10.0;
				});
			test.init(); //call back the UI thread
			m_UI_thread->SetCallbackInterface(&test);
			m_UI_thread->StartStreaming();
			assert(m_UI_thread);
			//since we are static we can leave the callback set, but only for this test
			//actual app needs to unhook properly
		}
		break;
		case 3:
		{
			//Making use of modern c++ as this is just like test 2, 
			//shows how much cleaner the code is without needing to use without the need of writing an interface
			//all the code is here in the same place!
			using namespace GG_Framework::Base;
			//m_UI_thread->SetUseSyntheticTimeDeltas(true);
			static Swerve_Robot_UI s_robot;
			s_robot.Initialize(); //setup the robot's properties here
			//set hooks here
			m_UI_thread->SetSceneCallback([&](osg::Group *rootNode, osg::Geode *geode) { s_robot.As_EPI().UpdateScene(geode, true); });

			//So due to legacy, Pos_m and IntendedOrientation are references, that means the caller owns the variables, which I suppose
			//is fine, but it may be confusing when looking at this example.
			static Vec2D Pos_m;
			static double IntendedOrientation;
			static Swerve_Robot_UI::SwerveRobot_State current_state =
			{ {}, {}, 0,
				IntendedOrientation
			};
			s_robot.SetSwerveRobot_Callback([&]()	{	return current_state;	});
			//Now the callback comes directly from the viewer
			m_UI_thread->SetUpdateCallback(
				[&](double dTime_s)
				{
					static double m_Rho;
					const double sample = SineInfluence(m_Rho);
					current_state.Att_r += NormalizeRotation2(sample * 0.02);
					IntendedOrientation += NormalizeRotation2(sample * 0.25);
					current_state.SwerveVelocitiesFromIndex[0] += NormalizeRotation2(sample * 0.02);
					current_state.SwerveVelocitiesFromIndex[5] += NormalizeRotation2(sample * 0.02);
					current_state.Pos_m.x = sample * 10.0;
					s_robot.TimeChange(dTime_s);
				});

			//Hooks set... now start the stream
			m_UI_thread->StartStreaming();
			//since we are static we can leave the callback set, but only for this test
			//actual app needs to unhook properly
		}
		break;

		}
	}
};


#pragma endregion

class OSG_Viewer_Internal
{
private:
	#pragma region _members_
	std::shared_ptr<GUIThread2> m_UI_thread = nullptr;
	#pragma endregion
public:
	void SetUpdateCallback(std::function<void(double dTime_s)> callback)
	{
		m_UI_thread->SetUpdateCallback(callback);
	}
	void SetSceneCallback(std::function<void(void *rootNode, void *geode)> callback)
	{
		//Gah need to cast... keeps header from being dependent on OSG parameters
		m_UI_thread->SetSceneCallback((std::function<void(osg::Group *rootNode, osg::Geode *geode)>)callback);
	}
	void init(bool useUserPrefs = true)
	{
		if (!m_UI_thread)
		{
			m_UI_thread = std::make_shared<GUIThread2>();
			m_UI_thread->init(useUserPrefs);
		}
	}
	void SetUseSyntheticTimeDeltas(bool UseSyntheticTimeDeltas)
	{
		m_UI_thread->SetUseSyntheticTimeDeltas(UseSyntheticTimeDeltas);
	}
	void StartStreaming()
	{
		m_UI_thread->StartStreaming();
	}
	void StopStreaming()
	{
		m_UI_thread->StopStreaming();
	}
	void Test(size_t index)
	{
		Tester test;
		test.Test(index,m_UI_thread);
	}
};

#pragma region _wrapper methods_

#pragma region _OSG Viewer_
OSG_Viewer::OSG_Viewer()
{
	m_OSG_Viewer = std::make_shared<OSG_Viewer_Internal>();
}
void OSG_Viewer::init()
{
	m_OSG_Viewer->init();
}
void OSG_Viewer::SetUpdateCallback(std::function<void(double dTime_s)> callback)
{
	m_OSG_Viewer->SetUpdateCallback(callback);
}
void OSG_Viewer::SetSceneCallback(std::function<void(void *rootNode, void *geode)> callback)
{
	//Gah need to cast... keeps header from being dependent on OSG parameters
	m_OSG_Viewer->SetSceneCallback(callback);
}
void OSG_Viewer::SetUseSyntheticTimeDeltas(bool UseSyntheticTimeDeltas)
{
	m_OSG_Viewer->SetUseSyntheticTimeDeltas(UseSyntheticTimeDeltas);
}
void OSG_Viewer::StartStreaming()
{
	m_OSG_Viewer->StartStreaming();
}
void OSG_Viewer::StopStreaming()
{
	m_OSG_Viewer->StopStreaming();
}
void OSG_Viewer::Zoom(double size)
{
	g_WorldScaleFactor = size;
}
void OSG_Viewer::Test(size_t index)
{
	m_OSG_Viewer->Test(index);
}
#pragma endregion
#pragma region _Swerve Robot UI_
SwerveRobot_UI::SwerveRobot_UI()
{
	//Instantiate now so we can set hooks before initialize
	m_Robot = std::make_shared<Swerve_Robot_UI>();
}
void SwerveRobot_UI::SetSwerveRobot_Callback(std::function<SwerveRobot_State()> callback)
{
	m_Robot->SetSwerveRobot_Callback(callback);
}
void SwerveRobot_UI::UpdateScene(void *geode, bool AddOrRemove)
{
	m_Robot->As_EPI().UpdateScene((osg::Geode *)geode, AddOrRemove);
}
void SwerveRobot_UI::Initialize()
{
	m_Robot->Initialize();
}
void SwerveRobot_UI::TimeChange(double dTime_s)
{
	m_Robot->TimeChange(dTime_s);
}

#pragma endregion
#pragma endregion
}
#pragma endregion

