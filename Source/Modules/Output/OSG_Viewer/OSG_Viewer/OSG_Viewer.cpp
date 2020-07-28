#pragma region _includes_
#include "stdafx.h"
#include <list>
#include <conio.h>
#include "OSG_Viewer.h"

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

//Note if we really need to debug in OSG itself we can add a DEBUG define with appropriate name and paths
//for now just adding release

#pragma comment (lib,"OpenThreads.lib")
#pragma comment (lib,"osg.lib")
#pragma comment (lib,"osgUtil.lib")
#pragma comment (lib,"osgDB.lib")
#pragma comment (lib,"osgText.lib")
#pragma comment (lib,"osgGA.lib")
#pragma comment (lib,"osgViewer.lib")
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

#pragma region _Font data (huge)_

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

static GLint strokeFont[][1 + MAX_STROKES * 3] = {
	{
		1,
			FONT_BEGIN, 0, 4,
			FONT_NEXT, 2, 2,
			FONT_END, 4, 4,
			FONT_BEGIN, 2, 8,
			FONT_END, 2, 2,
			FONT_ADVANCE, 6, 0
	},
	{
		2,
			FONT_BEGIN, 0, 3,
			FONT_NEXT, 0, 5,
			FONT_NEXT, 1, 6,
			FONT_NEXT, 2, 6,
			FONT_NEXT, 3, 5,
			FONT_END, 4, 6,
			FONT_BEGIN, 3, 5,
			FONT_NEXT, 3, 3,
			FONT_END, 4, 2,
			FONT_BEGIN, 3, 3,
			FONT_NEXT, 2, 2,
			FONT_NEXT, 1, 2,
			FONT_END, 0, 3,
			FONT_ADVANCE, 6, 0
	},
		{
			3,
				FONT_BEGIN, 0, 0,
				FONT_NEXT, 1, 1,
				FONT_NEXT, 1, 5,
				FONT_NEXT, 2, 6,
				FONT_NEXT, 4, 6,
				FONT_NEXT, 5, 5,
				FONT_NEXT, 4, 4,
				FONT_END, 1, 4,
				FONT_BEGIN, 4, 4,
				FONT_NEXT, 5, 3,
				FONT_NEXT, 4, 2,
				FONT_END, 1, 2,
				FONT_ADVANCE, 7, 0
		},
		{
			4,
				FONT_BEGIN, 0, 4,
				FONT_NEXT, 2, 6,
				FONT_END, 4, 4,
				FONT_ADVANCE, 6, 0
		},
			{
				5,
					FONT_BEGIN, 0, 5,
					FONT_NEXT, 4, 5,
					FONT_END, 4, 4,
					FONT_ADVANCE, 6, 0
			},
			{
				6,
					FONT_BEGIN, 1, 4,
					FONT_END, 3, 4,
					FONT_BEGIN, 3, 6,
					FONT_NEXT, 2, 6,
					FONT_NEXT, 1, 5,
					FONT_NEXT, 1, 3,
					FONT_NEXT, 2, 2,
					FONT_END, 3, 2,
					FONT_ADVANCE, 6, 0
			},
				{
					7,
						FONT_BEGIN, 1, 2,
						FONT_END, 1, 6,
						FONT_BEGIN, 0, 6,
						FONT_END, 4, 6,
						FONT_BEGIN, 3, 6,
						FONT_END, 3, 2,
						FONT_ADVANCE, 6, 0
				},
				{
					8,
						FONT_BEGIN, 0, 2,
						FONT_END, 2, 4,
						FONT_BEGIN, 0, 7,
						FONT_NEXT, 0, 6,
						FONT_END, 4, 2,
						FONT_ADVANCE, 6, 0
				},
					{
						10,
							FONT_ADVANCE, 0, -9
					},
					{
						11,
							FONT_BEGIN, 0, 2,
							FONT_NEXT, 1, 1,
							FONT_NEXT, 2, 2,
							FONT_NEXT, 2, 7,
							FONT_NEXT, 3, 8,
							FONT_END, 4, 7,
							FONT_ADVANCE, 6, 0
					},
						{
							12,
								FONT_BEGIN, 0, 3,
								FONT_END, 4, 3,
								FONT_BEGIN, 4, 6,
								FONT_END, 0, 6,
								FONT_BEGIN, 2, 8,
								FONT_END, 2, 4,
								FONT_ADVANCE, 6, 0
						},
						{
							14,
								FONT_BEGIN, 1, 3,
								FONT_NEXT, 2, 4,
								FONT_NEXT, 2, 5,
								FONT_NEXT, 3, 6,
								FONT_NEXT, 4, 5,
								FONT_NEXT, 4, 4,
								FONT_NEXT, 3, 3,
								FONT_END, 2, 4,
								FONT_BEGIN, 2, 5,
								FONT_NEXT, 1, 6,
								FONT_NEXT, 0, 5,
								FONT_NEXT, 0, 4,
								FONT_END, 1, 3,
								FONT_ADVANCE, 6, 0
						},
							{
								15,
									FONT_BEGIN, 0, 3,
									FONT_NEXT, 0, 4,
									FONT_NEXT, 1, 5,
									FONT_NEXT, 3, 5,
									FONT_NEXT, 4, 4,
									FONT_NEXT, 4, 6,
									FONT_NEXT, 2, 8,
									FONT_END, 1, 8,
									FONT_BEGIN, 4, 4,
									FONT_NEXT, 4, 3,
									FONT_NEXT, 3, 2,
									FONT_NEXT, 1, 2,
									FONT_END, 0, 3,
									FONT_ADVANCE, 6, 0
							},
							{
								16,
									FONT_BEGIN, 4, 7,
									FONT_NEXT, 1, 7,
									FONT_NEXT, 0, 6,
									FONT_NEXT, 0, 4,
									FONT_NEXT, 1, 3,
									FONT_END, 4, 3,
									FONT_ADVANCE, 6, 0
							},
								{
									17,
										FONT_BEGIN, 0, 3,
										FONT_NEXT, 3, 3,
										FONT_NEXT, 4, 4,
										FONT_NEXT, 4, 6,
										FONT_NEXT, 3, 7,
										FONT_END, 0, 7,
										FONT_ADVANCE, 6, 0
								},
								{
									18,
										FONT_BEGIN, 0, 4,
										FONT_NEXT, 0, 6,
										FONT_NEXT, 1, 7,
										FONT_NEXT, 3, 7,
										FONT_NEXT, 4, 6,
										FONT_END, 4, 4,
										FONT_ADVANCE, 6, 0
								},
									{
										19,
											FONT_BEGIN, 0, 7,
											FONT_NEXT, 0, 5,
											FONT_NEXT, 1, 4,
											FONT_NEXT, 3, 4,
											FONT_NEXT, 4, 5,
											FONT_END, 4, 7,
											FONT_ADVANCE, 6, 0
									},
									{
										20,
											FONT_BEGIN, 0, 8,
											FONT_NEXT, 2, 2,
											FONT_END, 4, 8,
											FONT_BEGIN, 3, 6,
											FONT_END, 1, 6,
											FONT_ADVANCE, 6, 0
									},
										{
											21,
												FONT_BEGIN, 0, 2,
												FONT_NEXT, 4, 2,
												FONT_NEXT, 4, 5,
												FONT_END, 1, 5,
												FONT_BEGIN, 0, 8,
												FONT_NEXT, 4, 8,
												FONT_END, 4, 5,
												FONT_ADVANCE, 6, 0
										},
										{
											22,
												FONT_BEGIN, 0, 4,
												FONT_NEXT, 0, 6,
												FONT_NEXT, 1, 7,
												FONT_NEXT, 3, 7,
												FONT_NEXT, 4, 6,
												FONT_NEXT, 4, 4,
												FONT_NEXT, 3, 3,
												FONT_NEXT, 1, 3,
												FONT_END, 0, 4,
												FONT_BEGIN, 1, 4,
												FONT_END, 3, 6,
												FONT_BEGIN, 1, 6,
												FONT_END, 3, 4,
												FONT_ADVANCE, 6, 0
										},
											{
												23,
													FONT_BEGIN, 2, 0,
													FONT_NEXT, 0, 2,
													FONT_NEXT, 4, 6,
													FONT_END, 2, 8,
													FONT_BEGIN, 0, 6,
													FONT_END, 4, 6,
													FONT_BEGIN, 0, 2,
													FONT_END, 4, 2,
													FONT_ADVANCE, 6, 0
											},
											{
												24,
													FONT_BEGIN, 0, 0,
													FONT_END, 4, 0,
													FONT_ADVANCE, 6, 0
											},
												{
													25,
														FONT_BEGIN, 2, 3,
														FONT_NEXT, 4, 5,
														FONT_END, 2, 7,
														FONT_BEGIN, 0, 5,
														FONT_END, 4, 5,
														FONT_ADVANCE, 6, 0
												},
												{
													26,
														FONT_BEGIN, 0, 8,
														FONT_NEXT, 1, 9,
														FONT_NEXT, 2, 9,
														FONT_NEXT, 2, 8,
														FONT_NEXT, 3, 8,
														FONT_END, 4, 9,
														FONT_ADVANCE, 6, 0
												},
													{
														27,
															FONT_BEGIN, 0, 1,
															FONT_END, 4, 7,
															FONT_BEGIN, 4, 5,
															FONT_END, 0, 5,
															FONT_BEGIN, 0, 3,
															FONT_END, 4, 3,
															FONT_ADVANCE, 6, 1
													},
													{
														28,
															FONT_BEGIN, 1, 2,
															FONT_END, 3, 2,
															FONT_BEGIN, 3, 4,
															FONT_NEXT, 1, 6,
															FONT_END, 3, 8,
															FONT_ADVANCE, 6, 0
													},
														{
															29,
																FONT_BEGIN, 1, 2,
																FONT_END, 3, 2,
																FONT_BEGIN, 1, 4,
																FONT_NEXT, 3, 6,
																FONT_END, 1, 8,
																FONT_ADVANCE, 6, 0
														},
														{
															30,
																FONT_BEGIN, 0, 3,
																FONT_END, 4, 3,
																FONT_BEGIN, 4, 5,
																FONT_END, 0, 5,
																FONT_BEGIN, 0, 7,
																FONT_END, 4, 7,
																FONT_ADVANCE, 6, 0
														},
															{
																31,
																	FONT_BEGIN, 0, 6,
																	FONT_NEXT, 2, 4,
																	FONT_END, 4, 6,
																	FONT_ADVANCE, 6, 0
															},
															{
																32,
																	FONT_ADVANCE, 6, 0
															},
																{
																	33,
																		FONT_BEGIN, 2, 1,
																		FONT_END, 2, 2,
																		FONT_BEGIN, 2, 4,
																		FONT_END, 2, 8,
																		FONT_ADVANCE, 6, 0
																},
																{
																	34,
																		FONT_BEGIN, 1, 7,
																		FONT_END, 1, 9,
																		FONT_BEGIN, 3, 9,
																		FONT_END, 3, 7,
																		FONT_ADVANCE, 6, 0
																},
																	{
																		35,
																			FONT_BEGIN, 1, 2,
																			FONT_END, 1, 7,
																			FONT_BEGIN, 3, 7,
																			FONT_END, 3, 2,
																			FONT_BEGIN, 4, 3,
																			FONT_END, 0, 3,
																			FONT_BEGIN, 0, 6,
																			FONT_END, 4, 6,
																			FONT_ADVANCE, 6, 0
																	},
																	{
																		36,
																			FONT_BEGIN, 2, 1,
																			FONT_END, 2, 9,
																			FONT_BEGIN, 4, 7,
																			FONT_NEXT, 3, 8,
																			FONT_NEXT, 1, 8,
																			FONT_NEXT, 0, 7,
																			FONT_NEXT, 0, 6,
																			FONT_NEXT, 1, 5,
																			FONT_NEXT, 3, 5,
																			FONT_NEXT, 4, 4,
																			FONT_NEXT, 4, 2,
																			FONT_NEXT, 3, 1,
																			FONT_NEXT, 1, 1,
																			FONT_END, 0, 2,
																			FONT_ADVANCE, 6, -1
																	},
																		{
																			37,
																				FONT_BEGIN, 0, 2,
																				FONT_NEXT, 0, 3,
																				FONT_NEXT, 4, 7,
																				FONT_END, 4, 8,
																				FONT_BEGIN, 1, 8,
																				FONT_NEXT, 0, 8,
																				FONT_NEXT, 0, 7,
																				FONT_NEXT, 1, 7,
																				FONT_END, 1, 8,
																				FONT_BEGIN, 4, 3,
																				FONT_NEXT, 5, 3,
																				FONT_NEXT, 5, 2,
																				FONT_NEXT, 4, 2,
																				FONT_END, 5, 2,
																				FONT_ADVANCE, 8, 1
																		},
																		{
																			38,
																				FONT_BEGIN, 4, 4,
																				FONT_NEXT, 2, 2,
																				FONT_NEXT, 1, 2,
																				FONT_NEXT, 0, 3,
																				FONT_NEXT, 0, 4,
																				FONT_NEXT, 2, 6,
																				FONT_NEXT, 2, 7,
																				FONT_NEXT, 1, 8,
																				FONT_NEXT, 0, 7,
																				FONT_NEXT, 0, 6,
																				FONT_END, 4, 2,
																				FONT_ADVANCE, 6, 0
																		},
																			{
																				39,
																					FONT_BEGIN, 0, 7,
																					FONT_NEXT, 1, 8,
																					FONT_NEXT, 1, 9,
																					FONT_NEXT, 2, 9,
																					FONT_NEXT, 2, 8,
																					FONT_END, 1, 8,
																					FONT_ADVANCE, 6, 1
																			},
																			{
																				40,
																					FONT_BEGIN, 4, 2,
																					FONT_NEXT, 2, 4,
																					FONT_NEXT, 2, 6,
																					FONT_END, 4, 8,
																					FONT_ADVANCE, 6, 0
																			},
																				{
																					41,
																						FONT_BEGIN, 0, 2,
																						FONT_NEXT, 2, 4,
																						FONT_NEXT, 2, 6,
																						FONT_END, 0, 8,
																						FONT_ADVANCE, 6, 0
																				},
																				{
																					42,
																						FONT_BEGIN, 2, 2,
																						FONT_END, 2, 8,
																						FONT_BEGIN, 0, 7,
																						FONT_END, 4, 3,
																						FONT_BEGIN, 4, 5,
																						FONT_END, 0, 5,
																						FONT_BEGIN, 0, 3,
																						FONT_END, 4, 7,
																						FONT_ADVANCE, 6, 0
																				},
																					{
																						43,
																							FONT_BEGIN, 2, 3,
																							FONT_END, 2, 7,
																							FONT_BEGIN, 0, 5,
																							FONT_END, 4, 5,
																							FONT_ADVANCE, 6, 0
																					},
																					{
																						44,
																							FONT_BEGIN, 0, 1,
																							FONT_NEXT, 1, 2,
																							FONT_END, 1, 3,
																							FONT_ADVANCE, 6, 0
																					},
																						{
																							45,
																								FONT_BEGIN, 0, 5,
																								FONT_END, 4, 5,
																								FONT_ADVANCE, 6, 0
																						},
																						{
																							46,
																								FONT_BEGIN, 1, 2,
																								FONT_END, 2, 2,
																								FONT_ADVANCE, 6, 0
																						},
																							{
																								47,
																									FONT_BEGIN, 0, 3,
																									FONT_END, 4, 7,
																									FONT_ADVANCE, 6, 0
																							},
																							{
																								48,
																									FONT_BEGIN, 0, 3,
																									FONT_NEXT, 4, 7,
																									FONT_NEXT, 3, 8,
																									FONT_NEXT, 1, 8,
																									FONT_NEXT, 0, 7,
																									FONT_NEXT, 0, 3,
																									FONT_NEXT, 1, 2,
																									FONT_NEXT, 3, 2,
																									FONT_NEXT, 4, 3,
																									FONT_END, 4, 7,
																									FONT_ADVANCE, 6, 0
																							},
																								{
																									49,
																										FONT_BEGIN, 1, 2,
																										FONT_END, 3, 2,
																										FONT_BEGIN, 2, 2,
																										FONT_NEXT, 2, 8,
																										FONT_END, 1, 7,
																										FONT_ADVANCE, 6, 0
																								},
																								{
																									50,
																										FONT_BEGIN, 0, 7,
																										FONT_NEXT, 1, 8,
																										FONT_NEXT, 3, 8,
																										FONT_NEXT, 4, 7,
																										FONT_NEXT, 4, 6,
																										FONT_NEXT, 0, 2,
																										FONT_END, 4, 2,
																										FONT_ADVANCE, 6, 0
																								},
																									{
																										51,
																											FONT_BEGIN, 0, 3,
																											FONT_NEXT, 1, 2,
																											FONT_NEXT, 3, 2,
																											FONT_NEXT, 4, 3,
																											FONT_NEXT, 4, 4,
																											FONT_NEXT, 3, 5,
																											FONT_END, 2, 5,
																											FONT_BEGIN, 3, 5,
																											FONT_NEXT, 4, 6,
																											FONT_NEXT, 4, 7,
																											FONT_NEXT, 3, 8,
																											FONT_NEXT, 1, 8,
																											FONT_END, 0, 7,
																											FONT_ADVANCE, 6, 0
																									},
																									{
																										52,
																											FONT_BEGIN, 3, 2,
																											FONT_NEXT, 3, 8,
																											FONT_NEXT, 0, 5,
																											FONT_NEXT, 0, 4,
																											FONT_END, 4, 4,
																											FONT_ADVANCE, 6, 0
																									},
																										{
																											53,
																												FONT_BEGIN, 0, 3,
																												FONT_NEXT, 1, 2,
																												FONT_NEXT, 3, 2,
																												FONT_NEXT, 4, 3,
																												FONT_NEXT, 4, 5,
																												FONT_NEXT, 3, 6,
																												FONT_NEXT, 0, 6,
																												FONT_NEXT, 0, 8,
																												FONT_END, 4, 8,
																												FONT_ADVANCE, 6, 0
																										},
																										{
																											54,
																												FONT_BEGIN, 0, 5,
																												FONT_NEXT, 0, 3,
																												FONT_NEXT, 1, 2,
																												FONT_NEXT, 3, 2,
																												FONT_NEXT, 4, 3,
																												FONT_NEXT, 4, 4,
																												FONT_NEXT, 3, 5,
																												FONT_NEXT, 0, 5,
																												FONT_NEXT, 0, 6,
																												FONT_NEXT, 2, 8,
																												FONT_END, 3, 8,
																												FONT_ADVANCE, 6, 0
																										},
																											{
																												55,
																													FONT_BEGIN, 0, 8,
																													FONT_NEXT, 4, 8,
																													FONT_NEXT, 2, 4,
																													FONT_END, 2, 2,
																													FONT_ADVANCE, 6, 0
																											},
																											{
																												56,
																													FONT_BEGIN, 1, 5,
																													FONT_NEXT, 0, 4,
																													FONT_NEXT, 0, 3,
																													FONT_NEXT, 1, 2,
																													FONT_NEXT, 3, 2,
																													FONT_NEXT, 4, 3,
																													FONT_NEXT, 4, 4,
																													FONT_NEXT, 3, 5,
																													FONT_NEXT, 1, 5,
																													FONT_NEXT, 0, 6,
																													FONT_NEXT, 0, 7,
																													FONT_NEXT, 1, 8,
																													FONT_NEXT, 3, 8,
																													FONT_NEXT, 4, 7,
																													FONT_NEXT, 4, 6,
																													FONT_END, 3, 5,
																													FONT_ADVANCE, 6, 0
																											},
																												{
																													57,
																														FONT_BEGIN, 1, 2,
																														FONT_NEXT, 2, 2,
																														FONT_NEXT, 4, 4,
																														FONT_NEXT, 4, 7,
																														FONT_NEXT, 3, 8,
																														FONT_NEXT, 1, 8,
																														FONT_NEXT, 0, 7,
																														FONT_NEXT, 0, 6,
																														FONT_NEXT, 1, 5,
																														FONT_END, 4, 5,
																														FONT_ADVANCE, 6, 0
																												},
																												{
																													58,
																														FONT_BEGIN, 0, 3,
																														FONT_END, 1, 3,
																														FONT_BEGIN, 1, 6,
																														FONT_END, 0, 6,
																														FONT_ADVANCE, 6, 0
																												},
																													{
																														59,
																															FONT_BEGIN, 0, 1,
																															FONT_NEXT, 1, 2,
																															FONT_END, 1, 3,
																															FONT_BEGIN, 1, 6,
																															FONT_END, 0, 6,
																															FONT_ADVANCE, 6, 0
																													},
																													{
																														60,
																															FONT_BEGIN, 3, 3,
																															FONT_NEXT, 1, 5,
																															FONT_END, 3, 7,
																															FONT_ADVANCE, 6, 0
																													},
																														{
																															61,
																																FONT_BEGIN, 0, 4,
																																FONT_END, 4, 4,
																																FONT_BEGIN, 4, 6,
																																FONT_END, 0, 6,
																																FONT_ADVANCE, 6, 0
																														},
																														{
																															62,
																																FONT_BEGIN, 1, 7,
																																FONT_NEXT, 3, 5,
																																FONT_END, 1, 3,
																																FONT_ADVANCE, 6, 0
																														},
																															{
																																63,
																																	FONT_BEGIN, 1, 2,
																																	FONT_END, 2, 2,
																																	FONT_BEGIN, 2, 4,
																																	FONT_NEXT, 2, 5,
																																	FONT_NEXT, 4, 7,
																																	FONT_NEXT, 3, 8,
																																	FONT_NEXT, 1, 8,
																																	FONT_END, 0, 7,
																																	FONT_ADVANCE, 6, 0
																															},
																															{
																																64,
																																	FONT_BEGIN, 3, 2,
																																	FONT_NEXT, 1, 2,
																																	FONT_NEXT, 0, 3,
																																	FONT_NEXT, 0, 7,
																																	FONT_NEXT, 1, 8,
																																	FONT_NEXT, 3, 8,
																																	FONT_NEXT, 4, 7,
																																	FONT_NEXT, 4, 4,
																																	FONT_NEXT, 2, 4,
																																	FONT_NEXT, 2, 6,
																																	FONT_END, 4, 6,
																																	FONT_ADVANCE, 6, 0
																															},
																																{
																																	65,
																																		FONT_BEGIN, 0, 2,
																																		FONT_NEXT, 0, 7,
																																		FONT_NEXT, 1, 8,
																																		FONT_NEXT, 3, 8,
																																		FONT_NEXT, 4, 7,
																																		FONT_END, 4, 2,
																																		FONT_BEGIN, 0, 5,
																																		FONT_END, 4, 5,
																																		FONT_ADVANCE, 6, 0
																																},
																																{
																																	66,
																																		FONT_BEGIN, 0, 2,
																																		FONT_NEXT, 3, 2,
																																		FONT_NEXT, 4, 3,
																																		FONT_NEXT, 4, 4,
																																		FONT_NEXT, 3, 5,
																																		FONT_END, 0, 5,
																																		FONT_BEGIN, 3, 5,
																																		FONT_NEXT, 4, 6,
																																		FONT_NEXT, 4, 7,
																																		FONT_NEXT, 3, 8,
																																		FONT_NEXT, 0, 8,
																																		FONT_END, 0, 2,
																																		FONT_ADVANCE, 6, 0
																																},
																																	{
																																		67,
																																			FONT_BEGIN, 4, 7,
																																			FONT_NEXT, 3, 8,
																																			FONT_NEXT, 1, 8,
																																			FONT_NEXT, 0, 7,
																																			FONT_NEXT, 0, 3,
																																			FONT_NEXT, 1, 2,
																																			FONT_NEXT, 3, 2,
																																			FONT_END, 4, 3,
																																			FONT_ADVANCE, 6, 0
																																	},
																																	{
																																		68,
																																			FONT_BEGIN, 0, 2,
																																			FONT_NEXT, 3, 2,
																																			FONT_NEXT, 4, 3,
																																			FONT_NEXT, 4, 7,
																																			FONT_NEXT, 3, 8,
																																			FONT_END, 0, 8,
																																			FONT_BEGIN, 1, 8,
																																			FONT_END, 1, 2,
																																			FONT_ADVANCE, 6, 0
																																	},
																																		{
																																			69,
																																				FONT_BEGIN, 4, 2,
																																				FONT_NEXT, 0, 2,
																																				FONT_NEXT, 0, 8,
																																				FONT_END, 4, 8,
																																				FONT_BEGIN, 3, 5,
																																				FONT_END, 0, 5,
																																				FONT_ADVANCE, 6, 0
																																		},
																																		{
																																			70,
																																				FONT_BEGIN, 0, 2,
																																				FONT_NEXT, 0, 8,
																																				FONT_END, 4, 8,
																																				FONT_BEGIN, 3, 5,
																																				FONT_END, 0, 5,
																																				FONT_ADVANCE, 6, 0
																																		},
																																			{
																																				71,
																																					FONT_BEGIN, 3, 4,
																																					FONT_NEXT, 4, 4,
																																					FONT_NEXT, 4, 3,
																																					FONT_NEXT, 3, 2,
																																					FONT_NEXT, 1, 2,
																																					FONT_NEXT, 0, 3,
																																					FONT_NEXT, 0, 7,
																																					FONT_NEXT, 1, 8,
																																					FONT_NEXT, 3, 8,
																																					FONT_END, 4, 7,
																																					FONT_ADVANCE, 6, 0
																																			},
																																			{
																																				72,
																																					FONT_BEGIN, 0, 2,
																																					FONT_END, 0, 8,
																																					FONT_BEGIN, 0, 5,
																																					FONT_END, 4, 5,
																																					FONT_BEGIN, 4, 8,
																																					FONT_END, 4, 2,
																																					FONT_ADVANCE, 6, 0
																																			},
																																				{
																																					73,
																																						FONT_BEGIN, 1, 2,
																																						FONT_END, 3, 2,
																																						FONT_BEGIN, 2, 2,
																																						FONT_END, 2, 8,
																																						FONT_BEGIN, 1, 8,
																																						FONT_END, 3, 8,
																																						FONT_ADVANCE, 6, 0
																																				},
																																				{
																																					74,
																																						FONT_BEGIN, 0, 3,
																																						FONT_NEXT, 1, 2,
																																						FONT_NEXT, 3, 2,
																																						FONT_NEXT, 4, 3,
																																						FONT_END, 4, 8,
																																						FONT_ADVANCE, 6, 0
																																				},
																																					{
																																						75,
																																							FONT_BEGIN, 0, 2,
																																							FONT_END, 0, 8,
																																							FONT_BEGIN, 4, 8,
																																							FONT_NEXT, 1, 5,
																																							FONT_END, 4, 2,
																																							FONT_ADVANCE, 8, 0
																																					},
																																					{
																																						76,
																																							FONT_BEGIN, 0, 8,
																																							FONT_NEXT, 0, 2,
																																							FONT_END, 4, 2,
																																							FONT_ADVANCE, 6, 0
																																					},
																																						{
																																							77,
																																								FONT_BEGIN, 0, 2,
																																								FONT_NEXT, 0, 8,
																																								FONT_NEXT, 2, 6,
																																								FONT_NEXT, 4, 8,
																																								FONT_END, 4, 2,
																																								FONT_ADVANCE, 6, 0
																																						},
																																						{
																																							78,
																																								FONT_BEGIN, 0, 2,
																																								FONT_NEXT, 0, 8,
																																								FONT_NEXT, 4, 2,
																																								FONT_END, 4, 8,
																																								FONT_ADVANCE, 6, 0
																																						},
																																							{
																																								79,
																																									FONT_BEGIN, 0, 3,
																																									FONT_NEXT, 0, 7,
																																									FONT_NEXT, 1, 8,
																																									FONT_NEXT, 3, 8,
																																									FONT_NEXT, 4, 7,
																																									FONT_NEXT, 4, 3,
																																									FONT_NEXT, 3, 2,
																																									FONT_NEXT, 1, 2,
																																									FONT_END, 0, 3,
																																									FONT_ADVANCE, 6, 0
																																							},
																																							{
																																								80,
																																									FONT_BEGIN, 0, 2,
																																									FONT_NEXT, 0, 8,
																																									FONT_NEXT, 3, 8,
																																									FONT_NEXT, 4, 7,
																																									FONT_NEXT, 4, 6,
																																									FONT_NEXT, 3, 5,
																																									FONT_END, 0, 5,
																																									FONT_ADVANCE, 6, 0
																																							},
																																								{
																																									81,
																																										FONT_BEGIN, 0, 3,
																																										FONT_NEXT, 0, 7,
																																										FONT_NEXT, 1, 8,
																																										FONT_NEXT, 3, 8,
																																										FONT_NEXT, 4, 7,
																																										FONT_NEXT, 4, 4,
																																										FONT_NEXT, 2, 2,
																																										FONT_NEXT, 1, 2,
																																										FONT_END, 0, 3,
																																										FONT_BEGIN, 2, 5,
																																										FONT_END, 4, 3,
																																										FONT_ADVANCE, 6, 0
																																								},
																																								{
																																									82,
																																										FONT_BEGIN, 0, 2,
																																										FONT_NEXT, 0, 8,
																																										FONT_NEXT, 3, 8,
																																										FONT_NEXT, 4, 7,
																																										FONT_NEXT, 4, 6,
																																										FONT_NEXT, 3, 5,
																																										FONT_END, 0, 5,
																																										FONT_BEGIN, 1, 5,
																																										FONT_END, 4, 2,
																																										FONT_ADVANCE, 6, 0
																																								},
																																									{
																																										83,
																																											FONT_BEGIN, 4, 7,
																																											FONT_NEXT, 3, 8,
																																											FONT_NEXT, 1, 8,
																																											FONT_NEXT, 0, 7,
																																											FONT_NEXT, 0, 6,
																																											FONT_NEXT, 1, 5,
																																											FONT_NEXT, 3, 5,
																																											FONT_NEXT, 4, 4,
																																											FONT_NEXT, 4, 3,
																																											FONT_NEXT, 3, 2,
																																											FONT_NEXT, 1, 2,
																																											FONT_END, 0, 3,
																																											FONT_ADVANCE, 6, 0
																																									},
																																									{
																																										84,
																																											FONT_BEGIN, 0, 8,
																																											FONT_END, 4, 8,
																																											FONT_BEGIN, 2, 8,
																																											FONT_END, 2, 2,
																																											FONT_ADVANCE, 6, 0
																																									},
																																										{
																																											85,
																																												FONT_BEGIN, 0, 3,
																																												FONT_END, 0, 8,
																																												FONT_BEGIN, 4, 8,
																																												FONT_NEXT, 4, 3,
																																												FONT_NEXT, 3, 2,
																																												FONT_NEXT, 1, 2,
																																												FONT_END, 0, 3,
																																												FONT_ADVANCE, 6, 0
																																										},
																																										{
																																											86,
																																												FONT_BEGIN, 0, 8,
																																												FONT_NEXT, 0, 5,
																																												FONT_NEXT, 2, 2,
																																												FONT_NEXT, 4, 5,
																																												FONT_END, 4, 8,
																																												FONT_ADVANCE, 6, 0
																																										},
																																											{
																																												87,
																																													FONT_BEGIN, 0, 2,
																																													FONT_END, 0, 8,
																																													FONT_BEGIN, 4, 8,
																																													FONT_NEXT, 4, 2,
																																													FONT_NEXT, 2, 4,
																																													FONT_END, 0, 2,
																																													FONT_ADVANCE, 6, 0
																																											},
																																											{
																																												88,
																																													FONT_BEGIN, 0, 2,
																																													FONT_END, 4, 8,
																																													FONT_BEGIN, 0, 8,
																																													FONT_END, 4, 2,
																																													FONT_ADVANCE, 6, 0
																																											},
																																												{
																																													89,
																																														FONT_BEGIN, 2, 2,
																																														FONT_NEXT, 2, 5,
																																														FONT_END, 0, 8,
																																														FONT_BEGIN, 4, 8,
																																														FONT_END, 2, 5,
																																														FONT_ADVANCE, 6, 0
																																												},
																																												{
																																													90,
																																														FONT_BEGIN, 0, 8,
																																														FONT_NEXT, 4, 8,
																																														FONT_NEXT, 0, 2,
																																														FONT_END, 4, 2,
																																														FONT_ADVANCE, 6, 0
																																												},
																																													{
																																														91,
																																															FONT_BEGIN, 3, 1,
																																															FONT_NEXT, 1, 1,
																																															FONT_NEXT, 1, 9,
																																															FONT_END, 3, 9,
																																															FONT_ADVANCE, 6, 0
																																													},
																																													{
																																														92,
																																															FONT_BEGIN, 0, 7,
																																															FONT_END, 4, 3,
																																															FONT_ADVANCE, 6, 0
																																													},
																																														{
																																															93,
																																																FONT_BEGIN, 1, 9,
																																																FONT_NEXT, 3, 9,
																																																FONT_NEXT, 3, 1,
																																																FONT_END, 1, 1,
																																																FONT_ADVANCE, 6, 0
																																														},
																																														{
																																															94,
																																																FONT_BEGIN, 2, 2,
																																																FONT_END, 2, 8,
																																																FONT_BEGIN, 0, 6,
																																																FONT_NEXT, 2, 8,
																																																FONT_END, 4, 6,
																																																FONT_ADVANCE, 6, 0
																																														},
																																															{
																																																95,
																																																	FONT_BEGIN, 2, 3,
																																																	FONT_NEXT, 0, 5,
																																																	FONT_END, 2, 7,
																																																	FONT_BEGIN, 0, 5,
																																																	FONT_END, 4, 5,
																																																	FONT_ADVANCE, 6, 0
																																															},
																																															{
																																																96,
																																																	FONT_BEGIN, 3, 8,
																																																	FONT_NEXT, 2, 8,
																																																	FONT_NEXT, 2, 9,
																																																	FONT_NEXT, 3, 9,
																																																	FONT_NEXT, 3, 8,
																																																	FONT_END, 4, 7,
																																																	FONT_ADVANCE, 6, 0
																																															},
																																																{
																																																	97,
																																																		FONT_BEGIN, 1, 6,
																																																		FONT_NEXT, 3, 6,
																																																		FONT_NEXT, 4, 5,
																																																		FONT_NEXT, 4, 2,
																																																		FONT_NEXT, 1, 2,
																																																		FONT_NEXT, 0, 3,
																																																		FONT_NEXT, 1, 4,
																																																		FONT_END, 4, 4,
																																																		FONT_ADVANCE, 6, 0
																																																},
																																																{
																																																	98,
																																																		FONT_BEGIN, 0, 8,
																																																		FONT_NEXT, 0, 2,
																																																		FONT_NEXT, 3, 2,
																																																		FONT_NEXT, 4, 3,
																																																		FONT_NEXT, 4, 5,
																																																		FONT_NEXT, 3, 6,
																																																		FONT_END, 0, 6,
																																																		FONT_ADVANCE, 6, 0
																																																},
																																																	{
																																																		99,
																																																			FONT_BEGIN, 4, 5,
																																																			FONT_NEXT, 3, 6,
																																																			FONT_NEXT, 1, 6,
																																																			FONT_NEXT, 0, 5,
																																																			FONT_NEXT, 0, 3,
																																																			FONT_NEXT, 1, 2,
																																																			FONT_END, 4, 2,
																																																			FONT_ADVANCE, 6, 0
																																																	},
																																																	{
																																																		100,
																																																			FONT_BEGIN, 4, 8,
																																																			FONT_NEXT, 4, 2,
																																																			FONT_NEXT, 1, 2,
																																																			FONT_NEXT, 0, 3,
																																																			FONT_NEXT, 0, 5,
																																																			FONT_NEXT, 1, 6,
																																																			FONT_END, 4, 6,
																																																			FONT_ADVANCE, 6, 0
																																																	},
																																																		{
																																																			101,
																																																				FONT_BEGIN, 3, 2,
																																																				FONT_NEXT, 1, 2,
																																																				FONT_NEXT, 0, 3,
																																																				FONT_NEXT, 0, 5,
																																																				FONT_NEXT, 1, 6,
																																																				FONT_NEXT, 3, 6,
																																																				FONT_NEXT, 4, 5,
																																																				FONT_NEXT, 3, 4,
																																																				FONT_END, 0, 4,
																																																				FONT_ADVANCE, 6, 0
																																																		},
																																																		{
																																																			102,
																																																				FONT_BEGIN, 1, 2,
																																																				FONT_NEXT, 1, 7,
																																																				FONT_NEXT, 2, 8,
																																																				FONT_NEXT, 3, 8,
																																																				FONT_END, 4, 7,
																																																				FONT_BEGIN, 2, 5,
																																																				FONT_END, 0, 5,
																																																				FONT_ADVANCE, 6, 0
																																																		},
																																																			{
																																																				103,
																																																					FONT_BEGIN, 1, 0,
																																																					FONT_NEXT, 3, 0,
																																																					FONT_NEXT, 4, 1,
																																																					FONT_NEXT, 4, 5,
																																																					FONT_NEXT, 3, 6,
																																																					FONT_NEXT, 1, 6,
																																																					FONT_NEXT, 0, 5,
																																																					FONT_NEXT, 0, 3,
																																																					FONT_NEXT, 1, 2,
																																																					FONT_END, 4, 2,
																																																					FONT_ADVANCE, 6, 0
																																																			},
																																																			{
																																																				104,
																																																					FONT_BEGIN, 0, 2,
																																																					FONT_END, 0, 8,
																																																					FONT_BEGIN, 0, 6,
																																																					FONT_NEXT, 3, 6,
																																																					FONT_NEXT, 4, 5,
																																																					FONT_END, 4, 2,
																																																					FONT_ADVANCE, 6, 0
																																																			},
																																																				{
																																																					105,
																																																						FONT_BEGIN, 2, 2,
																																																						FONT_END, 2, 5,
																																																						FONT_BEGIN, 2, 6,
																																																						FONT_END, 2, 7,
																																																						FONT_ADVANCE, 6, 0
																																																				},
																																																				{
																																																					106,
																																																						FONT_BEGIN, 0, 1,
																																																						FONT_NEXT, 1, 0,
																																																						FONT_NEXT, 3, 0,
																																																						FONT_NEXT, 4, 1,
																																																						FONT_END, 4, 5,
																																																						FONT_BEGIN, 4, 6,
																																																						FONT_END, 4, 7,
																																																						FONT_ADVANCE, 6, 0
																																																				},
																																																					{
																																																						107,
																																																							FONT_BEGIN, 0, 2,
																																																							FONT_END, 0, 8,
																																																							FONT_BEGIN, 4, 6,
																																																							FONT_NEXT, 2, 4,
																																																							FONT_END, 0, 4,
																																																							FONT_BEGIN, 2, 4,
																																																							FONT_END, 4, 2,
																																																							FONT_ADVANCE, 6, 0
																																																					},
																																																					{
																																																						108,
																																																							FONT_BEGIN, 1, 2,
																																																							FONT_END, 1, 8,
																																																							FONT_ADVANCE, 6, 0
																																																					},
																																																						{
																																																							109,
																																																								FONT_BEGIN, 0, 2,
																																																								FONT_NEXT, 0, 6,
																																																								FONT_NEXT, 1, 6,
																																																								FONT_NEXT, 2, 5,
																																																								FONT_NEXT, 3, 6,
																																																								FONT_NEXT, 4, 5,
																																																								FONT_END, 4, 2,
																																																								FONT_BEGIN, 2, 2,
																																																								FONT_END, 2, 5,
																																																								FONT_ADVANCE, 6, 0
																																																						},
																																																						{
																																																							110,
																																																								FONT_BEGIN, 0, 2,
																																																								FONT_NEXT, 0, 6,
																																																								FONT_NEXT, 1, 5,
																																																								FONT_NEXT, 2, 6,
																																																								FONT_NEXT, 3, 6,
																																																								FONT_NEXT, 4, 5,
																																																								FONT_END, 4, 2,
																																																								FONT_ADVANCE, 6, 0
																																																						},
																																																							{
																																																								111,
																																																									FONT_BEGIN, 0, 3,
																																																									FONT_NEXT, 0, 5,
																																																									FONT_NEXT, 1, 6,
																																																									FONT_NEXT, 3, 6,
																																																									FONT_NEXT, 4, 5,
																																																									FONT_NEXT, 4, 3,
																																																									FONT_NEXT, 3, 2,
																																																									FONT_NEXT, 1, 2,
																																																									FONT_END, 0, 3,
																																																									FONT_ADVANCE, 6, 0
																																																							},
																																																							{
																																																								112,
																																																									FONT_BEGIN, 0, 0,
																																																									FONT_NEXT, 0, 6,
																																																									FONT_NEXT, 3, 6,
																																																									FONT_NEXT, 4, 5,
																																																									FONT_NEXT, 4, 3,
																																																									FONT_NEXT, 3, 2,
																																																									FONT_END, 0, 2,
																																																									FONT_ADVANCE, 6, 0
																																																							},
																																																								{
																																																									113,
																																																										FONT_BEGIN, 4, 2,
																																																										FONT_NEXT, 1, 2,
																																																										FONT_NEXT, 0, 3,
																																																										FONT_NEXT, 0, 5,
																																																										FONT_NEXT, 1, 6,
																																																										FONT_NEXT, 3, 6,
																																																										FONT_NEXT, 4, 5,
																																																										FONT_END, 4, 0,
																																																										FONT_ADVANCE, 6, 0
																																																								},
																																																								{
																																																									114,
																																																										FONT_BEGIN, 0, 2,
																																																										FONT_END, 0, 6,
																																																										FONT_BEGIN, 0, 4,
																																																										FONT_NEXT, 2, 6,
																																																										FONT_NEXT, 3, 6,
																																																										FONT_END, 4, 5,
																																																										FONT_ADVANCE, 6, 0
																																																								},
																																																									{
																																																										115,
																																																											FONT_BEGIN, 0, 2,
																																																											FONT_NEXT, 3, 2,
																																																											FONT_NEXT, 4, 3,
																																																											FONT_NEXT, 3, 4,
																																																											FONT_NEXT, 1, 4,
																																																											FONT_NEXT, 0, 5,
																																																											FONT_NEXT, 1, 6,
																																																											FONT_END, 4, 6,
																																																											FONT_ADVANCE, 6, 0
																																																									},
																																																									{
																																																										116,
																																																											FONT_BEGIN, 0, 6,
																																																											FONT_END, 4, 6,
																																																											FONT_BEGIN, 2, 8,
																																																											FONT_NEXT, 2, 3,
																																																											FONT_NEXT, 3, 2,
																																																											FONT_END, 4, 2,
																																																											FONT_ADVANCE, 6, 0
																																																									},
																																																										{
																																																											117,
																																																												FONT_BEGIN, 0, 6,
																																																												FONT_NEXT, 0, 3,
																																																												FONT_NEXT, 1, 2,
																																																												FONT_NEXT, 3, 2,
																																																												FONT_NEXT, 4, 3,
																																																												FONT_END, 4, 6,
																																																												FONT_ADVANCE, 6, 0
																																																										},
																																																										{
																																																											118,
																																																												FONT_BEGIN, 0, 6,
																																																												FONT_NEXT, 0, 4,
																																																												FONT_NEXT, 2, 2,
																																																												FONT_NEXT, 4, 4,
																																																												FONT_END, 4, 6,
																																																												FONT_ADVANCE, 6, 0
																																																										},
																																																											{
																																																												119,
																																																													FONT_BEGIN, 0, 6,
																																																													FONT_NEXT, 0, 3,
																																																													FONT_NEXT, 1, 2,
																																																													FONT_NEXT, 2, 3,
																																																													FONT_NEXT, 3, 2,
																																																													FONT_NEXT, 4, 3,
																																																													FONT_END, 4, 6,
																																																													FONT_ADVANCE, 6, 0
																																																											},
																																																											{
																																																												120,
																																																													FONT_BEGIN, 0, 2,
																																																													FONT_END, 4, 6,
																																																													FONT_BEGIN, 0, 6,
																																																													FONT_END, 4, 2,
																																																													FONT_ADVANCE, 6, 0
																																																											},
																																																												{
																																																													121,
																																																														FONT_BEGIN, 0, 0,
																																																														FONT_NEXT, 4, 4,
																																																														FONT_END, 4, 6,
																																																														FONT_BEGIN, 0, 6,
																																																														FONT_NEXT, 0, 4,
																																																														FONT_END, 2, 2,
																																																														FONT_ADVANCE, 6, 0
																																																												},
																																																												{
																																																													122,
																																																														FONT_BEGIN, 0, 6,
																																																														FONT_NEXT, 4, 6,
																																																														FONT_NEXT, 0, 2,
																																																														FONT_END, 4, 2,
																																																														FONT_ADVANCE, 6, 0
																																																												},
																																																													{
																																																														123,
																																																															FONT_BEGIN, 4, 9,
																																																															FONT_NEXT, 3, 8,
																																																															FONT_NEXT, 3, 6,
																																																															FONT_NEXT, 2, 5,
																																																															FONT_NEXT, 3, 4,
																																																															FONT_NEXT, 3, 2,
																																																															FONT_END, 4, 1,
																																																															FONT_ADVANCE, 6, 0
																																																													},
																																																													{
																																																														124,
																																																															FONT_BEGIN, 2, 9,
																																																															FONT_END, 2, 0,
																																																															FONT_ADVANCE, 6, 0
																																																													},
																																																														{
																																																															125,
																																																																FONT_BEGIN, 2, 2,
																																																																FONT_NEXT, 2, 3,
																																																																FONT_NEXT, 0, 5,
																																																																FONT_NEXT, 2, 7,
																																																																FONT_END, 2, 8,
																																																																FONT_BEGIN, 2, 7,
																																																																FONT_NEXT, 4, 5,
																																																																FONT_END, 2, 3,
																																																																FONT_ADVANCE, 6, 0
																																																														},
																																																														{
																																																															126,
																																																																FONT_BEGIN, 0, 9,
																																																																FONT_NEXT, 1, 8,
																																																																FONT_NEXT, 1, 6,
																																																																FONT_NEXT, 2, 5,
																																																																FONT_NEXT, 1, 4,
																																																																FONT_NEXT, 1, 2,
																																																																FONT_END, 0, 1,
																																																																FONT_ADVANCE, 6, 0
																																																														},
																																																															{
																																																																END_OF_LIST
																																																															}
			};
static GLint outlineFont[][1 + MAX_STROKES * 3] = {
				{
					32,
						FONT_ADVANCE, 250, 0
				},
				{
					33,
						FONT_BEGIN, 234, 559,
						FONT_NEXT, 236, 605,
						FONT_NEXT, 229, 642,
						FONT_NEXT, 211, 667,
						FONT_NEXT, 182, 676,
						FONT_NEXT, 156, 668,
						FONT_NEXT, 140, 648,
						FONT_NEXT, 130, 592,
						FONT_NEXT, 131, 568,
						FONT_NEXT, 134, 534,
						FONT_NEXT, 146, 447,
						FONT_NEXT, 158, 355,
						FONT_NEXT, 163, 315,
						FONT_NEXT, 167, 284,
						FONT_NEXT, 176, 176,
						FONT_END, 189, 176,
						FONT_BEGIN, 219, 3,
						FONT_NEXT, 232, 18,
						FONT_NEXT, 238, 42,
						FONT_NEXT, 233, 62,
						FONT_NEXT, 222, 79,
						FONT_NEXT, 204, 91,
						FONT_NEXT, 183, 96,
						FONT_NEXT, 162, 91,
						FONT_NEXT, 145, 80,
						FONT_NEXT, 134, 63,
						FONT_NEXT, 130, 42,
						FONT_NEXT, 135, 18,
						FONT_NEXT, 148, 2,
						FONT_END, 183, -9,
						FONT_ADVANCE, 333, 0
				},
					{
						34,
							FONT_BEGIN, 308, 482,
							FONT_NEXT, 318, 543,
							FONT_NEXT, 327, 599,
							FONT_NEXT, 331, 635,
							FONT_NEXT, 317, 665,
							FONT_NEXT, 289, 676,
							FONT_NEXT, 260, 665,
							FONT_NEXT, 246, 635,
							FONT_NEXT, 249, 599,
							FONT_NEXT, 258, 543,
							FONT_NEXT, 268, 482,
							FONT_NEXT, 278, 431,
							FONT_END, 299, 431,
							FONT_BEGIN, 139, 482,
							FONT_NEXT, 149, 543,
							FONT_NEXT, 158, 599,
							FONT_NEXT, 162, 635,
							FONT_NEXT, 148, 665,
							FONT_NEXT, 120, 676,
							FONT_NEXT, 91, 665,
							FONT_NEXT, 77, 635,
							FONT_NEXT, 80, 599,
							FONT_NEXT, 89, 543,
							FONT_NEXT, 99, 482,
							FONT_NEXT, 109, 431,
							FONT_END, 130, 431,
							FONT_ADVANCE, 408, 0
					},
					{
						35,
							FONT_BEGIN, 371, 271,
							FONT_NEXT, 391, 405,
							FONT_NEXT, 496, 405,
							FONT_NEXT, 496, 460,
							FONT_NEXT, 399, 460,
							FONT_NEXT, 429, 662,
							FONT_NEXT, 371, 662,
							FONT_NEXT, 341, 460,
							FONT_NEXT, 208, 460,
							FONT_NEXT, 239, 662,
							FONT_NEXT, 181, 662,
							FONT_NEXT, 150, 460,
							FONT_NEXT, 32, 460,
							FONT_NEXT, 32, 405,
							FONT_NEXT, 142, 405,
							FONT_NEXT, 121, 271,
							FONT_NEXT, 5, 271,
							FONT_NEXT, 5, 216,
							FONT_NEXT, 112, 216,
							FONT_NEXT, 79, 0,
							FONT_NEXT, 137, 0,
							FONT_NEXT, 170, 216,
							FONT_NEXT, 304, 216,
							FONT_NEXT, 273, 0,
							FONT_NEXT, 331, 0,
							FONT_NEXT, 362, 216,
							FONT_NEXT, 471, 216,
							FONT_END, 471, 271,
							FONT_BEGIN, 313, 271,
							FONT_NEXT, 179, 271,
							FONT_NEXT, 200, 405,
							FONT_END, 333, 405,
							FONT_ADVANCE, 500, 0
					},
						{
							36,
								FONT_BEGIN, 425, 611,
								FONT_NEXT, 387, 634,
								FONT_NEXT, 345, 649,
								FONT_NEXT, 264, 664,
								FONT_NEXT, 264, 727,
								FONT_NEXT, 230, 727,
								FONT_NEXT, 230, 664,
								FONT_NEXT, 185, 658,
								FONT_NEXT, 148, 646,
								FONT_NEXT, 93, 611,
								FONT_NEXT, 61, 563,
								FONT_NEXT, 52, 511,
								FONT_NEXT, 55, 476,
								FONT_NEXT, 67, 445,
								FONT_NEXT, 106, 392,
								FONT_NEXT, 163, 349,
								FONT_NEXT, 230, 310,
								FONT_NEXT, 230, 28,
								FONT_NEXT, 166, 37,
								FONT_NEXT, 117, 66,
								FONT_NEXT, 81, 114,
								FONT_NEXT, 59, 181,
								FONT_NEXT, 44, 181,
								FONT_NEXT, 44, 51,
								FONT_NEXT, 70, 35,
								FONT_NEXT, 110, 19,
								FONT_NEXT, 162, 5,
								FONT_NEXT, 230, 0,
								FONT_NEXT, 230, -87,
								FONT_NEXT, 264, -87,
								FONT_NEXT, 264, 0,
								FONT_NEXT, 338, 16,
								FONT_NEXT, 399, 49,
								FONT_NEXT, 441, 100,
								FONT_NEXT, 453, 134,
								FONT_NEXT, 457, 174,
								FONT_NEXT, 453, 211,
								FONT_NEXT, 444, 243,
								FONT_NEXT, 408, 295,
								FONT_NEXT, 348, 341,
								FONT_NEXT, 264, 391,
								FONT_NEXT, 264, 637,
								FONT_NEXT, 305, 628,
								FONT_NEXT, 348, 607,
								FONT_NEXT, 385, 566,
								FONT_NEXT, 399, 537,
								FONT_NEXT, 410, 500,
								FONT_END, 425, 500,
								FONT_BEGIN, 187, 435,
								FONT_NEXT, 155, 464,
								FONT_NEXT, 133, 495,
								FONT_NEXT, 126, 532,
								FONT_NEXT, 128, 555,
								FONT_NEXT, 141, 586,
								FONT_NEXT, 171, 616,
								FONT_NEXT, 196, 628,
								FONT_NEXT, 229, 637,
								FONT_END, 229, 407,
								FONT_BEGIN, 308, 264,
								FONT_NEXT, 344, 234,
								FONT_NEXT, 369, 197,
								FONT_NEXT, 378, 151,
								FONT_NEXT, 374, 116,
								FONT_NEXT, 364, 89,
								FONT_NEXT, 333, 53,
								FONT_NEXT, 295, 35,
								FONT_NEXT, 264, 28,
								FONT_END, 264, 293,
								FONT_ADVANCE, 500, 0
						},
						{
							37,
								FONT_BEGIN, 622, 365,
								FONT_NEXT, 583, 349,
								FONT_NEXT, 514, 293,
								FONT_NEXT, 466, 218,
								FONT_NEXT, 453, 177,
								FONT_NEXT, 449, 137,
								FONT_NEXT, 455, 86,
								FONT_NEXT, 475, 42,
								FONT_NEXT, 513, 11,
								FONT_NEXT, 572, 0,
								FONT_NEXT, 615, 6,
								FONT_NEXT, 654, 25,
								FONT_NEXT, 688, 53,
								FONT_NEXT, 717, 88,
								FONT_NEXT, 757, 173,
								FONT_NEXT, 772, 261,
								FONT_NEXT, 768, 295,
								FONT_NEXT, 760, 321,
								FONT_NEXT, 731, 354,
								FONT_NEXT, 695, 368,
								FONT_END, 663, 371,
								FONT_BEGIN, 703, 332,
								FONT_NEXT, 726, 314,
								FONT_NEXT, 740, 287,
								FONT_NEXT, 746, 254,
								FONT_NEXT, 733, 183,
								FONT_NEXT, 700, 110,
								FONT_NEXT, 650, 53,
								FONT_NEXT, 620, 36,
								FONT_NEXT, 589, 30,
								FONT_NEXT, 552, 39,
								FONT_NEXT, 533, 60,
								FONT_NEXT, 525, 106,
								FONT_NEXT, 527, 128,
								FONT_NEXT, 534, 160,
								FONT_NEXT, 564, 238,
								FONT_NEXT, 611, 308,
								FONT_NEXT, 641, 330,
								FONT_END, 676, 339,
								FONT_BEGIN, 595, 676,
								FONT_NEXT, 536, 630,
								FONT_NEXT, 493, 614,
								FONT_NEXT, 438, 608,
								FONT_NEXT, 404, 610,
								FONT_NEXT, 381, 616,
								FONT_NEXT, 352, 635,
								FONT_NEXT, 325, 653,
								FONT_NEXT, 305, 659,
								FONT_NEXT, 276, 662,
								FONT_NEXT, 232, 655,
								FONT_NEXT, 192, 639,
								FONT_NEXT, 156, 613,
								FONT_NEXT, 124, 581,
								FONT_NEXT, 78, 504,
								FONT_NEXT, 65, 463,
								FONT_NEXT, 61, 425,
								FONT_NEXT, 64, 390,
								FONT_NEXT, 72, 361,
								FONT_NEXT, 102, 319,
								FONT_NEXT, 140, 296,
								FONT_NEXT, 178, 289,
								FONT_NEXT, 223, 295,
								FONT_NEXT, 264, 314,
								FONT_NEXT, 299, 343,
								FONT_NEXT, 329, 379,
								FONT_NEXT, 369, 463,
								FONT_NEXT, 384, 548,
								FONT_NEXT, 377, 593,
								FONT_NEXT, 401, 585,
								FONT_NEXT, 443, 583,
								FONT_NEXT, 478, 585,
								FONT_NEXT, 507, 592,
								FONT_NEXT, 548, 611,
								FONT_NEXT, 550, 609,
								FONT_NEXT, 201, -13,
								FONT_NEXT, 249, -13,
								FONT_END, 634, 676,
								FONT_BEGIN, 172, 326,
								FONT_NEXT, 153, 339,
								FONT_NEXT, 141, 362,
								FONT_NEXT, 137, 397,
								FONT_NEXT, 150, 468,
								FONT_NEXT, 184, 545,
								FONT_NEXT, 230, 606,
								FONT_NEXT, 255, 625,
								FONT_NEXT, 281, 632,
								FONT_NEXT, 299, 622,
								FONT_NEXT, 315, 611,
								FONT_NEXT, 350, 600,
								FONT_NEXT, 359, 553,
								FONT_NEXT, 345, 469,
								FONT_NEXT, 309, 396,
								FONT_NEXT, 257, 343,
								FONT_NEXT, 228, 328,
								FONT_END, 199, 323,
								FONT_ADVANCE, 833, 0
						},
							{
								38,
									FONT_BEGIN, 287, 596,
									FONT_NEXT, 304, 623,
									FONT_NEXT, 329, 639,
									FONT_NEXT, 359, 644,
									FONT_NEXT, 389, 637,
									FONT_NEXT, 414, 620,
									FONT_NEXT, 431, 594,
									FONT_NEXT, 438, 560,
									FONT_NEXT, 423, 504,
									FONT_NEXT, 389, 462,
									FONT_NEXT, 350, 432,
									FONT_NEXT, 321, 416,
									FONT_NEXT, 293, 483,
									FONT_END, 281, 557,
									FONT_BEGIN, 495, 426,
									FONT_NEXT, 495, 405,
									FONT_NEXT, 540, 395,
									FONT_NEXT, 554, 381,
									FONT_NEXT, 559, 359,
									FONT_NEXT, 548, 310,
									FONT_NEXT, 524, 259,
									FONT_NEXT, 494, 213,
									FONT_NEXT, 468, 178,
									FONT_NEXT, 396, 277,
									FONT_NEXT, 336, 384,
									FONT_NEXT, 387, 409,
									FONT_NEXT, 437, 444,
									FONT_NEXT, 475, 492,
									FONT_NEXT, 486, 521,
									FONT_NEXT, 491, 556,
									FONT_NEXT, 480, 606,
									FONT_NEXT, 450, 644,
									FONT_NEXT, 407, 667,
									FONT_NEXT, 355, 676,
									FONT_NEXT, 319, 672,
									FONT_NEXT, 288, 661,
									FONT_NEXT, 240, 624,
									FONT_NEXT, 211, 573,
									FONT_NEXT, 202, 519,
									FONT_NEXT, 211, 452,
									FONT_NEXT, 237, 377,
									FONT_NEXT, 199, 355,
									FONT_NEXT, 146, 320,
									FONT_NEXT, 95, 274,
									FONT_NEXT, 57, 215,
									FONT_NEXT, 42, 144,
									FONT_NEXT, 46, 99,
									FONT_NEXT, 59, 64,
									FONT_NEXT, 78, 36,
									FONT_NEXT, 101, 16,
									FONT_NEXT, 155, -7,
									FONT_NEXT, 207, -13,
									FONT_NEXT, 286, -4,
									FONT_NEXT, 347, 19,
									FONT_NEXT, 394, 49,
									FONT_NEXT, 429, 78,
									FONT_NEXT, 456, 51,
									FONT_NEXT, 494, 21,
									FONT_NEXT, 542, -3,
									FONT_NEXT, 599, -13,
									FONT_NEXT, 635, -9,
									FONT_NEXT, 667, 4,
									FONT_NEXT, 713, 42,
									FONT_NEXT, 740, 80,
									FONT_NEXT, 747, 93,
									FONT_NEXT, 750, 100,
									FONT_NEXT, 735, 111,
									FONT_NEXT, 697, 73,
									FONT_NEXT, 672, 62,
									FONT_NEXT, 639, 58,
									FONT_NEXT, 585, 70,
									FONT_NEXT, 540, 98,
									FONT_NEXT, 507, 129,
									FONT_NEXT, 491, 150,
									FONT_NEXT, 528, 200,
									FONT_NEXT, 550, 232,
									FONT_NEXT, 568, 261,
									FONT_NEXT, 591, 300,
									FONT_NEXT, 617, 346,
									FONT_NEXT, 638, 375,
									FONT_NEXT, 661, 392,
									FONT_NEXT, 711, 405,
									FONT_END, 711, 426,
									FONT_BEGIN, 146, 238,
									FONT_NEXT, 176, 283,
									FONT_NEXT, 214, 317,
									FONT_NEXT, 252, 343,
									FONT_NEXT, 322, 219,
									FONT_NEXT, 404, 104,
									FONT_NEXT, 356, 69,
									FONT_NEXT, 316, 49,
									FONT_NEXT, 285, 41,
									FONT_NEXT, 263, 39,
									FONT_NEXT, 212, 50,
									FONT_NEXT, 171, 80,
									FONT_NEXT, 144, 125,
									FONT_END, 134, 180,
									FONT_ADVANCE, 778, 0
							},
							{
								39,
									FONT_BEGIN, 136, 450,
									FONT_NEXT, 173, 483,
									FONT_NEXT, 204, 528,
									FONT_NEXT, 218, 580,
									FONT_NEXT, 208, 630,
									FONT_NEXT, 186, 659,
									FONT_NEXT, 159, 672,
									FONT_NEXT, 137, 676,
									FONT_NEXT, 98, 663,
									FONT_NEXT, 84, 645,
									FONT_NEXT, 79, 618,
									FONT_NEXT, 85, 593,
									FONT_NEXT, 100, 578,
									FONT_NEXT, 141, 568,
									FONT_NEXT, 156, 570,
									FONT_NEXT, 165, 572,
									FONT_NEXT, 174, 568,
									FONT_NEXT, 179, 558,
									FONT_NEXT, 175, 539,
									FONT_NEXT, 162, 513,
									FONT_NEXT, 137, 483,
									FONT_NEXT, 97, 452,
									FONT_END, 106, 433,
									FONT_ADVANCE, 333, 0
							},
								{
									40,
										FONT_BEGIN, 239, 638,
										FONT_NEXT, 191, 596,
										FONT_NEXT, 148, 549,
										FONT_NEXT, 113, 499,
										FONT_NEXT, 85, 444,
										FONT_NEXT, 64, 386,
										FONT_NEXT, 52, 325,
										FONT_NEXT, 48, 262,
										FONT_NEXT, 53, 169,
										FONT_NEXT, 70, 96,
										FONT_NEXT, 96, 35,
										FONT_NEXT, 130, -20,
										FONT_NEXT, 203, -108,
										FONT_NEXT, 247, -148,
										FONT_NEXT, 292, -177,
										FONT_NEXT, 304, -161,
										FONT_NEXT, 247, -116,
										FONT_NEXT, 205, -63,
										FONT_NEXT, 175, -4,
										FONT_NEXT, 155, 56,
										FONT_NEXT, 136, 174,
										FONT_NEXT, 134, 225,
										FONT_NEXT, 134, 269,
										FONT_NEXT, 138, 356,
										FONT_NEXT, 150, 428,
										FONT_NEXT, 168, 488,
										FONT_NEXT, 191, 537,
										FONT_NEXT, 246, 609,
										FONT_NEXT, 304, 660,
										FONT_END, 295, 676,
										FONT_ADVANCE, 333, 0
								},
								{
									41,
										FONT_BEGIN, 93, -140,
										FONT_NEXT, 141, -98,
										FONT_NEXT, 184, -51,
										FONT_NEXT, 219, 0,
										FONT_NEXT, 247, 54,
										FONT_NEXT, 268, 112,
										FONT_NEXT, 280, 173,
										FONT_NEXT, 285, 237,
										FONT_NEXT, 279, 329,
										FONT_NEXT, 262, 402,
										FONT_NEXT, 236, 463,
										FONT_NEXT, 203, 519,
										FONT_NEXT, 129, 606,
										FONT_NEXT, 85, 646,
										FONT_NEXT, 41, 676,
										FONT_NEXT, 29, 660,
										FONT_NEXT, 85, 614,
										FONT_NEXT, 127, 561,
										FONT_NEXT, 157, 502,
										FONT_NEXT, 177, 442,
										FONT_NEXT, 196, 324,
										FONT_NEXT, 198, 273,
										FONT_NEXT, 199, 230,
										FONT_NEXT, 194, 142,
										FONT_NEXT, 182, 70,
										FONT_NEXT, 164, 10,
										FONT_NEXT, 141, -39,
										FONT_NEXT, 86, -111,
										FONT_NEXT, 29, -161,
										FONT_END, 38, -177,
										FONT_ADVANCE, 333, 0
								},
									{
										42,
											FONT_BEGIN, 76, 362,
											FONT_NEXT, 103, 351,
											FONT_NEXT, 129, 358,
											FONT_NEXT, 143, 371,
											FONT_NEXT, 164, 394,
											FONT_NEXT, 192, 422,
											FONT_NEXT, 210, 435,
											FONT_NEXT, 240, 456,
											FONT_NEXT, 241, 449,
											FONT_NEXT, 240, 405,
											FONT_NEXT, 230, 365,
											FONT_NEXT, 219, 331,
											FONT_NEXT, 214, 304,
											FONT_NEXT, 225, 275,
											FONT_NEXT, 249, 265,
											FONT_NEXT, 277, 278,
											FONT_NEXT, 288, 307,
											FONT_NEXT, 274, 352,
											FONT_NEXT, 264, 389,
											FONT_NEXT, 260, 449,
											FONT_NEXT, 260, 456,
											FONT_NEXT, 267, 453,
											FONT_NEXT, 307, 425,
											FONT_NEXT, 338, 391,
											FONT_NEXT, 366, 362,
											FONT_NEXT, 396, 350,
											FONT_NEXT, 421, 359,
											FONT_NEXT, 432, 386,
											FONT_NEXT, 428, 404,
											FONT_NEXT, 419, 416,
											FONT_NEXT, 384, 428,
											FONT_NEXT, 334, 439,
											FONT_NEXT, 276, 465,
											FONT_NEXT, 268, 470,
											FONT_NEXT, 304, 489,
											FONT_NEXT, 336, 502,
											FONT_NEXT, 388, 513,
											FONT_NEXT, 420, 525,
											FONT_NEXT, 428, 537,
											FONT_NEXT, 431, 557,
											FONT_NEXT, 427, 574,
											FONT_NEXT, 417, 585,
											FONT_NEXT, 395, 593,
											FONT_NEXT, 368, 581,
											FONT_NEXT, 341, 554,
											FONT_NEXT, 309, 520,
											FONT_NEXT, 267, 488,
											FONT_NEXT, 260, 484,
											FONT_NEXT, 260, 510,
											FONT_NEXT, 273, 582,
											FONT_NEXT, 282, 613,
											FONT_NEXT, 287, 637,
											FONT_NEXT, 279, 661,
											FONT_NEXT, 269, 671,
											FONT_NEXT, 253, 676,
											FONT_NEXT, 231, 671,
											FONT_NEXT, 220, 659,
											FONT_NEXT, 216, 637,
											FONT_NEXT, 228, 581,
											FONT_NEXT, 237, 548,
											FONT_NEXT, 241, 499,
											FONT_NEXT, 241, 486,
											FONT_NEXT, 185, 523,
											FONT_NEXT, 152, 557,
											FONT_NEXT, 129, 581,
											FONT_NEXT, 102, 591,
											FONT_NEXT, 85, 586,
											FONT_NEXT, 75, 576,
											FONT_NEXT, 69, 557,
											FONT_NEXT, 72, 538,
											FONT_NEXT, 83, 526,
											FONT_NEXT, 120, 511,
											FONT_NEXT, 170, 500,
											FONT_NEXT, 223, 478,
											FONT_NEXT, 234, 471,
											FONT_NEXT, 172, 441,
											FONT_NEXT, 120, 428,
											FONT_NEXT, 82, 415,
											FONT_NEXT, 72, 402,
											FONT_END, 69, 383,
											FONT_ADVANCE, 500, 0
									},
									{
										43,
											FONT_BEGIN, 30, 286,
											FONT_NEXT, 30, 220,
											FONT_NEXT, 249, 220,
											FONT_NEXT, 249, 0,
											FONT_NEXT, 315, 0,
											FONT_NEXT, 315, 220,
											FONT_NEXT, 534, 220,
											FONT_NEXT, 534, 286,
											FONT_NEXT, 315, 286,
											FONT_NEXT, 315, 506,
											FONT_NEXT, 249, 506,
											FONT_END, 249, 286,
											FONT_ADVANCE, 564, 0
									},
										{
											44,
												FONT_BEGIN, 113, -124,
												FONT_NEXT, 150, -91,
												FONT_NEXT, 181, -46,
												FONT_NEXT, 195, 6,
												FONT_NEXT, 185, 56,
												FONT_NEXT, 163, 85,
												FONT_NEXT, 136, 98,
												FONT_NEXT, 114, 102,
												FONT_NEXT, 75, 89,
												FONT_NEXT, 61, 71,
												FONT_NEXT, 56, 44,
												FONT_NEXT, 62, 19,
												FONT_NEXT, 77, 4,
												FONT_NEXT, 118, -6,
												FONT_NEXT, 133, -4,
												FONT_NEXT, 142, -2,
												FONT_NEXT, 151, -6,
												FONT_NEXT, 156, -16,
												FONT_NEXT, 152, -35,
												FONT_NEXT, 139, -61,
												FONT_NEXT, 114, -91,
												FONT_NEXT, 74, -122,
												FONT_END, 83, -141,
												FONT_ADVANCE, 250, 0
										},
										{
											45,
												FONT_BEGIN, 39, 194,
												FONT_NEXT, 285, 194,
												FONT_NEXT, 285, 257,
												FONT_END, 39, 257,
												FONT_ADVANCE, 333, 0
										},
											{
												46,
													FONT_BEGIN, 160, 2,
													FONT_NEXT, 175, 18,
													FONT_NEXT, 181, 43,
													FONT_NEXT, 176, 64,
													FONT_NEXT, 164, 83,
													FONT_NEXT, 146, 95,
													FONT_NEXT, 125, 100,
													FONT_NEXT, 103, 95,
													FONT_NEXT, 86, 83,
													FONT_NEXT, 74, 65,
													FONT_NEXT, 70, 43,
													FONT_NEXT, 75, 18,
													FONT_NEXT, 89, 1,
													FONT_END, 125, -11,
													FONT_ADVANCE, 250, 0
											},
											{
												47,
													FONT_BEGIN, 220, 676,
													FONT_NEXT, -9, -14,
													FONT_NEXT, 59, -14,
													FONT_END, 287, 676,
													FONT_ADVANCE, 278, 0
											},
												{
													48,
														FONT_BEGIN, 292, 639,
														FONT_NEXT, 324, 609,
														FONT_NEXT, 347, 566,
														FONT_NEXT, 363, 516,
														FONT_NEXT, 377, 409,
														FONT_NEXT, 379, 364,
														FONT_NEXT, 380, 331,
														FONT_NEXT, 379, 294,
														FONT_NEXT, 377, 247,
														FONT_NEXT, 363, 141,
														FONT_NEXT, 347, 92,
														FONT_NEXT, 324, 50,
														FONT_NEXT, 292, 22,
														FONT_NEXT, 250, 12,
														FONT_NEXT, 207, 22,
														FONT_NEXT, 175, 50,
														FONT_NEXT, 152, 91,
														FONT_NEXT, 137, 141,
														FONT_NEXT, 122, 246,
														FONT_NEXT, 120, 294,
														FONT_NEXT, 120, 331,
														FONT_NEXT, 120, 364,
														FONT_NEXT, 122, 409,
														FONT_NEXT, 137, 516,
														FONT_NEXT, 152, 566,
														FONT_NEXT, 175, 609,
														FONT_NEXT, 207, 639,
														FONT_END, 250, 650,
														FONT_BEGIN, 187, 666,
														FONT_NEXT, 137, 638,
														FONT_NEXT, 97, 597,
														FONT_NEXT, 68, 547,
														FONT_NEXT, 33, 434,
														FONT_NEXT, 26, 379,
														FONT_NEXT, 24, 331,
														FONT_NEXT, 33, 226,
														FONT_NEXT, 68, 113,
														FONT_NEXT, 97, 63,
														FONT_NEXT, 137, 22,
														FONT_NEXT, 187, -5,
														FONT_NEXT, 250, -14,
														FONT_NEXT, 312, -5,
														FONT_NEXT, 362, 22,
														FONT_NEXT, 402, 63,
														FONT_NEXT, 432, 113,
														FONT_NEXT, 466, 226,
														FONT_NEXT, 476, 331,
														FONT_NEXT, 473, 379,
														FONT_NEXT, 466, 434,
														FONT_NEXT, 432, 547,
														FONT_NEXT, 402, 597,
														FONT_NEXT, 362, 638,
														FONT_NEXT, 312, 666,
														FONT_END, 250, 676,
														FONT_ADVANCE, 500, 0
												},
												{
													49,
														FONT_BEGIN, 394, 0,
														FONT_NEXT, 394, 15,
														FONT_NEXT, 347, 17,
														FONT_NEXT, 318, 26,
														FONT_NEXT, 303, 44,
														FONT_NEXT, 299, 74,
														FONT_NEXT, 299, 674,
														FONT_NEXT, 291, 676,
														FONT_NEXT, 111, 585,
														FONT_NEXT, 111, 571,
														FONT_NEXT, 143, 583,
														FONT_NEXT, 161, 590,
														FONT_NEXT, 179, 593,
														FONT_NEXT, 198, 588,
														FONT_NEXT, 208, 577,
														FONT_NEXT, 213, 546,
														FONT_NEXT, 213, 93,
														FONT_NEXT, 206, 49,
														FONT_NEXT, 187, 27,
														FONT_NEXT, 157, 17,
														FONT_NEXT, 118, 15,
														FONT_END, 118, 0,
														FONT_ADVANCE, 500, 0
												},
													{
														50,
															FONT_BEGIN, 462, 142,
															FONT_NEXT, 435, 105,
															FONT_NEXT, 413, 85,
															FONT_NEXT, 367, 76,
															FONT_NEXT, 128, 76,
															FONT_NEXT, 296, 252,
															FONT_NEXT, 334, 295,
															FONT_NEXT, 376, 351,
															FONT_NEXT, 410, 419,
															FONT_NEXT, 424, 496,
															FONT_NEXT, 419, 535,
															FONT_NEXT, 408, 571,
															FONT_NEXT, 367, 628,
															FONT_NEXT, 309, 663,
															FONT_NEXT, 243, 676,
															FONT_NEXT, 176, 666,
															FONT_NEXT, 115, 633,
															FONT_NEXT, 88, 606,
															FONT_NEXT, 64, 571,
															FONT_NEXT, 45, 528,
															FONT_NEXT, 31, 477,
															FONT_NEXT, 52, 472,
															FONT_NEXT, 70, 515,
															FONT_NEXT, 97, 557,
															FONT_NEXT, 137, 589,
															FONT_NEXT, 195, 602,
															FONT_NEXT, 235, 597,
															FONT_NEXT, 267, 586,
															FONT_NEXT, 310, 549,
															FONT_NEXT, 332, 502,
															FONT_NEXT, 338, 459,
															FONT_NEXT, 329, 400,
															FONT_NEXT, 305, 336,
															FONT_NEXT, 265, 270,
															FONT_NEXT, 208, 201,
															FONT_NEXT, 30, 12,
															FONT_NEXT, 30, 0,
															FONT_NEXT, 420, 0,
															FONT_END, 475, 137,
															FONT_ADVANCE, 500, 0
													},
													{
														51,
															FONT_BEGIN, 197, 330,
															FONT_NEXT, 230, 326,
															FONT_NEXT, 283, 308,
															FONT_NEXT, 321, 276,
															FONT_NEXT, 345, 239,
															FONT_NEXT, 356, 202,
															FONT_NEXT, 359, 176,
															FONT_NEXT, 351, 120,
															FONT_NEXT, 326, 70,
															FONT_NEXT, 285, 35,
															FONT_NEXT, 225, 22,
															FONT_NEXT, 184, 30,
															FONT_NEXT, 148, 50,
															FONT_NEXT, 115, 69,
															FONT_NEXT, 82, 78,
															FONT_NEXT, 56, 71,
															FONT_NEXT, 46, 60,
															FONT_NEXT, 43, 43,
															FONT_NEXT, 56, 12,
															FONT_NEXT, 89, -5,
															FONT_NEXT, 126, -13,
															FONT_NEXT, 154, -14,
															FONT_NEXT, 210, -10,
															FONT_NEXT, 263, 1,
															FONT_NEXT, 310, 21,
															FONT_NEXT, 351, 47,
															FONT_NEXT, 384, 80,
															FONT_NEXT, 409, 120,
															FONT_NEXT, 425, 165,
															FONT_NEXT, 431, 216,
															FONT_NEXT, 427, 261,
															FONT_NEXT, 417, 298,
															FONT_NEXT, 383, 351,
															FONT_NEXT, 341, 383,
															FONT_NEXT, 304, 401,
															FONT_NEXT, 346, 433,
															FONT_NEXT, 375, 466,
															FONT_NEXT, 391, 502,
															FONT_NEXT, 397, 541,
															FONT_NEXT, 390, 579,
															FONT_NEXT, 367, 623,
															FONT_NEXT, 319, 660,
															FONT_NEXT, 284, 671,
															FONT_NEXT, 241, 676,
															FONT_NEXT, 187, 669,
															FONT_NEXT, 132, 644,
															FONT_NEXT, 83, 594,
															FONT_NEXT, 62, 558,
															FONT_NEXT, 45, 514,
															FONT_NEXT, 60, 510,
															FONT_NEXT, 76, 538,
															FONT_NEXT, 105, 573,
															FONT_NEXT, 148, 603,
															FONT_NEXT, 208, 616,
															FONT_NEXT, 256, 606,
															FONT_NEXT, 290, 583,
															FONT_NEXT, 311, 549,
															FONT_NEXT, 318, 512,
															FONT_NEXT, 304, 450,
															FONT_NEXT, 269, 403,
															FONT_NEXT, 216, 368,
															FONT_NEXT, 152, 343,
															FONT_END, 153, 330,
															FONT_ADVANCE, 500, 0
													},
														{
															52,
																FONT_BEGIN, 370, 231,
																FONT_NEXT, 370, 676,
																FONT_NEXT, 326, 676,
																FONT_NEXT, 12, 231,
																FONT_NEXT, 12, 167,
																FONT_NEXT, 293, 167,
																FONT_NEXT, 293, 0,
																FONT_NEXT, 370, 0,
																FONT_NEXT, 370, 167,
																FONT_NEXT, 472, 167,
																FONT_END, 472, 231,
																FONT_BEGIN, 52, 231,
																FONT_NEXT, 290, 571,
																FONT_NEXT, 292, 571,
																FONT_END, 292, 231,
																FONT_ADVANCE, 500, 0
														},
														{
															53,
																FONT_BEGIN, 377, 583,
																FONT_NEXT, 391, 584,
																FONT_NEXT, 400, 592,
																FONT_NEXT, 438, 681,
																FONT_NEXT, 429, 688,
																FONT_NEXT, 411, 669,
																FONT_NEXT, 383, 662,
																FONT_NEXT, 174, 662,
																FONT_NEXT, 65, 425,
																FONT_NEXT, 64, 415,
																FONT_NEXT, 71, 412,
																FONT_NEXT, 161, 399,
																FONT_NEXT, 254, 365,
																FONT_NEXT, 294, 337,
																FONT_NEXT, 326, 300,
																FONT_NEXT, 348, 255,
																FONT_NEXT, 356, 201,
																FONT_NEXT, 352, 158,
																FONT_NEXT, 343, 121,
																FONT_NEXT, 310, 66,
																FONT_NEXT, 266, 33,
																FONT_NEXT, 217, 23,
																FONT_NEXT, 72, 85,
																FONT_NEXT, 48, 80,
																FONT_NEXT, 37, 69,
																FONT_NEXT, 32, 46,
																FONT_NEXT, 40, 21,
																FONT_NEXT, 65, 2,
																FONT_NEXT, 104, -10,
																FONT_NEXT, 158, -14,
																FONT_NEXT, 214, -10,
																FONT_NEXT, 266, 4,
																FONT_NEXT, 312, 26,
																FONT_NEXT, 351, 55,
																FONT_NEXT, 382, 90,
																FONT_NEXT, 406, 131,
																FONT_NEXT, 420, 176,
																FONT_NEXT, 426, 225,
																FONT_NEXT, 423, 281,
																FONT_NEXT, 415, 323,
																FONT_NEXT, 398, 359,
																FONT_NEXT, 372, 395,
																FONT_NEXT, 321, 439,
																FONT_NEXT, 263, 468,
																FONT_NEXT, 139, 498,
																FONT_END, 181, 583,
																FONT_ADVANCE, 500, 0
														},
															{
																54,
																	FONT_BEGIN, 358, 673,
																	FONT_NEXT, 279, 647,
																	FONT_NEXT, 209, 609,
																	FONT_NEXT, 149, 560,
																	FONT_NEXT, 100, 502,
																	FONT_NEXT, 64, 436,
																	FONT_NEXT, 41, 365,
																	FONT_NEXT, 34, 291,
																	FONT_NEXT, 38, 217,
																	FONT_NEXT, 51, 154,
																	FONT_NEXT, 72, 102,
																	FONT_NEXT, 99, 59,
																	FONT_NEXT, 132, 27,
																	FONT_NEXT, 169, 4,
																	FONT_NEXT, 210, -10,
																	FONT_NEXT, 254, -14,
																	FONT_NEXT, 314, -8,
																	FONT_NEXT, 362, 11,
																	FONT_NEXT, 399, 40,
																	FONT_NEXT, 427, 74,
																	FONT_NEXT, 459, 150,
																	FONT_NEXT, 468, 218,
																	FONT_NEXT, 464, 265,
																	FONT_NEXT, 454, 307,
																	FONT_NEXT, 437, 343,
																	FONT_NEXT, 415, 373,
																	FONT_NEXT, 357, 414,
																	FONT_NEXT, 284, 428,
																	FONT_NEXT, 247, 425,
																	FONT_NEXT, 217, 417,
																	FONT_NEXT, 187, 403,
																	FONT_NEXT, 152, 383,
																	FONT_NEXT, 181, 475,
																	FONT_NEXT, 207, 519,
																	FONT_NEXT, 240, 560,
																	FONT_NEXT, 281, 597,
																	FONT_NEXT, 329, 628,
																	FONT_NEXT, 384, 652,
																	FONT_NEXT, 448, 668,
																	FONT_END, 446, 684,
																	FONT_BEGIN, 278, 377,
																	FONT_NEXT, 307, 364,
																	FONT_NEXT, 331, 344,
																	FONT_NEXT, 349, 319,
																	FONT_NEXT, 371, 254,
																	FONT_NEXT, 378, 179,
																	FONT_NEXT, 369, 105,
																	FONT_NEXT, 347, 54,
																	FONT_NEXT, 312, 23,
																	FONT_NEXT, 269, 14,
																	FONT_NEXT, 215, 28,
																	FONT_NEXT, 191, 46,
																	FONT_NEXT, 169, 72,
																	FONT_NEXT, 152, 105,
																	FONT_NEXT, 138, 147,
																	FONT_NEXT, 130, 197,
																	FONT_NEXT, 127, 256,
																	FONT_NEXT, 128, 294,
																	FONT_NEXT, 133, 321,
																	FONT_NEXT, 147, 350,
																	FONT_NEXT, 191, 375,
																	FONT_END, 242, 382,
																	FONT_ADVANCE, 500, 0
															},
															{
																55,
																	FONT_BEGIN, 79, 662,
																	FONT_NEXT, 63, 618,
																	FONT_NEXT, 20, 515,
																	FONT_NEXT, 37, 507,
																	FONT_NEXT, 59, 541,
																	FONT_NEXT, 83, 567,
																	FONT_NEXT, 112, 582,
																	FONT_NEXT, 153, 588,
																	FONT_NEXT, 370, 588,
																	FONT_NEXT, 172, -8,
																	FONT_NEXT, 237, -8,
																	FONT_NEXT, 449, 646,
																	FONT_END, 449, 662,
																	FONT_ADVANCE, 500, 0
															},
																{
																	56,
																		FONT_BEGIN, 285, 258,
																		FONT_NEXT, 333, 216,
																		FONT_NEXT, 360, 174,
																		FONT_NEXT, 369, 126,
																		FONT_NEXT, 360, 78,
																		FONT_NEXT, 337, 43,
																		FONT_NEXT, 302, 21,
																		FONT_NEXT, 259, 14,
																		FONT_NEXT, 204, 25,
																		FONT_NEXT, 165, 56,
																		FONT_NEXT, 140, 101,
																		FONT_NEXT, 132, 156,
																		FONT_NEXT, 140, 215,
																		FONT_NEXT, 160, 258,
																		FONT_NEXT, 186, 289,
																		FONT_END, 212, 312,
																		FONT_BEGIN, 68, 88,
																		FONT_NEXT, 104, 36,
																		FONT_NEXT, 164, 0,
																		FONT_NEXT, 202, -11,
																		FONT_NEXT, 246, -14,
																		FONT_NEXT, 302, -10,
																		FONT_NEXT, 347, 4,
																		FONT_NEXT, 381, 24,
																		FONT_NEXT, 407, 48,
																		FONT_NEXT, 437, 102,
																		FONT_NEXT, 445, 150,
																		FONT_NEXT, 439, 199,
																		FONT_NEXT, 417, 249,
																		FONT_NEXT, 370, 305,
																		FONT_NEXT, 335, 336,
																		FONT_NEXT, 290, 371,
																		FONT_NEXT, 333, 395,
																		FONT_NEXT, 377, 428,
																		FONT_NEXT, 410, 473,
																		FONT_NEXT, 424, 534,
																		FONT_NEXT, 412, 588,
																		FONT_NEXT, 378, 633,
																		FONT_NEXT, 324, 664,
																		FONT_NEXT, 249, 676,
																		FONT_NEXT, 183, 665,
																		FONT_NEXT, 123, 636,
																		FONT_NEXT, 79, 587,
																		FONT_NEXT, 66, 556,
																		FONT_NEXT, 62, 521,
																		FONT_NEXT, 69, 467,
																		FONT_NEXT, 92, 423,
																		FONT_NEXT, 131, 381,
																		FONT_NEXT, 186, 332,
																		FONT_NEXT, 128, 286,
																		FONT_NEXT, 87, 246,
																		FONT_NEXT, 63, 202,
																		FONT_END, 56, 146,
																		FONT_BEGIN, 142, 583,
																		FONT_NEXT, 161, 615,
																		FONT_NEXT, 194, 638,
																		FONT_NEXT, 243, 648,
																		FONT_NEXT, 292, 638,
																		FONT_NEXT, 327, 614,
																		FONT_NEXT, 348, 579,
																		FONT_NEXT, 355, 539,
																		FONT_NEXT, 345, 481,
																		FONT_NEXT, 322, 440,
																		FONT_NEXT, 291, 410,
																		FONT_NEXT, 261, 389,
																		FONT_NEXT, 226, 412,
																		FONT_NEXT, 185, 447,
																		FONT_NEXT, 150, 492,
																		FONT_END, 136, 547,
																		FONT_ADVANCE, 500, 0
																},
																{
																	57,
																		FONT_BEGIN, 149, -9,
																		FONT_NEXT, 228, 20,
																		FONT_NEXT, 297, 61,
																		FONT_NEXT, 354, 113,
																		FONT_NEXT, 399, 174,
																		FONT_NEXT, 432, 243,
																		FONT_NEXT, 452, 317,
																		FONT_NEXT, 459, 396,
																		FONT_NEXT, 454, 455,
																		FONT_NEXT, 442, 510,
																		FONT_NEXT, 422, 557,
																		FONT_NEXT, 395, 598,
																		FONT_NEXT, 363, 631,
																		FONT_NEXT, 326, 655,
																		FONT_NEXT, 284, 670,
																		FONT_NEXT, 240, 676,
																		FONT_NEXT, 193, 671,
																		FONT_NEXT, 152, 657,
																		FONT_NEXT, 116, 634,
																		FONT_NEXT, 86, 605,
																		FONT_NEXT, 62, 569,
																		FONT_NEXT, 44, 528,
																		FONT_NEXT, 30, 435,
																		FONT_NEXT, 39, 369,
																		FONT_NEXT, 69, 305,
																		FONT_NEXT, 94, 278,
																		FONT_NEXT, 125, 256,
																		FONT_NEXT, 163, 242,
																		FONT_NEXT, 210, 237,
																		FONT_NEXT, 285, 250,
																		FONT_NEXT, 357, 292,
																		FONT_NEXT, 359, 290,
																		FONT_NEXT, 354, 271,
																		FONT_NEXT, 343, 239,
																		FONT_NEXT, 324, 197,
																		FONT_NEXT, 296, 149,
																		FONT_NEXT, 256, 101,
																		FONT_NEXT, 204, 56,
																		FONT_NEXT, 138, 21,
																		FONT_NEXT, 56, -2,
																		FONT_END, 59, -22,
																		FONT_BEGIN, 358, 332,
																		FONT_NEXT, 346, 315,
																		FONT_NEXT, 302, 291,
																		FONT_NEXT, 250, 280,
																		FONT_NEXT, 214, 285,
																		FONT_NEXT, 185, 299,
																		FONT_NEXT, 146, 348,
																		FONT_NEXT, 127, 410,
																		FONT_NEXT, 122, 473,
																		FONT_NEXT, 126, 534,
																		FONT_NEXT, 144, 591,
																		FONT_NEXT, 176, 632,
																		FONT_NEXT, 200, 643,
																		FONT_NEXT, 229, 648,
																		FONT_NEXT, 272, 639,
																		FONT_NEXT, 305, 616,
																		FONT_NEXT, 329, 583,
																		FONT_NEXT, 345, 543,
																		FONT_NEXT, 359, 458,
																		FONT_NEXT, 361, 421,
																		FONT_NEXT, 362, 394,
																		FONT_END, 362, 355,
																		FONT_ADVANCE, 500, 0
																},
																	{
																		58,
																			FONT_BEGIN, 171, 361,
																			FONT_NEXT, 186, 377,
																			FONT_NEXT, 192, 402,
																			FONT_NEXT, 187, 423,
																			FONT_NEXT, 175, 442,
																			FONT_NEXT, 157, 454,
																			FONT_NEXT, 136, 459,
																			FONT_NEXT, 114, 454,
																			FONT_NEXT, 97, 442,
																			FONT_NEXT, 85, 424,
																			FONT_NEXT, 81, 402,
																			FONT_NEXT, 86, 377,
																			FONT_NEXT, 100, 360,
																			FONT_END, 136, 348,
																			FONT_BEGIN, 171, 2,
																			FONT_NEXT, 186, 18,
																			FONT_NEXT, 192, 43,
																			FONT_NEXT, 187, 64,
																			FONT_NEXT, 175, 83,
																			FONT_NEXT, 157, 95,
																			FONT_NEXT, 136, 100,
																			FONT_NEXT, 114, 95,
																			FONT_NEXT, 97, 83,
																			FONT_NEXT, 85, 65,
																			FONT_NEXT, 81, 43,
																			FONT_NEXT, 86, 18,
																			FONT_NEXT, 100, 1,
																			FONT_END, 136, -11,
																			FONT_ADVANCE, 278, 0
																	},
																	{
																		59,
																			FONT_BEGIN, 138, -123,
																			FONT_NEXT, 175, -90,
																			FONT_NEXT, 206, -45,
																			FONT_NEXT, 219, 6,
																			FONT_NEXT, 209, 56,
																			FONT_NEXT, 186, 85,
																			FONT_NEXT, 160, 98,
																			FONT_NEXT, 139, 102,
																			FONT_NEXT, 125, 100,
																			FONT_NEXT, 105, 93,
																			FONT_NEXT, 87, 76,
																			FONT_NEXT, 80, 44,
																			FONT_NEXT, 85, 20,
																			FONT_NEXT, 99, 5,
																			FONT_NEXT, 142, -6,
																			FONT_NEXT, 157, -4,
																			FONT_NEXT, 166, -2,
																			FONT_NEXT, 175, -6,
																			FONT_NEXT, 180, -16,
																			FONT_NEXT, 176, -35,
																			FONT_NEXT, 163, -61,
																			FONT_NEXT, 138, -91,
																			FONT_NEXT, 98, -122,
																			FONT_END, 107, -141,
																			FONT_BEGIN, 171, 361,
																			FONT_NEXT, 186, 377,
																			FONT_NEXT, 192, 402,
																			FONT_NEXT, 187, 423,
																			FONT_NEXT, 175, 442,
																			FONT_NEXT, 157, 454,
																			FONT_NEXT, 136, 459,
																			FONT_NEXT, 114, 454,
																			FONT_NEXT, 97, 442,
																			FONT_NEXT, 85, 424,
																			FONT_NEXT, 81, 402,
																			FONT_NEXT, 86, 377,
																			FONT_NEXT, 100, 360,
																			FONT_END, 136, 348,
																			FONT_ADVANCE, 278, 0
																	},
																		{
																			60,
																				FONT_BEGIN, 111, 253,
																				FONT_NEXT, 536, 446,
																				FONT_NEXT, 536, 514,
																				FONT_NEXT, 28, 284,
																				FONT_NEXT, 28, 222,
																				FONT_NEXT, 536, -8,
																				FONT_END, 536, 60,
																				FONT_ADVANCE, 564, 0
																		},
																		{
																			61,
																				FONT_BEGIN, 30, 386,
																				FONT_NEXT, 30, 320,
																				FONT_NEXT, 534, 320,
																				FONT_END, 534, 386,
																				FONT_BEGIN, 30, 186,
																				FONT_NEXT, 30, 120,
																				FONT_NEXT, 534, 120,
																				FONT_END, 534, 186,
																				FONT_ADVANCE, 564, 0
																		},
																			{
																				62,
																					FONT_BEGIN, 28, -8,
																					FONT_NEXT, 536, 222,
																					FONT_NEXT, 536, 284,
																					FONT_NEXT, 28, 514,
																					FONT_NEXT, 28, 446,
																					FONT_NEXT, 453, 253,
																					FONT_END, 28, 60,
																					FONT_ADVANCE, 564, 0
																			},
																			{
																				63,
																					FONT_BEGIN, 257, 220,
																					FONT_NEXT, 276, 267,
																					FONT_NEXT, 302, 309,
																					FONT_NEXT, 339, 354,
																					FONT_NEXT, 363, 385,
																					FONT_NEXT, 387, 426,
																					FONT_NEXT, 406, 469,
																					FONT_NEXT, 414, 510,
																					FONT_NEXT, 412, 544,
																					FONT_NEXT, 406, 571,
																					FONT_NEXT, 378, 618,
																					FONT_NEXT, 352, 639,
																					FONT_NEXT, 316, 657,
																					FONT_NEXT, 274, 671,
																					FONT_NEXT, 231, 676,
																					FONT_NEXT, 176, 667,
																					FONT_NEXT, 123, 641,
																					FONT_NEXT, 83, 597,
																					FONT_NEXT, 72, 567,
																					FONT_NEXT, 68, 532,
																					FONT_NEXT, 78, 487,
																					FONT_NEXT, 93, 474,
																					FONT_NEXT, 118, 469,
																					FONT_NEXT, 135, 473,
																					FONT_NEXT, 148, 483,
																					FONT_NEXT, 157, 510,
																					FONT_NEXT, 119, 585,
																					FONT_NEXT, 127, 608,
																					FONT_NEXT, 149, 627,
																					FONT_NEXT, 179, 641,
																					FONT_NEXT, 212, 646,
																					FONT_NEXT, 261, 634,
																					FONT_NEXT, 295, 605,
																					FONT_NEXT, 315, 564,
																					FONT_NEXT, 322, 519,
																					FONT_NEXT, 310, 444,
																					FONT_NEXT, 285, 370,
																					FONT_NEXT, 257, 306,
																					FONT_NEXT, 249, 286,
																					FONT_NEXT, 239, 251,
																					FONT_NEXT, 227, 164,
																					FONT_END, 244, 164,
																					FONT_BEGIN, 273, 4,
																					FONT_NEXT, 286, 19,
																					FONT_NEXT, 292, 43,
																					FONT_NEXT, 287, 63,
																					FONT_NEXT, 276, 80,
																					FONT_NEXT, 258, 92,
																					FONT_NEXT, 237, 97,
																					FONT_NEXT, 216, 92,
																					FONT_NEXT, 199, 81,
																					FONT_NEXT, 188, 64,
																					FONT_NEXT, 184, 43,
																					FONT_NEXT, 189, 19,
																					FONT_NEXT, 202, 3,
																					FONT_END, 237, -8,
																					FONT_ADVANCE, 444, 0
																			},
																				{
																					64,
																						FONT_BEGIN, 554, 306,
																						FONT_NEXT, 524, 240,
																						FONT_NEXT, 489, 200,
																						FONT_NEXT, 455, 187,
																						FONT_NEXT, 430, 191,
																						FONT_NEXT, 410, 206,
																						FONT_NEXT, 396, 232,
																						FONT_NEXT, 392, 271,
																						FONT_NEXT, 395, 314,
																						FONT_NEXT, 406, 352,
																						FONT_NEXT, 442, 412,
																						FONT_NEXT, 487, 450,
																						FONT_NEXT, 529, 464,
																						FONT_NEXT, 548, 459,
																						FONT_NEXT, 561, 447,
																						FONT_NEXT, 569, 426,
																						FONT_END, 572, 399,
																						FONT_BEGIN, 601, 39,
																						FONT_NEXT, 550, 28,
																						FONT_NEXT, 490, 25,
																						FONT_NEXT, 427, 31,
																						FONT_NEXT, 371, 48,
																						FONT_NEXT, 321, 75,
																						FONT_NEXT, 279, 111,
																						FONT_NEXT, 245, 153,
																						FONT_NEXT, 220, 200,
																						FONT_NEXT, 205, 251,
																						FONT_NEXT, 200, 305,
																						FONT_NEXT, 205, 383,
																						FONT_NEXT, 222, 451,
																						FONT_NEXT, 248, 510,
																						FONT_NEXT, 283, 558,
																						FONT_NEXT, 324, 595,
																						FONT_NEXT, 372, 623,
																						FONT_NEXT, 424, 639,
																						FONT_NEXT, 481, 645,
																						FONT_NEXT, 545, 638,
																						FONT_NEXT, 603, 619,
																						FONT_NEXT, 652, 591,
																						FONT_NEXT, 693, 555,
																						FONT_NEXT, 726, 513,
																						FONT_NEXT, 749, 469,
																						FONT_NEXT, 764, 424,
																						FONT_NEXT, 769, 380,
																						FONT_NEXT, 760, 311,
																						FONT_NEXT, 734, 245,
																						FONT_NEXT, 693, 195,
																						FONT_NEXT, 667, 181,
																						FONT_NEXT, 639, 176,
																						FONT_NEXT, 613, 183,
																						FONT_NEXT, 603, 202,
																						FONT_NEXT, 603, 240,
																						FONT_NEXT, 668, 494,
																						FONT_NEXT, 599, 494,
																						FONT_NEXT, 589, 456,
																						FONT_NEXT, 565, 491,
																						FONT_NEXT, 546, 503,
																						FONT_NEXT, 518, 508,
																						FONT_NEXT, 470, 500,
																						FONT_NEXT, 430, 480,
																						FONT_NEXT, 396, 450,
																						FONT_NEXT, 368, 414,
																						FONT_NEXT, 332, 335,
																						FONT_NEXT, 323, 298,
																						FONT_NEXT, 321, 268,
																						FONT_NEXT, 323, 230,
																						FONT_NEXT, 332, 200,
																						FONT_NEXT, 358, 163,
																						FONT_NEXT, 390, 147,
																						FONT_NEXT, 417, 144,
																						FONT_NEXT, 457, 154,
																						FONT_NEXT, 492, 174,
																						FONT_NEXT, 517, 196,
																						FONT_NEXT, 532, 214,
																						FONT_NEXT, 534, 214,
																						FONT_NEXT, 538, 197,
																						FONT_NEXT, 553, 173,
																						FONT_NEXT, 581, 152,
																						FONT_NEXT, 623, 143,
																						FONT_NEXT, 665, 149,
																						FONT_NEXT, 702, 165,
																						FONT_NEXT, 734, 191,
																						FONT_NEXT, 760, 223,
																						FONT_NEXT, 796, 300,
																						FONT_NEXT, 809, 381,
																						FONT_NEXT, 802, 442,
																						FONT_NEXT, 783, 498,
																						FONT_NEXT, 753, 548,
																						FONT_NEXT, 714, 591,
																						FONT_NEXT, 665, 627,
																						FONT_NEXT, 609, 653,
																						FONT_NEXT, 547, 670,
																						FONT_NEXT, 481, 676,
																						FONT_NEXT, 408, 669,
																						FONT_NEXT, 340, 649,
																						FONT_NEXT, 278, 618,
																						FONT_NEXT, 224, 576,
																						FONT_NEXT, 179, 524,
																						FONT_NEXT, 145, 462,
																						FONT_NEXT, 123, 392,
																						FONT_NEXT, 116, 315,
																						FONT_NEXT, 121, 262,
																						FONT_NEXT, 137, 207,
																						FONT_NEXT, 165, 152,
																						FONT_NEXT, 205, 100,
																						FONT_NEXT, 257, 54,
																						FONT_NEXT, 322, 18,
																						FONT_NEXT, 399, -6,
																						FONT_NEXT, 491, -14,
																						FONT_NEXT, 547, -10,
																						FONT_NEXT, 601, 3,
																						FONT_NEXT, 700, 43,
																						FONT_END, 688, 73,
																						FONT_ADVANCE, 921, 0
																				},
																				{
																					65,
																						FONT_BEGIN, 677, 23,
																						FONT_NEXT, 656, 36,
																						FONT_NEXT, 637, 62,
																						FONT_NEXT, 616, 106,
																						FONT_NEXT, 367, 674,
																						FONT_NEXT, 347, 674,
																						FONT_NEXT, 139, 183,
																						FONT_NEXT, 104, 104,
																						FONT_NEXT, 84, 67,
																						FONT_NEXT, 66, 41,
																						FONT_NEXT, 39, 23,
																						FONT_NEXT, 15, 19,
																						FONT_NEXT, 15, 0,
																						FONT_NEXT, 213, 0,
																						FONT_NEXT, 213, 19,
																						FONT_NEXT, 171, 24,
																						FONT_NEXT, 152, 36,
																						FONT_NEXT, 145, 61,
																						FONT_NEXT, 153, 99,
																						FONT_NEXT, 199, 216,
																						FONT_NEXT, 461, 216,
																						FONT_NEXT, 502, 120,
																						FONT_NEXT, 521, 57,
																						FONT_NEXT, 515, 35,
																						FONT_NEXT, 499, 24,
																						FONT_NEXT, 451, 19,
																						FONT_NEXT, 451, 0,
																						FONT_NEXT, 706, 0,
																						FONT_END, 706, 19,
																						FONT_BEGIN, 331, 532,
																						FONT_NEXT, 447, 257,
																						FONT_END, 216, 257,
																						FONT_ADVANCE, 722, 0
																				},
																					{
																						66,
																							FONT_BEGIN, 66, 637,
																							FONT_NEXT, 95, 624,
																							FONT_NEXT, 109, 597,
																							FONT_NEXT, 113, 553,
																							FONT_NEXT, 113, 109,
																							FONT_NEXT, 109, 65,
																							FONT_NEXT, 96, 38,
																							FONT_NEXT, 67, 23,
																							FONT_NEXT, 17, 19,
																							FONT_NEXT, 17, 0,
																							FONT_NEXT, 351, 0,
																							FONT_NEXT, 425, 6,
																							FONT_NEXT, 482, 22,
																							FONT_NEXT, 525, 45,
																							FONT_NEXT, 555, 73,
																							FONT_NEXT, 574, 103,
																							FONT_NEXT, 586, 132,
																							FONT_NEXT, 593, 176,
																							FONT_NEXT, 589, 212,
																							FONT_NEXT, 579, 244,
																							FONT_NEXT, 542, 294,
																							FONT_NEXT, 488, 327,
																							FONT_NEXT, 426, 347,
																							FONT_NEXT, 426, 349,
																							FONT_NEXT, 465, 360,
																							FONT_NEXT, 509, 383,
																							FONT_NEXT, 544, 425,
																							FONT_NEXT, 555, 455,
																							FONT_NEXT, 559, 493,
																							FONT_NEXT, 551, 544,
																							FONT_NEXT, 530, 584,
																							FONT_NEXT, 499, 614,
																							FONT_NEXT, 460, 635,
																							FONT_NEXT, 374, 657,
																							FONT_NEXT, 297, 662,
																							FONT_NEXT, 17, 662,
																							FONT_END, 17, 643,
																							FONT_BEGIN, 276, 325,
																							FONT_NEXT, 319, 322,
																							FONT_NEXT, 377, 311,
																							FONT_NEXT, 414, 295,
																							FONT_NEXT, 447, 270,
																							FONT_NEXT, 469, 232,
																							FONT_NEXT, 478, 180,
																							FONT_NEXT, 472, 135,
																							FONT_NEXT, 458, 101,
																							FONT_NEXT, 436, 75,
																							FONT_NEXT, 409, 58,
																							FONT_NEXT, 348, 40,
																							FONT_NEXT, 291, 37,
																							FONT_NEXT, 254, 38,
																							FONT_NEXT, 231, 43,
																							FONT_NEXT, 218, 55,
																							FONT_NEXT, 215, 78,
																							FONT_END, 215, 326,
																							FONT_BEGIN, 218, 615,
																							FONT_NEXT, 237, 624,
																							FONT_NEXT, 255, 624,
																							FONT_NEXT, 282, 625,
																							FONT_NEXT, 339, 620,
																							FONT_NEXT, 396, 600,
																							FONT_NEXT, 420, 582,
																							FONT_NEXT, 439, 558,
																							FONT_NEXT, 452, 527,
																							FONT_NEXT, 457, 487,
																							FONT_NEXT, 451, 447,
																							FONT_NEXT, 437, 417,
																							FONT_NEXT, 394, 381,
																							FONT_NEXT, 344, 368,
																							FONT_NEXT, 310, 366,
																							FONT_NEXT, 215, 365,
																							FONT_END, 215, 595,
																							FONT_ADVANCE, 667, 0
																					},
																					{
																						67,
																							FONT_BEGIN, 611, 676,
																							FONT_NEXT, 590, 676,
																							FONT_NEXT, 580, 656,
																							FONT_NEXT, 565, 647,
																							FONT_NEXT, 544, 643,
																							FONT_NEXT, 519, 648,
																							FONT_NEXT, 481, 659,
																							FONT_NEXT, 429, 670,
																							FONT_NEXT, 368, 676,
																							FONT_NEXT, 304, 670,
																							FONT_NEXT, 242, 653,
																							FONT_NEXT, 184, 625,
																							FONT_NEXT, 133, 587,
																							FONT_NEXT, 90, 538,
																							FONT_NEXT, 56, 479,
																							FONT_NEXT, 35, 410,
																							FONT_NEXT, 28, 331,
																							FONT_NEXT, 35, 244,
																							FONT_NEXT, 57, 172,
																							FONT_NEXT, 91, 112,
																							FONT_NEXT, 134, 65,
																							FONT_NEXT, 185, 29,
																							FONT_NEXT, 241, 4,
																							FONT_NEXT, 301, -10,
																							FONT_NEXT, 362, -14,
																							FONT_NEXT, 426, -9,
																							FONT_NEXT, 481, 5,
																							FONT_NEXT, 528, 25,
																							FONT_NEXT, 566, 48,
																							FONT_NEXT, 595, 71,
																							FONT_NEXT, 616, 92,
																							FONT_NEXT, 628, 106,
																							FONT_NEXT, 633, 113,
																							FONT_NEXT, 615, 131,
																							FONT_NEXT, 583, 102,
																							FONT_NEXT, 536, 69,
																							FONT_NEXT, 471, 41,
																							FONT_NEXT, 389, 30,
																							FONT_NEXT, 342, 34,
																							FONT_NEXT, 297, 46,
																							FONT_NEXT, 256, 68,
																							FONT_NEXT, 219, 100,
																							FONT_NEXT, 188, 141,
																							FONT_NEXT, 164, 192,
																							FONT_NEXT, 149, 255,
																							FONT_NEXT, 144, 329,
																							FONT_NEXT, 151, 422,
																							FONT_NEXT, 170, 491,
																							FONT_NEXT, 197, 540,
																							FONT_NEXT, 227, 575,
																							FONT_NEXT, 261, 601,
																							FONT_NEXT, 299, 620,
																							FONT_NEXT, 377, 636,
																							FONT_NEXT, 420, 632,
																							FONT_NEXT, 459, 621,
																							FONT_NEXT, 523, 582,
																							FONT_NEXT, 569, 524,
																							FONT_NEXT, 597, 451,
																							FONT_END, 620, 451,
																							FONT_ADVANCE, 667, 0
																					},
																						{
																							68,
																								FONT_BEGIN, 300, 0,
																								FONT_NEXT, 404, 8,
																								FONT_NEXT, 489, 32,
																								FONT_NEXT, 556, 69,
																								FONT_NEXT, 607, 114,
																								FONT_NEXT, 643, 166,
																								FONT_NEXT, 667, 221,
																								FONT_NEXT, 681, 276,
																								FONT_NEXT, 685, 329,
																								FONT_NEXT, 677, 407,
																								FONT_NEXT, 656, 475,
																								FONT_NEXT, 622, 532,
																								FONT_NEXT, 575, 579,
																								FONT_NEXT, 517, 615,
																								FONT_NEXT, 449, 641,
																								FONT_NEXT, 372, 656,
																								FONT_NEXT, 286, 662,
																								FONT_NEXT, 16, 662,
																								FONT_NEXT, 16, 643,
																								FONT_NEXT, 62, 636,
																								FONT_NEXT, 88, 623,
																								FONT_NEXT, 101, 597,
																								FONT_NEXT, 104, 553,
																								FONT_NEXT, 104, 109,
																								FONT_NEXT, 100, 65,
																								FONT_NEXT, 88, 39,
																								FONT_NEXT, 61, 24,
																								FONT_NEXT, 16, 19,
																								FONT_END, 16, 0,
																								FONT_BEGIN, 211, 613,
																								FONT_NEXT, 225, 621,
																								FONT_NEXT, 253, 625,
																								FONT_NEXT, 335, 620,
																								FONT_NEXT, 397, 606,
																								FONT_NEXT, 444, 584,
																								FONT_NEXT, 483, 555,
																								FONT_NEXT, 526, 505,
																								FONT_NEXT, 555, 449,
																								FONT_NEXT, 571, 390,
																								FONT_NEXT, 576, 328,
																								FONT_NEXT, 573, 276,
																								FONT_NEXT, 565, 231,
																								FONT_NEXT, 536, 159,
																								FONT_NEXT, 494, 108,
																								FONT_NEXT, 445, 73,
																								FONT_NEXT, 392, 52,
																								FONT_NEXT, 341, 41,
																								FONT_NEXT, 297, 37,
																								FONT_NEXT, 266, 37,
																								FONT_NEXT, 236, 38,
																								FONT_NEXT, 218, 44,
																								FONT_NEXT, 208, 57,
																								FONT_NEXT, 206, 78,
																								FONT_END, 206, 586,
																								FONT_ADVANCE, 722, 0
																						},
																						{
																							69,
																								FONT_BEGIN, 569, 169,
																								FONT_NEXT, 533, 105,
																								FONT_NEXT, 489, 65,
																								FONT_NEXT, 430, 44,
																								FONT_NEXT, 350, 38,
																								FONT_NEXT, 300, 38,
																								FONT_NEXT, 263, 38,
																								FONT_NEXT, 237, 40,
																								FONT_NEXT, 219, 43,
																								FONT_NEXT, 203, 56,
																								FONT_NEXT, 201, 80,
																								FONT_NEXT, 201, 328,
																								FONT_NEXT, 355, 326,
																								FONT_NEXT, 405, 321,
																								FONT_NEXT, 436, 308,
																								FONT_NEXT, 454, 279,
																								FONT_NEXT, 465, 231,
																								FONT_NEXT, 488, 231,
																								FONT_NEXT, 488, 463,
																								FONT_NEXT, 465, 463,
																								FONT_NEXT, 455, 418,
																								FONT_NEXT, 439, 388,
																								FONT_NEXT, 408, 372,
																								FONT_NEXT, 355, 368,
																								FONT_NEXT, 201, 368,
																								FONT_NEXT, 201, 590,
																								FONT_NEXT, 203, 609,
																								FONT_NEXT, 209, 619,
																								FONT_NEXT, 234, 624,
																								FONT_NEXT, 369, 624,
																								FONT_NEXT, 437, 620,
																								FONT_NEXT, 481, 604,
																								FONT_NEXT, 506, 572,
																								FONT_NEXT, 521, 519,
																								FONT_NEXT, 546, 519,
																								FONT_NEXT, 543, 662,
																								FONT_NEXT, 12, 662,
																								FONT_NEXT, 12, 643,
																								FONT_NEXT, 52, 637,
																								FONT_NEXT, 79, 625,
																								FONT_NEXT, 94, 599,
																								FONT_NEXT, 99, 553,
																								FONT_NEXT, 99, 109,
																								FONT_NEXT, 94, 63,
																								FONT_NEXT, 79, 36,
																								FONT_NEXT, 52, 23,
																								FONT_NEXT, 12, 19,
																								FONT_NEXT, 12, 0,
																								FONT_NEXT, 552, 0,
																								FONT_END, 597, 169,
																								FONT_ADVANCE, 611, 0
																						},
																							{
																								70,
																									FONT_BEGIN, 479, 463,
																									FONT_NEXT, 456, 463,
																									FONT_NEXT, 446, 416,
																									FONT_NEXT, 428, 387,
																									FONT_NEXT, 396, 372,
																									FONT_NEXT, 346, 368,
																									FONT_NEXT, 201, 368,
																									FONT_NEXT, 201, 590,
																									FONT_NEXT, 202, 607,
																									FONT_NEXT, 207, 618,
																									FONT_NEXT, 233, 624,
																									FONT_NEXT, 369, 624,
																									FONT_NEXT, 437, 620,
																									FONT_NEXT, 481, 604,
																									FONT_NEXT, 506, 572,
																									FONT_NEXT, 521, 519,
																									FONT_NEXT, 546, 519,
																									FONT_NEXT, 543, 662,
																									FONT_NEXT, 12, 662,
																									FONT_NEXT, 12, 643,
																									FONT_NEXT, 52, 637,
																									FONT_NEXT, 79, 625,
																									FONT_NEXT, 94, 599,
																									FONT_NEXT, 99, 553,
																									FONT_NEXT, 99, 120,
																									FONT_NEXT, 95, 70,
																									FONT_NEXT, 83, 40,
																									FONT_NEXT, 57, 24,
																									FONT_NEXT, 12, 19,
																									FONT_NEXT, 12, 0,
																									FONT_NEXT, 292, 0,
																									FONT_NEXT, 292, 19,
																									FONT_NEXT, 247, 23,
																									FONT_NEXT, 219, 37,
																									FONT_NEXT, 205, 63,
																									FONT_NEXT, 201, 109,
																									FONT_NEXT, 201, 328,
																									FONT_NEXT, 346, 326,
																									FONT_NEXT, 393, 322,
																									FONT_NEXT, 425, 309,
																									FONT_NEXT, 444, 281,
																									FONT_NEXT, 456, 231,
																									FONT_END, 479, 231,
																									FONT_ADVANCE, 556, 0
																							},
																							{
																								71,
																									FONT_BEGIN, 454, 354,
																									FONT_NEXT, 454, 336,
																									FONT_NEXT, 496, 331,
																									FONT_NEXT, 523, 320,
																									FONT_NEXT, 537, 295,
																									FONT_NEXT, 542, 247,
																									FONT_NEXT, 542, 85,
																									FONT_NEXT, 530, 60,
																									FONT_NEXT, 499, 42,
																									FONT_NEXT, 455, 30,
																									FONT_NEXT, 405, 26,
																									FONT_NEXT, 349, 31,
																									FONT_NEXT, 299, 46,
																									FONT_NEXT, 254, 71,
																									FONT_NEXT, 217, 105,
																									FONT_NEXT, 187, 147,
																									FONT_NEXT, 164, 197,
																									FONT_NEXT, 150, 255,
																									FONT_NEXT, 146, 320,
																									FONT_NEXT, 152, 406,
																									FONT_NEXT, 170, 476,
																									FONT_NEXT, 197, 531,
																									FONT_NEXT, 231, 572,
																									FONT_NEXT, 269, 602,
																									FONT_NEXT, 310, 622,
																									FONT_NEXT, 388, 636,
																									FONT_NEXT, 436, 631,
																									FONT_NEXT, 477, 619,
																									FONT_NEXT, 539, 577,
																									FONT_NEXT, 581, 522,
																									FONT_NEXT, 607, 465,
																									FONT_NEXT, 630, 465,
																									FONT_NEXT, 622, 676,
																									FONT_NEXT, 600, 676,
																									FONT_NEXT, 591, 659,
																									FONT_NEXT, 577, 649,
																									FONT_NEXT, 553, 643,
																									FONT_NEXT, 525, 648,
																									FONT_NEXT, 489, 659,
																									FONT_NEXT, 440, 670,
																									FONT_NEXT, 374, 676,
																									FONT_NEXT, 298, 668,
																									FONT_NEXT, 231, 647,
																									FONT_NEXT, 172, 614,
																									FONT_NEXT, 123, 571,
																									FONT_NEXT, 84, 520,
																									FONT_NEXT, 55, 464,
																									FONT_NEXT, 38, 403,
																									FONT_NEXT, 32, 341,
																									FONT_NEXT, 37, 255,
																									FONT_NEXT, 53, 189,
																									FONT_NEXT, 79, 137,
																									FONT_NEXT, 115, 93,
																									FONT_NEXT, 174, 43,
																									FONT_NEXT, 245, 10,
																									FONT_NEXT, 320, -9,
																									FONT_NEXT, 396, -14,
																									FONT_NEXT, 480, -5,
																									FONT_NEXT, 558, 16,
																									FONT_NEXT, 616, 40,
																									FONT_NEXT, 633, 50,
																									FONT_NEXT, 639, 58,
																									FONT_NEXT, 639, 259,
																									FONT_NEXT, 643, 300,
																									FONT_NEXT, 657, 322,
																									FONT_NEXT, 679, 332,
																									FONT_NEXT, 709, 336,
																									FONT_END, 709, 354,
																									FONT_ADVANCE, 722, 0
																							},
																								{
																									72,
																										FONT_BEGIN, 297, 0,
																										FONT_NEXT, 297, 19,
																										FONT_NEXT, 250, 25,
																										FONT_NEXT, 223, 41,
																										FONT_NEXT, 211, 67,
																										FONT_NEXT, 209, 109,
																										FONT_NEXT, 209, 315,
																										FONT_NEXT, 512, 315,
																										FONT_NEXT, 512, 120,
																										FONT_NEXT, 510, 73,
																										FONT_NEXT, 499, 42,
																										FONT_NEXT, 473, 25,
																										FONT_NEXT, 424, 19,
																										FONT_NEXT, 424, 0,
																										FONT_NEXT, 702, 0,
																										FONT_NEXT, 702, 19,
																										FONT_NEXT, 655, 25,
																										FONT_NEXT, 628, 41,
																										FONT_NEXT, 616, 67,
																										FONT_NEXT, 614, 109,
																										FONT_NEXT, 614, 553,
																										FONT_NEXT, 617, 596,
																										FONT_NEXT, 629, 622,
																										FONT_NEXT, 656, 636,
																										FONT_NEXT, 702, 643,
																										FONT_NEXT, 702, 662,
																										FONT_NEXT, 424, 662,
																										FONT_NEXT, 424, 643,
																										FONT_NEXT, 468, 636,
																										FONT_NEXT, 495, 623,
																										FONT_NEXT, 508, 597,
																										FONT_NEXT, 512, 553,
																										FONT_NEXT, 512, 359,
																										FONT_NEXT, 209, 359,
																										FONT_NEXT, 209, 553,
																										FONT_NEXT, 212, 596,
																										FONT_NEXT, 224, 622,
																										FONT_NEXT, 251, 636,
																										FONT_NEXT, 297, 643,
																										FONT_NEXT, 297, 662,
																										FONT_NEXT, 19, 662,
																										FONT_NEXT, 19, 643,
																										FONT_NEXT, 63, 636,
																										FONT_NEXT, 90, 623,
																										FONT_NEXT, 103, 597,
																										FONT_NEXT, 107, 553,
																										FONT_NEXT, 107, 120,
																										FONT_NEXT, 105, 73,
																										FONT_NEXT, 94, 42,
																										FONT_NEXT, 68, 25,
																										FONT_NEXT, 19, 19,
																										FONT_END, 19, 0,
																										FONT_ADVANCE, 722, 0
																								},
																								{
																									73,
																										FONT_BEGIN, 315, 0,
																										FONT_NEXT, 315, 19,
																										FONT_NEXT, 268, 22,
																										FONT_NEXT, 238, 35,
																										FONT_NEXT, 221, 62,
																										FONT_NEXT, 217, 109,
																										FONT_NEXT, 217, 553,
																										FONT_NEXT, 222, 600,
																										FONT_NEXT, 238, 626,
																										FONT_NEXT, 268, 638,
																										FONT_NEXT, 315, 643,
																										FONT_NEXT, 315, 662,
																										FONT_NEXT, 18, 662,
																										FONT_NEXT, 18, 643,
																										FONT_NEXT, 68, 637,
																										FONT_NEXT, 97, 624,
																										FONT_NEXT, 111, 597,
																										FONT_NEXT, 115, 553,
																										FONT_NEXT, 115, 109,
																										FONT_NEXT, 111, 65,
																										FONT_NEXT, 98, 38,
																										FONT_NEXT, 69, 23,
																										FONT_NEXT, 18, 19,
																										FONT_END, 18, 0,
																										FONT_ADVANCE, 333, 0
																								},
																									{
																										74,
																											FONT_BEGIN, 281, 597,
																											FONT_NEXT, 294, 623,
																											FONT_NEXT, 322, 637,
																											FONT_NEXT, 370, 643,
																											FONT_NEXT, 370, 662,
																											FONT_NEXT, 83, 662,
																											FONT_NEXT, 83, 643,
																											FONT_NEXT, 131, 636,
																											FONT_NEXT, 159, 622,
																											FONT_NEXT, 172, 596,
																											FONT_NEXT, 176, 553,
																											FONT_NEXT, 176, 90,
																											FONT_NEXT, 173, 57,
																											FONT_NEXT, 165, 37,
																											FONT_NEXT, 153, 27,
																											FONT_NEXT, 137, 24,
																											FONT_NEXT, 117, 37,
																											FONT_NEXT, 107, 66,
																											FONT_NEXT, 92, 94,
																											FONT_NEXT, 77, 104,
																											FONT_NEXT, 56, 108,
																											FONT_NEXT, 36, 103,
																											FONT_NEXT, 22, 90,
																											FONT_NEXT, 10, 59,
																											FONT_NEXT, 15, 36,
																											FONT_NEXT, 32, 11,
																											FONT_NEXT, 59, -6,
																											FONT_NEXT, 110, -14,
																											FONT_NEXT, 124, -14,
																											FONT_NEXT, 147, -11,
																											FONT_NEXT, 204, 11,
																											FONT_NEXT, 231, 35,
																											FONT_NEXT, 255, 70,
																											FONT_NEXT, 271, 118,
																											FONT_NEXT, 278, 183,
																											FONT_END, 278, 553,
																											FONT_ADVANCE, 389, 0
																									},
																									{
																										75,
																											FONT_BEGIN, 444, 641,
																											FONT_NEXT, 465, 636,
																											FONT_NEXT, 477, 627,
																											FONT_NEXT, 481, 612,
																											FONT_NEXT, 472, 584,
																											FONT_NEXT, 459, 567,
																											FONT_NEXT, 438, 543,
																											FONT_NEXT, 406, 511,
																											FONT_NEXT, 361, 470,
																											FONT_NEXT, 301, 416,
																											FONT_NEXT, 226, 348,
																											FONT_NEXT, 226, 553,
																											FONT_NEXT, 229, 597,
																											FONT_NEXT, 242, 623,
																											FONT_NEXT, 270, 637,
																											FONT_NEXT, 318, 643,
																											FONT_NEXT, 318, 662,
																											FONT_NEXT, 34, 662,
																											FONT_NEXT, 34, 643,
																											FONT_NEXT, 76, 637,
																											FONT_NEXT, 104, 625,
																											FONT_NEXT, 119, 599,
																											FONT_NEXT, 124, 553,
																											FONT_NEXT, 124, 120,
																											FONT_NEXT, 120, 70,
																											FONT_NEXT, 108, 40,
																											FONT_NEXT, 80, 24,
																											FONT_NEXT, 34, 19,
																											FONT_NEXT, 34, 0,
																											FONT_NEXT, 316, 0,
																											FONT_NEXT, 316, 19,
																											FONT_NEXT, 269, 24,
																											FONT_NEXT, 241, 39,
																											FONT_NEXT, 229, 65,
																											FONT_NEXT, 226, 109,
																											FONT_NEXT, 226, 296,
																											FONT_NEXT, 252, 317,
																											FONT_NEXT, 358, 212,
																											FONT_NEXT, 392, 175,
																											FONT_NEXT, 435, 126,
																											FONT_NEXT, 472, 78,
																											FONT_NEXT, 483, 58,
																											FONT_NEXT, 488, 44,
																											FONT_NEXT, 484, 31,
																											FONT_NEXT, 472, 24,
																											FONT_NEXT, 451, 20,
																											FONT_NEXT, 418, 19,
																											FONT_NEXT, 418, 0,
																											FONT_NEXT, 723, 0,
																											FONT_NEXT, 723, 19,
																											FONT_NEXT, 683, 24,
																											FONT_NEXT, 647, 45,
																											FONT_NEXT, 609, 79,
																											FONT_NEXT, 566, 127,
																											FONT_NEXT, 333, 377,
																											FONT_NEXT, 523, 565,
																											FONT_NEXT, 568, 606,
																											FONT_NEXT, 603, 628,
																											FONT_NEXT, 635, 638,
																											FONT_NEXT, 675, 643,
																											FONT_NEXT, 675, 662,
																											FONT_NEXT, 413, 662,
																											FONT_END, 413, 643,
																											FONT_ADVANCE, 722, 0
																									},
																										{
																											76,
																												FONT_BEGIN, 12, 662,
																												FONT_NEXT, 12, 643,
																												FONT_NEXT, 52, 637,
																												FONT_NEXT, 79, 625,
																												FONT_NEXT, 94, 599,
																												FONT_NEXT, 99, 553,
																												FONT_NEXT, 99, 109,
																												FONT_NEXT, 94, 63,
																												FONT_NEXT, 79, 36,
																												FONT_NEXT, 52, 23,
																												FONT_NEXT, 12, 19,
																												FONT_NEXT, 12, 0,
																												FONT_NEXT, 550, 0,
																												FONT_NEXT, 598, 174,
																												FONT_NEXT, 573, 174,
																												FONT_NEXT, 551, 130,
																												FONT_NEXT, 527, 97,
																												FONT_NEXT, 501, 73,
																												FONT_NEXT, 472, 57,
																												FONT_NEXT, 399, 41,
																												FONT_NEXT, 302, 39,
																												FONT_NEXT, 251, 39,
																												FONT_NEXT, 220, 44,
																												FONT_NEXT, 205, 56,
																												FONT_NEXT, 201, 80,
																												FONT_NEXT, 201, 553,
																												FONT_NEXT, 205, 599,
																												FONT_NEXT, 222, 626,
																												FONT_NEXT, 250, 638,
																												FONT_NEXT, 294, 643,
																												FONT_END, 294, 662,
																												FONT_ADVANCE, 611, 0
																										},
																										{
																											77,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 443, 157,
																												FONT_NEXT, 212, 662,
																												FONT_NEXT, 14, 662,
																												FONT_NEXT, 14, 643,
																												FONT_NEXT, 60, 638,
																												FONT_NEXT, 89, 625,
																												FONT_NEXT, 104, 599,
																												FONT_NEXT, 109, 553,
																												FONT_NEXT, 109, 147,
																												FONT_NEXT, 107, 110,
																												FONT_NEXT, 104, 82,
																												FONT_NEXT, 89, 44,
																												FONT_NEXT, 59, 25,
																												FONT_NEXT, 12, 19,
																												FONT_NEXT, 12, 0,
																												FONT_NEXT, 247, 0,
																												FONT_NEXT, 247, 19,
																												FONT_NEXT, 201, 25,
																												FONT_NEXT, 172, 45,
																												FONT_NEXT, 157, 83,
																												FONT_NEXT, 153, 147,
																												FONT_NEXT, 153, 546,
																												FONT_NEXT, 155, 546,
																												FONT_NEXT, 404, 0,
																												FONT_NEXT, 418, 0,
																												FONT_NEXT, 672, 569,
																												FONT_NEXT, 674, 569,
																												FONT_NEXT, 674, 120,
																												FONT_NEXT, 670, 70,
																												FONT_NEXT, 657, 40,
																												FONT_NEXT, 630, 24,
																												FONT_NEXT, 583, 19,
																												FONT_NEXT, 583, 0,
																												FONT_NEXT, 863, 0,
																												FONT_NEXT, 863, 19,
																												FONT_NEXT, 822, 23,
																												FONT_NEXT, 795, 36,
																												FONT_NEXT, 780, 63,
																												FONT_NEXT, 776, 109,
																												FONT_NEXT, 776, 553,
																												FONT_NEXT, 780, 599,
																												FONT_NEXT, 795, 625,
																												FONT_NEXT, 822, 637,
																												FONT_NEXT, 863, 643,
																												FONT_END, 863, 662,
																												FONT_ADVANCE, 889, 0
																										},
																											{
																												78,
																													FONT_BEGIN, 472, 662,
																													FONT_NEXT, 472, 643,
																													FONT_NEXT, 515, 637,
																													FONT_NEXT, 545, 620,
																													FONT_NEXT, 562, 582,
																													FONT_NEXT, 566, 553,
																													FONT_NEXT, 568, 515,
																													FONT_NEXT, 568, 181,
																													FONT_NEXT, 566, 181,
																													FONT_NEXT, 183, 662,
																													FONT_NEXT, 12, 662,
																													FONT_NEXT, 12, 643,
																													FONT_NEXT, 60, 634,
																													FONT_NEXT, 82, 617,
																													FONT_NEXT, 109, 588,
																													FONT_NEXT, 109, 147,
																													FONT_NEXT, 107, 110,
																													FONT_NEXT, 104, 82,
																													FONT_NEXT, 89, 44,
																													FONT_NEXT, 59, 25,
																													FONT_NEXT, 12, 19,
																													FONT_NEXT, 12, 0,
																													FONT_NEXT, 247, 0,
																													FONT_NEXT, 247, 19,
																													FONT_NEXT, 201, 25,
																													FONT_NEXT, 172, 45,
																													FONT_NEXT, 157, 83,
																													FONT_NEXT, 153, 147,
																													FONT_NEXT, 153, 537,
																													FONT_NEXT, 155, 537,
																													FONT_NEXT, 595, -11,
																													FONT_NEXT, 612, -11,
																													FONT_NEXT, 612, 515,
																													FONT_NEXT, 613, 553,
																													FONT_NEXT, 617, 583,
																													FONT_NEXT, 635, 620,
																													FONT_NEXT, 665, 636,
																													FONT_NEXT, 707, 643,
																													FONT_END, 707, 662,
																													FONT_ADVANCE, 722, 0
																											},
																											{
																												79,
																													FONT_BEGIN, 427, -8,
																													FONT_NEXT, 488, 9,
																													FONT_NEXT, 544, 38,
																													FONT_NEXT, 592, 78,
																													FONT_NEXT, 632, 127,
																													FONT_NEXT, 662, 186,
																													FONT_NEXT, 681, 254,
																													FONT_NEXT, 688, 330,
																													FONT_NEXT, 680, 410,
																													FONT_NEXT, 659, 481,
																													FONT_NEXT, 627, 540,
																													FONT_NEXT, 586, 589,
																													FONT_NEXT, 536, 627,
																													FONT_NEXT, 481, 654,
																													FONT_NEXT, 422, 670,
																													FONT_NEXT, 361, 676,
																													FONT_NEXT, 299, 670,
																													FONT_NEXT, 240, 654,
																													FONT_NEXT, 185, 627,
																													FONT_NEXT, 136, 589,
																													FONT_NEXT, 94, 540,
																													FONT_NEXT, 62, 481,
																													FONT_NEXT, 41, 410,
																													FONT_NEXT, 34, 330,
																													FONT_NEXT, 40, 254,
																													FONT_NEXT, 59, 186,
																													FONT_NEXT, 89, 127,
																													FONT_NEXT, 129, 78,
																													FONT_NEXT, 177, 38,
																													FONT_NEXT, 233, 9,
																													FONT_NEXT, 294, -8,
																													FONT_END, 361, -14,
																													FONT_BEGIN, 319, 26,
																													FONT_NEXT, 280, 39,
																													FONT_NEXT, 244, 61,
																													FONT_NEXT, 212, 94,
																													FONT_NEXT, 186, 136,
																													FONT_NEXT, 165, 189,
																													FONT_NEXT, 152, 254,
																													FONT_NEXT, 148, 331,
																													FONT_NEXT, 152, 409,
																													FONT_NEXT, 166, 475,
																													FONT_NEXT, 188, 528,
																													FONT_NEXT, 216, 571,
																													FONT_NEXT, 248, 602,
																													FONT_NEXT, 284, 623,
																													FONT_NEXT, 322, 636,
																													FONT_NEXT, 361, 640,
																													FONT_NEXT, 399, 636,
																													FONT_NEXT, 437, 623,
																													FONT_NEXT, 473, 602,
																													FONT_NEXT, 505, 571,
																													FONT_NEXT, 533, 528,
																													FONT_NEXT, 555, 475,
																													FONT_NEXT, 569, 409,
																													FONT_NEXT, 574, 331,
																													FONT_NEXT, 569, 254,
																													FONT_NEXT, 556, 189,
																													FONT_NEXT, 535, 136,
																													FONT_NEXT, 509, 94,
																													FONT_NEXT, 477, 61,
																													FONT_NEXT, 441, 39,
																													FONT_NEXT, 402, 26,
																													FONT_END, 361, 22,
																													FONT_ADVANCE, 722, 0
																											},
																												{
																													80,
																														FONT_BEGIN, 59, 635,
																														FONT_NEXT, 85, 622,
																														FONT_NEXT, 97, 596,
																														FONT_NEXT, 100, 553,
																														FONT_NEXT, 100, 120,
																														FONT_NEXT, 98, 74,
																														FONT_NEXT, 89, 44,
																														FONT_NEXT, 64, 27,
																														FONT_NEXT, 16, 19,
																														FONT_NEXT, 16, 0,
																														FONT_NEXT, 296, 0,
																														FONT_NEXT, 296, 19,
																														FONT_NEXT, 248, 23,
																														FONT_NEXT, 220, 37,
																														FONT_NEXT, 205, 64,
																														FONT_NEXT, 202, 109,
																														FONT_NEXT, 202, 291,
																														FONT_NEXT, 269, 288,
																														FONT_NEXT, 340, 291,
																														FONT_NEXT, 398, 302,
																														FONT_NEXT, 444, 320,
																														FONT_NEXT, 481, 347,
																														FONT_NEXT, 497, 363,
																														FONT_NEXT, 517, 391,
																														FONT_NEXT, 534, 430,
																														FONT_NEXT, 542, 482,
																														FONT_NEXT, 536, 526,
																														FONT_NEXT, 520, 563,
																														FONT_NEXT, 495, 594,
																														FONT_NEXT, 462, 619,
																														FONT_NEXT, 379, 651,
																														FONT_NEXT, 280, 662,
																														FONT_NEXT, 16, 662,
																														FONT_END, 16, 643,
																														FONT_BEGIN, 204, 610,
																														FONT_NEXT, 211, 620,
																														FONT_NEXT, 243, 625,
																														FONT_NEXT, 291, 622,
																														FONT_NEXT, 354, 606,
																														FONT_NEXT, 384, 588,
																														FONT_NEXT, 409, 562,
																														FONT_NEXT, 426, 527,
																														FONT_NEXT, 433, 480,
																														FONT_NEXT, 427, 429,
																														FONT_NEXT, 411, 392,
																														FONT_NEXT, 388, 365,
																														FONT_NEXT, 361, 347,
																														FONT_NEXT, 304, 330,
																														FONT_NEXT, 262, 328,
																														FONT_NEXT, 202, 331,
																														FONT_END, 202, 591,
																														FONT_ADVANCE, 556, 0
																												},
																												{
																													81,
																														FONT_BEGIN, 638, -153,
																														FONT_NEXT, 586, -138,
																														FONT_NEXT, 505, -90,
																														FONT_NEXT, 426, -7,
																														FONT_NEXT, 468, 2,
																														FONT_NEXT, 512, 20,
																														FONT_NEXT, 556, 47,
																														FONT_NEXT, 597, 83,
																														FONT_NEXT, 633, 130,
																														FONT_NEXT, 662, 186,
																														FONT_NEXT, 681, 252,
																														FONT_NEXT, 688, 330,
																														FONT_NEXT, 680, 410,
																														FONT_NEXT, 659, 481,
																														FONT_NEXT, 627, 540,
																														FONT_NEXT, 586, 589,
																														FONT_NEXT, 536, 627,
																														FONT_NEXT, 481, 654,
																														FONT_NEXT, 422, 670,
																														FONT_NEXT, 361, 676,
																														FONT_NEXT, 299, 670,
																														FONT_NEXT, 240, 654,
																														FONT_NEXT, 185, 626,
																														FONT_NEXT, 136, 588,
																														FONT_NEXT, 94, 540,
																														FONT_NEXT, 62, 480,
																														FONT_NEXT, 41, 410,
																														FONT_NEXT, 34, 330,
																														FONT_NEXT, 40, 255,
																														FONT_NEXT, 57, 191,
																														FONT_NEXT, 82, 138,
																														FONT_NEXT, 114, 94,
																														FONT_NEXT, 151, 58,
																														FONT_NEXT, 189, 31,
																														FONT_NEXT, 265, -1,
																														FONT_NEXT, 312, -56,
																														FONT_NEXT, 379, -113,
																														FONT_NEXT, 462, -151,
																														FONT_NEXT, 555, -172,
																														FONT_NEXT, 654, -178,
																														FONT_NEXT, 701, -177,
																														FONT_END, 701, -159,
																														FONT_BEGIN, 399, 636,
																														FONT_NEXT, 437, 623,
																														FONT_NEXT, 473, 602,
																														FONT_NEXT, 505, 570,
																														FONT_NEXT, 533, 528,
																														FONT_NEXT, 555, 475,
																														FONT_NEXT, 569, 409,
																														FONT_NEXT, 574, 330,
																														FONT_NEXT, 569, 253,
																														FONT_NEXT, 556, 189,
																														FONT_NEXT, 535, 136,
																														FONT_NEXT, 509, 93,
																														FONT_NEXT, 477, 61,
																														FONT_NEXT, 441, 39,
																														FONT_NEXT, 402, 26,
																														FONT_NEXT, 361, 22,
																														FONT_NEXT, 319, 26,
																														FONT_NEXT, 280, 39,
																														FONT_NEXT, 244, 61,
																														FONT_NEXT, 212, 93,
																														FONT_NEXT, 186, 136,
																														FONT_NEXT, 165, 189,
																														FONT_NEXT, 152, 253,
																														FONT_NEXT, 148, 330,
																														FONT_NEXT, 152, 409,
																														FONT_NEXT, 166, 475,
																														FONT_NEXT, 188, 528,
																														FONT_NEXT, 216, 570,
																														FONT_NEXT, 248, 602,
																														FONT_NEXT, 284, 623,
																														FONT_NEXT, 322, 636,
																														FONT_END, 361, 640,
																														FONT_ADVANCE, 722, 0
																												},
																													{
																														82,
																															FONT_BEGIN, 608, 33,
																															FONT_NEXT, 572, 66,
																															FONT_NEXT, 366, 319,
																															FONT_NEXT, 415, 330,
																															FONT_NEXT, 475, 356,
																															FONT_NEXT, 503, 377,
																															FONT_NEXT, 525, 405,
																															FONT_NEXT, 541, 441,
																															FONT_NEXT, 547, 487,
																															FONT_NEXT, 539, 538,
																															FONT_NEXT, 519, 579,
																															FONT_NEXT, 490, 610,
																															FONT_NEXT, 453, 632,
																															FONT_NEXT, 370, 656,
																															FONT_NEXT, 293, 662,
																															FONT_NEXT, 17, 662,
																															FONT_NEXT, 17, 643,
																															FONT_NEXT, 59, 636,
																															FONT_NEXT, 85, 623,
																															FONT_NEXT, 98, 597,
																															FONT_NEXT, 102, 553,
																															FONT_NEXT, 102, 120,
																															FONT_NEXT, 100, 73,
																															FONT_NEXT, 89, 43,
																															FONT_NEXT, 64, 26,
																															FONT_NEXT, 17, 19,
																															FONT_NEXT, 17, 0,
																															FONT_NEXT, 294, 0,
																															FONT_NEXT, 294, 19,
																															FONT_NEXT, 246, 25,
																															FONT_NEXT, 219, 40,
																															FONT_NEXT, 206, 67,
																															FONT_NEXT, 204, 109,
																															FONT_NEXT, 204, 306,
																															FONT_NEXT, 260, 308,
																															FONT_NEXT, 498, 0,
																															FONT_NEXT, 659, 0,
																															FONT_END, 659, 19,
																															FONT_BEGIN, 206, 606,
																															FONT_NEXT, 215, 617,
																															FONT_NEXT, 233, 623,
																															FONT_NEXT, 266, 625,
																															FONT_NEXT, 310, 622,
																															FONT_NEXT, 367, 606,
																															FONT_NEXT, 394, 590,
																															FONT_NEXT, 416, 566,
																															FONT_NEXT, 432, 533,
																															FONT_NEXT, 438, 489,
																															FONT_NEXT, 431, 441,
																															FONT_NEXT, 413, 406,
																															FONT_NEXT, 386, 380,
																															FONT_NEXT, 352, 362,
																															FONT_NEXT, 276, 346,
																															FONT_NEXT, 204, 343,
																															FONT_END, 204, 589,
																															FONT_ADVANCE, 667, 0
																													},
																													{
																														83,
																															FONT_BEGIN, 426, 676,
																															FONT_NEXT, 418, 654,
																															FONT_NEXT, 408, 645,
																															FONT_NEXT, 391, 642,
																															FONT_NEXT, 367, 647,
																															FONT_NEXT, 334, 659,
																															FONT_NEXT, 252, 676,
																															FONT_NEXT, 185, 665,
																															FONT_NEXT, 127, 633,
																															FONT_NEXT, 86, 580,
																															FONT_NEXT, 75, 546,
																															FONT_NEXT, 71, 506,
																															FONT_NEXT, 82, 441,
																															FONT_NEXT, 114, 390,
																															FONT_NEXT, 163, 349,
																															FONT_NEXT, 227, 310,
																															FONT_NEXT, 276, 280,
																															FONT_NEXT, 314, 254,
																															FONT_NEXT, 343, 231,
																															FONT_NEXT, 363, 210,
																															FONT_NEXT, 385, 172,
																															FONT_NEXT, 390, 133,
																															FONT_NEXT, 382, 92,
																															FONT_NEXT, 360, 57,
																															FONT_NEXT, 322, 31,
																															FONT_NEXT, 270, 22,
																															FONT_NEXT, 223, 27,
																															FONT_NEXT, 183, 43,
																															FONT_NEXT, 124, 92,
																															FONT_NEXT, 86, 151,
																															FONT_NEXT, 65, 199,
																															FONT_NEXT, 42, 199,
																															FONT_NEXT, 72, -13,
																															FONT_NEXT, 94, -13,
																															FONT_NEXT, 101, 9,
																															FONT_NEXT, 125, 20,
																															FONT_NEXT, 151, 14,
																															FONT_NEXT, 185, 3,
																															FONT_NEXT, 227, -9,
																															FONT_NEXT, 280, -14,
																															FONT_NEXT, 330, -10,
																															FONT_NEXT, 373, 2,
																															FONT_NEXT, 409, 20,
																															FONT_NEXT, 439, 44,
																															FONT_NEXT, 478, 102,
																															FONT_NEXT, 491, 167,
																															FONT_NEXT, 476, 235,
																															FONT_NEXT, 438, 290,
																															FONT_NEXT, 385, 336,
																															FONT_NEXT, 324, 375,
																															FONT_NEXT, 209, 448,
																															FONT_NEXT, 171, 491,
																															FONT_NEXT, 157, 542,
																															FONT_NEXT, 165, 582,
																															FONT_NEXT, 188, 611,
																															FONT_NEXT, 221, 629,
																															FONT_NEXT, 258, 635,
																															FONT_NEXT, 295, 630,
																															FONT_NEXT, 328, 619,
																															FONT_NEXT, 384, 579,
																															FONT_NEXT, 422, 524,
																															FONT_NEXT, 444, 463,
																															FONT_NEXT, 469, 463,
																															FONT_END, 447, 676,
																															FONT_ADVANCE, 556, 0
																													},
																														{
																															84,
																																FONT_BEGIN, 401, 24,
																																FONT_NEXT, 372, 39,
																																FONT_NEXT, 359, 66,
																																FONT_NEXT, 356, 109,
																																FONT_NEXT, 356, 620,
																																FONT_NEXT, 410, 620,
																																FONT_NEXT, 478, 615,
																																FONT_NEXT, 522, 597,
																																FONT_NEXT, 550, 558,
																																FONT_NEXT, 569, 492,
																																FONT_NEXT, 593, 492,
																																FONT_NEXT, 587, 662,
																																FONT_NEXT, 23, 662,
																																FONT_NEXT, 17, 492,
																																FONT_NEXT, 41, 492,
																																FONT_NEXT, 59, 557,
																																FONT_NEXT, 87, 596,
																																FONT_NEXT, 131, 615,
																																FONT_NEXT, 200, 620,
																																FONT_NEXT, 254, 620,
																																FONT_NEXT, 254, 120,
																																FONT_NEXT, 252, 73,
																																FONT_NEXT, 241, 43,
																																FONT_NEXT, 213, 26,
																																FONT_NEXT, 160, 19,
																																FONT_NEXT, 160, 0,
																																FONT_NEXT, 452, 0,
																																FONT_END, 452, 19,
																																FONT_ADVANCE, 611, 0
																														},
																														{
																															85,
																																FONT_BEGIN, 473, 662,
																																FONT_NEXT, 473, 643,
																																FONT_NEXT, 517, 636,
																																FONT_NEXT, 546, 617,
																																FONT_NEXT, 562, 579,
																																FONT_NEXT, 567, 515,
																																FONT_NEXT, 567, 245,
																																FONT_NEXT, 565, 192,
																																FONT_NEXT, 560, 154,
																																FONT_NEXT, 544, 109,
																																FONT_NEXT, 517, 76,
																																FONT_NEXT, 480, 51,
																																FONT_NEXT, 434, 35,
																																FONT_NEXT, 380, 30,
																																FONT_NEXT, 322, 36,
																																FONT_NEXT, 279, 52,
																																FONT_NEXT, 248, 77,
																																FONT_NEXT, 227, 107,
																																FONT_NEXT, 208, 174,
																																FONT_NEXT, 206, 233,
																																FONT_NEXT, 206, 553,
																																FONT_NEXT, 209, 596,
																																FONT_NEXT, 221, 622,
																																FONT_NEXT, 249, 636,
																																FONT_NEXT, 297, 643,
																																FONT_NEXT, 297, 662,
																																FONT_NEXT, 14, 662,
																																FONT_NEXT, 14, 643,
																																FONT_NEXT, 60, 637,
																																FONT_NEXT, 87, 623,
																																FONT_NEXT, 100, 597,
																																FONT_NEXT, 104, 553,
																																FONT_NEXT, 104, 241,
																																FONT_NEXT, 104, 212,
																																FONT_NEXT, 107, 175,
																																FONT_NEXT, 117, 133,
																																FONT_NEXT, 135, 91,
																																FONT_NEXT, 164, 50,
																																FONT_NEXT, 208, 17,
																																FONT_NEXT, 270, -6,
																																FONT_NEXT, 352, -14,
																																FONT_NEXT, 437, -6,
																																FONT_NEXT, 501, 16,
																																FONT_NEXT, 547, 49,
																																FONT_NEXT, 578, 90,
																																FONT_NEXT, 597, 134,
																																FONT_NEXT, 606, 178,
																																FONT_NEXT, 611, 254,
																																FONT_NEXT, 611, 515,
																																FONT_NEXT, 614, 578,
																																FONT_NEXT, 629, 615,
																																FONT_NEXT, 657, 634,
																																FONT_NEXT, 705, 643,
																																FONT_END, 705, 662,
																																FONT_ADVANCE, 722, 0
																														},
																															{
																																86,
																																	FONT_BEGIN, 492, 662,
																																	FONT_NEXT, 492, 643,
																																	FONT_NEXT, 542, 634,
																																	FONT_NEXT, 558, 621,
																																	FONT_NEXT, 565, 597,
																																	FONT_NEXT, 556, 556,
																																	FONT_NEXT, 549, 538,
																																	FONT_NEXT, 546, 528,
																																	FONT_NEXT, 399, 161,
																																	FONT_NEXT, 248, 499,
																																	FONT_NEXT, 218, 566,
																																	FONT_NEXT, 210, 590,
																																	FONT_NEXT, 207, 606,
																																	FONT_NEXT, 210, 624,
																																	FONT_NEXT, 223, 635,
																																	FONT_NEXT, 246, 640,
																																	FONT_NEXT, 282, 643,
																																	FONT_NEXT, 282, 662,
																																	FONT_NEXT, 16, 662,
																																	FONT_NEXT, 16, 643,
																																	FONT_NEXT, 46, 638,
																																	FONT_NEXT, 71, 623,
																																	FONT_NEXT, 94, 592,
																																	FONT_NEXT, 122, 538,
																																	FONT_NEXT, 368, -11,
																																	FONT_NEXT, 383, -11,
																																	FONT_NEXT, 605, 550,
																																	FONT_NEXT, 625, 596,
																																	FONT_NEXT, 644, 623,
																																	FONT_NEXT, 666, 637,
																																	FONT_NEXT, 697, 643,
																																	FONT_END, 697, 662,
																																	FONT_ADVANCE, 722, 0
																															},
																															{
																																87,
																																	FONT_BEGIN, 734, 662,
																																	FONT_NEXT, 734, 643,
																																	FONT_NEXT, 781, 633,
																																	FONT_NEXT, 797, 620,
																																	FONT_NEXT, 803, 597,
																																	FONT_NEXT, 787, 525,
																																	FONT_NEXT, 662, 186,
																																	FONT_NEXT, 530, 527,
																																	FONT_NEXT, 510, 574,
																																	FONT_NEXT, 503, 605,
																																	FONT_NEXT, 510, 626,
																																	FONT_NEXT, 529, 637,
																																	FONT_NEXT, 580, 643,
																																	FONT_NEXT, 580, 662,
																																	FONT_NEXT, 313, 662,
																																	FONT_NEXT, 313, 643,
																																	FONT_NEXT, 346, 640,
																																	FONT_NEXT, 371, 628,
																																	FONT_NEXT, 391, 601,
																																	FONT_NEXT, 414, 553,
																																	FONT_NEXT, 447, 471,
																																	FONT_NEXT, 340, 189,
																																	FONT_NEXT, 196, 565,
																																	FONT_NEXT, 185, 609,
																																	FONT_NEXT, 188, 625,
																																	FONT_NEXT, 200, 635,
																																	FONT_NEXT, 220, 640,
																																	FONT_NEXT, 250, 643,
																																	FONT_NEXT, 250, 662,
																																	FONT_NEXT, 5, 662,
																																	FONT_NEXT, 5, 643,
																																	FONT_NEXT, 40, 634,
																																	FONT_NEXT, 65, 613,
																																	FONT_NEXT, 86, 578,
																																	FONT_NEXT, 108, 526,
																																	FONT_NEXT, 125, 480,
																																	FONT_NEXT, 147, 421,
																																	FONT_NEXT, 199, 277,
																																	FONT_NEXT, 254, 123,
																																	FONT_NEXT, 279, 51,
																																	FONT_NEXT, 301, -11,
																																	FONT_NEXT, 316, -11,
																																	FONT_NEXT, 470, 412,
																																	FONT_NEXT, 630, -11,
																																	FONT_NEXT, 645, -11,
																																	FONT_NEXT, 745, 288,
																																	FONT_NEXT, 853, 582,
																																	FONT_NEXT, 864, 606,
																																	FONT_NEXT, 879, 624,
																																	FONT_NEXT, 900, 635,
																																	FONT_NEXT, 932, 643,
																																	FONT_END, 932, 662,
																																	FONT_ADVANCE, 944, 0
																															},
																																{
																																	88,
																																		FONT_BEGIN, 458, 662,
																																		FONT_NEXT, 458, 643,
																																		FONT_NEXT, 505, 637,
																																		FONT_NEXT, 521, 628,
																																		FONT_NEXT, 528, 610,
																																		FONT_NEXT, 516, 579,
																																		FONT_NEXT, 488, 542,
																																		FONT_NEXT, 375, 404,
																																		FONT_NEXT, 333, 463,
																																		FONT_NEXT, 291, 523,
																																		FONT_NEXT, 265, 565,
																																		FONT_NEXT, 251, 592,
																																		FONT_NEXT, 248, 611,
																																		FONT_NEXT, 252, 628,
																																		FONT_NEXT, 266, 637,
																																		FONT_NEXT, 290, 641,
																																		FONT_NEXT, 324, 643,
																																		FONT_NEXT, 324, 662,
																																		FONT_NEXT, 22, 662,
																																		FONT_NEXT, 22, 643,
																																		FONT_NEXT, 60, 637,
																																		FONT_NEXT, 96, 618,
																																		FONT_NEXT, 116, 600,
																																		FONT_NEXT, 139, 573,
																																		FONT_NEXT, 168, 535,
																																		FONT_NEXT, 203, 486,
																																		FONT_NEXT, 312, 326,
																																		FONT_NEXT, 155, 133,
																																		FONT_NEXT, 128, 100,
																																		FONT_NEXT, 106, 74,
																																		FONT_NEXT, 73, 41,
																																		FONT_NEXT, 44, 24,
																																		FONT_NEXT, 10, 19,
																																		FONT_NEXT, 10, 0,
																																		FONT_NEXT, 243, 0,
																																		FONT_NEXT, 243, 19,
																																		FONT_NEXT, 190, 26,
																																		FONT_NEXT, 173, 37,
																																		FONT_NEXT, 167, 56,
																																		FONT_NEXT, 172, 73,
																																		FONT_NEXT, 185, 95,
																																		FONT_NEXT, 219, 140,
																																		FONT_NEXT, 338, 288,
																																		FONT_NEXT, 433, 148,
																																		FONT_NEXT, 464, 98,
																																		FONT_NEXT, 478, 72,
																																		FONT_NEXT, 484, 53,
																																		FONT_NEXT, 479, 35,
																																		FONT_NEXT, 464, 25,
																																		FONT_NEXT, 440, 21,
																																		FONT_NEXT, 407, 19,
																																		FONT_NEXT, 407, 0,
																																		FONT_NEXT, 704, 0,
																																		FONT_NEXT, 704, 19,
																																		FONT_NEXT, 670, 24,
																																		FONT_NEXT, 643, 37,
																																		FONT_NEXT, 619, 58,
																																		FONT_NEXT, 593, 93,
																																		FONT_NEXT, 401, 367,
																																		FONT_NEXT, 547, 549,
																																		FONT_NEXT, 592, 599,
																																		FONT_NEXT, 627, 627,
																																		FONT_NEXT, 660, 638,
																																		FONT_NEXT, 696, 643,
																																		FONT_END, 696, 662,
																																		FONT_ADVANCE, 722, 0
																																},
																																{
																																	89,
																																		FONT_BEGIN, 484, 662,
																																		FONT_NEXT, 484, 643,
																																		FONT_NEXT, 530, 638,
																																		FONT_NEXT, 546, 628,
																																		FONT_NEXT, 553, 610,
																																		FONT_NEXT, 539, 573,
																																		FONT_NEXT, 396, 347,
																																		FONT_NEXT, 248, 569,
																																		FONT_NEXT, 231, 612,
																																		FONT_NEXT, 235, 628,
																																		FONT_NEXT, 249, 637,
																																		FONT_NEXT, 271, 641,
																																		FONT_NEXT, 302, 643,
																																		FONT_NEXT, 302, 662,
																																		FONT_NEXT, 22, 662,
																																		FONT_NEXT, 22, 643,
																																		FONT_NEXT, 51, 637,
																																		FONT_NEXT, 81, 619,
																																		FONT_NEXT, 99, 600,
																																		FONT_NEXT, 121, 573,
																																		FONT_NEXT, 149, 535,
																																		FONT_NEXT, 184, 486,
																																		FONT_NEXT, 315, 294,
																																		FONT_NEXT, 315, 120,
																																		FONT_NEXT, 313, 72,
																																		FONT_NEXT, 301, 41,
																																		FONT_NEXT, 271, 24,
																																		FONT_NEXT, 246, 20,
																																		FONT_NEXT, 214, 19,
																																		FONT_NEXT, 214, 0,
																																		FONT_NEXT, 520, 0,
																																		FONT_NEXT, 520, 19,
																																		FONT_NEXT, 465, 23,
																																		FONT_NEXT, 434, 37,
																																		FONT_NEXT, 420, 64,
																																		FONT_NEXT, 417, 109,
																																		FONT_NEXT, 417, 303,
																																		FONT_NEXT, 565, 529,
																																		FONT_NEXT, 589, 564,
																																		FONT_NEXT, 610, 590,
																																		FONT_NEXT, 646, 624,
																																		FONT_NEXT, 675, 638,
																																		FONT_NEXT, 703, 643,
																																		FONT_END, 703, 662,
																																		FONT_ADVANCE, 722, 0
																																},
																																	{
																																		90,
																																			FONT_BEGIN, 556, 123,
																																			FONT_NEXT, 528, 79,
																																			FONT_NEXT, 481, 49,
																																			FONT_NEXT, 446, 40,
																																			FONT_NEXT, 402, 38,
																																			FONT_NEXT, 145, 38,
																																			FONT_NEXT, 577, 647,
																																			FONT_NEXT, 577, 662,
																																			FONT_NEXT, 51, 662,
																																			FONT_NEXT, 31, 491,
																																			FONT_NEXT, 57, 491,
																																			FONT_NEXT, 68, 540,
																																			FONT_NEXT, 92, 582,
																																			FONT_NEXT, 112, 599,
																																			FONT_NEXT, 140, 612,
																																			FONT_NEXT, 177, 621,
																																			FONT_NEXT, 225, 624,
																																			FONT_NEXT, 446, 624,
																																			FONT_NEXT, 9, 15,
																																			FONT_NEXT, 9, 0,
																																			FONT_NEXT, 573, 0,
																																			FONT_NEXT, 597, 176,
																																			FONT_END, 574, 176,
																																			FONT_ADVANCE, 611, 0
																																	},
																																	{
																																		91,
																																			FONT_BEGIN, 299, 662,
																																			FONT_NEXT, 88, 662,
																																			FONT_NEXT, 88, -156,
																																			FONT_NEXT, 299, -156,
																																			FONT_NEXT, 299, -131,
																																			FONT_NEXT, 213, -131,
																																			FONT_NEXT, 184, -125,
																																			FONT_NEXT, 170, -110,
																																			FONT_NEXT, 164, -79,
																																			FONT_NEXT, 164, 593,
																																			FONT_NEXT, 169, 620,
																																			FONT_NEXT, 182, 632,
																																			FONT_NEXT, 209, 637,
																																			FONT_END, 299, 637,
																																			FONT_ADVANCE, 333, 0
																																	},
																																		{
																																			92,
																																				FONT_BEGIN, 219, -14,
																																				FONT_NEXT, 287, -14,
																																				FONT_NEXT, 58, 676,
																																				FONT_END, -9, 676,
																																				FONT_ADVANCE, 278, 0
																																		},
																																		{
																																			93,
																																				FONT_BEGIN, 34, -156,
																																				FONT_NEXT, 245, -156,
																																				FONT_NEXT, 245, 662,
																																				FONT_NEXT, 34, 662,
																																				FONT_NEXT, 34, 637,
																																				FONT_NEXT, 120, 637,
																																				FONT_NEXT, 148, 630,
																																				FONT_NEXT, 162, 615,
																																				FONT_NEXT, 169, 585,
																																				FONT_NEXT, 169, -87,
																																				FONT_NEXT, 163, -115,
																																				FONT_NEXT, 150, -127,
																																				FONT_NEXT, 124, -131,
																																				FONT_END, 34, -131,
																																				FONT_ADVANCE, 333, 0
																																		},
																																			{
																																				94,
																																					FONT_BEGIN, 235, 586,
																																					FONT_NEXT, 378, 297,
																																					FONT_NEXT, 446, 297,
																																					FONT_NEXT, 265, 662,
																																					FONT_NEXT, 205, 662,
																																					FONT_NEXT, 24, 297,
																																					FONT_END, 92, 297,
																																					FONT_ADVANCE, 469, 0
																																			},
																																			{
																																				95,
																																					FONT_BEGIN, 500, -75,
																																					FONT_NEXT, 0, -75,
																																					FONT_NEXT, 0, -125,
																																					FONT_END, 500, -125,
																																					FONT_ADVANCE, 500, 0
																																			},
																																				{
																																					96,
																																						FONT_BEGIN, 196, 658,
																																						FONT_NEXT, 159, 625,
																																						FONT_NEXT, 128, 580,
																																						FONT_NEXT, 115, 529,
																																						FONT_NEXT, 124, 478,
																																						FONT_NEXT, 146, 449,
																																						FONT_NEXT, 173, 436,
																																						FONT_NEXT, 196, 433,
																																						FONT_NEXT, 234, 445,
																																						FONT_NEXT, 248, 463,
																																						FONT_NEXT, 254, 491,
																																						FONT_NEXT, 247, 515,
																																						FONT_NEXT, 232, 531,
																																						FONT_NEXT, 192, 541,
																																						FONT_NEXT, 177, 539,
																																						FONT_NEXT, 168, 537,
																																						FONT_NEXT, 158, 540,
																																						FONT_NEXT, 154, 551,
																																						FONT_NEXT, 157, 569,
																																						FONT_NEXT, 170, 595,
																																						FONT_NEXT, 195, 625,
																																						FONT_NEXT, 236, 657,
																																						FONT_END, 227, 676,
																																						FONT_ADVANCE, 333, 0
																																				},
																																				{
																																					97,
																																						FONT_BEGIN, 281, 88,
																																						FONT_NEXT, 271, 77,
																																						FONT_NEXT, 253, 66,
																																						FONT_NEXT, 226, 55,
																																						FONT_NEXT, 191, 48,
																																						FONT_NEXT, 165, 53,
																																						FONT_NEXT, 144, 69,
																																						FONT_NEXT, 130, 93,
																																						FONT_NEXT, 125, 126,
																																						FONT_NEXT, 125, 128,
																																						FONT_NEXT, 129, 157,
																																						FONT_NEXT, 149, 192,
																																						FONT_NEXT, 169, 210,
																																						FONT_NEXT, 197, 229,
																																						FONT_NEXT, 236, 249,
																																						FONT_NEXT, 287, 268,
																																						FONT_END, 287, 123,
																																						FONT_BEGIN, 423, 53,
																																						FONT_NEXT, 397, 47,
																																						FONT_NEXT, 380, 51,
																																						FONT_NEXT, 371, 65,
																																						FONT_NEXT, 368, 105,
																																						FONT_NEXT, 368, 300,
																																						FONT_NEXT, 365, 350,
																																						FONT_NEXT, 349, 402,
																																						FONT_NEXT, 331, 425,
																																						FONT_NEXT, 305, 443,
																																						FONT_NEXT, 268, 455,
																																						FONT_NEXT, 219, 460,
																																						FONT_NEXT, 178, 456,
																																						FONT_NEXT, 144, 448,
																																						FONT_NEXT, 94, 419,
																																						FONT_NEXT, 65, 383,
																																						FONT_NEXT, 56, 349,
																																						FONT_NEXT, 65, 321,
																																						FONT_NEXT, 79, 309,
																																						FONT_NEXT, 99, 305,
																																						FONT_NEXT, 131, 318,
																																						FONT_NEXT, 144, 348,
																																						FONT_NEXT, 139, 387,
																																						FONT_NEXT, 145, 407,
																																						FONT_NEXT, 162, 423,
																																						FONT_NEXT, 210, 436,
																																						FONT_NEXT, 234, 433,
																																						FONT_NEXT, 259, 421,
																																						FONT_NEXT, 279, 396,
																																						FONT_NEXT, 287, 353,
																																						FONT_NEXT, 287, 292,
																																						FONT_NEXT, 179, 248,
																																						FONT_NEXT, 100, 206,
																																						FONT_NEXT, 73, 183,
																																						FONT_NEXT, 53, 158,
																																						FONT_NEXT, 41, 128,
																																						FONT_NEXT, 37, 94,
																																						FONT_NEXT, 45, 49,
																																						FONT_NEXT, 69, 16,
																																						FONT_NEXT, 103, -4,
																																						FONT_NEXT, 143, -10,
																																						FONT_NEXT, 183, -4,
																																						FONT_NEXT, 222, 14,
																																						FONT_NEXT, 288, 63,
																																						FONT_NEXT, 297, 21,
																																						FONT_NEXT, 315, -1,
																																						FONT_NEXT, 353, -10,
																																						FONT_NEXT, 387, -4,
																																						FONT_NEXT, 412, 11,
																																						FONT_NEXT, 442, 40,
																																						FONT_END, 442, 66,
																																						FONT_ADVANCE, 444, 0
																																				},
																																					{
																																						98,
																																							FONT_BEGIN, 165, 357,
																																							FONT_NEXT, 192, 380,
																																							FONT_NEXT, 224, 393,
																																							FONT_NEXT, 253, 397,
																																							FONT_NEXT, 284, 392,
																																							FONT_NEXT, 311, 379,
																																							FONT_NEXT, 350, 333,
																																							FONT_NEXT, 372, 270,
																																							FONT_NEXT, 380, 201,
																																							FONT_NEXT, 375, 145,
																																							FONT_NEXT, 357, 86,
																																							FONT_NEXT, 340, 61,
																																							FONT_NEXT, 317, 40,
																																							FONT_NEXT, 287, 26,
																																							FONT_NEXT, 249, 22,
																																							FONT_NEXT, 223, 24,
																																							FONT_NEXT, 191, 32,
																																							FONT_NEXT, 164, 47,
																																							FONT_NEXT, 153, 70,
																																							FONT_END, 153, 322,
																																							FONT_BEGIN, 73, 40,
																																							FONT_NEXT, 86, 28,
																																							FONT_NEXT, 127, 8,
																																							FONT_NEXT, 178, -6,
																																							FONT_NEXT, 224, -10,
																																							FONT_NEXT, 287, -4,
																																							FONT_NEXT, 340, 16,
																																							FONT_NEXT, 382, 44,
																																							FONT_NEXT, 415, 80,
																																							FONT_NEXT, 439, 119,
																																							FONT_NEXT, 455, 161,
																																							FONT_NEXT, 468, 238,
																																							FONT_NEXT, 457, 317,
																																							FONT_NEXT, 424, 389,
																																							FONT_NEXT, 399, 417,
																																							FONT_NEXT, 368, 440,
																																							FONT_NEXT, 332, 454,
																																							FONT_NEXT, 290, 460,
																																							FONT_NEXT, 237, 450,
																																							FONT_NEXT, 196, 428,
																																							FONT_NEXT, 167, 401,
																																							FONT_NEXT, 155, 379,
																																							FONT_NEXT, 153, 379,
																																							FONT_NEXT, 153, 681,
																																							FONT_NEXT, 148, 683,
																																							FONT_NEXT, 3, 639,
																																							FONT_NEXT, 3, 623,
																																							FONT_NEXT, 25, 624,
																																							FONT_NEXT, 50, 620,
																																							FONT_NEXT, 63, 609,
																																							FONT_NEXT, 69, 573,
																																							FONT_END, 69, 54,
																																							FONT_ADVANCE, 500, 0
																																					},
																																					{
																																						99,
																																							FONT_BEGIN, 340, 89,
																																							FONT_NEXT, 302, 69,
																																							FONT_NEXT, 255, 62,
																																							FONT_NEXT, 195, 75,
																																							FONT_NEXT, 147, 114,
																																							FONT_NEXT, 114, 174,
																																							FONT_NEXT, 102, 253,
																																							FONT_NEXT, 106, 302,
																																							FONT_NEXT, 116, 342,
																																							FONT_NEXT, 133, 373,
																																							FONT_NEXT, 152, 397,
																																							FONT_NEXT, 195, 423,
																																							FONT_NEXT, 231, 431,
																																							FONT_NEXT, 262, 427,
																																							FONT_NEXT, 280, 418,
																																							FONT_NEXT, 297, 383,
																																							FONT_NEXT, 303, 361,
																																							FONT_NEXT, 320, 329,
																																							FONT_NEXT, 334, 318,
																																							FONT_NEXT, 352, 315,
																																							FONT_NEXT, 386, 329,
																																							FONT_NEXT, 398, 359,
																																							FONT_NEXT, 387, 392,
																																							FONT_NEXT, 355, 429,
																																							FONT_NEXT, 306, 450,
																																							FONT_NEXT, 249, 460,
																																							FONT_NEXT, 168, 444,
																																							FONT_NEXT, 130, 425,
																																							FONT_NEXT, 96, 398,
																																							FONT_NEXT, 67, 363,
																																							FONT_NEXT, 44, 320,
																																							FONT_NEXT, 30, 270,
																																							FONT_NEXT, 25, 213,
																																							FONT_NEXT, 29, 158,
																																							FONT_NEXT, 42, 111,
																																							FONT_NEXT, 63, 73,
																																							FONT_NEXT, 88, 42,
																																							FONT_NEXT, 149, 2,
																																							FONT_NEXT, 212, -10,
																																							FONT_NEXT, 269, -2,
																																							FONT_NEXT, 322, 25,
																																							FONT_NEXT, 369, 74,
																																							FONT_NEXT, 412, 147,
																																							FONT_END, 398, 156,
																																							FONT_ADVANCE, 444, 0
																																					},
																																						{
																																							100,
																																								FONT_BEGIN, 328, 77,
																																								FONT_NEXT, 308, 58,
																																								FONT_NEXT, 281, 46,
																																								FONT_NEXT, 251, 42,
																																								FONT_NEXT, 201, 53,
																																								FONT_NEXT, 157, 88,
																																								FONT_NEXT, 139, 116,
																																								FONT_NEXT, 125, 152,
																																								FONT_NEXT, 116, 195,
																																								FONT_NEXT, 113, 247,
																																								FONT_NEXT, 116, 293,
																																								FONT_NEXT, 124, 333,
																																								FONT_NEXT, 154, 390,
																																								FONT_NEXT, 194, 422,
																																								FONT_NEXT, 237, 432,
																																								FONT_NEXT, 271, 425,
																																								FONT_NEXT, 303, 407,
																																								FONT_NEXT, 327, 376,
																																								FONT_NEXT, 340, 332,
																																								FONT_END, 340, 102,
																																								FONT_BEGIN, 452, 58,
																																								FONT_NEXT, 432, 68,
																																								FONT_NEXT, 425, 87,
																																								FONT_NEXT, 424, 114,
																																								FONT_NEXT, 424, 681,
																																								FONT_NEXT, 419, 683,
																																								FONT_NEXT, 272, 639,
																																								FONT_NEXT, 272, 623,
																																								FONT_NEXT, 296, 624,
																																								FONT_NEXT, 321, 620,
																																								FONT_NEXT, 334, 609,
																																								FONT_NEXT, 340, 573,
																																								FONT_NEXT, 340, 417,
																																								FONT_NEXT, 290, 449,
																																								FONT_NEXT, 234, 460,
																																								FONT_NEXT, 192, 454,
																																								FONT_NEXT, 154, 439,
																																								FONT_NEXT, 119, 416,
																																								FONT_NEXT, 88, 386,
																																								FONT_NEXT, 43, 307,
																																								FONT_NEXT, 31, 262,
																																								FONT_NEXT, 27, 214,
																																								FONT_NEXT, 33, 145,
																																								FONT_NEXT, 49, 92,
																																								FONT_NEXT, 74, 53,
																																								FONT_NEXT, 103, 25,
																																								FONT_NEXT, 164, -4,
																																								FONT_NEXT, 209, -10,
																																								FONT_NEXT, 257, -4,
																																								FONT_NEXT, 294, 11,
																																								FONT_NEXT, 320, 32,
																																								FONT_NEXT, 338, 54,
																																								FONT_NEXT, 340, 54,
																																								FONT_NEXT, 340, -7,
																																								FONT_NEXT, 344, -10,
																																								FONT_NEXT, 491, 42,
																																								FONT_END, 491, 58,
																																								FONT_ADVANCE, 500, 0
																																						},
																																						{
																																							101,
																																								FONT_BEGIN, 386, 131,
																																								FONT_NEXT, 355, 97,
																																								FONT_NEXT, 311, 70,
																																								FONT_NEXT, 253, 59,
																																								FONT_NEXT, 202, 68,
																																								FONT_NEXT, 153, 101,
																																								FONT_NEXT, 132, 130,
																																								FONT_NEXT, 114, 168,
																																								FONT_NEXT, 101, 216,
																																								FONT_NEXT, 94, 277,
																																								FONT_NEXT, 405, 277,
																																								FONT_NEXT, 398, 322,
																																								FONT_NEXT, 386, 360,
																																								FONT_NEXT, 348, 417,
																																								FONT_NEXT, 294, 449,
																																								FONT_NEXT, 229, 460,
																																								FONT_NEXT, 163, 447,
																																								FONT_NEXT, 129, 430,
																																								FONT_NEXT, 97, 406,
																																								FONT_NEXT, 68, 373,
																																								FONT_NEXT, 45, 330,
																																								FONT_NEXT, 30, 277,
																																								FONT_NEXT, 25, 214,
																																								FONT_NEXT, 28, 163,
																																								FONT_NEXT, 38, 119,
																																								FONT_NEXT, 54, 80,
																																								FONT_NEXT, 76, 48,
																																								FONT_NEXT, 104, 23,
																																								FONT_NEXT, 135, 5,
																																								FONT_NEXT, 172, -7,
																																								FONT_NEXT, 212, -10,
																																								FONT_NEXT, 262, -5,
																																								FONT_NEXT, 304, 11,
																																								FONT_NEXT, 368, 61,
																																								FONT_NEXT, 406, 117,
																																								FONT_NEXT, 417, 140,
																																								FONT_NEXT, 424, 157,
																																								FONT_END, 408, 164,
																																								FONT_BEGIN, 113, 364,
																																								FONT_NEXT, 137, 399,
																																								FONT_NEXT, 168, 418,
																																								FONT_NEXT, 207, 424,
																																								FONT_NEXT, 254, 412,
																																								FONT_NEXT, 282, 384,
																																								FONT_NEXT, 303, 309,
																																								FONT_END, 97, 309,
																																								FONT_ADVANCE, 444, 0
																																						},
																																							{
																																								102,
																																									FONT_BEGIN, 186, 450,
																																									FONT_NEXT, 186, 566,
																																									FONT_NEXT, 195, 624,
																																									FONT_NEXT, 213, 646,
																																									FONT_NEXT, 246, 655,
																																									FONT_NEXT, 340, 580,
																																									FONT_NEXT, 369, 590,
																																									FONT_NEXT, 379, 603,
																																									FONT_NEXT, 383, 622,
																																									FONT_NEXT, 375, 645,
																																									FONT_NEXT, 354, 665,
																																									FONT_NEXT, 322, 678,
																																									FONT_NEXT, 283, 683,
																																									FONT_NEXT, 223, 675,
																																									FONT_NEXT, 178, 654,
																																									FONT_NEXT, 146, 624,
																																									FONT_NEXT, 125, 587,
																																									FONT_NEXT, 105, 510,
																																									FONT_NEXT, 103, 476,
																																									FONT_NEXT, 103, 450,
																																									FONT_NEXT, 21, 450,
																																									FONT_NEXT, 21, 418,
																																									FONT_NEXT, 103, 418,
																																									FONT_NEXT, 103, 104,
																																									FONT_NEXT, 99, 58,
																																									FONT_NEXT, 86, 31,
																																									FONT_NEXT, 60, 19,
																																									FONT_NEXT, 20, 15,
																																									FONT_NEXT, 20, 0,
																																									FONT_NEXT, 280, 0,
																																									FONT_NEXT, 280, 15,
																																									FONT_NEXT, 226, 21,
																																									FONT_NEXT, 199, 37,
																																									FONT_NEXT, 188, 64,
																																									FONT_NEXT, 187, 104,
																																									FONT_NEXT, 187, 418,
																																									FONT_NEXT, 309, 418,
																																									FONT_END, 309, 450,
																																									FONT_ADVANCE, 333, 0
																																							},
																																							{
																																								103,
																																									FONT_BEGIN, 110, -49,
																																									FONT_NEXT, 147, -2,
																																									FONT_NEXT, 210, -12,
																																									FONT_NEXT, 251, -14,
																																									FONT_NEXT, 310, -15,
																																									FONT_NEXT, 360, -18,
																																									FONT_NEXT, 399, -24,
																																									FONT_NEXT, 424, -38,
																																									FONT_NEXT, 433, -64,
																																									FONT_NEXT, 420, -102,
																																									FONT_NEXT, 384, -133,
																																									FONT_NEXT, 326, -154,
																																									FONT_NEXT, 248, -161,
																																									FONT_NEXT, 176, -155,
																																									FONT_NEXT, 130, -138,
																																									FONT_NEXT, 105, -115,
																																									FONT_END, 98, -90,
																																									FONT_BEGIN, 158, 384,
																																									FONT_NEXT, 176, 412,
																																									FONT_NEXT, 200, 427,
																																									FONT_NEXT, 227, 432,
																																									FONT_NEXT, 256, 426,
																																									FONT_NEXT, 279, 410,
																																									FONT_NEXT, 310, 361,
																																									FONT_NEXT, 325, 307,
																																									FONT_NEXT, 329, 270,
																																									FONT_NEXT, 327, 245,
																																									FONT_NEXT, 318, 213,
																																									FONT_NEXT, 296, 185,
																																									FONT_NEXT, 256, 174,
																																									FONT_NEXT, 229, 177,
																																									FONT_NEXT, 206, 189,
																																									FONT_NEXT, 174, 228,
																																									FONT_NEXT, 157, 282,
																																									FONT_END, 152, 342,
																																									FONT_BEGIN, 470, 427,
																																									FONT_NEXT, 393, 427,
																																									FONT_NEXT, 338, 437,
																																									FONT_NEXT, 287, 454,
																																									FONT_NEXT, 241, 460,
																																									FONT_NEXT, 182, 450,
																																									FONT_NEXT, 126, 421,
																																									FONT_NEXT, 85, 369,
																																									FONT_NEXT, 73, 334,
																																									FONT_NEXT, 69, 293,
																																									FONT_NEXT, 75, 251,
																																									FONT_NEXT, 93, 216,
																																									FONT_NEXT, 122, 186,
																																									FONT_NEXT, 162, 163,
																																									FONT_NEXT, 132, 136,
																																									FONT_NEXT, 110, 115,
																																									FONT_NEXT, 84, 85,
																																									FONT_NEXT, 74, 67,
																																									FONT_NEXT, 73, 54,
																																									FONT_NEXT, 77, 35,
																																									FONT_NEXT, 89, 21,
																																									FONT_NEXT, 126, 1,
																																									FONT_NEXT, 57, -57,
																																									FONT_NEXT, 36, -89,
																																									FONT_NEXT, 28, -121,
																																									FONT_NEXT, 41, -159,
																																									FONT_NEXT, 78, -190,
																																									FONT_NEXT, 134, -211,
																																									FONT_NEXT, 205, -218,
																																									FONT_NEXT, 282, -208,
																																									FONT_NEXT, 366, -176,
																																									FONT_NEXT, 403, -153,
																																									FONT_NEXT, 433, -124,
																																									FONT_NEXT, 453, -89,
																																									FONT_NEXT, 461, -49,
																																									FONT_NEXT, 457, -22,
																																									FONT_NEXT, 440, 13,
																																									FONT_NEXT, 404, 43,
																																									FONT_NEXT, 375, 53,
																																									FONT_NEXT, 340, 58,
																																									FONT_NEXT, 211, 64,
																																									FONT_NEXT, 160, 69,
																																									FONT_NEXT, 140, 77,
																																									FONT_NEXT, 133, 91,
																																									FONT_NEXT, 139, 109,
																																									FONT_NEXT, 155, 129,
																																									FONT_NEXT, 175, 146,
																																									FONT_NEXT, 193, 154,
																																									FONT_NEXT, 247, 149,
																																									FONT_NEXT, 295, 156,
																																									FONT_NEXT, 347, 182,
																																									FONT_NEXT, 389, 227,
																																									FONT_NEXT, 401, 258,
																																									FONT_NEXT, 406, 296,
																																									FONT_NEXT, 404, 328,
																																									FONT_NEXT, 400, 352,
																																									FONT_NEXT, 387, 388,
																																									FONT_END, 470, 388,
																																									FONT_ADVANCE, 500, 0
																																							},
																																								{
																																									104,
																																										FONT_BEGIN, 456, 22,
																																										FONT_NEXT, 438, 34,
																																										FONT_NEXT, 429, 58,
																																										FONT_NEXT, 427, 102,
																																										FONT_NEXT, 427, 301,
																																										FONT_NEXT, 424, 342,
																																										FONT_NEXT, 411, 395,
																																										FONT_NEXT, 396, 420,
																																										FONT_NEXT, 374, 440,
																																										FONT_NEXT, 343, 454,
																																										FONT_NEXT, 303, 460,
																																										FONT_NEXT, 260, 454,
																																										FONT_NEXT, 221, 437,
																																										FONT_NEXT, 187, 411,
																																										FONT_NEXT, 159, 378,
																																										FONT_NEXT, 157, 378,
																																										FONT_NEXT, 157, 680,
																																										FONT_NEXT, 152, 683,
																																										FONT_NEXT, 10, 639,
																																										FONT_NEXT, 10, 623,
																																										FONT_NEXT, 29, 624,
																																										FONT_NEXT, 54, 620,
																																										FONT_NEXT, 67, 609,
																																										FONT_NEXT, 73, 573,
																																										FONT_NEXT, 73, 102,
																																										FONT_NEXT, 70, 58,
																																										FONT_NEXT, 61, 33,
																																										FONT_NEXT, 41, 21,
																																										FONT_NEXT, 9, 15,
																																										FONT_NEXT, 9, 0,
																																										FONT_NEXT, 225, 0,
																																										FONT_NEXT, 225, 15,
																																										FONT_NEXT, 193, 19,
																																										FONT_NEXT, 172, 31,
																																										FONT_NEXT, 160, 56,
																																										FONT_NEXT, 157, 102,
																																										FONT_NEXT, 157, 343,
																																										FONT_NEXT, 187, 373,
																																										FONT_NEXT, 216, 392,
																																										FONT_NEXT, 266, 406,
																																										FONT_NEXT, 301, 398,
																																										FONT_NEXT, 325, 378,
																																										FONT_NEXT, 338, 344,
																																										FONT_NEXT, 343, 300,
																																										FONT_NEXT, 343, 102,
																																										FONT_NEXT, 339, 56,
																																										FONT_NEXT, 327, 31,
																																										FONT_NEXT, 306, 19,
																																										FONT_NEXT, 275, 15,
																																										FONT_NEXT, 275, 0,
																																										FONT_NEXT, 487, 0,
																																										FONT_END, 487, 15,
																																										FONT_ADVANCE, 500, 0
																																								},
																																								{
																																									105,
																																										FONT_BEGIN, 253, 0,
																																										FONT_NEXT, 253, 15,
																																										FONT_NEXT, 212, 21,
																																										FONT_NEXT, 190, 35,
																																										FONT_NEXT, 180, 61,
																																										FONT_NEXT, 179, 102,
																																										FONT_NEXT, 179, 457,
																																										FONT_NEXT, 175, 460,
																																										FONT_NEXT, 20, 405,
																																										FONT_NEXT, 20, 390,
																																										FONT_NEXT, 60, 394,
																																										FONT_NEXT, 83, 386,
																																										FONT_NEXT, 91, 368,
																																										FONT_NEXT, 95, 334,
																																										FONT_NEXT, 95, 102,
																																										FONT_NEXT, 91, 55,
																																										FONT_NEXT, 78, 30,
																																										FONT_NEXT, 54, 18,
																																										FONT_NEXT, 16, 15,
																																										FONT_END, 16, 0,
																																										FONT_BEGIN, 89, 599,
																																										FONT_NEXT, 105, 586,
																																										FONT_NEXT, 128, 581,
																																										FONT_NEXT, 148, 584,
																																										FONT_NEXT, 165, 595,
																																										FONT_NEXT, 176, 611,
																																										FONT_NEXT, 180, 632,
																																										FONT_NEXT, 165, 668,
																																										FONT_NEXT, 148, 678,
																																										FONT_NEXT, 128, 683,
																																										FONT_NEXT, 105, 677,
																																										FONT_NEXT, 89, 664,
																																										FONT_END, 78, 632,
																																										FONT_ADVANCE, 278, 0
																																								},
																																									{
																																										106,
																																											FONT_BEGIN, 108, -102,
																																											FONT_NEXT, 102, -146,
																																											FONT_NEXT, 87, -174,
																																											FONT_NEXT, 59, -184,
																																											FONT_NEXT, -30, -124,
																																											FONT_NEXT, -58, -135,
																																											FONT_NEXT, -70, -162,
																																											FONT_NEXT, -63, -187,
																																											FONT_NEXT, -42, -204,
																																											FONT_NEXT, -12, -215,
																																											FONT_NEXT, 23, -218,
																																											FONT_NEXT, 67, -214,
																																											FONT_NEXT, 103, -201,
																																											FONT_NEXT, 133, -180,
																																											FONT_NEXT, 156, -153,
																																											FONT_NEXT, 184, -84,
																																											FONT_NEXT, 193, 0,
																																											FONT_NEXT, 193, 457,
																																											FONT_NEXT, 188, 460,
																																											FONT_NEXT, 32, 406,
																																											FONT_NEXT, 32, 390,
																																											FONT_NEXT, 74, 394,
																																											FONT_NEXT, 97, 386,
																																											FONT_NEXT, 105, 368,
																																											FONT_NEXT, 109, 334,
																																											FONT_END, 109, -45,
																																											FONT_BEGIN, 103, 599,
																																											FONT_NEXT, 119, 586,
																																											FONT_NEXT, 142, 581,
																																											FONT_NEXT, 162, 584,
																																											FONT_NEXT, 179, 595,
																																											FONT_NEXT, 190, 611,
																																											FONT_NEXT, 194, 632,
																																											FONT_NEXT, 179, 668,
																																											FONT_NEXT, 162, 678,
																																											FONT_NEXT, 142, 683,
																																											FONT_NEXT, 119, 677,
																																											FONT_NEXT, 103, 664,
																																											FONT_END, 92, 632,
																																											FONT_ADVANCE, 278, 0
																																									},
																																									{
																																										107,
																																											FONT_BEGIN, 241, 0,
																																											FONT_NEXT, 241, 15,
																																											FONT_NEXT, 221, 16,
																																											FONT_NEXT, 181, 26,
																																											FONT_NEXT, 169, 42,
																																											FONT_NEXT, 166, 67,
																																											FONT_NEXT, 166, 248,
																																											FONT_NEXT, 168, 248,
																																											FONT_NEXT, 306, 64,
																																											FONT_NEXT, 321, 43,
																																											FONT_NEXT, 327, 31,
																																											FONT_NEXT, 319, 18,
																																											FONT_NEXT, 306, 15,
																																											FONT_NEXT, 287, 15,
																																											FONT_NEXT, 287, 0,
																																											FONT_NEXT, 505, 0,
																																											FONT_NEXT, 505, 15,
																																											FONT_NEXT, 482, 18,
																																											FONT_NEXT, 455, 28,
																																											FONT_NEXT, 424, 49,
																																											FONT_NEXT, 388, 88,
																																											FONT_NEXT, 235, 282,
																																											FONT_NEXT, 264, 309,
																																											FONT_NEXT, 309, 350,
																																											FONT_NEXT, 347, 381,
																																											FONT_NEXT, 377, 404,
																																											FONT_NEXT, 402, 419,
																																											FONT_NEXT, 442, 433,
																																											FONT_NEXT, 480, 435,
																																											FONT_NEXT, 480, 450,
																																											FONT_NEXT, 276, 450,
																																											FONT_NEXT, 276, 436,
																																											FONT_NEXT, 301, 434,
																																											FONT_NEXT, 316, 430,
																																											FONT_NEXT, 326, 417,
																																											FONT_NEXT, 320, 401,
																																											FONT_NEXT, 303, 383,
																																											FONT_NEXT, 168, 263,
																																											FONT_NEXT, 166, 265,
																																											FONT_NEXT, 166, 681,
																																											FONT_NEXT, 162, 683,
																																											FONT_NEXT, 7, 639,
																																											FONT_NEXT, 7, 623,
																																											FONT_NEXT, 37, 625,
																																											FONT_NEXT, 60, 621,
																																											FONT_NEXT, 74, 611,
																																											FONT_NEXT, 80, 592,
																																											FONT_NEXT, 82, 564,
																																											FONT_NEXT, 82, 82,
																																											FONT_NEXT, 79, 50,
																																											FONT_NEXT, 69, 32,
																																											FONT_NEXT, 46, 22,
																																											FONT_NEXT, 7, 15,
																																											FONT_END, 7, 0,
																																											FONT_ADVANCE, 500, 0
																																									},
																																										{
																																											108,
																																												FONT_BEGIN, 257, 0,
																																												FONT_NEXT, 257, 15,
																																												FONT_NEXT, 220, 18,
																																												FONT_NEXT, 197, 28,
																																												FONT_NEXT, 185, 49,
																																												FONT_NEXT, 182, 84,
																																												FONT_NEXT, 182, 681,
																																												FONT_NEXT, 178, 683,
																																												FONT_NEXT, 19, 639,
																																												FONT_NEXT, 19, 623,
																																												FONT_NEXT, 53, 625,
																																												FONT_NEXT, 76, 621,
																																												FONT_NEXT, 90, 611,
																																												FONT_NEXT, 96, 592,
																																												FONT_NEXT, 98, 564,
																																												FONT_NEXT, 98, 87,
																																												FONT_NEXT, 94, 53,
																																												FONT_NEXT, 81, 31,
																																												FONT_NEXT, 57, 20,
																																												FONT_NEXT, 21, 15,
																																												FONT_END, 21, 0,
																																												FONT_ADVANCE, 278, 0
																																										},
																																										{
																																											109,
																																												FONT_BEGIN, 80, 45,
																																												FONT_NEXT, 66, 25,
																																												FONT_NEXT, 43, 16,
																																												FONT_NEXT, 16, 15,
																																												FONT_NEXT, 16, 0,
																																												FONT_NEXT, 238, 0,
																																												FONT_NEXT, 238, 15,
																																												FONT_NEXT, 190, 23,
																																												FONT_NEXT, 175, 38,
																																												FONT_NEXT, 170, 67,
																																												FONT_NEXT, 170, 349,
																																												FONT_NEXT, 178, 360,
																																												FONT_NEXT, 201, 380,
																																												FONT_NEXT, 237, 399,
																																												FONT_NEXT, 285, 408,
																																												FONT_NEXT, 317, 400,
																																												FONT_NEXT, 339, 380,
																																												FONT_NEXT, 350, 347,
																																												FONT_NEXT, 354, 303,
																																												FONT_NEXT, 354, 86,
																																												FONT_NEXT, 349, 47,
																																												FONT_NEXT, 336, 27,
																																												FONT_NEXT, 315, 18,
																																												FONT_NEXT, 286, 15,
																																												FONT_NEXT, 286, 0,
																																												FONT_NEXT, 510, 0,
																																												FONT_NEXT, 510, 15,
																																												FONT_NEXT, 481, 17,
																																												FONT_NEXT, 458, 26,
																																												FONT_NEXT, 443, 50,
																																												FONT_NEXT, 438, 95,
																																												FONT_NEXT, 438, 347,
																																												FONT_NEXT, 481, 390,
																																												FONT_NEXT, 511, 403,
																																												FONT_NEXT, 549, 408,
																																												FONT_NEXT, 590, 397,
																																												FONT_NEXT, 612, 372,
																																												FONT_NEXT, 622, 298,
																																												FONT_NEXT, 622, 87,
																																												FONT_NEXT, 618, 49,
																																												FONT_NEXT, 607, 29,
																																												FONT_NEXT, 587, 19,
																																												FONT_NEXT, 556, 15,
																																												FONT_NEXT, 556, 0,
																																												FONT_NEXT, 775, 0,
																																												FONT_NEXT, 775, 15,
																																												FONT_NEXT, 749, 17,
																																												FONT_NEXT, 723, 26,
																																												FONT_NEXT, 711, 40,
																																												FONT_NEXT, 706, 76,
																																												FONT_NEXT, 706, 282,
																																												FONT_NEXT, 702, 341,
																																												FONT_NEXT, 686, 398,
																																												FONT_NEXT, 650, 442,
																																												FONT_NEXT, 622, 455,
																																												FONT_NEXT, 588, 460,
																																												FONT_NEXT, 532, 450,
																																												FONT_NEXT, 485, 427,
																																												FONT_NEXT, 449, 399,
																																												FONT_NEXT, 427, 376,
																																												FONT_NEXT, 414, 406,
																																												FONT_NEXT, 393, 433,
																																												FONT_NEXT, 362, 452,
																																												FONT_NEXT, 320, 460,
																																												FONT_NEXT, 281, 454,
																																												FONT_NEXT, 244, 438,
																																												FONT_NEXT, 166, 383,
																																												FONT_NEXT, 166, 458,
																																												FONT_NEXT, 159, 460,
																																												FONT_NEXT, 19, 415,
																																												FONT_NEXT, 19, 398,
																																												FONT_NEXT, 51, 402,
																																												FONT_NEXT, 74, 393,
																																												FONT_NEXT, 82, 374,
																																												FONT_NEXT, 86, 338,
																																												FONT_END, 86, 85,
																																												FONT_ADVANCE, 778, 0
																																										},
																																											{
																																												110,
																																													FONT_BEGIN, 76, 53,
																																													FONT_NEXT, 66, 31,
																																													FONT_NEXT, 47, 19,
																																													FONT_NEXT, 18, 15,
																																													FONT_NEXT, 18, 0,
																																													FONT_NEXT, 230, 0,
																																													FONT_NEXT, 230, 15,
																																													FONT_NEXT, 197, 19,
																																													FONT_NEXT, 177, 29,
																																													FONT_NEXT, 167, 45,
																																													FONT_NEXT, 164, 67,
																																													FONT_NEXT, 164, 348,
																																													FONT_NEXT, 216, 390,
																																													FONT_NEXT, 263, 405,
																																													FONT_NEXT, 299, 398,
																																													FONT_NEXT, 323, 379,
																																													FONT_NEXT, 336, 348,
																																													FONT_NEXT, 340, 308,
																																													FONT_NEXT, 340, 99,
																																													FONT_NEXT, 335, 54,
																																													FONT_NEXT, 322, 29,
																																													FONT_NEXT, 302, 18,
																																													FONT_NEXT, 277, 15,
																																													FONT_NEXT, 277, 0,
																																													FONT_NEXT, 485, 0,
																																													FONT_NEXT, 485, 15,
																																													FONT_NEXT, 454, 20,
																																													FONT_NEXT, 436, 31,
																																													FONT_NEXT, 426, 51,
																																													FONT_NEXT, 424, 81,
																																													FONT_NEXT, 424, 310,
																																													FONT_NEXT, 420, 351,
																																													FONT_NEXT, 412, 385,
																																													FONT_NEXT, 382, 431,
																																													FONT_NEXT, 344, 453,
																																													FONT_NEXT, 307, 460,
																																													FONT_NEXT, 266, 453,
																																													FONT_NEXT, 230, 436,
																																													FONT_NEXT, 161, 379,
																																													FONT_NEXT, 161, 458,
																																													FONT_NEXT, 154, 460,
																																													FONT_NEXT, 16, 415,
																																													FONT_NEXT, 16, 398,
																																													FONT_NEXT, 45, 402,
																																													FONT_NEXT, 68, 393,
																																													FONT_NEXT, 76, 374,
																																													FONT_NEXT, 80, 338,
																																													FONT_END, 80, 90,
																																													FONT_ADVANCE, 500, 0
																																											},
																																											{
																																												111,
																																													FONT_BEGIN, 197, 455,
																																													FONT_NEXT, 154, 441,
																																													FONT_NEXT, 117, 419,
																																													FONT_NEXT, 86, 390,
																																													FONT_NEXT, 62, 355,
																																													FONT_NEXT, 43, 316,
																																													FONT_NEXT, 29, 228,
																																													FONT_NEXT, 33, 179,
																																													FONT_NEXT, 45, 133,
																																													FONT_NEXT, 64, 93,
																																													FONT_NEXT, 90, 58,
																																													FONT_NEXT, 122, 29,
																																													FONT_NEXT, 159, 8,
																																													FONT_NEXT, 201, -6,
																																													FONT_NEXT, 248, -10,
																																													FONT_NEXT, 301, -4,
																																													FONT_NEXT, 346, 13,
																																													FONT_NEXT, 385, 39,
																																													FONT_NEXT, 416, 71,
																																													FONT_NEXT, 456, 149,
																																													FONT_NEXT, 470, 228,
																																													FONT_NEXT, 465, 279,
																																													FONT_NEXT, 453, 325,
																																													FONT_NEXT, 433, 365,
																																													FONT_NEXT, 406, 398,
																																													FONT_NEXT, 373, 424,
																																													FONT_NEXT, 335, 444,
																																													FONT_NEXT, 292, 455,
																																													FONT_END, 245, 460,
																																													FONT_BEGIN, 221, 26,
																																													FONT_NEXT, 189, 47,
																																													FONT_NEXT, 164, 80,
																																													FONT_NEXT, 145, 119,
																																													FONT_NEXT, 124, 204,
																																													FONT_NEXT, 119, 276,
																																													FONT_NEXT, 128, 344,
																																													FONT_NEXT, 154, 393,
																																													FONT_NEXT, 191, 422,
																																													FONT_NEXT, 237, 432,
																																													FONT_NEXT, 269, 427,
																																													FONT_NEXT, 298, 413,
																																													FONT_NEXT, 322, 390,
																																													FONT_NEXT, 343, 362,
																																													FONT_NEXT, 370, 287,
																																													FONT_NEXT, 380, 199,
																																													FONT_NEXT, 377, 150,
																																													FONT_NEXT, 368, 110,
																																													FONT_NEXT, 356, 78,
																																													FONT_NEXT, 340, 54,
																																													FONT_NEXT, 301, 26,
																																													FONT_END, 262, 18,
																																													FONT_ADVANCE, 500, 0
																																											},
																																												{
																																													112,
																																														FONT_BEGIN, 73, -160,
																																														FONT_NEXT, 64, -180,
																																														FONT_NEXT, 43, -194,
																																														FONT_NEXT, 5, -200,
																																														FONT_NEXT, 5, -217,
																																														FONT_NEXT, 247, -217,
																																														FONT_NEXT, 247, -199,
																																														FONT_NEXT, 202, -197,
																																														FONT_NEXT, 176, -186,
																																														FONT_NEXT, 162, -163,
																																														FONT_NEXT, 159, -124,
																																														FONT_NEXT, 159, 33,
																																														FONT_NEXT, 206, -1,
																																														FONT_NEXT, 260, -10,
																																														FONT_NEXT, 308, -4,
																																														FONT_NEXT, 351, 13,
																																														FONT_NEXT, 387, 40,
																																														FONT_NEXT, 417, 74,
																																														FONT_NEXT, 456, 156,
																																														FONT_NEXT, 470, 245,
																																														FONT_NEXT, 466, 292,
																																														FONT_NEXT, 456, 334,
																																														FONT_NEXT, 420, 402,
																																														FONT_NEXT, 368, 445,
																																														FONT_NEXT, 305, 460,
																																														FONT_NEXT, 252, 451,
																																														FONT_NEXT, 212, 432,
																																														FONT_NEXT, 181, 406,
																																														FONT_NEXT, 161, 383,
																																														FONT_NEXT, 159, 385,
																																														FONT_NEXT, 159, 458,
																																														FONT_NEXT, 153, 460,
																																														FONT_NEXT, 9, 409,
																																														FONT_NEXT, 9, 393,
																																														FONT_NEXT, 37, 394,
																																														FONT_NEXT, 57, 390,
																																														FONT_NEXT, 69, 380,
																																														FONT_NEXT, 73, 362,
																																														FONT_NEXT, 75, 337,
																																														FONT_END, 75, -131,
																																														FONT_BEGIN, 171, 360,
																																														FONT_NEXT, 196, 381,
																																														FONT_NEXT, 227, 395,
																																														FONT_NEXT, 257, 400,
																																														FONT_NEXT, 297, 392,
																																														FONT_NEXT, 328, 374,
																																														FONT_NEXT, 350, 347,
																																														FONT_NEXT, 366, 316,
																																														FONT_NEXT, 381, 253,
																																														FONT_NEXT, 383, 229,
																																														FONT_NEXT, 384, 215,
																																														FONT_NEXT, 380, 156,
																																														FONT_NEXT, 369, 111,
																																														FONT_NEXT, 353, 77,
																																														FONT_NEXT, 334, 53,
																																														FONT_NEXT, 294, 27,
																																														FONT_NEXT, 261, 22,
																																														FONT_NEXT, 221, 29,
																																														FONT_NEXT, 189, 46,
																																														FONT_NEXT, 167, 67,
																																														FONT_NEXT, 159, 88,
																																														FONT_END, 159, 334,
																																														FONT_ADVANCE, 500, 0
																																												},
																																												{
																																													113,
																																														FONT_BEGIN, 308, 450,
																																														FONT_NEXT, 247, 460,
																																														FONT_NEXT, 201, 455,
																																														FONT_NEXT, 159, 440,
																																														FONT_NEXT, 121, 417,
																																														FONT_NEXT, 88, 387,
																																														FONT_NEXT, 61, 351,
																																														FONT_NEXT, 41, 308,
																																														FONT_NEXT, 28, 262,
																																														FONT_NEXT, 24, 212,
																																														FONT_NEXT, 27, 159,
																																														FONT_NEXT, 37, 113,
																																														FONT_NEXT, 52, 75,
																																														FONT_NEXT, 72, 44,
																																														FONT_NEXT, 124, 3,
																																														FONT_NEXT, 188, -10,
																																														FONT_NEXT, 234, -3,
																																														FONT_NEXT, 278, 14,
																																														FONT_NEXT, 314, 38,
																																														FONT_NEXT, 336, 62,
																																														FONT_NEXT, 341, 62,
																																														FONT_NEXT, 341, -124,
																																														FONT_NEXT, 337, -157,
																																														FONT_NEXT, 324, -180,
																																														FONT_NEXT, 297, -194,
																																														FONT_NEXT, 252, -200,
																																														FONT_NEXT, 252, -217,
																																														FONT_NEXT, 488, -217,
																																														FONT_NEXT, 488, -203,
																																														FONT_NEXT, 459, -196,
																																														FONT_NEXT, 439, -187,
																																														FONT_NEXT, 428, -170,
																																														FONT_NEXT, 425, -141,
																																														FONT_NEXT, 425, 456,
																																														FONT_NEXT, 421, 460,
																																														FONT_NEXT, 414, 457,
																																														FONT_END, 360, 425,
																																														FONT_BEGIN, 333, 88,
																																														FONT_NEXT, 318, 73,
																																														FONT_NEXT, 288, 60,
																																														FONT_NEXT, 238, 51,
																																														FONT_NEXT, 206, 55,
																																														FONT_NEXT, 180, 67,
																																														FONT_NEXT, 140, 110,
																																														FONT_NEXT, 117, 169,
																																														FONT_NEXT, 110, 238,
																																														FONT_NEXT, 119, 319,
																																														FONT_NEXT, 147, 380,
																																														FONT_NEXT, 189, 418,
																																														FONT_NEXT, 245, 432,
																																														FONT_NEXT, 276, 428,
																																														FONT_NEXT, 300, 420,
																																														FONT_NEXT, 328, 393,
																																														FONT_NEXT, 339, 360,
																																														FONT_NEXT, 341, 333,
																																														FONT_END, 341, 127,
																																														FONT_ADVANCE, 500, 0
																																												},
																																													{
																																														114,
																																															FONT_BEGIN, 72, 51,
																																															FONT_NEXT, 61, 32,
																																															FONT_NEXT, 39, 22,
																																															FONT_NEXT, 5, 15,
																																															FONT_NEXT, 5, 0,
																																															FONT_NEXT, 245, 0,
																																															FONT_NEXT, 245, 15,
																																															FONT_NEXT, 210, 17,
																																															FONT_NEXT, 183, 27,
																																															FONT_NEXT, 166, 49,
																																															FONT_NEXT, 160, 90,
																																															FONT_NEXT, 160, 315,
																																															FONT_NEXT, 167, 341,
																																															FONT_NEXT, 185, 368,
																																															FONT_NEXT, 207, 388,
																																															FONT_NEXT, 228, 397,
																																															FONT_NEXT, 258, 379,
																																															FONT_NEXT, 274, 367,
																																															FONT_NEXT, 297, 362,
																																															FONT_NEXT, 325, 374,
																																															FONT_NEXT, 335, 406,
																																															FONT_NEXT, 331, 428,
																																															FONT_NEXT, 320, 445,
																																															FONT_NEXT, 303, 456,
																																															FONT_NEXT, 281, 460,
																																															FONT_NEXT, 252, 455,
																																															FONT_NEXT, 224, 441,
																																															FONT_NEXT, 195, 413,
																																															FONT_NEXT, 162, 369,
																																															FONT_NEXT, 160, 369,
																																															FONT_NEXT, 160, 458,
																																															FONT_NEXT, 155, 460,
																																															FONT_NEXT, 7, 406,
																																															FONT_NEXT, 7, 390,
																																															FONT_NEXT, 41, 394,
																																															FONT_NEXT, 64, 386,
																																															FONT_NEXT, 72, 368,
																																															FONT_NEXT, 76, 334,
																																															FONT_END, 76, 84,
																																															FONT_ADVANCE, 333, 0
																																													},
																																													{
																																														115,
																																															FONT_BEGIN, 311, 451,
																																															FONT_NEXT, 300, 451,
																																															FONT_NEXT, 283, 440,
																																															FONT_NEXT, 247, 450,
																																															FONT_NEXT, 191, 460,
																																															FONT_NEXT, 128, 448,
																																															FONT_NEXT, 84, 420,
																																															FONT_NEXT, 59, 380,
																																															FONT_NEXT, 51, 338,
																																															FONT_NEXT, 60, 288,
																																															FONT_NEXT, 86, 250,
																																															FONT_NEXT, 123, 219,
																																															FONT_NEXT, 168, 192,
																																															FONT_NEXT, 226, 159,
																																															FONT_NEXT, 264, 127,
																																															FONT_NEXT, 278, 85,
																																															FONT_NEXT, 270, 49,
																																															FONT_NEXT, 250, 27,
																																															FONT_NEXT, 224, 15,
																																															FONT_NEXT, 197, 12,
																																															FONT_NEXT, 163, 15,
																																															FONT_NEXT, 137, 26,
																																															FONT_NEXT, 100, 62,
																																															FONT_NEXT, 79, 107,
																																															FONT_NEXT, 68, 152,
																																															FONT_NEXT, 52, 152,
																																															FONT_NEXT, 52, -4,
																																															FONT_NEXT, 65, -4,
																																															FONT_NEXT, 73, 5,
																																															FONT_NEXT, 87, 8,
																																															FONT_NEXT, 138, -1,
																																															FONT_NEXT, 203, -10,
																																															FONT_NEXT, 252, -3,
																																															FONT_NEXT, 299, 21,
																																															FONT_NEXT, 334, 62,
																																															FONT_NEXT, 348, 119,
																																															FONT_NEXT, 341, 158,
																																															FONT_NEXT, 324, 189,
																																															FONT_NEXT, 264, 237,
																																															FONT_NEXT, 156, 301,
																																															FONT_NEXT, 133, 318,
																																															FONT_NEXT, 120, 337,
																																															FONT_NEXT, 113, 372,
																																															FONT_NEXT, 117, 393,
																																															FONT_NEXT, 130, 414,
																																															FONT_NEXT, 153, 430,
																																															FONT_NEXT, 189, 437,
																																															FONT_NEXT, 221, 432,
																																															FONT_NEXT, 252, 413,
																																															FONT_NEXT, 279, 376,
																																															FONT_NEXT, 300, 314,
																																															FONT_END, 315, 314,
																																															FONT_ADVANCE, 389, 0
																																													},
																																														{
																																															116,
																																																FONT_BEGIN, 154, 450,
																																																FONT_NEXT, 154, 566,
																																																FONT_NEXT, 147, 579,
																																																FONT_NEXT, 122, 544,
																																																FONT_NEXT, 102, 516,
																																																FONT_NEXT, 57, 465,
																																																FONT_NEXT, 26, 441,
																																																FONT_NEXT, 13, 425,
																																																FONT_NEXT, 17, 418,
																																																FONT_NEXT, 70, 418,
																																																FONT_NEXT, 70, 117,
																																																FONT_NEXT, 77, 54,
																																																FONT_NEXT, 96, 16,
																																																FONT_NEXT, 126, -5,
																																																FONT_NEXT, 162, -10,
																																																FONT_NEXT, 196, -5,
																																																FONT_NEXT, 228, 11,
																																																FONT_NEXT, 257, 35,
																																																FONT_NEXT, 279, 66,
																																																FONT_NEXT, 266, 77,
																																																FONT_NEXT, 243, 54,
																																																FONT_NEXT, 205, 42,
																																																FONT_NEXT, 175, 51,
																																																FONT_NEXT, 160, 75,
																																																FONT_NEXT, 154, 132,
																																																FONT_NEXT, 154, 418,
																																																FONT_NEXT, 255, 418,
																																																FONT_END, 255, 450,
																																																FONT_ADVANCE, 278, 0
																																														},
																																														{
																																															117,
																																																FONT_BEGIN, 444, 53,
																																																FONT_NEXT, 425, 64,
																																																FONT_NEXT, 418, 82,
																																																FONT_NEXT, 417, 107,
																																																FONT_NEXT, 417, 450,
																																																FONT_NEXT, 259, 450,
																																																FONT_NEXT, 259, 433,
																																																FONT_NEXT, 300, 427,
																																																FONT_NEXT, 322, 415,
																																																FONT_NEXT, 331, 396,
																																																FONT_NEXT, 333, 370,
																																																FONT_NEXT, 333, 135,
																																																FONT_NEXT, 328, 100,
																																																FONT_NEXT, 317, 84,
																																																FONT_NEXT, 274, 57,
																																																FONT_NEXT, 230, 48,
																																																FONT_NEXT, 190, 57,
																																																FONT_NEXT, 167, 80,
																																																FONT_NEXT, 157, 106,
																																																FONT_NEXT, 155, 124,
																																																FONT_NEXT, 155, 450,
																																																FONT_NEXT, 9, 450,
																																																FONT_NEXT, 9, 436,
																																																FONT_NEXT, 51, 425,
																																																FONT_NEXT, 65, 407,
																																																FONT_NEXT, 71, 372,
																																																FONT_NEXT, 71, 120,
																																																FONT_NEXT, 74, 83,
																																																FONT_NEXT, 83, 54,
																																																FONT_NEXT, 115, 15,
																																																FONT_NEXT, 154, -5,
																																																FONT_NEXT, 190, -10,
																																																FONT_NEXT, 208, -9,
																																																FONT_NEXT, 234, -4,
																																																FONT_NEXT, 264, 9,
																																																FONT_NEXT, 295, 33,
																																																FONT_NEXT, 338, 76,
																																																FONT_NEXT, 338, -7,
																																																FONT_NEXT, 340, -9,
																																																FONT_NEXT, 479, 36,
																																																FONT_END, 479, 50,
																																																FONT_ADVANCE, 500, 0
																																														},
																																															{
																																																118,
																																																	FONT_BEGIN, 338, 450,
																																																	FONT_NEXT, 338, 435,
																																																	FONT_NEXT, 370, 426,
																																																	FONT_NEXT, 385, 402,
																																																	FONT_NEXT, 381, 383,
																																																	FONT_NEXT, 373, 355,
																																																	FONT_NEXT, 345, 280,
																																																	FONT_NEXT, 280, 114,
																																																	FONT_NEXT, 178, 370,
																																																	FONT_NEXT, 169, 405,
																																																	FONT_NEXT, 172, 419,
																																																	FONT_NEXT, 183, 428,
																																																	FONT_NEXT, 215, 435,
																																																	FONT_NEXT, 215, 450,
																																																	FONT_NEXT, 19, 450,
																																																	FONT_NEXT, 19, 435,
																																																	FONT_NEXT, 48, 427,
																																																	FONT_NEXT, 74, 398,
																																																	FONT_NEXT, 90, 364,
																																																	FONT_NEXT, 110, 319,
																																																	FONT_NEXT, 156, 212,
																																																	FONT_NEXT, 200, 106,
																																																	FONT_NEXT, 217, 63,
																																																	FONT_NEXT, 230, 33,
																																																	FONT_NEXT, 239, 7,
																																																	FONT_NEXT, 246, -7,
																																																	FONT_NEXT, 257, -14,
																																																	FONT_NEXT, 269, 0,
																																																	FONT_NEXT, 284, 36,
																																																	FONT_NEXT, 412, 357,
																																																	FONT_NEXT, 427, 393,
																																																	FONT_NEXT, 440, 417,
																																																	FONT_NEXT, 455, 429,
																																																	FONT_NEXT, 477, 435,
																																																	FONT_END, 477, 450,
																																																	FONT_ADVANCE, 500, 0
																																															},
																																															{
																																																119,
																																																	FONT_BEGIN, 571, 435,
																																																	FONT_NEXT, 602, 425,
																																																	FONT_NEXT, 611, 416,
																																																	FONT_NEXT, 615, 402,
																																																	FONT_NEXT, 610, 374,
																																																	FONT_NEXT, 598, 338,
																																																	FONT_NEXT, 508, 116,
																																																	FONT_NEXT, 428, 330,
																																																	FONT_NEXT, 411, 371,
																																																	FONT_NEXT, 407, 398,
																																																	FONT_NEXT, 411, 415,
																																																	FONT_NEXT, 423, 425,
																																																	FONT_NEXT, 465, 435,
																																																	FONT_NEXT, 465, 450,
																																																	FONT_NEXT, 262, 450,
																																																	FONT_NEXT, 262, 435,
																																																	FONT_NEXT, 288, 429,
																																																	FONT_NEXT, 308, 413,
																																																	FONT_NEXT, 327, 376,
																																																	FONT_NEXT, 338, 347,
																																																	FONT_NEXT, 351, 310,
																																																	FONT_NEXT, 260, 111,
																																																	FONT_NEXT, 224, 205,
																																																	FONT_NEXT, 190, 290,
																																																	FONT_NEXT, 164, 358,
																																																	FONT_NEXT, 157, 383,
																																																	FONT_NEXT, 155, 401,
																																																	FONT_NEXT, 158, 417,
																																																	FONT_NEXT, 169, 427,
																																																	FONT_NEXT, 201, 435,
																																																	FONT_NEXT, 201, 450,
																																																	FONT_NEXT, 21, 450,
																																																	FONT_NEXT, 21, 435,
																																																	FONT_NEXT, 49, 420,
																																																	FONT_NEXT, 60, 401,
																																																	FONT_NEXT, 74, 372,
																																																	FONT_NEXT, 209, 30,
																																																	FONT_NEXT, 217, 8,
																																																	FONT_NEXT, 224, -5,
																																																	FONT_NEXT, 235, -14,
																																																	FONT_NEXT, 244, -5,
																																																	FONT_NEXT, 260, 25,
																																																	FONT_NEXT, 372, 265,
																																																	FONT_NEXT, 463, 29,
																																																	FONT_NEXT, 471, 7,
																																																	FONT_NEXT, 477, -6,
																																																	FONT_NEXT, 487, -14,
																																																	FONT_NEXT, 498, -5,
																																																	FONT_NEXT, 505, 10,
																																																	FONT_NEXT, 515, 35,
																																																	FONT_NEXT, 653, 381,
																																																	FONT_NEXT, 671, 419,
																																																	FONT_NEXT, 694, 435,
																																																	FONT_NEXT, 694, 450,
																																																	FONT_END, 571, 450,
																																																	FONT_ADVANCE, 722, 0
																																															},
																																																{
																																																	120,
																																																		FONT_BEGIN, 311, 60,
																																																		FONT_NEXT, 318, 41,
																																																		FONT_NEXT, 309, 20,
																																																		FONT_NEXT, 278, 15,
																																																		FONT_NEXT, 278, 0,
																																																		FONT_NEXT, 479, 0,
																																																		FONT_NEXT, 479, 15,
																																																		FONT_NEXT, 445, 23,
																																																		FONT_NEXT, 423, 41,
																																																		FONT_NEXT, 397, 75,
																																																		FONT_NEXT, 269, 271,
																																																		FONT_NEXT, 352, 391,
																																																		FONT_NEXT, 379, 417,
																																																		FONT_NEXT, 403, 429,
																																																		FONT_NEXT, 433, 435,
																																																		FONT_NEXT, 433, 450,
																																																		FONT_NEXT, 275, 450,
																																																		FONT_NEXT, 275, 435,
																																																		FONT_NEXT, 301, 431,
																																																		FONT_NEXT, 311, 424,
																																																		FONT_NEXT, 315, 412,
																																																		FONT_NEXT, 313, 402,
																																																		FONT_NEXT, 304, 386,
																																																		FONT_NEXT, 284, 356,
																																																		FONT_NEXT, 248, 304,
																																																		FONT_NEXT, 210, 362,
																																																		FONT_NEXT, 194, 391,
																																																		FONT_NEXT, 188, 412,
																																																		FONT_NEXT, 192, 424,
																																																		FONT_NEXT, 202, 431,
																																																		FONT_NEXT, 231, 435,
																																																		FONT_NEXT, 231, 450,
																																																		FONT_NEXT, 24, 450,
																																																		FONT_NEXT, 24, 435,
																																																		FONT_NEXT, 64, 428,
																																																		FONT_NEXT, 83, 410,
																																																		FONT_NEXT, 110, 375,
																																																		FONT_NEXT, 204, 231,
																																																		FONT_NEXT, 90, 66,
																																																		FONT_NEXT, 68, 39,
																																																		FONT_NEXT, 51, 24,
																																																		FONT_NEXT, 17, 15,
																																																		FONT_NEXT, 17, 0,
																																																		FONT_NEXT, 162, 0,
																																																		FONT_NEXT, 162, 15,
																																																		FONT_NEXT, 136, 17,
																																																		FONT_NEXT, 122, 33,
																																																		FONT_NEXT, 142, 74,
																																																		FONT_NEXT, 221, 197,
																																																		FONT_END, 302, 73,
																																																		FONT_ADVANCE, 500, 0
																																																},
																																																{
																																																	121,
																																																		FONT_BEGIN, 340, 450,
																																																		FONT_NEXT, 340, 435,
																																																		FONT_NEXT, 376, 428,
																																																		FONT_NEXT, 388, 407,
																																																		FONT_NEXT, 380, 379,
																																																		FONT_NEXT, 365, 339,
																																																		FONT_NEXT, 287, 117,
																																																		FONT_NEXT, 172, 370,
																																																		FONT_NEXT, 162, 406,
																																																		FONT_NEXT, 167, 421,
																																																		FONT_NEXT, 182, 430,
																																																		FONT_NEXT, 220, 435,
																																																		FONT_NEXT, 220, 450,
																																																		FONT_NEXT, 14, 450,
																																																		FONT_NEXT, 14, 436,
																																																		FONT_NEXT, 39, 428,
																																																		FONT_NEXT, 65, 404,
																																																		FONT_NEXT, 179, 158,
																																																		FONT_NEXT, 194, 125,
																																																		FONT_NEXT, 215, 83,
																																																		FONT_NEXT, 233, 43,
																																																		FONT_NEXT, 241, 20,
																																																		FONT_NEXT, 239, 8,
																																																		FONT_NEXT, 233, -11,
																																																		FONT_NEXT, 214, -63,
																																																		FONT_NEXT, 185, -113,
																																																		FONT_NEXT, 167, -129,
																																																		FONT_NEXT, 149, -134,
																																																		FONT_NEXT, 118, -125,
																																																		FONT_NEXT, 78, -116,
																																																		FONT_NEXT, 51, -123,
																																																		FONT_NEXT, 36, -136,
																																																		FONT_NEXT, 30, -160,
																																																		FONT_NEXT, 34, -181,
																																																		FONT_NEXT, 47, -199,
																																																		FONT_NEXT, 70, -213,
																																																		FONT_NEXT, 103, -218,
																																																		FONT_NEXT, 154, -206,
																																																		FONT_NEXT, 197, -169,
																																																		FONT_NEXT, 235, -106,
																																																		FONT_NEXT, 273, -18,
																																																		FONT_NEXT, 427, 390,
																																																		FONT_NEXT, 439, 414,
																																																		FONT_NEXT, 451, 427,
																																																		FONT_NEXT, 475, 435,
																																																		FONT_END, 475, 450,
																																																		FONT_ADVANCE, 500, 0
																																																},
																																																	{
																																																		122,
																																																			FONT_BEGIN, 400, 139,
																																																			FONT_NEXT, 388, 89,
																																																			FONT_NEXT, 369, 55,
																																																			FONT_NEXT, 334, 36,
																																																			FONT_NEXT, 307, 31,
																																																			FONT_NEXT, 272, 30,
																																																			FONT_NEXT, 134, 30,
																																																			FONT_NEXT, 403, 435,
																																																			FONT_NEXT, 403, 450,
																																																			FONT_NEXT, 56, 450,
																																																			FONT_NEXT, 53, 332,
																																																			FONT_NEXT, 71, 332,
																																																			FONT_NEXT, 76, 368,
																																																			FONT_NEXT, 88, 396,
																																																			FONT_NEXT, 112, 413,
																																																			FONT_NEXT, 155, 420,
																																																			FONT_NEXT, 293, 420,
																																																			FONT_NEXT, 27, 15,
																																																			FONT_NEXT, 27, 0,
																																																			FONT_NEXT, 404, 0,
																																																			FONT_END, 418, 135,
																																																			FONT_ADVANCE, 444, 0
																																																	},
																																																	{
																																																		123,
																																																			FONT_BEGIN, 187, -82,
																																																			FONT_NEXT, 196, -114,
																																																			FONT_NEXT, 210, -139,
																																																			FONT_NEXT, 228, -156,
																																																			FONT_NEXT, 280, -176,
																																																			FONT_NEXT, 350, -181,
																																																			FONT_NEXT, 350, -170,
																																																			FONT_NEXT, 305, -154,
																																																			FONT_NEXT, 277, -129,
																																																			FONT_NEXT, 263, -94,
																																																			FONT_NEXT, 259, -47,
																																																			FONT_NEXT, 259, 121,
																																																			FONT_NEXT, 255, 168,
																																																			FONT_NEXT, 242, 204,
																																																			FONT_NEXT, 215, 230,
																																																			FONT_NEXT, 168, 248,
																																																			FONT_NEXT, 168, 250,
																																																			FONT_NEXT, 215, 267,
																																																			FONT_NEXT, 242, 294,
																																																			FONT_NEXT, 255, 330,
																																																			FONT_NEXT, 259, 378,
																																																			FONT_NEXT, 259, 546,
																																																			FONT_NEXT, 263, 592,
																																																			FONT_NEXT, 277, 627,
																																																			FONT_NEXT, 305, 652,
																																																			FONT_NEXT, 350, 669,
																																																			FONT_NEXT, 350, 680,
																																																			FONT_NEXT, 280, 675,
																																																			FONT_NEXT, 228, 655,
																																																			FONT_NEXT, 210, 637,
																																																			FONT_NEXT, 196, 612,
																																																			FONT_NEXT, 187, 581,
																																																			FONT_NEXT, 185, 541,
																																																			FONT_NEXT, 185, 363,
																																																			FONT_NEXT, 181, 320,
																																																			FONT_NEXT, 168, 289,
																																																			FONT_NEXT, 142, 266,
																																																			FONT_NEXT, 100, 250,
																																																			FONT_NEXT, 142, 233,
																																																			FONT_NEXT, 168, 209,
																																																			FONT_NEXT, 181, 178,
																																																			FONT_NEXT, 185, 136,
																																																			FONT_END, 185, -41,
																																																			FONT_ADVANCE, 480, 0
																																																	},
																																																		{
																																																			124,
																																																				FONT_BEGIN, 133, -14,
																																																				FONT_NEXT, 133, 676,
																																																				FONT_NEXT, 67, 676,
																																																				FONT_END, 67, -14,
																																																				FONT_ADVANCE, 200, 0
																																																		},
																																																		{
																																																			125,
																																																				FONT_BEGIN, 292, 580,
																																																				FONT_NEXT, 283, 612,
																																																				FONT_NEXT, 269, 637,
																																																				FONT_NEXT, 251, 655,
																																																				FONT_NEXT, 199, 675,
																																																				FONT_NEXT, 130, 680,
																																																				FONT_NEXT, 130, 669,
																																																				FONT_NEXT, 174, 652,
																																																				FONT_NEXT, 202, 627,
																																																				FONT_NEXT, 216, 592,
																																																				FONT_NEXT, 221, 546,
																																																				FONT_NEXT, 221, 378,
																																																				FONT_NEXT, 224, 330,
																																																				FONT_NEXT, 237, 294,
																																																				FONT_NEXT, 264, 268,
																																																				FONT_NEXT, 312, 251,
																																																				FONT_NEXT, 312, 249,
																																																				FONT_NEXT, 264, 231,
																																																				FONT_NEXT, 237, 204,
																																																				FONT_NEXT, 224, 168,
																																																				FONT_NEXT, 221, 121,
																																																				FONT_NEXT, 221, -47,
																																																				FONT_NEXT, 216, -94,
																																																				FONT_NEXT, 202, -129,
																																																				FONT_NEXT, 174, -154,
																																																				FONT_NEXT, 130, -170,
																																																				FONT_NEXT, 130, -181,
																																																				FONT_NEXT, 199, -177,
																																																				FONT_NEXT, 251, -157,
																																																				FONT_NEXT, 269, -139,
																																																				FONT_NEXT, 283, -114,
																																																				FONT_NEXT, 292, -83,
																																																				FONT_NEXT, 295, -42,
																																																				FONT_NEXT, 295, 136,
																																																				FONT_NEXT, 298, 178,
																																																				FONT_NEXT, 311, 209,
																																																				FONT_NEXT, 337, 233,
																																																				FONT_NEXT, 380, 250,
																																																				FONT_NEXT, 337, 266,
																																																				FONT_NEXT, 311, 289,
																																																				FONT_NEXT, 298, 320,
																																																				FONT_NEXT, 295, 363,
																																																				FONT_END, 295, 540,
																																																				FONT_ADVANCE, 480, 0
																																																		},
																																																			{
																																																				126,
																																																					FONT_BEGIN, 428, 273,
																																																					FONT_NEXT, 405, 257,
																																																					FONT_NEXT, 378, 251,
																																																					FONT_NEXT, 330, 263,
																																																					FONT_NEXT, 276, 287,
																																																					FONT_NEXT, 219, 309,
																																																					FONT_NEXT, 165, 319,
																																																					FONT_NEXT, 120, 311,
																																																					FONT_NEXT, 85, 291,
																																																					FONT_NEXT, 40, 233,
																																																					FONT_NEXT, 76, 183,
																																																					FONT_NEXT, 107, 232,
																																																					FONT_NEXT, 130, 248,
																																																					FONT_NEXT, 160, 255,
																																																					FONT_NEXT, 222, 244,
																																																					FONT_NEXT, 275, 220,
																																																					FONT_NEXT, 324, 196,
																																																					FONT_NEXT, 377, 187,
																																																					FONT_NEXT, 418, 194,
																																																					FONT_NEXT, 451, 214,
																																																					FONT_NEXT, 502, 273,
																																																					FONT_END, 466, 323,
																																																					FONT_ADVANCE, 541, 0
																																																			},
																																																			{
																																																				END_OF_LIST
																																																			}
			};
static GLint filledFont[][1 + MAX_STROKES * 3] = {
				{
					32,
						FONT_ADVANCE, 250, 0
				},
				{
					33,
						FONT_BEGIN, 236, 605,
						FONT_NEXT, 234, 559,
						FONT_NEXT, 229, 642,
						FONT_NEXT, 189, 176,
						FONT_NEXT, 211, 667,
						FONT_END, 182, 676,
						FONT_BEGIN, 182, 676,
						FONT_NEXT, 189, 176,
						FONT_END, 176, 176,
						FONT_BEGIN, 182, 676,
						FONT_NEXT, 176, 176,
						FONT_NEXT, 156, 668,
						FONT_NEXT, 167, 284,
						FONT_END, 163, 315,
						FONT_BEGIN, 156, 668,
						FONT_NEXT, 163, 315,
						FONT_END, 158, 355,
						FONT_BEGIN, 156, 668,
						FONT_NEXT, 158, 355,
						FONT_END, 146, 447,
						FONT_BEGIN, 156, 668,
						FONT_NEXT, 146, 447,
						FONT_NEXT, 140, 648,
						FONT_NEXT, 134, 534,
						FONT_NEXT, 130, 592,
						FONT_END, 131, 568,
						FONT_BEGIN, 238, 42,
						FONT_NEXT, 232, 18,
						FONT_NEXT, 233, 62,
						FONT_END, 222, 79,
						FONT_BEGIN, 222, 79,
						FONT_NEXT, 232, 18,
						FONT_END, 219, 3,
						FONT_BEGIN, 222, 79,
						FONT_NEXT, 219, 3,
						FONT_NEXT, 204, 91,
						FONT_NEXT, 183, -9,
						FONT_NEXT, 183, 96,
						FONT_END, 162, 91,
						FONT_BEGIN, 162, 91,
						FONT_NEXT, 183, -9,
						FONT_END, 148, 2,
						FONT_BEGIN, 162, 91,
						FONT_NEXT, 148, 2,
						FONT_NEXT, 145, 80,
						FONT_NEXT, 135, 18,
						FONT_NEXT, 134, 63,
						FONT_END, 130, 42,
						FONT_ADVANCE, 333, 0
				},
					{
						34,
							FONT_BEGIN, 162, 635,
							FONT_NEXT, 158, 599,
							FONT_NEXT, 148, 665,
							FONT_NEXT, 149, 543,
							FONT_END, 139, 482,
							FONT_BEGIN, 148, 665,
							FONT_NEXT, 139, 482,
							FONT_NEXT, 120, 676,
							FONT_NEXT, 130, 431,
							FONT_END, 109, 431,
							FONT_BEGIN, 120, 676,
							FONT_NEXT, 109, 431,
							FONT_NEXT, 91, 665,
							FONT_NEXT, 99, 482,
							FONT_END, 89, 543,
							FONT_BEGIN, 91, 665,
							FONT_NEXT, 89, 543,
							FONT_NEXT, 77, 635,
							FONT_END, 80, 599,
							FONT_BEGIN, 331, 635,
							FONT_NEXT, 327, 599,
							FONT_NEXT, 317, 665,
							FONT_NEXT, 318, 543,
							FONT_END, 308, 482,
							FONT_BEGIN, 317, 665,
							FONT_NEXT, 308, 482,
							FONT_NEXT, 289, 676,
							FONT_NEXT, 299, 431,
							FONT_END, 278, 431,
							FONT_BEGIN, 289, 676,
							FONT_NEXT, 278, 431,
							FONT_NEXT, 260, 665,
							FONT_NEXT, 268, 482,
							FONT_END, 258, 543,
							FONT_BEGIN, 260, 665,
							FONT_NEXT, 258, 543,
							FONT_NEXT, 246, 635,
							FONT_END, 249, 599,
							FONT_ADVANCE, 408, 0
					},
					{
						35,
							FONT_BEGIN, 32, 405,
							FONT_NEXT, 32, 460,
							FONT_NEXT, 142, 405,
							FONT_NEXT, 150, 460,
							FONT_END, 137, 0,
							FONT_BEGIN, 142, 405,
							FONT_NEXT, 137, 0,
							FONT_NEXT, 121, 271,
							FONT_NEXT, 79, 0,
							FONT_END, 112, 216,
							FONT_BEGIN, 121, 271,
							FONT_NEXT, 112, 216,
							FONT_NEXT, 5, 271,
							FONT_END, 5, 216,
							FONT_BEGIN, 239, 662,
							FONT_NEXT, 208, 460,
							FONT_NEXT, 181, 662,
							FONT_NEXT, 200, 405,
							FONT_END, 179, 271,
							FONT_BEGIN, 181, 662,
							FONT_NEXT, 179, 271,
							FONT_NEXT, 150, 460,
							FONT_NEXT, 170, 216,
							FONT_END, 137, 0,
							FONT_BEGIN, 170, 216,
							FONT_NEXT, 179, 271,
							FONT_NEXT, 304, 216,
							FONT_NEXT, 313, 271,
							FONT_NEXT, 273, 0,
							FONT_END, 331, 0,
							FONT_BEGIN, 331, 0,
							FONT_NEXT, 313, 271,
							FONT_END, 333, 405,
							FONT_BEGIN, 331, 0,
							FONT_NEXT, 333, 405,
							FONT_NEXT, 341, 460,
							FONT_END, 208, 460,
							FONT_BEGIN, 208, 460,
							FONT_NEXT, 333, 405,
							FONT_END, 200, 405,
							FONT_BEGIN, 208, 460,
							FONT_END, 200, 405,
							FONT_BEGIN, 429, 662,
							FONT_NEXT, 399, 460,
							FONT_NEXT, 371, 662,
							FONT_NEXT, 391, 405,
							FONT_END, 371, 271,
							FONT_BEGIN, 371, 662,
							FONT_NEXT, 371, 271,
							FONT_NEXT, 341, 460,
							FONT_NEXT, 362, 216,
							FONT_END, 331, 0,
							FONT_BEGIN, 496, 460,
							FONT_NEXT, 496, 405,
							FONT_NEXT, 399, 460,
							FONT_END, 391, 405,
							FONT_BEGIN, 471, 271,
							FONT_NEXT, 471, 216,
							FONT_NEXT, 371, 271,
							FONT_END, 362, 216,
							FONT_ADVANCE, 500, 0
					},
						{
							36,
								FONT_BEGIN, 264, 727,
								FONT_NEXT, 264, 664,
								FONT_NEXT, 230, 727,
								FONT_NEXT, 264, 637,
								FONT_END, 264, 391,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 264, 391,
								FONT_END, 264, 293,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 264, 293,
								FONT_END, 264, 28,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 264, 28,
								FONT_END, 264, 0,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 264, 0,
								FONT_END, 264, -87,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 264, -87,
								FONT_END, 230, -87,
								FONT_BEGIN, 230, 727,
								FONT_NEXT, 230, -87,
								FONT_NEXT, 230, 664,
								FONT_END, 230, 310,
								FONT_BEGIN, 230, 310,
								FONT_NEXT, 230, -87,
								FONT_NEXT, 230, 28,
								FONT_NEXT, 230, 0,
								FONT_NEXT, 166, 37,
								FONT_NEXT, 162, 5,
								FONT_NEXT, 117, 66,
								FONT_NEXT, 110, 19,
								FONT_NEXT, 81, 114,
								FONT_NEXT, 70, 35,
								FONT_NEXT, 59, 181,
								FONT_NEXT, 44, 51,
								FONT_END, 44, 181,
								FONT_BEGIN, 425, 611,
								FONT_NEXT, 425, 500,
								FONT_NEXT, 387, 634,
								FONT_NEXT, 410, 500,
								FONT_END, 399, 537,
								FONT_BEGIN, 387, 634,
								FONT_NEXT, 399, 537,
								FONT_END, 385, 566,
								FONT_BEGIN, 387, 634,
								FONT_NEXT, 385, 566,
								FONT_NEXT, 345, 649,
								FONT_NEXT, 348, 607,
								FONT_END, 305, 628,
								FONT_BEGIN, 345, 649,
								FONT_NEXT, 305, 628,
								FONT_NEXT, 264, 664,
								FONT_END, 264, 637,
								FONT_BEGIN, 457, 174,
								FONT_NEXT, 453, 134,
								FONT_NEXT, 453, 211,
								FONT_END, 444, 243,
								FONT_BEGIN, 444, 243,
								FONT_NEXT, 453, 134,
								FONT_END, 441, 100,
								FONT_BEGIN, 444, 243,
								FONT_NEXT, 441, 100,
								FONT_NEXT, 408, 295,
								FONT_NEXT, 399, 49,
								FONT_NEXT, 348, 341,
								FONT_NEXT, 378, 151,
								FONT_END, 369, 197,
								FONT_BEGIN, 348, 341,
								FONT_NEXT, 369, 197,
								FONT_END, 344, 234,
								FONT_BEGIN, 348, 341,
								FONT_NEXT, 344, 234,
								FONT_NEXT, 264, 391,
								FONT_NEXT, 308, 264,
								FONT_END, 264, 293,
								FONT_BEGIN, 264, 28,
								FONT_NEXT, 295, 35,
								FONT_NEXT, 264, 0,
								FONT_NEXT, 333, 53,
								FONT_NEXT, 338, 16,
								FONT_NEXT, 364, 89,
								FONT_END, 374, 116,
								FONT_BEGIN, 338, 16,
								FONT_NEXT, 374, 116,
								FONT_NEXT, 399, 49,
								FONT_END, 378, 151,
								FONT_BEGIN, 126, 532,
								FONT_NEXT, 133, 495,
								FONT_NEXT, 106, 392,
								FONT_NEXT, 155, 464,
								FONT_NEXT, 163, 349,
								FONT_NEXT, 187, 435,
								FONT_END, 229, 407,
								FONT_BEGIN, 163, 349,
								FONT_NEXT, 229, 407,
								FONT_NEXT, 230, 310,
								FONT_NEXT, 229, 637,
								FONT_NEXT, 230, 664,
								FONT_END, 185, 658,
								FONT_BEGIN, 185, 658,
								FONT_NEXT, 229, 637,
								FONT_END, 196, 628,
								FONT_BEGIN, 185, 658,
								FONT_NEXT, 196, 628,
								FONT_END, 171, 616,
								FONT_BEGIN, 185, 658,
								FONT_NEXT, 171, 616,
								FONT_NEXT, 148, 646,
								FONT_NEXT, 141, 586,
								FONT_NEXT, 93, 611,
								FONT_NEXT, 128, 555,
								FONT_END, 126, 532,
								FONT_BEGIN, 93, 611,
								FONT_NEXT, 126, 532,
								FONT_END, 106, 392,
								FONT_BEGIN, 93, 611,
								FONT_NEXT, 106, 392,
								FONT_END, 67, 445,
								FONT_BEGIN, 93, 611,
								FONT_NEXT, 67, 445,
								FONT_NEXT, 61, 563,
								FONT_NEXT, 55, 476,
								FONT_END, 52, 511,
								FONT_ADVANCE, 500, 0
						},
						{
							37,
								FONT_BEGIN, 102, 319,
								FONT_NEXT, 137, 397,
								FONT_NEXT, 140, 296,
								FONT_NEXT, 141, 362,
								FONT_END, 153, 339,
								FONT_BEGIN, 140, 296,
								FONT_NEXT, 153, 339,
								FONT_END, 172, 326,
								FONT_BEGIN, 140, 296,
								FONT_NEXT, 172, 326,
								FONT_NEXT, 178, 289,
								FONT_NEXT, 199, 323,
								FONT_NEXT, 223, 295,
								FONT_NEXT, 228, 328,
								FONT_END, 257, 343,
								FONT_BEGIN, 223, 295,
								FONT_NEXT, 257, 343,
								FONT_NEXT, 264, 314,
								FONT_END, 299, 343,
								FONT_BEGIN, 299, 343,
								FONT_NEXT, 257, 343,
								FONT_END, 309, 396,
								FONT_BEGIN, 299, 343,
								FONT_NEXT, 309, 396,
								FONT_NEXT, 329, 379,
								FONT_NEXT, 345, 469,
								FONT_NEXT, 369, 463,
								FONT_NEXT, 359, 553,
								FONT_NEXT, 352, 635,
								FONT_NEXT, 350, 600,
								FONT_NEXT, 325, 653,
								FONT_NEXT, 315, 611,
								FONT_NEXT, 305, 659,
								FONT_NEXT, 299, 622,
								FONT_NEXT, 276, 662,
								FONT_NEXT, 281, 632,
								FONT_END, 255, 625,
								FONT_BEGIN, 276, 662,
								FONT_NEXT, 255, 625,
								FONT_NEXT, 232, 655,
								FONT_NEXT, 230, 606,
								FONT_NEXT, 192, 639,
								FONT_NEXT, 184, 545,
								FONT_NEXT, 156, 613,
								FONT_NEXT, 150, 468,
								FONT_NEXT, 124, 581,
								FONT_NEXT, 137, 397,
								FONT_END, 102, 319,
								FONT_BEGIN, 124, 581,
								FONT_NEXT, 102, 319,
								FONT_NEXT, 78, 504,
								FONT_NEXT, 72, 361,
								FONT_NEXT, 65, 463,
								FONT_NEXT, 64, 390,
								FONT_END, 61, 425,
								FONT_BEGIN, 201, -13,
								FONT_NEXT, 550, 609,
								FONT_NEXT, 249, -13,
								FONT_END, 595, 676,
								FONT_BEGIN, 595, 676,
								FONT_NEXT, 550, 609,
								FONT_NEXT, 536, 630,
								FONT_NEXT, 548, 611,
								FONT_END, 507, 592,
								FONT_BEGIN, 536, 630,
								FONT_NEXT, 507, 592,
								FONT_NEXT, 493, 614,
								FONT_NEXT, 478, 585,
								FONT_NEXT, 438, 608,
								FONT_NEXT, 443, 583,
								FONT_END, 401, 585,
								FONT_BEGIN, 438, 608,
								FONT_NEXT, 401, 585,
								FONT_NEXT, 404, 610,
								FONT_END, 381, 616,
								FONT_BEGIN, 381, 616,
								FONT_NEXT, 401, 585,
								FONT_END, 377, 593,
								FONT_BEGIN, 381, 616,
								FONT_NEXT, 377, 593,
								FONT_NEXT, 352, 635,
								FONT_END, 369, 463,
								FONT_BEGIN, 369, 463,
								FONT_NEXT, 377, 593,
								FONT_END, 384, 548,
								FONT_BEGIN, 369, 463,
								FONT_END, 384, 548,
								FONT_BEGIN, 249, -13,
								FONT_NEXT, 595, 676,
								FONT_END, 634, 676,
								FONT_BEGIN, 772, 261,
								FONT_NEXT, 757, 173,
								FONT_NEXT, 768, 295,
								FONT_END, 760, 321,
								FONT_BEGIN, 760, 321,
								FONT_NEXT, 757, 173,
								FONT_END, 746, 254,
								FONT_BEGIN, 760, 321,
								FONT_NEXT, 746, 254,
								FONT_NEXT, 731, 354,
								FONT_NEXT, 740, 287,
								FONT_END, 726, 314,
								FONT_BEGIN, 731, 354,
								FONT_NEXT, 726, 314,
								FONT_NEXT, 695, 368,
								FONT_NEXT, 703, 332,
								FONT_END, 676, 339,
								FONT_BEGIN, 695, 368,
								FONT_NEXT, 676, 339,
								FONT_NEXT, 663, 371,
								FONT_NEXT, 641, 330,
								FONT_NEXT, 622, 365,
								FONT_NEXT, 611, 308,
								FONT_NEXT, 583, 349,
								FONT_NEXT, 564, 238,
								FONT_NEXT, 514, 293,
								FONT_NEXT, 534, 160,
								FONT_END, 527, 128,
								FONT_BEGIN, 514, 293,
								FONT_NEXT, 527, 128,
								FONT_END, 525, 106,
								FONT_BEGIN, 514, 293,
								FONT_NEXT, 525, 106,
								FONT_END, 513, 11,
								FONT_BEGIN, 514, 293,
								FONT_NEXT, 513, 11,
								FONT_NEXT, 466, 218,
								FONT_NEXT, 475, 42,
								FONT_END, 455, 86,
								FONT_BEGIN, 466, 218,
								FONT_NEXT, 455, 86,
								FONT_NEXT, 453, 177,
								FONT_END, 449, 137,
								FONT_BEGIN, 525, 106,
								FONT_NEXT, 533, 60,
								FONT_NEXT, 513, 11,
								FONT_NEXT, 552, 39,
								FONT_NEXT, 572, 0,
								FONT_NEXT, 589, 30,
								FONT_NEXT, 615, 6,
								FONT_NEXT, 620, 36,
								FONT_END, 650, 53,
								FONT_BEGIN, 615, 6,
								FONT_NEXT, 650, 53,
								FONT_NEXT, 654, 25,
								FONT_END, 688, 53,
								FONT_BEGIN, 688, 53,
								FONT_NEXT, 650, 53,
								FONT_END, 700, 110,
								FONT_BEGIN, 688, 53,
								FONT_NEXT, 700, 110,
								FONT_NEXT, 717, 88,
								FONT_NEXT, 733, 183,
								FONT_END, 746, 254,
								FONT_BEGIN, 717, 88,
								FONT_NEXT, 746, 254,
								FONT_END, 757, 173,
								FONT_ADVANCE, 833, 0
						},
							{
								38,
									FONT_BEGIN, 42, 144,
									FONT_NEXT, 57, 215,
									FONT_NEXT, 46, 99,
									FONT_END, 59, 64,
									FONT_BEGIN, 59, 64,
									FONT_NEXT, 57, 215,
									FONT_NEXT, 78, 36,
									FONT_NEXT, 95, 274,
									FONT_NEXT, 101, 16,
									FONT_END, 134, 180,
									FONT_BEGIN, 134, 180,
									FONT_NEXT, 95, 274,
									FONT_NEXT, 146, 238,
									FONT_NEXT, 146, 320,
									FONT_NEXT, 176, 283,
									FONT_NEXT, 199, 355,
									FONT_NEXT, 214, 317,
									FONT_NEXT, 237, 377,
									FONT_NEXT, 252, 343,
									FONT_END, 240, 624,
									FONT_BEGIN, 240, 624,
									FONT_NEXT, 237, 377,
									FONT_NEXT, 211, 573,
									FONT_NEXT, 211, 452,
									FONT_END, 202, 519,
									FONT_BEGIN, 321, 416,
									FONT_NEXT, 350, 432,
									FONT_NEXT, 336, 384,
									FONT_END, 387, 409,
									FONT_BEGIN, 387, 409,
									FONT_NEXT, 350, 432,
									FONT_END, 389, 462,
									FONT_BEGIN, 387, 409,
									FONT_NEXT, 389, 462,
									FONT_END, 423, 504,
									FONT_BEGIN, 387, 409,
									FONT_NEXT, 423, 504,
									FONT_NEXT, 437, 444,
									FONT_NEXT, 438, 560,
									FONT_NEXT, 450, 644,
									FONT_NEXT, 431, 594,
									FONT_NEXT, 407, 667,
									FONT_NEXT, 414, 620,
									FONT_END, 389, 637,
									FONT_BEGIN, 407, 667,
									FONT_NEXT, 389, 637,
									FONT_NEXT, 355, 676,
									FONT_NEXT, 359, 644,
									FONT_END, 329, 639,
									FONT_BEGIN, 355, 676,
									FONT_NEXT, 329, 639,
									FONT_NEXT, 319, 672,
									FONT_NEXT, 304, 623,
									FONT_NEXT, 288, 661,
									FONT_NEXT, 287, 596,
									FONT_NEXT, 240, 624,
									FONT_NEXT, 281, 557,
									FONT_NEXT, 252, 343,
									FONT_NEXT, 293, 483,
									FONT_END, 321, 416,
									FONT_BEGIN, 252, 343,
									FONT_NEXT, 321, 416,
									FONT_NEXT, 322, 219,
									FONT_NEXT, 336, 384,
									FONT_END, 396, 277,
									FONT_BEGIN, 322, 219,
									FONT_NEXT, 396, 277,
									FONT_NEXT, 404, 104,
									FONT_NEXT, 429, 78,
									FONT_END, 394, 49,
									FONT_BEGIN, 404, 104,
									FONT_NEXT, 394, 49,
									FONT_NEXT, 356, 69,
									FONT_NEXT, 347, 19,
									FONT_NEXT, 316, 49,
									FONT_NEXT, 286, -4,
									FONT_NEXT, 285, 41,
									FONT_NEXT, 207, -13,
									FONT_NEXT, 263, 39,
									FONT_END, 212, 50,
									FONT_BEGIN, 212, 50,
									FONT_NEXT, 207, -13,
									FONT_NEXT, 171, 80,
									FONT_NEXT, 155, -7,
									FONT_NEXT, 144, 125,
									FONT_NEXT, 101, 16,
									FONT_END, 134, 180,
									FONT_BEGIN, 491, 556,
									FONT_NEXT, 486, 521,
									FONT_NEXT, 480, 606,
									FONT_NEXT, 475, 492,
									FONT_NEXT, 450, 644,
									FONT_END, 437, 444,
									FONT_BEGIN, 495, 405,
									FONT_NEXT, 495, 426,
									FONT_NEXT, 540, 395,
									FONT_NEXT, 617, 346,
									FONT_NEXT, 554, 381,
									FONT_NEXT, 591, 300,
									FONT_NEXT, 559, 359,
									FONT_NEXT, 568, 261,
									FONT_END, 550, 232,
									FONT_BEGIN, 559, 359,
									FONT_NEXT, 550, 232,
									FONT_NEXT, 548, 310,
									FONT_NEXT, 528, 200,
									FONT_NEXT, 524, 259,
									FONT_NEXT, 491, 150,
									FONT_NEXT, 494, 213,
									FONT_END, 468, 178,
									FONT_BEGIN, 468, 178,
									FONT_NEXT, 491, 150,
									FONT_END, 456, 51,
									FONT_BEGIN, 468, 178,
									FONT_NEXT, 456, 51,
									FONT_NEXT, 396, 277,
									FONT_END, 429, 78,
									FONT_BEGIN, 711, 426,
									FONT_NEXT, 711, 405,
									FONT_NEXT, 495, 426,
									FONT_NEXT, 661, 392,
									FONT_END, 638, 375,
									FONT_BEGIN, 495, 426,
									FONT_NEXT, 638, 375,
									FONT_END, 617, 346,
									FONT_BEGIN, 750, 100,
									FONT_NEXT, 747, 93,
									FONT_NEXT, 735, 111,
									FONT_NEXT, 740, 80,
									FONT_END, 713, 42,
									FONT_BEGIN, 735, 111,
									FONT_NEXT, 713, 42,
									FONT_NEXT, 697, 73,
									FONT_NEXT, 667, 4,
									FONT_NEXT, 672, 62,
									FONT_END, 639, 58,
									FONT_BEGIN, 639, 58,
									FONT_NEXT, 667, 4,
									FONT_END, 635, -9,
									FONT_BEGIN, 639, 58,
									FONT_NEXT, 635, -9,
									FONT_NEXT, 585, 70,
									FONT_NEXT, 599, -13,
									FONT_END, 542, -3,
									FONT_BEGIN, 585, 70,
									FONT_NEXT, 542, -3,
									FONT_NEXT, 540, 98,
									FONT_NEXT, 494, 21,
									FONT_NEXT, 507, 129,
									FONT_END, 491, 150,
									FONT_BEGIN, 491, 150,
									FONT_NEXT, 494, 21,
									FONT_END, 456, 51,
									FONT_BEGIN, 491, 150,
									FONT_END, 456, 51,
									FONT_ADVANCE, 778, 0
							},
							{
								39,
									FONT_BEGIN, 106, 433,
									FONT_NEXT, 97, 452,
									FONT_NEXT, 136, 450,
									FONT_NEXT, 137, 483,
									FONT_END, 162, 513,
									FONT_BEGIN, 136, 450,
									FONT_NEXT, 162, 513,
									FONT_NEXT, 173, 483,
									FONT_NEXT, 175, 539,
									FONT_END, 179, 558,
									FONT_BEGIN, 173, 483,
									FONT_NEXT, 179, 558,
									FONT_NEXT, 204, 528,
									FONT_END, 186, 659,
									FONT_BEGIN, 186, 659,
									FONT_NEXT, 179, 558,
									FONT_NEXT, 159, 672,
									FONT_NEXT, 174, 568,
									FONT_END, 165, 572,
									FONT_BEGIN, 159, 672,
									FONT_NEXT, 165, 572,
									FONT_END, 156, 570,
									FONT_BEGIN, 159, 672,
									FONT_NEXT, 156, 570,
									FONT_NEXT, 137, 676,
									FONT_NEXT, 141, 568,
									FONT_END, 100, 578,
									FONT_BEGIN, 137, 676,
									FONT_NEXT, 100, 578,
									FONT_NEXT, 98, 663,
									FONT_NEXT, 85, 593,
									FONT_NEXT, 84, 645,
									FONT_END, 79, 618,
									FONT_BEGIN, 218, 580,
									FONT_NEXT, 204, 528,
									FONT_NEXT, 208, 630,
									FONT_END, 186, 659,
									FONT_ADVANCE, 333, 0
							},
								{
									40,
										FONT_BEGIN, 48, 262,
										FONT_NEXT, 52, 325,
										FONT_NEXT, 53, 169,
										FONT_NEXT, 64, 386,
										FONT_NEXT, 70, 96,
										FONT_NEXT, 85, 444,
										FONT_NEXT, 96, 35,
										FONT_NEXT, 113, 499,
										FONT_NEXT, 130, -20,
										FONT_NEXT, 134, 225,
										FONT_END, 136, 174,
										FONT_BEGIN, 130, -20,
										FONT_NEXT, 136, 174,
										FONT_END, 155, 56,
										FONT_BEGIN, 130, -20,
										FONT_NEXT, 155, 56,
										FONT_END, 175, -4,
										FONT_BEGIN, 130, -20,
										FONT_NEXT, 175, -4,
										FONT_NEXT, 203, -108,
										FONT_NEXT, 205, -63,
										FONT_NEXT, 247, -148,
										FONT_NEXT, 247, -116,
										FONT_NEXT, 292, -177,
										FONT_END, 304, -161,
										FONT_BEGIN, 304, 660,
										FONT_NEXT, 246, 609,
										FONT_NEXT, 295, 676,
										FONT_END, 239, 638,
										FONT_BEGIN, 239, 638,
										FONT_NEXT, 246, 609,
										FONT_END, 191, 537,
										FONT_BEGIN, 239, 638,
										FONT_NEXT, 191, 537,
										FONT_NEXT, 191, 596,
										FONT_END, 148, 549,
										FONT_BEGIN, 148, 549,
										FONT_NEXT, 191, 537,
										FONT_END, 168, 488,
										FONT_BEGIN, 148, 549,
										FONT_NEXT, 168, 488,
										FONT_END, 150, 428,
										FONT_BEGIN, 148, 549,
										FONT_NEXT, 150, 428,
										FONT_END, 138, 356,
										FONT_BEGIN, 148, 549,
										FONT_NEXT, 138, 356,
										FONT_NEXT, 113, 499,
										FONT_NEXT, 134, 269,
										FONT_END, 134, 225,
										FONT_ADVANCE, 333, 0
								},
								{
									41,
										FONT_BEGIN, 29, -161,
										FONT_NEXT, 86, -111,
										FONT_NEXT, 38, -177,
										FONT_END, 93, -140,
										FONT_BEGIN, 93, -140,
										FONT_NEXT, 86, -111,
										FONT_NEXT, 141, -98,
										FONT_NEXT, 141, -39,
										FONT_END, 164, 10,
										FONT_BEGIN, 141, -98,
										FONT_NEXT, 164, 10,
										FONT_END, 182, 70,
										FONT_BEGIN, 141, -98,
										FONT_NEXT, 182, 70,
										FONT_NEXT, 184, -51,
										FONT_NEXT, 194, 142,
										FONT_END, 199, 230,
										FONT_BEGIN, 184, -51,
										FONT_NEXT, 199, 230,
										FONT_NEXT, 219, 0,
										FONT_END, 203, 519,
										FONT_BEGIN, 203, 519,
										FONT_NEXT, 199, 230,
										FONT_END, 198, 273,
										FONT_BEGIN, 203, 519,
										FONT_NEXT, 198, 273,
										FONT_END, 196, 324,
										FONT_BEGIN, 203, 519,
										FONT_NEXT, 196, 324,
										FONT_END, 177, 442,
										FONT_BEGIN, 203, 519,
										FONT_NEXT, 177, 442,
										FONT_NEXT, 129, 606,
										FONT_NEXT, 157, 502,
										FONT_END, 127, 561,
										FONT_BEGIN, 129, 606,
										FONT_NEXT, 127, 561,
										FONT_NEXT, 85, 646,
										FONT_NEXT, 85, 614,
										FONT_NEXT, 41, 676,
										FONT_END, 29, 660,
										FONT_BEGIN, 285, 237,
										FONT_NEXT, 280, 173,
										FONT_NEXT, 279, 329,
										FONT_NEXT, 268, 112,
										FONT_NEXT, 262, 402,
										FONT_NEXT, 247, 54,
										FONT_NEXT, 236, 463,
										FONT_NEXT, 219, 0,
										FONT_END, 203, 519,
										FONT_ADVANCE, 333, 0
								},
									{
										42,
											FONT_BEGIN, 69, 557,
											FONT_NEXT, 75, 576,
											FONT_NEXT, 72, 538,
											FONT_END, 83, 526,
											FONT_BEGIN, 83, 526,
											FONT_NEXT, 75, 576,
											FONT_END, 85, 586,
											FONT_BEGIN, 83, 526,
											FONT_NEXT, 85, 586,
											FONT_END, 102, 591,
											FONT_BEGIN, 83, 526,
											FONT_NEXT, 102, 591,
											FONT_NEXT, 120, 511,
											FONT_NEXT, 129, 581,
											FONT_END, 152, 557,
											FONT_BEGIN, 120, 511,
											FONT_NEXT, 152, 557,
											FONT_NEXT, 170, 500,
											FONT_NEXT, 185, 523,
											FONT_NEXT, 223, 478,
											FONT_END, 234, 471,
											FONT_BEGIN, 234, 471,
											FONT_NEXT, 185, 523,
											FONT_END, 241, 486,
											FONT_BEGIN, 234, 471,
											FONT_NEXT, 241, 486,
											FONT_END, 240, 456,
											FONT_BEGIN, 234, 471,
											FONT_NEXT, 240, 456,
											FONT_END, 210, 435,
											FONT_BEGIN, 234, 471,
											FONT_NEXT, 210, 435,
											FONT_NEXT, 172, 441,
											FONT_NEXT, 192, 422,
											FONT_END, 164, 394,
											FONT_BEGIN, 172, 441,
											FONT_NEXT, 164, 394,
											FONT_NEXT, 120, 428,
											FONT_NEXT, 143, 371,
											FONT_END, 129, 358,
											FONT_BEGIN, 120, 428,
											FONT_NEXT, 129, 358,
											FONT_END, 103, 351,
											FONT_BEGIN, 120, 428,
											FONT_NEXT, 103, 351,
											FONT_NEXT, 82, 415,
											FONT_NEXT, 76, 362,
											FONT_NEXT, 72, 402,
											FONT_END, 69, 383,
											FONT_BEGIN, 214, 304,
											FONT_NEXT, 219, 331,
											FONT_NEXT, 225, 275,
											FONT_NEXT, 230, 365,
											FONT_END, 240, 405,
											FONT_BEGIN, 225, 275,
											FONT_NEXT, 240, 405,
											FONT_NEXT, 249, 265,
											FONT_NEXT, 241, 449,
											FONT_NEXT, 241, 499,
											FONT_END, 241, 486,
											FONT_BEGIN, 241, 486,
											FONT_NEXT, 241, 449,
											FONT_END, 240, 456,
											FONT_BEGIN, 241, 486,
											FONT_END, 240, 456,
											FONT_BEGIN, 216, 637,
											FONT_NEXT, 220, 659,
											FONT_NEXT, 228, 581,
											FONT_NEXT, 231, 671,
											FONT_NEXT, 237, 548,
											FONT_END, 241, 499,
											FONT_BEGIN, 241, 499,
											FONT_NEXT, 231, 671,
											FONT_END, 253, 676,
											FONT_BEGIN, 241, 499,
											FONT_NEXT, 253, 676,
											FONT_NEXT, 249, 265,
											FONT_END, 260, 449,
											FONT_BEGIN, 260, 449,
											FONT_NEXT, 253, 676,
											FONT_NEXT, 260, 456,
											FONT_END, 260, 484,
											FONT_BEGIN, 260, 484,
											FONT_NEXT, 253, 676,
											FONT_NEXT, 260, 510,
											FONT_NEXT, 269, 671,
											FONT_NEXT, 273, 582,
											FONT_NEXT, 279, 661,
											FONT_NEXT, 282, 613,
											FONT_END, 287, 637,
											FONT_BEGIN, 288, 307,
											FONT_NEXT, 277, 278,
											FONT_NEXT, 274, 352,
											FONT_NEXT, 249, 265,
											FONT_NEXT, 264, 389,
											FONT_END, 260, 449,
											FONT_BEGIN, 431, 557,
											FONT_NEXT, 428, 537,
											FONT_NEXT, 427, 574,
											FONT_NEXT, 420, 525,
											FONT_NEXT, 417, 585,
											FONT_NEXT, 388, 513,
											FONT_NEXT, 395, 593,
											FONT_END, 368, 581,
											FONT_BEGIN, 368, 581,
											FONT_NEXT, 388, 513,
											FONT_END, 336, 502,
											FONT_BEGIN, 368, 581,
											FONT_NEXT, 336, 502,
											FONT_NEXT, 341, 554,
											FONT_END, 309, 520,
											FONT_BEGIN, 309, 520,
											FONT_NEXT, 336, 502,
											FONT_END, 304, 489,
											FONT_BEGIN, 309, 520,
											FONT_NEXT, 304, 489,
											FONT_NEXT, 267, 488,
											FONT_NEXT, 268, 470,
											FONT_END, 267, 453,
											FONT_BEGIN, 267, 488,
											FONT_NEXT, 267, 453,
											FONT_NEXT, 260, 484,
											FONT_END, 260, 456,
											FONT_BEGIN, 432, 386,
											FONT_NEXT, 421, 359,
											FONT_NEXT, 428, 404,
											FONT_END, 419, 416,
											FONT_BEGIN, 419, 416,
											FONT_NEXT, 421, 359,
											FONT_END, 396, 350,
											FONT_BEGIN, 419, 416,
											FONT_NEXT, 396, 350,
											FONT_NEXT, 384, 428,
											FONT_NEXT, 366, 362,
											FONT_NEXT, 334, 439,
											FONT_NEXT, 338, 391,
											FONT_END, 307, 425,
											FONT_BEGIN, 334, 439,
											FONT_NEXT, 307, 425,
											FONT_NEXT, 276, 465,
											FONT_NEXT, 267, 453,
											FONT_END, 268, 470,
											FONT_ADVANCE, 500, 0
									},
									{
										43,
											FONT_BEGIN, 315, 506,
											FONT_NEXT, 315, 286,
											FONT_NEXT, 249, 506,
											FONT_NEXT, 315, 220,
											FONT_END, 315, 0,
											FONT_BEGIN, 249, 506,
											FONT_NEXT, 315, 0,
											FONT_END, 249, 0,
											FONT_BEGIN, 249, 506,
											FONT_NEXT, 249, 0,
											FONT_NEXT, 249, 286,
											FONT_NEXT, 249, 220,
											FONT_NEXT, 30, 286,
											FONT_END, 30, 220,
											FONT_BEGIN, 534, 286,
											FONT_NEXT, 534, 220,
											FONT_NEXT, 315, 286,
											FONT_END, 315, 220,
											FONT_ADVANCE, 564, 0
									},
										{
											44,
												FONT_BEGIN, 83, -141,
												FONT_NEXT, 74, -122,
												FONT_NEXT, 113, -124,
												FONT_NEXT, 114, -91,
												FONT_END, 139, -61,
												FONT_BEGIN, 113, -124,
												FONT_NEXT, 139, -61,
												FONT_NEXT, 150, -91,
												FONT_NEXT, 152, -35,
												FONT_END, 156, -16,
												FONT_BEGIN, 150, -91,
												FONT_NEXT, 156, -16,
												FONT_NEXT, 181, -46,
												FONT_END, 163, 85,
												FONT_BEGIN, 163, 85,
												FONT_NEXT, 156, -16,
												FONT_NEXT, 136, 98,
												FONT_NEXT, 151, -6,
												FONT_END, 142, -2,
												FONT_BEGIN, 136, 98,
												FONT_NEXT, 142, -2,
												FONT_END, 133, -4,
												FONT_BEGIN, 136, 98,
												FONT_NEXT, 133, -4,
												FONT_NEXT, 114, 102,
												FONT_NEXT, 118, -6,
												FONT_END, 77, 4,
												FONT_BEGIN, 114, 102,
												FONT_NEXT, 77, 4,
												FONT_NEXT, 75, 89,
												FONT_NEXT, 62, 19,
												FONT_NEXT, 61, 71,
												FONT_END, 56, 44,
												FONT_BEGIN, 195, 6,
												FONT_NEXT, 181, -46,
												FONT_NEXT, 185, 56,
												FONT_END, 163, 85,
												FONT_ADVANCE, 250, 0
										},
										{
											45,
												FONT_BEGIN, 285, 257,
												FONT_NEXT, 285, 194,
												FONT_NEXT, 39, 257,
												FONT_END, 39, 194,
												FONT_ADVANCE, 333, 0
										},
											{
												46,
													FONT_BEGIN, 181, 43,
													FONT_NEXT, 175, 18,
													FONT_NEXT, 176, 64,
													FONT_END, 164, 83,
													FONT_BEGIN, 164, 83,
													FONT_NEXT, 175, 18,
													FONT_END, 160, 2,
													FONT_BEGIN, 164, 83,
													FONT_NEXT, 160, 2,
													FONT_NEXT, 146, 95,
													FONT_NEXT, 125, -11,
													FONT_NEXT, 125, 100,
													FONT_END, 103, 95,
													FONT_BEGIN, 103, 95,
													FONT_NEXT, 125, -11,
													FONT_END, 89, 1,
													FONT_BEGIN, 103, 95,
													FONT_NEXT, 89, 1,
													FONT_NEXT, 86, 83,
													FONT_NEXT, 75, 18,
													FONT_NEXT, 74, 65,
													FONT_END, 70, 43,
													FONT_ADVANCE, 250, 0
											},
											{
												47,
													FONT_BEGIN, 287, 676,
													FONT_NEXT, 59, -14,
													FONT_NEXT, 220, 676,
													FONT_END, -9, -14,
													FONT_ADVANCE, 278, 0
											},
												{
													48,
														FONT_BEGIN, 476, 331,
														FONT_NEXT, 466, 226,
														FONT_NEXT, 473, 379,
														FONT_END, 466, 434,
														FONT_BEGIN, 466, 434,
														FONT_NEXT, 466, 226,
														FONT_NEXT, 432, 547,
														FONT_NEXT, 432, 113,
														FONT_NEXT, 402, 597,
														FONT_NEXT, 402, 63,
														FONT_NEXT, 362, 638,
														FONT_NEXT, 377, 409,
														FONT_END, 363, 516,
														FONT_BEGIN, 362, 638,
														FONT_NEXT, 363, 516,
														FONT_END, 347, 566,
														FONT_BEGIN, 362, 638,
														FONT_NEXT, 347, 566,
														FONT_NEXT, 312, 666,
														FONT_NEXT, 324, 609,
														FONT_END, 292, 639,
														FONT_BEGIN, 312, 666,
														FONT_NEXT, 292, 639,
														FONT_NEXT, 250, 676,
														FONT_NEXT, 250, 650,
														FONT_NEXT, 187, 666,
														FONT_NEXT, 207, 639,
														FONT_END, 175, 609,
														FONT_BEGIN, 187, 666,
														FONT_NEXT, 175, 609,
														FONT_NEXT, 137, 638,
														FONT_NEXT, 152, 566,
														FONT_END, 137, 516,
														FONT_BEGIN, 137, 638,
														FONT_NEXT, 137, 516,
														FONT_NEXT, 97, 597,
														FONT_NEXT, 122, 409,
														FONT_END, 120, 364,
														FONT_BEGIN, 97, 597,
														FONT_NEXT, 120, 364,
														FONT_END, 120, 331,
														FONT_BEGIN, 97, 597,
														FONT_NEXT, 120, 331,
														FONT_END, 120, 294,
														FONT_BEGIN, 97, 597,
														FONT_NEXT, 120, 294,
														FONT_END, 97, 63,
														FONT_BEGIN, 97, 597,
														FONT_NEXT, 97, 63,
														FONT_NEXT, 68, 547,
														FONT_NEXT, 68, 113,
														FONT_NEXT, 33, 434,
														FONT_NEXT, 33, 226,
														FONT_NEXT, 26, 379,
														FONT_END, 24, 331,
														FONT_BEGIN, 120, 294,
														FONT_NEXT, 122, 246,
														FONT_NEXT, 97, 63,
														FONT_END, 137, 22,
														FONT_BEGIN, 137, 22,
														FONT_NEXT, 122, 246,
														FONT_END, 137, 141,
														FONT_BEGIN, 137, 22,
														FONT_NEXT, 137, 141,
														FONT_END, 152, 91,
														FONT_BEGIN, 137, 22,
														FONT_NEXT, 152, 91,
														FONT_END, 175, 50,
														FONT_BEGIN, 137, 22,
														FONT_NEXT, 175, 50,
														FONT_NEXT, 187, -5,
														FONT_NEXT, 207, 22,
														FONT_NEXT, 250, -14,
														FONT_NEXT, 250, 12,
														FONT_END, 292, 22,
														FONT_BEGIN, 250, -14,
														FONT_NEXT, 292, 22,
														FONT_NEXT, 312, -5,
														FONT_NEXT, 324, 50,
														FONT_END, 347, 92,
														FONT_BEGIN, 312, -5,
														FONT_NEXT, 347, 92,
														FONT_NEXT, 362, 22,
														FONT_NEXT, 363, 141,
														FONT_END, 377, 247,
														FONT_BEGIN, 362, 22,
														FONT_NEXT, 377, 247,
														FONT_NEXT, 402, 63,
														FONT_NEXT, 379, 294,
														FONT_END, 380, 331,
														FONT_BEGIN, 402, 63,
														FONT_NEXT, 380, 331,
														FONT_END, 379, 364,
														FONT_BEGIN, 402, 63,
														FONT_NEXT, 379, 364,
														FONT_END, 377, 409,
														FONT_ADVANCE, 500, 0
												},
												{
													49,
														FONT_BEGIN, 299, 674,
														FONT_NEXT, 299, 74,
														FONT_NEXT, 291, 676,
														FONT_NEXT, 213, 93,
														FONT_END, 213, 546,
														FONT_BEGIN, 291, 676,
														FONT_NEXT, 213, 546,
														FONT_END, 208, 577,
														FONT_BEGIN, 291, 676,
														FONT_NEXT, 208, 577,
														FONT_END, 198, 588,
														FONT_BEGIN, 291, 676,
														FONT_NEXT, 198, 588,
														FONT_END, 179, 593,
														FONT_BEGIN, 291, 676,
														FONT_NEXT, 179, 593,
														FONT_NEXT, 111, 585,
														FONT_NEXT, 161, 590,
														FONT_END, 143, 583,
														FONT_BEGIN, 111, 585,
														FONT_NEXT, 143, 583,
														FONT_END, 111, 571,
														FONT_BEGIN, 394, 15,
														FONT_NEXT, 394, 0,
														FONT_NEXT, 347, 17,
														FONT_NEXT, 118, 0,
														FONT_NEXT, 318, 26,
														FONT_END, 303, 44,
														FONT_BEGIN, 303, 44,
														FONT_NEXT, 118, 0,
														FONT_END, 187, 27,
														FONT_BEGIN, 303, 44,
														FONT_NEXT, 187, 27,
														FONT_NEXT, 299, 74,
														FONT_NEXT, 206, 49,
														FONT_END, 213, 93,
														FONT_BEGIN, 118, 15,
														FONT_NEXT, 157, 17,
														FONT_NEXT, 118, 0,
														FONT_END, 187, 27,
														FONT_ADVANCE, 500, 0
												},
													{
														50,
															FONT_BEGIN, 475, 137,
															FONT_NEXT, 420, 0,
															FONT_NEXT, 462, 142,
															FONT_END, 435, 105,
															FONT_BEGIN, 435, 105,
															FONT_NEXT, 420, 0,
															FONT_NEXT, 413, 85,
															FONT_NEXT, 30, 0,
															FONT_NEXT, 367, 76,
															FONT_END, 128, 76,
															FONT_BEGIN, 128, 76,
															FONT_NEXT, 30, 0,
															FONT_END, 30, 12,
															FONT_BEGIN, 128, 76,
															FONT_NEXT, 30, 12,
															FONT_END, 208, 201,
															FONT_BEGIN, 128, 76,
															FONT_NEXT, 208, 201,
															FONT_END, 265, 270,
															FONT_BEGIN, 128, 76,
															FONT_NEXT, 265, 270,
															FONT_NEXT, 296, 252,
															FONT_NEXT, 305, 336,
															FONT_END, 329, 400,
															FONT_BEGIN, 296, 252,
															FONT_NEXT, 329, 400,
															FONT_NEXT, 334, 295,
															FONT_NEXT, 338, 459,
															FONT_NEXT, 367, 628,
															FONT_NEXT, 332, 502,
															FONT_NEXT, 309, 663,
															FONT_NEXT, 310, 549,
															FONT_END, 267, 586,
															FONT_BEGIN, 309, 663,
															FONT_NEXT, 267, 586,
															FONT_NEXT, 243, 676,
															FONT_NEXT, 235, 597,
															FONT_NEXT, 176, 666,
															FONT_NEXT, 195, 602,
															FONT_END, 137, 589,
															FONT_BEGIN, 176, 666,
															FONT_NEXT, 137, 589,
															FONT_NEXT, 115, 633,
															FONT_NEXT, 97, 557,
															FONT_NEXT, 88, 606,
															FONT_NEXT, 70, 515,
															FONT_NEXT, 64, 571,
															FONT_NEXT, 52, 472,
															FONT_NEXT, 45, 528,
															FONT_END, 31, 477,
															FONT_BEGIN, 424, 496,
															FONT_NEXT, 410, 419,
															FONT_NEXT, 419, 535,
															FONT_END, 408, 571,
															FONT_BEGIN, 408, 571,
															FONT_NEXT, 410, 419,
															FONT_END, 376, 351,
															FONT_BEGIN, 408, 571,
															FONT_NEXT, 376, 351,
															FONT_NEXT, 367, 628,
															FONT_END, 334, 295,
															FONT_ADVANCE, 500, 0
													},
													{
														51,
															FONT_BEGIN, 153, 330,
															FONT_NEXT, 152, 343,
															FONT_NEXT, 197, 330,
															FONT_NEXT, 216, 368,
															FONT_NEXT, 230, 326,
															FONT_NEXT, 269, 403,
															FONT_NEXT, 283, 308,
															FONT_END, 304, 401,
															FONT_BEGIN, 304, 401,
															FONT_NEXT, 269, 403,
															FONT_END, 304, 450,
															FONT_BEGIN, 304, 401,
															FONT_NEXT, 304, 450,
															FONT_END, 318, 512,
															FONT_BEGIN, 304, 401,
															FONT_NEXT, 318, 512,
															FONT_NEXT, 346, 433,
															FONT_END, 319, 660,
															FONT_BEGIN, 319, 660,
															FONT_NEXT, 318, 512,
															FONT_END, 311, 549,
															FONT_BEGIN, 319, 660,
															FONT_NEXT, 311, 549,
															FONT_NEXT, 284, 671,
															FONT_NEXT, 290, 583,
															FONT_END, 256, 606,
															FONT_BEGIN, 284, 671,
															FONT_NEXT, 256, 606,
															FONT_NEXT, 241, 676,
															FONT_NEXT, 208, 616,
															FONT_NEXT, 187, 669,
															FONT_NEXT, 148, 603,
															FONT_NEXT, 132, 644,
															FONT_NEXT, 105, 573,
															FONT_NEXT, 83, 594,
															FONT_NEXT, 76, 538,
															FONT_NEXT, 62, 558,
															FONT_NEXT, 60, 510,
															FONT_END, 45, 514,
															FONT_BEGIN, 43, 43,
															FONT_NEXT, 46, 60,
															FONT_NEXT, 56, 12,
															FONT_NEXT, 56, 71,
															FONT_END, 82, 78,
															FONT_BEGIN, 56, 12,
															FONT_NEXT, 82, 78,
															FONT_NEXT, 89, -5,
															FONT_NEXT, 115, 69,
															FONT_NEXT, 126, -13,
															FONT_NEXT, 148, 50,
															FONT_NEXT, 154, -14,
															FONT_NEXT, 184, 30,
															FONT_NEXT, 210, -10,
															FONT_NEXT, 225, 22,
															FONT_NEXT, 263, 1,
															FONT_NEXT, 285, 35,
															FONT_NEXT, 310, 21,
															FONT_NEXT, 326, 70,
															FONT_NEXT, 351, 47,
															FONT_NEXT, 351, 120,
															FONT_END, 359, 176,
															FONT_BEGIN, 351, 47,
															FONT_NEXT, 359, 176,
															FONT_NEXT, 383, 351,
															FONT_END, 341, 383,
															FONT_BEGIN, 341, 383,
															FONT_NEXT, 359, 176,
															FONT_END, 356, 202,
															FONT_BEGIN, 341, 383,
															FONT_NEXT, 356, 202,
															FONT_END, 345, 239,
															FONT_BEGIN, 341, 383,
															FONT_NEXT, 345, 239,
															FONT_END, 321, 276,
															FONT_BEGIN, 341, 383,
															FONT_NEXT, 321, 276,
															FONT_NEXT, 304, 401,
															FONT_END, 283, 308,
															FONT_BEGIN, 431, 216,
															FONT_NEXT, 425, 165,
															FONT_NEXT, 427, 261,
															FONT_END, 417, 298,
															FONT_BEGIN, 417, 298,
															FONT_NEXT, 425, 165,
															FONT_END, 409, 120,
															FONT_BEGIN, 417, 298,
															FONT_NEXT, 409, 120,
															FONT_NEXT, 383, 351,
															FONT_NEXT, 384, 80,
															FONT_END, 351, 47,
															FONT_BEGIN, 397, 541,
															FONT_NEXT, 391, 502,
															FONT_NEXT, 390, 579,
															FONT_NEXT, 375, 466,
															FONT_NEXT, 367, 623,
															FONT_NEXT, 346, 433,
															FONT_END, 319, 660,
															FONT_ADVANCE, 500, 0
													},
														{
															52,
																FONT_BEGIN, 292, 571,
																FONT_NEXT, 293, 167,
																FONT_NEXT, 292, 231,
																FONT_NEXT, 12, 167,
																FONT_NEXT, 52, 231,
																FONT_NEXT, 12, 231,
																FONT_NEXT, 290, 571,
																FONT_NEXT, 326, 676,
																FONT_NEXT, 292, 571,
																FONT_END, 293, 167,
																FONT_BEGIN, 293, 167,
																FONT_NEXT, 326, 676,
																FONT_NEXT, 293, 0,
																FONT_END, 370, 0,
																FONT_BEGIN, 370, 0,
																FONT_NEXT, 326, 676,
																FONT_NEXT, 370, 167,
																FONT_END, 370, 231,
																FONT_BEGIN, 370, 231,
																FONT_NEXT, 326, 676,
																FONT_END, 370, 676,
																FONT_BEGIN, 370, 231,
																FONT_END, 370, 676,
																FONT_BEGIN, 472, 231,
																FONT_NEXT, 472, 167,
																FONT_NEXT, 370, 231,
																FONT_END, 370, 167,
																FONT_ADVANCE, 500, 0
														},
														{
															53,
																FONT_BEGIN, 64, 415,
																FONT_NEXT, 65, 425,
																FONT_NEXT, 71, 412,
																FONT_NEXT, 139, 498,
																FONT_NEXT, 161, 399,
																FONT_END, 254, 365,
																FONT_BEGIN, 254, 365,
																FONT_NEXT, 139, 498,
																FONT_END, 263, 468,
																FONT_BEGIN, 254, 365,
																FONT_NEXT, 263, 468,
																FONT_NEXT, 294, 337,
																FONT_NEXT, 321, 439,
																FONT_NEXT, 326, 300,
																FONT_END, 348, 255,
																FONT_BEGIN, 348, 255,
																FONT_NEXT, 321, 439,
																FONT_NEXT, 356, 201,
																FONT_NEXT, 372, 395,
																FONT_END, 351, 55,
																FONT_BEGIN, 356, 201,
																FONT_NEXT, 351, 55,
																FONT_NEXT, 352, 158,
																FONT_END, 343, 121,
																FONT_BEGIN, 343, 121,
																FONT_NEXT, 351, 55,
																FONT_END, 312, 26,
																FONT_BEGIN, 343, 121,
																FONT_NEXT, 312, 26,
																FONT_NEXT, 310, 66,
																FONT_NEXT, 266, 4,
																FONT_NEXT, 266, 33,
																FONT_END, 217, 23,
																FONT_BEGIN, 217, 23,
																FONT_NEXT, 266, 4,
																FONT_END, 214, -10,
																FONT_BEGIN, 217, 23,
																FONT_NEXT, 214, -10,
																FONT_NEXT, 72, 85,
																FONT_NEXT, 158, -14,
																FONT_END, 104, -10,
																FONT_BEGIN, 72, 85,
																FONT_NEXT, 104, -10,
																FONT_END, 65, 2,
																FONT_BEGIN, 72, 85,
																FONT_NEXT, 65, 2,
																FONT_NEXT, 48, 80,
																FONT_NEXT, 40, 21,
																FONT_NEXT, 37, 69,
																FONT_END, 32, 46,
																FONT_BEGIN, 438, 681,
																FONT_NEXT, 400, 592,
																FONT_NEXT, 429, 688,
																FONT_END, 411, 669,
																FONT_BEGIN, 411, 669,
																FONT_NEXT, 400, 592,
																FONT_NEXT, 383, 662,
																FONT_NEXT, 391, 584,
																FONT_END, 377, 583,
																FONT_BEGIN, 383, 662,
																FONT_NEXT, 377, 583,
																FONT_NEXT, 174, 662,
																FONT_NEXT, 181, 583,
																FONT_END, 139, 498,
																FONT_BEGIN, 174, 662,
																FONT_NEXT, 139, 498,
																FONT_END, 65, 425,
																FONT_BEGIN, 426, 225,
																FONT_NEXT, 420, 176,
																FONT_NEXT, 423, 281,
																FONT_END, 415, 323,
																FONT_BEGIN, 415, 323,
																FONT_NEXT, 420, 176,
																FONT_END, 406, 131,
																FONT_BEGIN, 415, 323,
																FONT_NEXT, 406, 131,
																FONT_NEXT, 398, 359,
																FONT_NEXT, 382, 90,
																FONT_NEXT, 372, 395,
																FONT_END, 351, 55,
																FONT_ADVANCE, 500, 0
														},
															{
																54,
																	FONT_BEGIN, 448, 668,
																	FONT_NEXT, 384, 652,
																	FONT_NEXT, 446, 684,
																	FONT_END, 358, 673,
																	FONT_BEGIN, 358, 673,
																	FONT_NEXT, 384, 652,
																	FONT_END, 329, 628,
																	FONT_BEGIN, 358, 673,
																	FONT_NEXT, 329, 628,
																	FONT_NEXT, 279, 647,
																	FONT_NEXT, 281, 597,
																	FONT_END, 240, 560,
																	FONT_BEGIN, 279, 647,
																	FONT_NEXT, 240, 560,
																	FONT_NEXT, 209, 609,
																	FONT_NEXT, 207, 519,
																	FONT_NEXT, 149, 560,
																	FONT_NEXT, 181, 475,
																	FONT_END, 152, 383,
																	FONT_BEGIN, 149, 560,
																	FONT_NEXT, 152, 383,
																	FONT_END, 147, 350,
																	FONT_BEGIN, 149, 560,
																	FONT_NEXT, 147, 350,
																	FONT_NEXT, 100, 502,
																	FONT_NEXT, 133, 321,
																	FONT_END, 128, 294,
																	FONT_BEGIN, 100, 502,
																	FONT_NEXT, 128, 294,
																	FONT_END, 127, 256,
																	FONT_BEGIN, 100, 502,
																	FONT_NEXT, 127, 256,
																	FONT_END, 99, 59,
																	FONT_BEGIN, 100, 502,
																	FONT_NEXT, 99, 59,
																	FONT_NEXT, 64, 436,
																	FONT_NEXT, 72, 102,
																	FONT_END, 51, 154,
																	FONT_BEGIN, 64, 436,
																	FONT_NEXT, 51, 154,
																	FONT_NEXT, 41, 365,
																	FONT_NEXT, 38, 217,
																	FONT_END, 34, 291,
																	FONT_BEGIN, 468, 218,
																	FONT_NEXT, 459, 150,
																	FONT_NEXT, 464, 265,
																	FONT_END, 454, 307,
																	FONT_BEGIN, 454, 307,
																	FONT_NEXT, 459, 150,
																	FONT_END, 427, 74,
																	FONT_BEGIN, 454, 307,
																	FONT_NEXT, 427, 74,
																	FONT_NEXT, 437, 343,
																	FONT_END, 415, 373,
																	FONT_BEGIN, 415, 373,
																	FONT_NEXT, 427, 74,
																	FONT_END, 399, 40,
																	FONT_BEGIN, 415, 373,
																	FONT_NEXT, 399, 40,
																	FONT_NEXT, 357, 414,
																	FONT_NEXT, 378, 179,
																	FONT_END, 371, 254,
																	FONT_BEGIN, 357, 414,
																	FONT_NEXT, 371, 254,
																	FONT_END, 349, 319,
																	FONT_BEGIN, 357, 414,
																	FONT_NEXT, 349, 319,
																	FONT_NEXT, 284, 428,
																	FONT_NEXT, 331, 344,
																	FONT_END, 307, 364,
																	FONT_BEGIN, 284, 428,
																	FONT_NEXT, 307, 364,
																	FONT_END, 278, 377,
																	FONT_BEGIN, 284, 428,
																	FONT_NEXT, 278, 377,
																	FONT_NEXT, 247, 425,
																	FONT_NEXT, 242, 382,
																	FONT_NEXT, 217, 417,
																	FONT_NEXT, 191, 375,
																	FONT_NEXT, 187, 403,
																	FONT_NEXT, 147, 350,
																	FONT_END, 152, 383,
																	FONT_BEGIN, 127, 256,
																	FONT_NEXT, 130, 197,
																	FONT_NEXT, 99, 59,
																	FONT_END, 132, 27,
																	FONT_BEGIN, 132, 27,
																	FONT_NEXT, 130, 197,
																	FONT_END, 138, 147,
																	FONT_BEGIN, 132, 27,
																	FONT_NEXT, 138, 147,
																	FONT_END, 152, 105,
																	FONT_BEGIN, 132, 27,
																	FONT_NEXT, 152, 105,
																	FONT_NEXT, 169, 4,
																	FONT_NEXT, 169, 72,
																	FONT_END, 191, 46,
																	FONT_BEGIN, 169, 4,
																	FONT_NEXT, 191, 46,
																	FONT_NEXT, 210, -10,
																	FONT_NEXT, 215, 28,
																	FONT_NEXT, 254, -14,
																	FONT_NEXT, 269, 14,
																	FONT_END, 312, 23,
																	FONT_BEGIN, 254, -14,
																	FONT_NEXT, 312, 23,
																	FONT_NEXT, 314, -8,
																	FONT_NEXT, 347, 54,
																	FONT_NEXT, 362, 11,
																	FONT_NEXT, 369, 105,
																	FONT_END, 378, 179,
																	FONT_BEGIN, 362, 11,
																	FONT_NEXT, 378, 179,
																	FONT_END, 399, 40,
																	FONT_ADVANCE, 500, 0
															},
															{
																55,
																	FONT_BEGIN, 449, 662,
																	FONT_NEXT, 449, 646,
																	FONT_NEXT, 79, 662,
																	FONT_END, 370, 588,
																	FONT_BEGIN, 370, 588,
																	FONT_NEXT, 449, 646,
																	FONT_END, 237, -8,
																	FONT_BEGIN, 370, 588,
																	FONT_NEXT, 237, -8,
																	FONT_END, 172, -8,
																	FONT_BEGIN, 37, 507,
																	FONT_NEXT, 20, 515,
																	FONT_NEXT, 59, 541,
																	FONT_NEXT, 63, 618,
																	FONT_END, 79, 662,
																	FONT_BEGIN, 59, 541,
																	FONT_NEXT, 79, 662,
																	FONT_NEXT, 83, 567,
																	FONT_END, 112, 582,
																	FONT_BEGIN, 112, 582,
																	FONT_NEXT, 79, 662,
																	FONT_NEXT, 153, 588,
																	FONT_END, 370, 588,
																	FONT_ADVANCE, 500, 0
															},
																{
																	56,
																		FONT_BEGIN, 136, 547,
																		FONT_NEXT, 150, 492,
																		FONT_NEXT, 131, 381,
																		FONT_NEXT, 185, 447,
																		FONT_NEXT, 186, 332,
																		FONT_END, 212, 312,
																		FONT_BEGIN, 212, 312,
																		FONT_NEXT, 185, 447,
																		FONT_END, 226, 412,
																		FONT_BEGIN, 212, 312,
																		FONT_NEXT, 226, 412,
																		FONT_END, 261, 389,
																		FONT_BEGIN, 212, 312,
																		FONT_NEXT, 261, 389,
																		FONT_NEXT, 285, 258,
																		FONT_END, 290, 371,
																		FONT_BEGIN, 290, 371,
																		FONT_NEXT, 261, 389,
																		FONT_END, 291, 410,
																		FONT_BEGIN, 290, 371,
																		FONT_NEXT, 291, 410,
																		FONT_END, 322, 440,
																		FONT_BEGIN, 290, 371,
																		FONT_NEXT, 322, 440,
																		FONT_NEXT, 333, 395,
																		FONT_NEXT, 345, 481,
																		FONT_END, 355, 539,
																		FONT_BEGIN, 333, 395,
																		FONT_NEXT, 355, 539,
																		FONT_NEXT, 377, 428,
																		FONT_NEXT, 348, 579,
																		FONT_NEXT, 378, 633,
																		FONT_END, 324, 664,
																		FONT_BEGIN, 324, 664,
																		FONT_NEXT, 348, 579,
																		FONT_END, 327, 614,
																		FONT_BEGIN, 324, 664,
																		FONT_NEXT, 327, 614,
																		FONT_END, 292, 638,
																		FONT_BEGIN, 324, 664,
																		FONT_NEXT, 292, 638,
																		FONT_NEXT, 249, 676,
																		FONT_NEXT, 243, 648,
																		FONT_NEXT, 183, 665,
																		FONT_NEXT, 194, 638,
																		FONT_END, 161, 615,
																		FONT_BEGIN, 183, 665,
																		FONT_NEXT, 161, 615,
																		FONT_NEXT, 123, 636,
																		FONT_NEXT, 142, 583,
																		FONT_END, 136, 547,
																		FONT_BEGIN, 123, 636,
																		FONT_NEXT, 136, 547,
																		FONT_END, 131, 381,
																		FONT_BEGIN, 123, 636,
																		FONT_NEXT, 131, 381,
																		FONT_END, 92, 423,
																		FONT_BEGIN, 123, 636,
																		FONT_NEXT, 92, 423,
																		FONT_NEXT, 79, 587,
																		FONT_NEXT, 69, 467,
																		FONT_NEXT, 66, 556,
																		FONT_END, 62, 521,
																		FONT_BEGIN, 56, 146,
																		FONT_NEXT, 63, 202,
																		FONT_NEXT, 68, 88,
																		FONT_NEXT, 87, 246,
																		FONT_NEXT, 104, 36,
																		FONT_NEXT, 128, 286,
																		FONT_NEXT, 132, 156,
																		FONT_END, 140, 215,
																		FONT_BEGIN, 140, 215,
																		FONT_NEXT, 128, 286,
																		FONT_NEXT, 160, 258,
																		FONT_END, 186, 289,
																		FONT_BEGIN, 186, 289,
																		FONT_NEXT, 128, 286,
																		FONT_END, 186, 332,
																		FONT_BEGIN, 186, 289,
																		FONT_NEXT, 186, 332,
																		FONT_END, 212, 312,
																		FONT_BEGIN, 285, 258,
																		FONT_NEXT, 290, 371,
																		FONT_NEXT, 333, 216,
																		FONT_NEXT, 335, 336,
																		FONT_NEXT, 360, 174,
																		FONT_END, 369, 126,
																		FONT_BEGIN, 369, 126,
																		FONT_NEXT, 335, 336,
																		FONT_END, 370, 305,
																		FONT_BEGIN, 369, 126,
																		FONT_NEXT, 370, 305,
																		FONT_END, 381, 24,
																		FONT_BEGIN, 369, 126,
																		FONT_NEXT, 381, 24,
																		FONT_END, 347, 4,
																		FONT_BEGIN, 369, 126,
																		FONT_NEXT, 347, 4,
																		FONT_NEXT, 360, 78,
																		FONT_END, 337, 43,
																		FONT_BEGIN, 337, 43,
																		FONT_NEXT, 347, 4,
																		FONT_END, 302, -10,
																		FONT_BEGIN, 337, 43,
																		FONT_NEXT, 302, -10,
																		FONT_NEXT, 302, 21,
																		FONT_END, 259, 14,
																		FONT_BEGIN, 259, 14,
																		FONT_NEXT, 302, -10,
																		FONT_END, 246, -14,
																		FONT_BEGIN, 259, 14,
																		FONT_NEXT, 246, -14,
																		FONT_NEXT, 204, 25,
																		FONT_NEXT, 202, -11,
																		FONT_NEXT, 165, 56,
																		FONT_NEXT, 164, 0,
																		FONT_NEXT, 140, 101,
																		FONT_NEXT, 104, 36,
																		FONT_END, 132, 156,
																		FONT_BEGIN, 445, 150,
																		FONT_NEXT, 437, 102,
																		FONT_NEXT, 439, 199,
																		FONT_END, 417, 249,
																		FONT_BEGIN, 417, 249,
																		FONT_NEXT, 437, 102,
																		FONT_END, 407, 48,
																		FONT_BEGIN, 417, 249,
																		FONT_NEXT, 407, 48,
																		FONT_NEXT, 370, 305,
																		FONT_END, 381, 24,
																		FONT_BEGIN, 424, 534,
																		FONT_NEXT, 410, 473,
																		FONT_NEXT, 412, 588,
																		FONT_END, 378, 633,
																		FONT_BEGIN, 378, 633,
																		FONT_NEXT, 410, 473,
																		FONT_END, 377, 428,
																		FONT_BEGIN, 378, 633,
																		FONT_END, 377, 428,
																		FONT_ADVANCE, 500, 0
																},
																{
																	57,
																		FONT_BEGIN, 459, 396,
																		FONT_NEXT, 452, 317,
																		FONT_NEXT, 454, 455,
																		FONT_END, 442, 510,
																		FONT_BEGIN, 442, 510,
																		FONT_NEXT, 452, 317,
																		FONT_END, 432, 243,
																		FONT_BEGIN, 442, 510,
																		FONT_NEXT, 432, 243,
																		FONT_NEXT, 422, 557,
																		FONT_NEXT, 399, 174,
																		FONT_NEXT, 395, 598,
																		FONT_NEXT, 354, 113,
																		FONT_END, 362, 355,
																		FONT_BEGIN, 395, 598,
																		FONT_NEXT, 362, 355,
																		FONT_NEXT, 363, 631,
																		FONT_NEXT, 362, 394,
																		FONT_END, 361, 421,
																		FONT_BEGIN, 363, 631,
																		FONT_NEXT, 361, 421,
																		FONT_END, 359, 458,
																		FONT_BEGIN, 363, 631,
																		FONT_NEXT, 359, 458,
																		FONT_END, 345, 543,
																		FONT_BEGIN, 363, 631,
																		FONT_NEXT, 345, 543,
																		FONT_NEXT, 326, 655,
																		FONT_NEXT, 329, 583,
																		FONT_END, 305, 616,
																		FONT_BEGIN, 326, 655,
																		FONT_NEXT, 305, 616,
																		FONT_NEXT, 284, 670,
																		FONT_NEXT, 272, 639,
																		FONT_NEXT, 240, 676,
																		FONT_NEXT, 229, 648,
																		FONT_NEXT, 193, 671,
																		FONT_NEXT, 200, 643,
																		FONT_END, 176, 632,
																		FONT_BEGIN, 193, 671,
																		FONT_NEXT, 176, 632,
																		FONT_NEXT, 152, 657,
																		FONT_NEXT, 144, 591,
																		FONT_NEXT, 116, 634,
																		FONT_NEXT, 126, 534,
																		FONT_END, 122, 473,
																		FONT_BEGIN, 116, 634,
																		FONT_NEXT, 122, 473,
																		FONT_END, 94, 278,
																		FONT_BEGIN, 116, 634,
																		FONT_NEXT, 94, 278,
																		FONT_NEXT, 86, 605,
																		FONT_NEXT, 69, 305,
																		FONT_NEXT, 62, 569,
																		FONT_NEXT, 39, 369,
																		FONT_NEXT, 44, 528,
																		FONT_END, 30, 435,
																		FONT_BEGIN, 56, -2,
																		FONT_NEXT, 138, 21,
																		FONT_NEXT, 59, -22,
																		FONT_END, 149, -9,
																		FONT_BEGIN, 149, -9,
																		FONT_NEXT, 138, 21,
																		FONT_END, 204, 56,
																		FONT_BEGIN, 149, -9,
																		FONT_NEXT, 204, 56,
																		FONT_NEXT, 228, 20,
																		FONT_NEXT, 256, 101,
																		FONT_END, 296, 149,
																		FONT_BEGIN, 228, 20,
																		FONT_NEXT, 296, 149,
																		FONT_NEXT, 297, 61,
																		FONT_NEXT, 324, 197,
																		FONT_END, 343, 239,
																		FONT_BEGIN, 297, 61,
																		FONT_NEXT, 343, 239,
																		FONT_NEXT, 354, 113,
																		FONT_NEXT, 354, 271,
																		FONT_END, 359, 290,
																		FONT_BEGIN, 354, 113,
																		FONT_NEXT, 359, 290,
																		FONT_NEXT, 362, 355,
																		FONT_END, 358, 332,
																		FONT_BEGIN, 358, 332,
																		FONT_NEXT, 359, 290,
																		FONT_END, 357, 292,
																		FONT_BEGIN, 358, 332,
																		FONT_NEXT, 357, 292,
																		FONT_NEXT, 346, 315,
																		FONT_NEXT, 285, 250,
																		FONT_NEXT, 302, 291,
																		FONT_END, 250, 280,
																		FONT_BEGIN, 250, 280,
																		FONT_NEXT, 285, 250,
																		FONT_END, 210, 237,
																		FONT_BEGIN, 250, 280,
																		FONT_NEXT, 210, 237,
																		FONT_NEXT, 214, 285,
																		FONT_END, 185, 299,
																		FONT_BEGIN, 185, 299,
																		FONT_NEXT, 210, 237,
																		FONT_END, 163, 242,
																		FONT_BEGIN, 185, 299,
																		FONT_NEXT, 163, 242,
																		FONT_NEXT, 146, 348,
																		FONT_NEXT, 125, 256,
																		FONT_NEXT, 127, 410,
																		FONT_END, 122, 473,
																		FONT_BEGIN, 122, 473,
																		FONT_NEXT, 125, 256,
																		FONT_END, 94, 278,
																		FONT_BEGIN, 122, 473,
																		FONT_END, 94, 278,
																		FONT_ADVANCE, 500, 0
																},
																	{
																		58,
																			FONT_BEGIN, 192, 43,
																			FONT_NEXT, 186, 18,
																			FONT_NEXT, 187, 64,
																			FONT_END, 175, 83,
																			FONT_BEGIN, 175, 83,
																			FONT_NEXT, 186, 18,
																			FONT_END, 171, 2,
																			FONT_BEGIN, 175, 83,
																			FONT_NEXT, 171, 2,
																			FONT_NEXT, 157, 95,
																			FONT_NEXT, 136, -11,
																			FONT_NEXT, 136, 100,
																			FONT_END, 114, 95,
																			FONT_BEGIN, 114, 95,
																			FONT_NEXT, 136, -11,
																			FONT_END, 100, 1,
																			FONT_BEGIN, 114, 95,
																			FONT_NEXT, 100, 1,
																			FONT_NEXT, 97, 83,
																			FONT_NEXT, 86, 18,
																			FONT_NEXT, 85, 65,
																			FONT_END, 81, 43,
																			FONT_BEGIN, 192, 402,
																			FONT_NEXT, 186, 377,
																			FONT_NEXT, 187, 423,
																			FONT_END, 175, 442,
																			FONT_BEGIN, 175, 442,
																			FONT_NEXT, 186, 377,
																			FONT_END, 171, 361,
																			FONT_BEGIN, 175, 442,
																			FONT_NEXT, 171, 361,
																			FONT_NEXT, 157, 454,
																			FONT_NEXT, 136, 348,
																			FONT_NEXT, 136, 459,
																			FONT_END, 114, 454,
																			FONT_BEGIN, 114, 454,
																			FONT_NEXT, 136, 348,
																			FONT_END, 100, 360,
																			FONT_BEGIN, 114, 454,
																			FONT_NEXT, 100, 360,
																			FONT_NEXT, 97, 442,
																			FONT_NEXT, 86, 377,
																			FONT_NEXT, 85, 424,
																			FONT_END, 81, 402,
																			FONT_ADVANCE, 278, 0
																	},
																	{
																		59,
																			FONT_BEGIN, 107, -141,
																			FONT_NEXT, 98, -122,
																			FONT_NEXT, 138, -123,
																			FONT_NEXT, 138, -91,
																			FONT_END, 163, -61,
																			FONT_BEGIN, 138, -123,
																			FONT_NEXT, 163, -61,
																			FONT_NEXT, 175, -90,
																			FONT_NEXT, 176, -35,
																			FONT_END, 180, -16,
																			FONT_BEGIN, 175, -90,
																			FONT_NEXT, 180, -16,
																			FONT_NEXT, 206, -45,
																			FONT_END, 186, 85,
																			FONT_BEGIN, 186, 85,
																			FONT_NEXT, 180, -16,
																			FONT_NEXT, 160, 98,
																			FONT_NEXT, 175, -6,
																			FONT_END, 166, -2,
																			FONT_BEGIN, 160, 98,
																			FONT_NEXT, 166, -2,
																			FONT_END, 157, -4,
																			FONT_BEGIN, 160, 98,
																			FONT_NEXT, 157, -4,
																			FONT_NEXT, 139, 102,
																			FONT_NEXT, 142, -6,
																			FONT_END, 99, 5,
																			FONT_BEGIN, 139, 102,
																			FONT_NEXT, 99, 5,
																			FONT_NEXT, 125, 100,
																			FONT_END, 105, 93,
																			FONT_BEGIN, 105, 93,
																			FONT_NEXT, 99, 5,
																			FONT_NEXT, 87, 76,
																			FONT_NEXT, 85, 20,
																			FONT_END, 80, 44,
																			FONT_BEGIN, 219, 6,
																			FONT_NEXT, 206, -45,
																			FONT_NEXT, 209, 56,
																			FONT_END, 186, 85,
																			FONT_BEGIN, 192, 402,
																			FONT_NEXT, 186, 377,
																			FONT_NEXT, 187, 423,
																			FONT_END, 175, 442,
																			FONT_BEGIN, 175, 442,
																			FONT_NEXT, 186, 377,
																			FONT_END, 171, 361,
																			FONT_BEGIN, 175, 442,
																			FONT_NEXT, 171, 361,
																			FONT_NEXT, 157, 454,
																			FONT_NEXT, 136, 348,
																			FONT_NEXT, 136, 459,
																			FONT_END, 114, 454,
																			FONT_BEGIN, 114, 454,
																			FONT_NEXT, 136, 348,
																			FONT_END, 100, 360,
																			FONT_BEGIN, 114, 454,
																			FONT_NEXT, 100, 360,
																			FONT_NEXT, 97, 442,
																			FONT_NEXT, 86, 377,
																			FONT_NEXT, 85, 424,
																			FONT_END, 81, 402,
																			FONT_ADVANCE, 278, 0
																	},
																		{
																			60,
																				FONT_BEGIN, 536, 514,
																				FONT_NEXT, 536, 446,
																				FONT_NEXT, 28, 284,
																				FONT_NEXT, 111, 253,
																				FONT_NEXT, 28, 222,
																				FONT_END, 536, -8,
																				FONT_BEGIN, 536, -8,
																				FONT_NEXT, 111, 253,
																				FONT_END, 536, 60,
																				FONT_BEGIN, 536, -8,
																				FONT_END, 536, 60,
																				FONT_ADVANCE, 564, 0
																		},
																		{
																			61,
																				FONT_BEGIN, 534, 186,
																				FONT_NEXT, 534, 120,
																				FONT_NEXT, 30, 186,
																				FONT_END, 30, 120,
																				FONT_BEGIN, 534, 386,
																				FONT_NEXT, 534, 320,
																				FONT_NEXT, 30, 386,
																				FONT_END, 30, 320,
																				FONT_ADVANCE, 564, 0
																		},
																			{
																				62,
																					FONT_BEGIN, 28, 446,
																					FONT_NEXT, 28, 514,
																					FONT_NEXT, 453, 253,
																					FONT_NEXT, 536, 222,
																					FONT_END, 28, -8,
																					FONT_BEGIN, 453, 253,
																					FONT_NEXT, 28, -8,
																					FONT_END, 28, 60,
																					FONT_BEGIN, 536, 222,
																					FONT_NEXT, 28, 514,
																					FONT_END, 536, 284,
																					FONT_ADVANCE, 564, 0
																			},
																			{
																				63,
																					FONT_BEGIN, 68, 532,
																					FONT_NEXT, 72, 567,
																					FONT_NEXT, 78, 487,
																					FONT_NEXT, 83, 597,
																					FONT_NEXT, 93, 474,
																					FONT_END, 118, 469,
																					FONT_BEGIN, 118, 469,
																					FONT_NEXT, 83, 597,
																					FONT_END, 119, 585,
																					FONT_BEGIN, 118, 469,
																					FONT_NEXT, 119, 585,
																					FONT_NEXT, 135, 473,
																					FONT_END, 148, 483,
																					FONT_BEGIN, 148, 483,
																					FONT_NEXT, 119, 585,
																					FONT_END, 157, 510,
																					FONT_BEGIN, 148, 483,
																					FONT_END, 157, 510,
																					FONT_BEGIN, 414, 510,
																					FONT_NEXT, 406, 469,
																					FONT_NEXT, 412, 544,
																					FONT_END, 406, 571,
																					FONT_BEGIN, 406, 571,
																					FONT_NEXT, 406, 469,
																					FONT_NEXT, 378, 618,
																					FONT_NEXT, 387, 426,
																					FONT_END, 363, 385,
																					FONT_BEGIN, 378, 618,
																					FONT_NEXT, 363, 385,
																					FONT_NEXT, 352, 639,
																					FONT_NEXT, 339, 354,
																					FONT_NEXT, 316, 657,
																					FONT_NEXT, 322, 519,
																					FONT_END, 315, 564,
																					FONT_BEGIN, 316, 657,
																					FONT_NEXT, 315, 564,
																					FONT_NEXT, 274, 671,
																					FONT_NEXT, 295, 605,
																					FONT_END, 261, 634,
																					FONT_BEGIN, 274, 671,
																					FONT_NEXT, 261, 634,
																					FONT_NEXT, 231, 676,
																					FONT_NEXT, 212, 646,
																					FONT_NEXT, 176, 667,
																					FONT_NEXT, 179, 641,
																					FONT_END, 149, 627,
																					FONT_BEGIN, 176, 667,
																					FONT_NEXT, 149, 627,
																					FONT_NEXT, 123, 641,
																					FONT_NEXT, 127, 608,
																					FONT_END, 119, 585,
																					FONT_BEGIN, 123, 641,
																					FONT_NEXT, 119, 585,
																					FONT_END, 83, 597,
																					FONT_BEGIN, 227, 164,
																					FONT_NEXT, 239, 251,
																					FONT_NEXT, 244, 164,
																					FONT_NEXT, 249, 286,
																					FONT_NEXT, 257, 220,
																					FONT_NEXT, 257, 306,
																					FONT_NEXT, 276, 267,
																					FONT_NEXT, 285, 370,
																					FONT_NEXT, 302, 309,
																					FONT_NEXT, 310, 444,
																					FONT_END, 322, 519,
																					FONT_BEGIN, 302, 309,
																					FONT_NEXT, 322, 519,
																					FONT_END, 339, 354,
																					FONT_BEGIN, 292, 43,
																					FONT_NEXT, 286, 19,
																					FONT_NEXT, 287, 63,
																					FONT_END, 276, 80,
																					FONT_BEGIN, 276, 80,
																					FONT_NEXT, 286, 19,
																					FONT_END, 273, 4,
																					FONT_BEGIN, 276, 80,
																					FONT_NEXT, 273, 4,
																					FONT_NEXT, 258, 92,
																					FONT_NEXT, 237, -8,
																					FONT_NEXT, 237, 97,
																					FONT_END, 216, 92,
																					FONT_BEGIN, 216, 92,
																					FONT_NEXT, 237, -8,
																					FONT_END, 202, 3,
																					FONT_BEGIN, 216, 92,
																					FONT_NEXT, 202, 3,
																					FONT_NEXT, 199, 81,
																					FONT_NEXT, 189, 19,
																					FONT_NEXT, 188, 64,
																					FONT_END, 184, 43,
																					FONT_ADVANCE, 444, 0
																			},
																				{
																					64,
																						FONT_BEGIN, 321, 268,
																						FONT_NEXT, 323, 298,
																						FONT_NEXT, 323, 230,
																						FONT_END, 332, 200,
																						FONT_BEGIN, 332, 200,
																						FONT_NEXT, 323, 298,
																						FONT_END, 332, 335,
																						FONT_BEGIN, 332, 200,
																						FONT_NEXT, 332, 335,
																						FONT_NEXT, 358, 163,
																						FONT_NEXT, 368, 414,
																						FONT_NEXT, 390, 147,
																						FONT_NEXT, 392, 271,
																						FONT_END, 396, 232,
																						FONT_BEGIN, 390, 147,
																						FONT_NEXT, 396, 232,
																						FONT_END, 410, 206,
																						FONT_BEGIN, 390, 147,
																						FONT_NEXT, 410, 206,
																						FONT_NEXT, 417, 144,
																						FONT_NEXT, 430, 191,
																						FONT_END, 455, 187,
																						FONT_BEGIN, 417, 144,
																						FONT_NEXT, 455, 187,
																						FONT_NEXT, 457, 154,
																						FONT_NEXT, 489, 200,
																						FONT_NEXT, 492, 174,
																						FONT_END, 517, 196,
																						FONT_BEGIN, 517, 196,
																						FONT_NEXT, 489, 200,
																						FONT_END, 524, 240,
																						FONT_BEGIN, 517, 196,
																						FONT_NEXT, 524, 240,
																						FONT_NEXT, 532, 214,
																						FONT_END, 534, 214,
																						FONT_BEGIN, 534, 214,
																						FONT_NEXT, 524, 240,
																						FONT_END, 553, 173,
																						FONT_BEGIN, 534, 214,
																						FONT_NEXT, 553, 173,
																						FONT_END, 538, 197,
																						FONT_BEGIN, 668, 494,
																						FONT_NEXT, 603, 240,
																						FONT_NEXT, 599, 494,
																						FONT_NEXT, 603, 202,
																						FONT_END, 581, 152,
																						FONT_BEGIN, 599, 494,
																						FONT_NEXT, 581, 152,
																						FONT_NEXT, 589, 456,
																						FONT_NEXT, 572, 399,
																						FONT_NEXT, 565, 491,
																						FONT_NEXT, 569, 426,
																						FONT_END, 561, 447,
																						FONT_BEGIN, 565, 491,
																						FONT_NEXT, 561, 447,
																						FONT_NEXT, 546, 503,
																						FONT_NEXT, 548, 459,
																						FONT_END, 529, 464,
																						FONT_BEGIN, 546, 503,
																						FONT_NEXT, 529, 464,
																						FONT_NEXT, 518, 508,
																						FONT_NEXT, 487, 450,
																						FONT_NEXT, 470, 500,
																						FONT_NEXT, 442, 412,
																						FONT_NEXT, 430, 480,
																						FONT_NEXT, 406, 352,
																						FONT_NEXT, 396, 450,
																						FONT_NEXT, 395, 314,
																						FONT_NEXT, 368, 414,
																						FONT_END, 392, 271,
																						FONT_BEGIN, 116, 315,
																						FONT_NEXT, 123, 392,
																						FONT_NEXT, 121, 262,
																						FONT_END, 137, 207,
																						FONT_BEGIN, 137, 207,
																						FONT_NEXT, 123, 392,
																						FONT_END, 145, 462,
																						FONT_BEGIN, 137, 207,
																						FONT_NEXT, 145, 462,
																						FONT_NEXT, 165, 152,
																						FONT_NEXT, 179, 524,
																						FONT_NEXT, 200, 305,
																						FONT_END, 205, 383,
																						FONT_BEGIN, 205, 383,
																						FONT_NEXT, 179, 524,
																						FONT_NEXT, 222, 451,
																						FONT_NEXT, 224, 576,
																						FONT_NEXT, 248, 510,
																						FONT_NEXT, 278, 618,
																						FONT_NEXT, 283, 558,
																						FONT_END, 324, 595,
																						FONT_BEGIN, 324, 595,
																						FONT_NEXT, 278, 618,
																						FONT_END, 340, 649,
																						FONT_BEGIN, 324, 595,
																						FONT_NEXT, 340, 649,
																						FONT_NEXT, 372, 623,
																						FONT_NEXT, 408, 669,
																						FONT_NEXT, 424, 639,
																						FONT_END, 481, 645,
																						FONT_BEGIN, 481, 645,
																						FONT_NEXT, 408, 669,
																						FONT_END, 481, 676,
																						FONT_BEGIN, 481, 645,
																						FONT_NEXT, 481, 676,
																						FONT_NEXT, 545, 638,
																						FONT_NEXT, 547, 670,
																						FONT_NEXT, 603, 619,
																						FONT_NEXT, 609, 653,
																						FONT_NEXT, 652, 591,
																						FONT_NEXT, 665, 627,
																						FONT_NEXT, 693, 555,
																						FONT_NEXT, 714, 591,
																						FONT_NEXT, 726, 513,
																						FONT_END, 749, 469,
																						FONT_BEGIN, 749, 469,
																						FONT_NEXT, 714, 591,
																						FONT_END, 753, 548,
																						FONT_BEGIN, 749, 469,
																						FONT_NEXT, 753, 548,
																						FONT_NEXT, 764, 424,
																						FONT_END, 769, 380,
																						FONT_BEGIN, 769, 380,
																						FONT_NEXT, 753, 548,
																						FONT_END, 783, 498,
																						FONT_BEGIN, 769, 380,
																						FONT_NEXT, 783, 498,
																						FONT_END, 760, 223,
																						FONT_BEGIN, 769, 380,
																						FONT_NEXT, 760, 223,
																						FONT_NEXT, 760, 311,
																						FONT_END, 734, 245,
																						FONT_BEGIN, 734, 245,
																						FONT_NEXT, 760, 223,
																						FONT_END, 734, 191,
																						FONT_BEGIN, 734, 245,
																						FONT_NEXT, 734, 191,
																						FONT_NEXT, 693, 195,
																						FONT_NEXT, 702, 165,
																						FONT_END, 665, 149,
																						FONT_BEGIN, 693, 195,
																						FONT_NEXT, 665, 149,
																						FONT_NEXT, 667, 181,
																						FONT_END, 639, 176,
																						FONT_BEGIN, 639, 176,
																						FONT_NEXT, 665, 149,
																						FONT_END, 623, 143,
																						FONT_BEGIN, 639, 176,
																						FONT_NEXT, 623, 143,
																						FONT_NEXT, 613, 183,
																						FONT_NEXT, 581, 152,
																						FONT_END, 603, 202,
																						FONT_BEGIN, 700, 43,
																						FONT_NEXT, 601, 3,
																						FONT_NEXT, 688, 73,
																						FONT_END, 601, 39,
																						FONT_BEGIN, 601, 39,
																						FONT_NEXT, 601, 3,
																						FONT_NEXT, 550, 28,
																						FONT_NEXT, 547, -10,
																						FONT_NEXT, 490, 25,
																						FONT_NEXT, 491, -14,
																						FONT_END, 399, -6,
																						FONT_BEGIN, 490, 25,
																						FONT_NEXT, 399, -6,
																						FONT_NEXT, 427, 31,
																						FONT_END, 371, 48,
																						FONT_BEGIN, 371, 48,
																						FONT_NEXT, 399, -6,
																						FONT_END, 322, 18,
																						FONT_BEGIN, 371, 48,
																						FONT_NEXT, 322, 18,
																						FONT_NEXT, 321, 75,
																						FONT_NEXT, 257, 54,
																						FONT_NEXT, 279, 111,
																						FONT_END, 245, 153,
																						FONT_BEGIN, 245, 153,
																						FONT_NEXT, 257, 54,
																						FONT_END, 205, 100,
																						FONT_BEGIN, 245, 153,
																						FONT_NEXT, 205, 100,
																						FONT_NEXT, 220, 200,
																						FONT_END, 205, 251,
																						FONT_BEGIN, 205, 251,
																						FONT_NEXT, 205, 100,
																						FONT_NEXT, 200, 305,
																						FONT_END, 165, 152,
																						FONT_BEGIN, 809, 381,
																						FONT_NEXT, 796, 300,
																						FONT_NEXT, 802, 442,
																						FONT_END, 783, 498,
																						FONT_BEGIN, 783, 498,
																						FONT_NEXT, 796, 300,
																						FONT_END, 760, 223,
																						FONT_BEGIN, 783, 498,
																						FONT_END, 760, 223,
																						FONT_BEGIN, 524, 240,
																						FONT_NEXT, 554, 306,
																						FONT_NEXT, 553, 173,
																						FONT_NEXT, 572, 399,
																						FONT_END, 581, 152,
																						FONT_ADVANCE, 921, 0
																				},
																				{
																					65,
																						FONT_BEGIN, 451, 19,
																						FONT_NEXT, 499, 24,
																						FONT_NEXT, 451, 0,
																						FONT_END, 637, 62,
																						FONT_BEGIN, 637, 62,
																						FONT_NEXT, 499, 24,
																						FONT_END, 515, 35,
																						FONT_BEGIN, 637, 62,
																						FONT_NEXT, 515, 35,
																						FONT_NEXT, 616, 106,
																						FONT_NEXT, 521, 57,
																						FONT_NEXT, 367, 674,
																						FONT_NEXT, 502, 120,
																						FONT_END, 461, 216,
																						FONT_BEGIN, 367, 674,
																						FONT_NEXT, 461, 216,
																						FONT_END, 447, 257,
																						FONT_BEGIN, 367, 674,
																						FONT_NEXT, 447, 257,
																						FONT_END, 331, 532,
																						FONT_BEGIN, 367, 674,
																						FONT_NEXT, 331, 532,
																						FONT_NEXT, 347, 674,
																						FONT_END, 139, 183,
																						FONT_BEGIN, 139, 183,
																						FONT_NEXT, 331, 532,
																						FONT_END, 216, 257,
																						FONT_BEGIN, 139, 183,
																						FONT_NEXT, 216, 257,
																						FONT_END, 199, 216,
																						FONT_BEGIN, 139, 183,
																						FONT_NEXT, 199, 216,
																						FONT_END, 153, 99,
																						FONT_BEGIN, 139, 183,
																						FONT_NEXT, 153, 99,
																						FONT_END, 145, 61,
																						FONT_BEGIN, 139, 183,
																						FONT_NEXT, 145, 61,
																						FONT_NEXT, 104, 104,
																						FONT_END, 84, 67,
																						FONT_BEGIN, 84, 67,
																						FONT_NEXT, 145, 61,
																						FONT_NEXT, 66, 41,
																						FONT_NEXT, 15, 0,
																						FONT_NEXT, 39, 23,
																						FONT_END, 15, 19,
																						FONT_BEGIN, 706, 19,
																						FONT_NEXT, 706, 0,
																						FONT_NEXT, 677, 23,
																						FONT_NEXT, 451, 0,
																						FONT_NEXT, 656, 36,
																						FONT_END, 637, 62,
																						FONT_BEGIN, 216, 257,
																						FONT_NEXT, 447, 257,
																						FONT_NEXT, 199, 216,
																						FONT_END, 461, 216,
																						FONT_BEGIN, 213, 19,
																						FONT_NEXT, 213, 0,
																						FONT_NEXT, 171, 24,
																						FONT_NEXT, 15, 0,
																						FONT_NEXT, 152, 36,
																						FONT_END, 145, 61,
																						FONT_ADVANCE, 722, 0
																				},
																					{
																						66,
																							FONT_BEGIN, 17, 19,
																							FONT_NEXT, 67, 23,
																							FONT_NEXT, 17, 0,
																							FONT_END, 215, 78,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 67, 23,
																							FONT_END, 96, 38,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 96, 38,
																							FONT_END, 109, 65,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 109, 65,
																							FONT_END, 113, 109,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 113, 109,
																							FONT_END, 113, 553,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 113, 553,
																							FONT_END, 109, 597,
																							FONT_BEGIN, 215, 78,
																							FONT_NEXT, 109, 597,
																							FONT_NEXT, 215, 326,
																							FONT_END, 215, 365,
																							FONT_BEGIN, 215, 365,
																							FONT_NEXT, 109, 597,
																							FONT_NEXT, 215, 595,
																							FONT_NEXT, 95, 624,
																							FONT_NEXT, 17, 662,
																							FONT_NEXT, 66, 637,
																							FONT_END, 17, 643,
																							FONT_BEGIN, 215, 326,
																							FONT_NEXT, 215, 365,
																							FONT_NEXT, 276, 325,
																							FONT_NEXT, 310, 366,
																							FONT_NEXT, 319, 322,
																							FONT_NEXT, 344, 368,
																							FONT_NEXT, 377, 311,
																							FONT_NEXT, 394, 381,
																							FONT_NEXT, 414, 295,
																							FONT_NEXT, 426, 347,
																							FONT_NEXT, 447, 270,
																							FONT_END, 469, 232,
																							FONT_BEGIN, 469, 232,
																							FONT_NEXT, 426, 347,
																							FONT_END, 488, 327,
																							FONT_BEGIN, 469, 232,
																							FONT_NEXT, 488, 327,
																							FONT_NEXT, 478, 180,
																							FONT_NEXT, 482, 22,
																							FONT_NEXT, 472, 135,
																							FONT_NEXT, 425, 6,
																							FONT_NEXT, 458, 101,
																							FONT_END, 436, 75,
																							FONT_BEGIN, 436, 75,
																							FONT_NEXT, 425, 6,
																							FONT_NEXT, 409, 58,
																							FONT_NEXT, 351, 0,
																							FONT_NEXT, 348, 40,
																							FONT_NEXT, 17, 0,
																							FONT_NEXT, 291, 37,
																							FONT_END, 254, 38,
																							FONT_BEGIN, 254, 38,
																							FONT_NEXT, 17, 0,
																							FONT_NEXT, 231, 43,
																							FONT_END, 218, 55,
																							FONT_BEGIN, 218, 55,
																							FONT_NEXT, 17, 0,
																							FONT_END, 215, 78,
																							FONT_BEGIN, 218, 55,
																							FONT_END, 215, 78,
																							FONT_BEGIN, 215, 595,
																							FONT_NEXT, 17, 662,
																							FONT_NEXT, 218, 615,
																							FONT_END, 237, 624,
																							FONT_BEGIN, 237, 624,
																							FONT_NEXT, 17, 662,
																							FONT_NEXT, 255, 624,
																							FONT_END, 282, 625,
																							FONT_BEGIN, 282, 625,
																							FONT_NEXT, 17, 662,
																							FONT_END, 297, 662,
																							FONT_BEGIN, 282, 625,
																							FONT_NEXT, 297, 662,
																							FONT_NEXT, 339, 620,
																							FONT_NEXT, 374, 657,
																							FONT_NEXT, 396, 600,
																							FONT_END, 420, 582,
																							FONT_BEGIN, 420, 582,
																							FONT_NEXT, 374, 657,
																							FONT_NEXT, 439, 558,
																							FONT_NEXT, 460, 635,
																							FONT_NEXT, 452, 527,
																							FONT_END, 457, 487,
																							FONT_BEGIN, 457, 487,
																							FONT_NEXT, 460, 635,
																							FONT_END, 465, 360,
																							FONT_BEGIN, 457, 487,
																							FONT_NEXT, 465, 360,
																							FONT_NEXT, 451, 447,
																							FONT_NEXT, 426, 349,
																							FONT_NEXT, 437, 417,
																							FONT_END, 394, 381,
																							FONT_BEGIN, 394, 381,
																							FONT_NEXT, 426, 349,
																							FONT_END, 426, 347,
																							FONT_BEGIN, 394, 381,
																							FONT_END, 426, 347,
																							FONT_BEGIN, 559, 493,
																							FONT_NEXT, 555, 455,
																							FONT_NEXT, 551, 544,
																							FONT_NEXT, 544, 425,
																							FONT_NEXT, 530, 584,
																							FONT_NEXT, 509, 383,
																							FONT_NEXT, 499, 614,
																							FONT_NEXT, 465, 360,
																							FONT_END, 460, 635,
																							FONT_BEGIN, 593, 176,
																							FONT_NEXT, 586, 132,
																							FONT_NEXT, 589, 212,
																							FONT_END, 579, 244,
																							FONT_BEGIN, 579, 244,
																							FONT_NEXT, 586, 132,
																							FONT_END, 574, 103,
																							FONT_BEGIN, 579, 244,
																							FONT_NEXT, 574, 103,
																							FONT_NEXT, 542, 294,
																							FONT_NEXT, 555, 73,
																							FONT_END, 525, 45,
																							FONT_BEGIN, 542, 294,
																							FONT_NEXT, 525, 45,
																							FONT_NEXT, 488, 327,
																							FONT_END, 482, 22,
																							FONT_ADVANCE, 667, 0
																					},
																					{
																						67,
																							FONT_BEGIN, 620, 451,
																							FONT_NEXT, 597, 451,
																							FONT_NEXT, 611, 676,
																							FONT_END, 590, 676,
																							FONT_BEGIN, 590, 676,
																							FONT_NEXT, 597, 451,
																							FONT_END, 569, 524,
																							FONT_BEGIN, 590, 676,
																							FONT_NEXT, 569, 524,
																							FONT_NEXT, 580, 656,
																							FONT_END, 565, 647,
																							FONT_BEGIN, 565, 647,
																							FONT_NEXT, 569, 524,
																							FONT_END, 523, 582,
																							FONT_BEGIN, 565, 647,
																							FONT_NEXT, 523, 582,
																							FONT_NEXT, 544, 643,
																							FONT_END, 519, 648,
																							FONT_BEGIN, 519, 648,
																							FONT_NEXT, 523, 582,
																							FONT_END, 459, 621,
																							FONT_BEGIN, 519, 648,
																							FONT_NEXT, 459, 621,
																							FONT_NEXT, 481, 659,
																							FONT_END, 429, 670,
																							FONT_BEGIN, 429, 670,
																							FONT_NEXT, 459, 621,
																							FONT_END, 420, 632,
																							FONT_BEGIN, 429, 670,
																							FONT_NEXT, 420, 632,
																							FONT_NEXT, 368, 676,
																							FONT_NEXT, 377, 636,
																							FONT_END, 299, 620,
																							FONT_BEGIN, 368, 676,
																							FONT_NEXT, 299, 620,
																							FONT_NEXT, 304, 670,
																							FONT_END, 242, 653,
																							FONT_BEGIN, 242, 653,
																							FONT_NEXT, 299, 620,
																							FONT_END, 261, 601,
																							FONT_BEGIN, 242, 653,
																							FONT_NEXT, 261, 601,
																							FONT_END, 227, 575,
																							FONT_BEGIN, 242, 653,
																							FONT_NEXT, 227, 575,
																							FONT_NEXT, 184, 625,
																							FONT_NEXT, 197, 540,
																							FONT_END, 170, 491,
																							FONT_BEGIN, 184, 625,
																							FONT_NEXT, 170, 491,
																							FONT_NEXT, 133, 587,
																							FONT_NEXT, 151, 422,
																							FONT_END, 144, 329,
																							FONT_BEGIN, 133, 587,
																							FONT_NEXT, 144, 329,
																							FONT_END, 134, 65,
																							FONT_BEGIN, 133, 587,
																							FONT_NEXT, 134, 65,
																							FONT_END, 91, 112,
																							FONT_BEGIN, 133, 587,
																							FONT_NEXT, 91, 112,
																							FONT_NEXT, 90, 538,
																							FONT_NEXT, 57, 172,
																							FONT_NEXT, 56, 479,
																							FONT_NEXT, 35, 244,
																							FONT_NEXT, 35, 410,
																							FONT_END, 28, 331,
																							FONT_BEGIN, 633, 113,
																							FONT_NEXT, 628, 106,
																							FONT_NEXT, 615, 131,
																							FONT_NEXT, 616, 92,
																							FONT_END, 595, 71,
																							FONT_BEGIN, 615, 131,
																							FONT_NEXT, 595, 71,
																							FONT_NEXT, 583, 102,
																							FONT_NEXT, 566, 48,
																							FONT_NEXT, 536, 69,
																							FONT_NEXT, 528, 25,
																							FONT_NEXT, 471, 41,
																							FONT_NEXT, 481, 5,
																							FONT_END, 426, -9,
																							FONT_BEGIN, 471, 41,
																							FONT_NEXT, 426, -9,
																							FONT_NEXT, 389, 30,
																							FONT_NEXT, 362, -14,
																							FONT_NEXT, 342, 34,
																							FONT_NEXT, 301, -10,
																							FONT_NEXT, 297, 46,
																							FONT_NEXT, 241, 4,
																							FONT_NEXT, 256, 68,
																							FONT_END, 219, 100,
																							FONT_BEGIN, 219, 100,
																							FONT_NEXT, 241, 4,
																							FONT_END, 185, 29,
																							FONT_BEGIN, 219, 100,
																							FONT_NEXT, 185, 29,
																							FONT_NEXT, 188, 141,
																							FONT_END, 164, 192,
																							FONT_BEGIN, 164, 192,
																							FONT_NEXT, 185, 29,
																							FONT_END, 134, 65,
																							FONT_BEGIN, 164, 192,
																							FONT_NEXT, 134, 65,
																							FONT_NEXT, 149, 255,
																							FONT_END, 144, 329,
																							FONT_ADVANCE, 667, 0
																					},
																						{
																							68,
																								FONT_BEGIN, 16, 643,
																								FONT_NEXT, 16, 662,
																								FONT_NEXT, 62, 636,
																								FONT_END, 88, 623,
																								FONT_BEGIN, 88, 623,
																								FONT_NEXT, 16, 662,
																								FONT_END, 206, 586,
																								FONT_BEGIN, 88, 623,
																								FONT_NEXT, 206, 586,
																								FONT_NEXT, 101, 597,
																								FONT_NEXT, 206, 78,
																								FONT_NEXT, 104, 553,
																								FONT_END, 104, 109,
																								FONT_BEGIN, 104, 109,
																								FONT_NEXT, 206, 78,
																								FONT_NEXT, 100, 65,
																								FONT_END, 88, 39,
																								FONT_BEGIN, 88, 39,
																								FONT_NEXT, 206, 78,
																								FONT_NEXT, 61, 24,
																								FONT_NEXT, 16, 0,
																								FONT_END, 16, 19,
																								FONT_BEGIN, 685, 329,
																								FONT_NEXT, 681, 276,
																								FONT_NEXT, 677, 407,
																								FONT_NEXT, 667, 221,
																								FONT_NEXT, 656, 475,
																								FONT_NEXT, 643, 166,
																								FONT_NEXT, 622, 532,
																								FONT_NEXT, 607, 114,
																								FONT_NEXT, 575, 579,
																								FONT_NEXT, 576, 328,
																								FONT_END, 571, 390,
																								FONT_BEGIN, 575, 579,
																								FONT_NEXT, 571, 390,
																								FONT_NEXT, 517, 615,
																								FONT_NEXT, 555, 449,
																								FONT_END, 526, 505,
																								FONT_BEGIN, 517, 615,
																								FONT_NEXT, 526, 505,
																								FONT_END, 483, 555,
																								FONT_BEGIN, 517, 615,
																								FONT_NEXT, 483, 555,
																								FONT_NEXT, 449, 641,
																								FONT_NEXT, 444, 584,
																								FONT_NEXT, 372, 656,
																								FONT_NEXT, 397, 606,
																								FONT_END, 335, 620,
																								FONT_BEGIN, 372, 656,
																								FONT_NEXT, 335, 620,
																								FONT_NEXT, 286, 662,
																								FONT_NEXT, 253, 625,
																								FONT_NEXT, 16, 662,
																								FONT_NEXT, 225, 621,
																								FONT_END, 211, 613,
																								FONT_BEGIN, 16, 662,
																								FONT_NEXT, 211, 613,
																								FONT_END, 206, 586,
																								FONT_BEGIN, 206, 78,
																								FONT_NEXT, 208, 57,
																								FONT_NEXT, 16, 0,
																								FONT_NEXT, 218, 44,
																								FONT_END, 236, 38,
																								FONT_BEGIN, 16, 0,
																								FONT_NEXT, 236, 38,
																								FONT_END, 266, 37,
																								FONT_BEGIN, 16, 0,
																								FONT_NEXT, 266, 37,
																								FONT_END, 297, 37,
																								FONT_BEGIN, 16, 0,
																								FONT_NEXT, 297, 37,
																								FONT_NEXT, 300, 0,
																								FONT_NEXT, 341, 41,
																								FONT_END, 392, 52,
																								FONT_BEGIN, 300, 0,
																								FONT_NEXT, 392, 52,
																								FONT_NEXT, 404, 8,
																								FONT_NEXT, 445, 73,
																								FONT_NEXT, 489, 32,
																								FONT_NEXT, 494, 108,
																								FONT_END, 536, 159,
																								FONT_BEGIN, 489, 32,
																								FONT_NEXT, 536, 159,
																								FONT_NEXT, 556, 69,
																								FONT_NEXT, 565, 231,
																								FONT_END, 573, 276,
																								FONT_BEGIN, 556, 69,
																								FONT_NEXT, 573, 276,
																								FONT_NEXT, 607, 114,
																								FONT_END, 576, 328,
																								FONT_ADVANCE, 722, 0
																						},
																						{
																							69,
																								FONT_BEGIN, 12, 19,
																								FONT_NEXT, 52, 23,
																								FONT_NEXT, 12, 0,
																								FONT_NEXT, 79, 36,
																								FONT_NEXT, 201, 80,
																								FONT_NEXT, 94, 63,
																								FONT_END, 99, 109,
																								FONT_BEGIN, 201, 80,
																								FONT_NEXT, 99, 109,
																								FONT_END, 99, 553,
																								FONT_BEGIN, 201, 80,
																								FONT_NEXT, 99, 553,
																								FONT_END, 94, 599,
																								FONT_BEGIN, 201, 80,
																								FONT_NEXT, 94, 599,
																								FONT_NEXT, 201, 328,
																								FONT_NEXT, 201, 368,
																								FONT_NEXT, 355, 326,
																								FONT_NEXT, 355, 368,
																								FONT_NEXT, 405, 321,
																								FONT_NEXT, 408, 372,
																								FONT_NEXT, 436, 308,
																								FONT_NEXT, 439, 388,
																								FONT_NEXT, 454, 279,
																								FONT_NEXT, 455, 418,
																								FONT_NEXT, 465, 231,
																								FONT_NEXT, 465, 463,
																								FONT_NEXT, 488, 231,
																								FONT_END, 488, 463,
																								FONT_BEGIN, 597, 169,
																								FONT_NEXT, 552, 0,
																								FONT_NEXT, 569, 169,
																								FONT_END, 533, 105,
																								FONT_BEGIN, 533, 105,
																								FONT_NEXT, 552, 0,
																								FONT_NEXT, 489, 65,
																								FONT_END, 430, 44,
																								FONT_BEGIN, 430, 44,
																								FONT_NEXT, 552, 0,
																								FONT_END, 12, 0,
																								FONT_BEGIN, 430, 44,
																								FONT_NEXT, 12, 0,
																								FONT_NEXT, 350, 38,
																								FONT_END, 300, 38,
																								FONT_BEGIN, 300, 38,
																								FONT_NEXT, 12, 0,
																								FONT_NEXT, 263, 38,
																								FONT_END, 237, 40,
																								FONT_BEGIN, 237, 40,
																								FONT_NEXT, 12, 0,
																								FONT_NEXT, 219, 43,
																								FONT_END, 203, 56,
																								FONT_BEGIN, 203, 56,
																								FONT_NEXT, 12, 0,
																								FONT_END, 201, 80,
																								FONT_BEGIN, 203, 56,
																								FONT_END, 201, 80,
																								FONT_BEGIN, 12, 643,
																								FONT_NEXT, 12, 662,
																								FONT_NEXT, 52, 637,
																								FONT_END, 79, 625,
																								FONT_BEGIN, 79, 625,
																								FONT_NEXT, 12, 662,
																								FONT_END, 201, 590,
																								FONT_BEGIN, 79, 625,
																								FONT_NEXT, 201, 590,
																								FONT_NEXT, 94, 599,
																								FONT_END, 201, 368,
																								FONT_BEGIN, 546, 519,
																								FONT_NEXT, 521, 519,
																								FONT_NEXT, 543, 662,
																								FONT_NEXT, 506, 572,
																								FONT_END, 481, 604,
																								FONT_BEGIN, 543, 662,
																								FONT_NEXT, 481, 604,
																								FONT_END, 437, 620,
																								FONT_BEGIN, 543, 662,
																								FONT_NEXT, 437, 620,
																								FONT_NEXT, 12, 662,
																								FONT_NEXT, 369, 624,
																								FONT_END, 234, 624,
																								FONT_BEGIN, 12, 662,
																								FONT_NEXT, 234, 624,
																								FONT_END, 209, 619,
																								FONT_BEGIN, 12, 662,
																								FONT_NEXT, 209, 619,
																								FONT_END, 203, 609,
																								FONT_BEGIN, 12, 662,
																								FONT_NEXT, 203, 609,
																								FONT_END, 201, 590,
																								FONT_ADVANCE, 611, 0
																						},
																							{
																								70,
																									FONT_BEGIN, 12, 643,
																									FONT_NEXT, 12, 662,
																									FONT_NEXT, 52, 637,
																									FONT_END, 79, 625,
																									FONT_BEGIN, 79, 625,
																									FONT_NEXT, 12, 662,
																									FONT_END, 201, 590,
																									FONT_BEGIN, 79, 625,
																									FONT_NEXT, 201, 590,
																									FONT_NEXT, 94, 599,
																									FONT_NEXT, 201, 368,
																									FONT_END, 201, 328,
																									FONT_BEGIN, 94, 599,
																									FONT_NEXT, 201, 328,
																									FONT_END, 201, 109,
																									FONT_BEGIN, 94, 599,
																									FONT_NEXT, 201, 109,
																									FONT_NEXT, 99, 553,
																									FONT_END, 99, 120,
																									FONT_BEGIN, 99, 120,
																									FONT_NEXT, 201, 109,
																									FONT_NEXT, 95, 70,
																									FONT_END, 83, 40,
																									FONT_BEGIN, 83, 40,
																									FONT_NEXT, 201, 109,
																									FONT_NEXT, 57, 24,
																									FONT_NEXT, 205, 63,
																									FONT_END, 12, 0,
																									FONT_BEGIN, 57, 24,
																									FONT_NEXT, 12, 0,
																									FONT_END, 12, 19,
																									FONT_BEGIN, 546, 519,
																									FONT_NEXT, 521, 519,
																									FONT_NEXT, 543, 662,
																									FONT_NEXT, 506, 572,
																									FONT_END, 481, 604,
																									FONT_BEGIN, 543, 662,
																									FONT_NEXT, 481, 604,
																									FONT_END, 437, 620,
																									FONT_BEGIN, 543, 662,
																									FONT_NEXT, 437, 620,
																									FONT_NEXT, 12, 662,
																									FONT_NEXT, 369, 624,
																									FONT_END, 233, 624,
																									FONT_BEGIN, 12, 662,
																									FONT_NEXT, 233, 624,
																									FONT_END, 207, 618,
																									FONT_BEGIN, 12, 662,
																									FONT_NEXT, 207, 618,
																									FONT_END, 202, 607,
																									FONT_BEGIN, 12, 662,
																									FONT_NEXT, 202, 607,
																									FONT_END, 201, 590,
																									FONT_BEGIN, 479, 463,
																									FONT_NEXT, 479, 231,
																									FONT_NEXT, 456, 463,
																									FONT_NEXT, 456, 231,
																									FONT_NEXT, 446, 416,
																									FONT_NEXT, 444, 281,
																									FONT_NEXT, 428, 387,
																									FONT_NEXT, 425, 309,
																									FONT_NEXT, 396, 372,
																									FONT_NEXT, 393, 322,
																									FONT_NEXT, 346, 368,
																									FONT_NEXT, 346, 326,
																									FONT_NEXT, 201, 368,
																									FONT_END, 201, 328,
																									FONT_BEGIN, 292, 19,
																									FONT_NEXT, 292, 0,
																									FONT_NEXT, 247, 23,
																									FONT_NEXT, 12, 0,
																									FONT_NEXT, 219, 37,
																									FONT_END, 205, 63,
																									FONT_ADVANCE, 556, 0
																							},
																							{
																								71,
																									FONT_BEGIN, 630, 465,
																									FONT_NEXT, 607, 465,
																									FONT_NEXT, 622, 676,
																									FONT_END, 600, 676,
																									FONT_BEGIN, 600, 676,
																									FONT_NEXT, 607, 465,
																									FONT_END, 581, 522,
																									FONT_BEGIN, 600, 676,
																									FONT_NEXT, 581, 522,
																									FONT_NEXT, 591, 659,
																									FONT_END, 577, 649,
																									FONT_BEGIN, 577, 649,
																									FONT_NEXT, 581, 522,
																									FONT_END, 539, 577,
																									FONT_BEGIN, 577, 649,
																									FONT_NEXT, 539, 577,
																									FONT_NEXT, 553, 643,
																									FONT_END, 525, 648,
																									FONT_BEGIN, 525, 648,
																									FONT_NEXT, 539, 577,
																									FONT_END, 477, 619,
																									FONT_BEGIN, 525, 648,
																									FONT_NEXT, 477, 619,
																									FONT_NEXT, 489, 659,
																									FONT_END, 440, 670,
																									FONT_BEGIN, 440, 670,
																									FONT_NEXT, 477, 619,
																									FONT_END, 436, 631,
																									FONT_BEGIN, 440, 670,
																									FONT_NEXT, 436, 631,
																									FONT_NEXT, 374, 676,
																									FONT_NEXT, 388, 636,
																									FONT_END, 310, 622,
																									FONT_BEGIN, 374, 676,
																									FONT_NEXT, 310, 622,
																									FONT_NEXT, 298, 668,
																									FONT_NEXT, 269, 602,
																									FONT_NEXT, 231, 647,
																									FONT_NEXT, 231, 572,
																									FONT_NEXT, 172, 614,
																									FONT_NEXT, 197, 531,
																									FONT_END, 170, 476,
																									FONT_BEGIN, 172, 614,
																									FONT_NEXT, 170, 476,
																									FONT_NEXT, 123, 571,
																									FONT_NEXT, 152, 406,
																									FONT_END, 146, 320,
																									FONT_BEGIN, 123, 571,
																									FONT_NEXT, 146, 320,
																									FONT_END, 115, 93,
																									FONT_BEGIN, 123, 571,
																									FONT_NEXT, 115, 93,
																									FONT_NEXT, 84, 520,
																									FONT_NEXT, 79, 137,
																									FONT_NEXT, 55, 464,
																									FONT_NEXT, 53, 189,
																									FONT_NEXT, 38, 403,
																									FONT_NEXT, 37, 255,
																									FONT_END, 32, 341,
																									FONT_BEGIN, 454, 336,
																									FONT_NEXT, 454, 354,
																									FONT_NEXT, 496, 331,
																									FONT_END, 523, 320,
																									FONT_BEGIN, 523, 320,
																									FONT_NEXT, 454, 354,
																									FONT_END, 643, 300,
																									FONT_BEGIN, 523, 320,
																									FONT_NEXT, 643, 300,
																									FONT_END, 639, 259,
																									FONT_BEGIN, 523, 320,
																									FONT_NEXT, 639, 259,
																									FONT_NEXT, 537, 295,
																									FONT_NEXT, 639, 58,
																									FONT_END, 633, 50,
																									FONT_BEGIN, 537, 295,
																									FONT_NEXT, 633, 50,
																									FONT_END, 616, 40,
																									FONT_BEGIN, 537, 295,
																									FONT_NEXT, 616, 40,
																									FONT_NEXT, 542, 247,
																									FONT_NEXT, 558, 16,
																									FONT_NEXT, 542, 85,
																									FONT_END, 530, 60,
																									FONT_BEGIN, 530, 60,
																									FONT_NEXT, 558, 16,
																									FONT_END, 480, -5,
																									FONT_BEGIN, 530, 60,
																									FONT_NEXT, 480, -5,
																									FONT_NEXT, 499, 42,
																									FONT_END, 455, 30,
																									FONT_BEGIN, 455, 30,
																									FONT_NEXT, 480, -5,
																									FONT_END, 396, -14,
																									FONT_BEGIN, 455, 30,
																									FONT_NEXT, 396, -14,
																									FONT_NEXT, 405, 26,
																									FONT_END, 349, 31,
																									FONT_BEGIN, 349, 31,
																									FONT_NEXT, 396, -14,
																									FONT_END, 320, -9,
																									FONT_BEGIN, 349, 31,
																									FONT_NEXT, 320, -9,
																									FONT_NEXT, 299, 46,
																									FONT_NEXT, 245, 10,
																									FONT_NEXT, 254, 71,
																									FONT_END, 217, 105,
																									FONT_BEGIN, 217, 105,
																									FONT_NEXT, 245, 10,
																									FONT_END, 174, 43,
																									FONT_BEGIN, 217, 105,
																									FONT_NEXT, 174, 43,
																									FONT_NEXT, 187, 147,
																									FONT_END, 164, 197,
																									FONT_BEGIN, 164, 197,
																									FONT_NEXT, 174, 43,
																									FONT_END, 115, 93,
																									FONT_BEGIN, 164, 197,
																									FONT_NEXT, 115, 93,
																									FONT_NEXT, 150, 255,
																									FONT_END, 146, 320,
																									FONT_BEGIN, 709, 354,
																									FONT_NEXT, 709, 336,
																									FONT_NEXT, 454, 354,
																									FONT_NEXT, 679, 332,
																									FONT_END, 657, 322,
																									FONT_BEGIN, 454, 354,
																									FONT_NEXT, 657, 322,
																									FONT_END, 643, 300,
																									FONT_ADVANCE, 722, 0
																							},
																								{
																									72,
																										FONT_BEGIN, 19, 643,
																										FONT_NEXT, 19, 662,
																										FONT_NEXT, 63, 636,
																										FONT_END, 90, 623,
																										FONT_BEGIN, 90, 623,
																										FONT_NEXT, 19, 662,
																										FONT_END, 212, 596,
																										FONT_BEGIN, 90, 623,
																										FONT_NEXT, 212, 596,
																										FONT_END, 209, 553,
																										FONT_BEGIN, 90, 623,
																										FONT_NEXT, 209, 553,
																										FONT_NEXT, 103, 597,
																										FONT_NEXT, 209, 359,
																										FONT_END, 209, 315,
																										FONT_BEGIN, 103, 597,
																										FONT_NEXT, 209, 315,
																										FONT_END, 209, 109,
																										FONT_BEGIN, 103, 597,
																										FONT_NEXT, 209, 109,
																										FONT_NEXT, 107, 553,
																										FONT_END, 107, 120,
																										FONT_BEGIN, 107, 120,
																										FONT_NEXT, 209, 109,
																										FONT_NEXT, 105, 73,
																										FONT_END, 94, 42,
																										FONT_BEGIN, 94, 42,
																										FONT_NEXT, 209, 109,
																										FONT_NEXT, 68, 25,
																										FONT_NEXT, 211, 67,
																										FONT_END, 19, 0,
																										FONT_BEGIN, 68, 25,
																										FONT_NEXT, 19, 0,
																										FONT_END, 19, 19,
																										FONT_BEGIN, 297, 662,
																										FONT_NEXT, 297, 643,
																										FONT_NEXT, 19, 662,
																										FONT_NEXT, 251, 636,
																										FONT_END, 224, 622,
																										FONT_BEGIN, 19, 662,
																										FONT_NEXT, 224, 622,
																										FONT_END, 212, 596,
																										FONT_BEGIN, 424, 643,
																										FONT_NEXT, 424, 662,
																										FONT_NEXT, 468, 636,
																										FONT_END, 495, 623,
																										FONT_BEGIN, 495, 623,
																										FONT_NEXT, 424, 662,
																										FONT_END, 617, 596,
																										FONT_BEGIN, 495, 623,
																										FONT_NEXT, 617, 596,
																										FONT_END, 614, 553,
																										FONT_BEGIN, 495, 623,
																										FONT_NEXT, 614, 553,
																										FONT_NEXT, 508, 597,
																										FONT_NEXT, 614, 109,
																										FONT_NEXT, 512, 553,
																										FONT_NEXT, 512, 120,
																										FONT_NEXT, 512, 359,
																										FONT_NEXT, 512, 315,
																										FONT_NEXT, 209, 359,
																										FONT_END, 209, 315,
																										FONT_BEGIN, 702, 662,
																										FONT_NEXT, 702, 643,
																										FONT_NEXT, 424, 662,
																										FONT_NEXT, 656, 636,
																										FONT_END, 629, 622,
																										FONT_BEGIN, 424, 662,
																										FONT_NEXT, 629, 622,
																										FONT_END, 617, 596,
																										FONT_BEGIN, 424, 19,
																										FONT_NEXT, 473, 25,
																										FONT_NEXT, 424, 0,
																										FONT_END, 616, 67,
																										FONT_BEGIN, 616, 67,
																										FONT_NEXT, 473, 25,
																										FONT_NEXT, 614, 109,
																										FONT_NEXT, 499, 42,
																										FONT_END, 510, 73,
																										FONT_BEGIN, 614, 109,
																										FONT_NEXT, 510, 73,
																										FONT_END, 512, 120,
																										FONT_BEGIN, 702, 19,
																										FONT_NEXT, 702, 0,
																										FONT_NEXT, 655, 25,
																										FONT_NEXT, 424, 0,
																										FONT_NEXT, 628, 41,
																										FONT_END, 616, 67,
																										FONT_BEGIN, 297, 19,
																										FONT_NEXT, 297, 0,
																										FONT_NEXT, 250, 25,
																										FONT_NEXT, 19, 0,
																										FONT_NEXT, 223, 41,
																										FONT_END, 211, 67,
																										FONT_ADVANCE, 722, 0
																								},
																								{
																									73,
																										FONT_BEGIN, 18, 643,
																										FONT_NEXT, 18, 662,
																										FONT_NEXT, 68, 637,
																										FONT_END, 97, 624,
																										FONT_BEGIN, 97, 624,
																										FONT_NEXT, 18, 662,
																										FONT_END, 222, 600,
																										FONT_BEGIN, 97, 624,
																										FONT_NEXT, 222, 600,
																										FONT_END, 217, 553,
																										FONT_BEGIN, 97, 624,
																										FONT_NEXT, 217, 553,
																										FONT_NEXT, 111, 597,
																										FONT_NEXT, 217, 109,
																										FONT_NEXT, 115, 553,
																										FONT_END, 115, 109,
																										FONT_BEGIN, 115, 109,
																										FONT_NEXT, 217, 109,
																										FONT_NEXT, 111, 65,
																										FONT_END, 98, 38,
																										FONT_BEGIN, 98, 38,
																										FONT_NEXT, 217, 109,
																										FONT_END, 221, 62,
																										FONT_BEGIN, 98, 38,
																										FONT_NEXT, 221, 62,
																										FONT_NEXT, 69, 23,
																										FONT_NEXT, 18, 0,
																										FONT_END, 18, 19,
																										FONT_BEGIN, 315, 662,
																										FONT_NEXT, 315, 643,
																										FONT_NEXT, 18, 662,
																										FONT_NEXT, 268, 638,
																										FONT_END, 238, 626,
																										FONT_BEGIN, 18, 662,
																										FONT_NEXT, 238, 626,
																										FONT_END, 222, 600,
																										FONT_BEGIN, 315, 19,
																										FONT_NEXT, 315, 0,
																										FONT_NEXT, 268, 22,
																										FONT_NEXT, 18, 0,
																										FONT_NEXT, 238, 35,
																										FONT_END, 221, 62,
																										FONT_ADVANCE, 333, 0
																								},
																									{
																										74,
																											FONT_BEGIN, 83, 643,
																											FONT_NEXT, 83, 662,
																											FONT_NEXT, 131, 636,
																											FONT_END, 159, 622,
																											FONT_BEGIN, 159, 622,
																											FONT_NEXT, 83, 662,
																											FONT_END, 281, 597,
																											FONT_BEGIN, 159, 622,
																											FONT_NEXT, 281, 597,
																											FONT_END, 278, 553,
																											FONT_BEGIN, 159, 622,
																											FONT_NEXT, 278, 553,
																											FONT_NEXT, 172, 596,
																											FONT_NEXT, 278, 183,
																											FONT_END, 271, 118,
																											FONT_BEGIN, 172, 596,
																											FONT_NEXT, 271, 118,
																											FONT_END, 255, 70,
																											FONT_BEGIN, 172, 596,
																											FONT_NEXT, 255, 70,
																											FONT_END, 231, 35,
																											FONT_BEGIN, 172, 596,
																											FONT_NEXT, 231, 35,
																											FONT_NEXT, 176, 553,
																											FONT_NEXT, 204, 11,
																											FONT_NEXT, 176, 90,
																											FONT_END, 173, 57,
																											FONT_BEGIN, 173, 57,
																											FONT_NEXT, 204, 11,
																											FONT_END, 147, -11,
																											FONT_BEGIN, 173, 57,
																											FONT_NEXT, 147, -11,
																											FONT_NEXT, 165, 37,
																											FONT_END, 153, 27,
																											FONT_BEGIN, 153, 27,
																											FONT_NEXT, 147, -11,
																											FONT_NEXT, 137, 24,
																											FONT_NEXT, 124, -14,
																											FONT_NEXT, 117, 37,
																											FONT_NEXT, 110, -14,
																											FONT_NEXT, 107, 66,
																											FONT_NEXT, 59, -6,
																											FONT_NEXT, 92, 94,
																											FONT_END, 77, 104,
																											FONT_BEGIN, 77, 104,
																											FONT_NEXT, 59, -6,
																											FONT_NEXT, 56, 108,
																											FONT_NEXT, 32, 11,
																											FONT_NEXT, 36, 103,
																											FONT_END, 22, 90,
																											FONT_BEGIN, 22, 90,
																											FONT_NEXT, 32, 11,
																											FONT_END, 15, 36,
																											FONT_BEGIN, 22, 90,
																											FONT_NEXT, 15, 36,
																											FONT_END, 10, 59,
																											FONT_BEGIN, 370, 662,
																											FONT_NEXT, 370, 643,
																											FONT_NEXT, 83, 662,
																											FONT_NEXT, 322, 637,
																											FONT_END, 294, 623,
																											FONT_BEGIN, 83, 662,
																											FONT_NEXT, 294, 623,
																											FONT_END, 281, 597,
																											FONT_ADVANCE, 389, 0
																									},
																									{
																										75,
																											FONT_BEGIN, 34, 643,
																											FONT_NEXT, 34, 662,
																											FONT_NEXT, 76, 637,
																											FONT_END, 104, 625,
																											FONT_BEGIN, 104, 625,
																											FONT_NEXT, 34, 662,
																											FONT_END, 229, 597,
																											FONT_BEGIN, 104, 625,
																											FONT_NEXT, 229, 597,
																											FONT_END, 226, 553,
																											FONT_BEGIN, 104, 625,
																											FONT_NEXT, 226, 553,
																											FONT_NEXT, 119, 599,
																											FONT_NEXT, 226, 348,
																											FONT_END, 226, 296,
																											FONT_BEGIN, 119, 599,
																											FONT_NEXT, 226, 296,
																											FONT_END, 226, 109,
																											FONT_BEGIN, 119, 599,
																											FONT_NEXT, 226, 109,
																											FONT_NEXT, 124, 553,
																											FONT_END, 124, 120,
																											FONT_BEGIN, 124, 120,
																											FONT_NEXT, 226, 109,
																											FONT_NEXT, 120, 70,
																											FONT_END, 108, 40,
																											FONT_BEGIN, 108, 40,
																											FONT_NEXT, 226, 109,
																											FONT_END, 229, 65,
																											FONT_BEGIN, 108, 40,
																											FONT_NEXT, 229, 65,
																											FONT_NEXT, 80, 24,
																											FONT_NEXT, 34, 0,
																											FONT_END, 34, 19,
																											FONT_BEGIN, 318, 662,
																											FONT_NEXT, 318, 643,
																											FONT_NEXT, 34, 662,
																											FONT_NEXT, 270, 637,
																											FONT_END, 242, 623,
																											FONT_BEGIN, 34, 662,
																											FONT_NEXT, 242, 623,
																											FONT_END, 229, 597,
																											FONT_BEGIN, 413, 643,
																											FONT_NEXT, 413, 662,
																											FONT_NEXT, 444, 641,
																											FONT_END, 465, 636,
																											FONT_BEGIN, 465, 636,
																											FONT_NEXT, 413, 662,
																											FONT_END, 568, 606,
																											FONT_BEGIN, 465, 636,
																											FONT_NEXT, 568, 606,
																											FONT_NEXT, 477, 627,
																											FONT_NEXT, 523, 565,
																											FONT_NEXT, 481, 612,
																											FONT_END, 472, 584,
																											FONT_BEGIN, 472, 584,
																											FONT_NEXT, 523, 565,
																											FONT_END, 333, 377,
																											FONT_BEGIN, 472, 584,
																											FONT_NEXT, 333, 377,
																											FONT_NEXT, 459, 567,
																											FONT_END, 438, 543,
																											FONT_BEGIN, 438, 543,
																											FONT_NEXT, 333, 377,
																											FONT_NEXT, 406, 511,
																											FONT_END, 361, 470,
																											FONT_BEGIN, 361, 470,
																											FONT_NEXT, 333, 377,
																											FONT_NEXT, 301, 416,
																											FONT_NEXT, 252, 317,
																											FONT_NEXT, 226, 348,
																											FONT_END, 226, 296,
																											FONT_BEGIN, 675, 662,
																											FONT_NEXT, 675, 643,
																											FONT_NEXT, 413, 662,
																											FONT_NEXT, 635, 638,
																											FONT_END, 603, 628,
																											FONT_BEGIN, 413, 662,
																											FONT_NEXT, 603, 628,
																											FONT_END, 568, 606,
																											FONT_BEGIN, 723, 19,
																											FONT_NEXT, 723, 0,
																											FONT_NEXT, 683, 24,
																											FONT_NEXT, 418, 0,
																											FONT_NEXT, 647, 45,
																											FONT_END, 609, 79,
																											FONT_BEGIN, 609, 79,
																											FONT_NEXT, 418, 0,
																											FONT_END, 472, 24,
																											FONT_BEGIN, 609, 79,
																											FONT_NEXT, 472, 24,
																											FONT_END, 484, 31,
																											FONT_BEGIN, 609, 79,
																											FONT_NEXT, 484, 31,
																											FONT_NEXT, 566, 127,
																											FONT_NEXT, 488, 44,
																											FONT_END, 483, 58,
																											FONT_BEGIN, 566, 127,
																											FONT_NEXT, 483, 58,
																											FONT_NEXT, 333, 377,
																											FONT_NEXT, 472, 78,
																											FONT_END, 435, 126,
																											FONT_BEGIN, 333, 377,
																											FONT_NEXT, 435, 126,
																											FONT_END, 392, 175,
																											FONT_BEGIN, 333, 377,
																											FONT_NEXT, 392, 175,
																											FONT_END, 358, 212,
																											FONT_BEGIN, 333, 377,
																											FONT_NEXT, 358, 212,
																											FONT_END, 252, 317,
																											FONT_BEGIN, 418, 19,
																											FONT_NEXT, 451, 20,
																											FONT_NEXT, 418, 0,
																											FONT_END, 472, 24,
																											FONT_BEGIN, 316, 19,
																											FONT_NEXT, 316, 0,
																											FONT_NEXT, 269, 24,
																											FONT_NEXT, 34, 0,
																											FONT_NEXT, 241, 39,
																											FONT_END, 229, 65,
																											FONT_ADVANCE, 722, 0
																									},
																										{
																											76,
																												FONT_BEGIN, 12, 19,
																												FONT_NEXT, 52, 23,
																												FONT_NEXT, 12, 0,
																												FONT_NEXT, 79, 36,
																												FONT_NEXT, 201, 80,
																												FONT_NEXT, 94, 63,
																												FONT_END, 99, 109,
																												FONT_BEGIN, 201, 80,
																												FONT_NEXT, 99, 109,
																												FONT_END, 99, 553,
																												FONT_BEGIN, 201, 80,
																												FONT_NEXT, 99, 553,
																												FONT_END, 94, 599,
																												FONT_BEGIN, 201, 80,
																												FONT_NEXT, 94, 599,
																												FONT_NEXT, 201, 553,
																												FONT_NEXT, 79, 625,
																												FONT_NEXT, 205, 599,
																												FONT_END, 12, 662,
																												FONT_BEGIN, 12, 662,
																												FONT_NEXT, 79, 625,
																												FONT_END, 52, 637,
																												FONT_BEGIN, 12, 662,
																												FONT_NEXT, 52, 637,
																												FONT_END, 12, 643,
																												FONT_BEGIN, 598, 174,
																												FONT_NEXT, 550, 0,
																												FONT_NEXT, 573, 174,
																												FONT_END, 551, 130,
																												FONT_BEGIN, 551, 130,
																												FONT_NEXT, 550, 0,
																												FONT_NEXT, 527, 97,
																												FONT_END, 501, 73,
																												FONT_BEGIN, 501, 73,
																												FONT_NEXT, 550, 0,
																												FONT_NEXT, 472, 57,
																												FONT_END, 399, 41,
																												FONT_BEGIN, 399, 41,
																												FONT_NEXT, 550, 0,
																												FONT_END, 12, 0,
																												FONT_BEGIN, 399, 41,
																												FONT_NEXT, 12, 0,
																												FONT_NEXT, 302, 39,
																												FONT_END, 251, 39,
																												FONT_BEGIN, 251, 39,
																												FONT_NEXT, 12, 0,
																												FONT_NEXT, 220, 44,
																												FONT_END, 205, 56,
																												FONT_BEGIN, 205, 56,
																												FONT_NEXT, 12, 0,
																												FONT_END, 201, 80,
																												FONT_BEGIN, 205, 56,
																												FONT_END, 201, 80,
																												FONT_BEGIN, 294, 662,
																												FONT_NEXT, 294, 643,
																												FONT_NEXT, 12, 662,
																												FONT_NEXT, 250, 638,
																												FONT_END, 222, 626,
																												FONT_BEGIN, 12, 662,
																												FONT_NEXT, 222, 626,
																												FONT_END, 205, 599,
																												FONT_ADVANCE, 611, 0
																										},
																										{
																											77,
																												FONT_BEGIN, 14, 643,
																												FONT_NEXT, 14, 662,
																												FONT_NEXT, 60, 638,
																												FONT_END, 89, 625,
																												FONT_BEGIN, 89, 625,
																												FONT_NEXT, 14, 662,
																												FONT_END, 212, 662,
																												FONT_BEGIN, 89, 625,
																												FONT_NEXT, 212, 662,
																												FONT_END, 155, 546,
																												FONT_BEGIN, 89, 625,
																												FONT_NEXT, 155, 546,
																												FONT_END, 153, 546,
																												FONT_BEGIN, 89, 625,
																												FONT_NEXT, 153, 546,
																												FONT_NEXT, 104, 599,
																												FONT_END, 109, 553,
																												FONT_BEGIN, 109, 553,
																												FONT_NEXT, 153, 546,
																												FONT_END, 153, 147,
																												FONT_BEGIN, 109, 553,
																												FONT_NEXT, 153, 147,
																												FONT_NEXT, 109, 147,
																												FONT_END, 107, 110,
																												FONT_BEGIN, 107, 110,
																												FONT_NEXT, 153, 147,
																												FONT_NEXT, 104, 82,
																												FONT_END, 89, 44,
																												FONT_BEGIN, 89, 44,
																												FONT_NEXT, 153, 147,
																												FONT_END, 157, 83,
																												FONT_BEGIN, 89, 44,
																												FONT_NEXT, 157, 83,
																												FONT_NEXT, 59, 25,
																												FONT_NEXT, 172, 45,
																												FONT_END, 12, 0,
																												FONT_BEGIN, 59, 25,
																												FONT_NEXT, 12, 0,
																												FONT_END, 12, 19,
																												FONT_BEGIN, 863, 662,
																												FONT_NEXT, 863, 643,
																												FONT_NEXT, 664, 662,
																												FONT_NEXT, 822, 637,
																												FONT_END, 795, 625,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 795, 625,
																												FONT_END, 780, 599,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 780, 599,
																												FONT_END, 776, 553,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 776, 553,
																												FONT_END, 776, 109,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 776, 109,
																												FONT_END, 674, 569,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 674, 569,
																												FONT_END, 672, 569,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 672, 569,
																												FONT_END, 418, 0,
																												FONT_BEGIN, 664, 662,
																												FONT_NEXT, 418, 0,
																												FONT_NEXT, 443, 157,
																												FONT_END, 212, 662,
																												FONT_BEGIN, 212, 662,
																												FONT_NEXT, 418, 0,
																												FONT_END, 404, 0,
																												FONT_BEGIN, 212, 662,
																												FONT_NEXT, 404, 0,
																												FONT_END, 155, 546,
																												FONT_BEGIN, 583, 19,
																												FONT_NEXT, 630, 24,
																												FONT_NEXT, 583, 0,
																												FONT_END, 780, 63,
																												FONT_BEGIN, 780, 63,
																												FONT_NEXT, 630, 24,
																												FONT_NEXT, 776, 109,
																												FONT_NEXT, 657, 40,
																												FONT_END, 670, 70,
																												FONT_BEGIN, 776, 109,
																												FONT_NEXT, 670, 70,
																												FONT_END, 674, 120,
																												FONT_BEGIN, 776, 109,
																												FONT_NEXT, 674, 120,
																												FONT_END, 674, 569,
																												FONT_BEGIN, 863, 19,
																												FONT_NEXT, 863, 0,
																												FONT_NEXT, 822, 23,
																												FONT_NEXT, 583, 0,
																												FONT_NEXT, 795, 36,
																												FONT_END, 780, 63,
																												FONT_BEGIN, 247, 19,
																												FONT_NEXT, 247, 0,
																												FONT_NEXT, 201, 25,
																												FONT_NEXT, 12, 0,
																												FONT_END, 172, 45,
																												FONT_ADVANCE, 889, 0
																										},
																											{
																												78,
																													FONT_BEGIN, 12, 643,
																													FONT_NEXT, 12, 662,
																													FONT_NEXT, 60, 634,
																													FONT_NEXT, 183, 662,
																													FONT_NEXT, 82, 617,
																													FONT_END, 109, 588,
																													FONT_BEGIN, 109, 588,
																													FONT_NEXT, 183, 662,
																													FONT_END, 155, 537,
																													FONT_BEGIN, 109, 588,
																													FONT_NEXT, 155, 537,
																													FONT_END, 153, 537,
																													FONT_BEGIN, 109, 588,
																													FONT_NEXT, 153, 537,
																													FONT_END, 153, 147,
																													FONT_BEGIN, 109, 588,
																													FONT_NEXT, 153, 147,
																													FONT_NEXT, 109, 147,
																													FONT_END, 107, 110,
																													FONT_BEGIN, 107, 110,
																													FONT_NEXT, 153, 147,
																													FONT_NEXT, 104, 82,
																													FONT_END, 89, 44,
																													FONT_BEGIN, 89, 44,
																													FONT_NEXT, 153, 147,
																													FONT_END, 157, 83,
																													FONT_BEGIN, 89, 44,
																													FONT_NEXT, 157, 83,
																													FONT_NEXT, 59, 25,
																													FONT_NEXT, 172, 45,
																													FONT_END, 12, 0,
																													FONT_BEGIN, 59, 25,
																													FONT_NEXT, 12, 0,
																													FONT_END, 12, 19,
																													FONT_BEGIN, 472, 643,
																													FONT_NEXT, 472, 662,
																													FONT_NEXT, 515, 637,
																													FONT_END, 545, 620,
																													FONT_BEGIN, 545, 620,
																													FONT_NEXT, 472, 662,
																													FONT_END, 617, 583,
																													FONT_BEGIN, 545, 620,
																													FONT_NEXT, 617, 583,
																													FONT_END, 613, 553,
																													FONT_BEGIN, 545, 620,
																													FONT_NEXT, 613, 553,
																													FONT_END, 612, 515,
																													FONT_BEGIN, 545, 620,
																													FONT_NEXT, 612, 515,
																													FONT_NEXT, 562, 582,
																													FONT_END, 566, 553,
																													FONT_BEGIN, 566, 553,
																													FONT_NEXT, 612, 515,
																													FONT_END, 612, -11,
																													FONT_BEGIN, 566, 553,
																													FONT_NEXT, 612, -11,
																													FONT_NEXT, 568, 515,
																													FONT_NEXT, 595, -11,
																													FONT_NEXT, 568, 181,
																													FONT_END, 566, 181,
																													FONT_BEGIN, 566, 181,
																													FONT_NEXT, 595, -11,
																													FONT_END, 155, 537,
																													FONT_BEGIN, 566, 181,
																													FONT_NEXT, 155, 537,
																													FONT_END, 183, 662,
																													FONT_BEGIN, 707, 662,
																													FONT_NEXT, 707, 643,
																													FONT_NEXT, 472, 662,
																													FONT_NEXT, 665, 636,
																													FONT_END, 635, 620,
																													FONT_BEGIN, 472, 662,
																													FONT_NEXT, 635, 620,
																													FONT_END, 617, 583,
																													FONT_BEGIN, 247, 19,
																													FONT_NEXT, 247, 0,
																													FONT_NEXT, 201, 25,
																													FONT_NEXT, 12, 0,
																													FONT_END, 172, 45,
																													FONT_ADVANCE, 722, 0
																											},
																											{
																												79,
																													FONT_BEGIN, 148, 331,
																													FONT_NEXT, 152, 254,
																													FONT_NEXT, 129, 78,
																													FONT_NEXT, 165, 189,
																													FONT_NEXT, 177, 38,
																													FONT_NEXT, 186, 136,
																													FONT_END, 212, 94,
																													FONT_BEGIN, 177, 38,
																													FONT_NEXT, 212, 94,
																													FONT_NEXT, 233, 9,
																													FONT_NEXT, 244, 61,
																													FONT_END, 280, 39,
																													FONT_BEGIN, 233, 9,
																													FONT_NEXT, 280, 39,
																													FONT_NEXT, 294, -8,
																													FONT_NEXT, 319, 26,
																													FONT_NEXT, 361, -14,
																													FONT_NEXT, 361, 22,
																													FONT_END, 402, 26,
																													FONT_BEGIN, 361, -14,
																													FONT_NEXT, 402, 26,
																													FONT_NEXT, 427, -8,
																													FONT_NEXT, 441, 39,
																													FONT_END, 477, 61,
																													FONT_BEGIN, 427, -8,
																													FONT_NEXT, 477, 61,
																													FONT_NEXT, 488, 9,
																													FONT_NEXT, 509, 94,
																													FONT_END, 535, 136,
																													FONT_BEGIN, 488, 9,
																													FONT_NEXT, 535, 136,
																													FONT_NEXT, 544, 38,
																													FONT_NEXT, 556, 189,
																													FONT_END, 569, 254,
																													FONT_BEGIN, 544, 38,
																													FONT_NEXT, 569, 254,
																													FONT_NEXT, 592, 78,
																													FONT_NEXT, 574, 331,
																													FONT_NEXT, 586, 589,
																													FONT_NEXT, 569, 409,
																													FONT_NEXT, 536, 627,
																													FONT_NEXT, 555, 475,
																													FONT_END, 533, 528,
																													FONT_BEGIN, 536, 627,
																													FONT_NEXT, 533, 528,
																													FONT_NEXT, 481, 654,
																													FONT_NEXT, 505, 571,
																													FONT_END, 473, 602,
																													FONT_BEGIN, 481, 654,
																													FONT_NEXT, 473, 602,
																													FONT_NEXT, 422, 670,
																													FONT_NEXT, 437, 623,
																													FONT_END, 399, 636,
																													FONT_BEGIN, 422, 670,
																													FONT_NEXT, 399, 636,
																													FONT_NEXT, 361, 676,
																													FONT_NEXT, 361, 640,
																													FONT_NEXT, 299, 670,
																													FONT_NEXT, 322, 636,
																													FONT_END, 284, 623,
																													FONT_BEGIN, 299, 670,
																													FONT_NEXT, 284, 623,
																													FONT_NEXT, 240, 654,
																													FONT_NEXT, 248, 602,
																													FONT_END, 216, 571,
																													FONT_BEGIN, 240, 654,
																													FONT_NEXT, 216, 571,
																													FONT_NEXT, 185, 627,
																													FONT_NEXT, 188, 528,
																													FONT_END, 166, 475,
																													FONT_BEGIN, 185, 627,
																													FONT_NEXT, 166, 475,
																													FONT_NEXT, 136, 589,
																													FONT_NEXT, 152, 409,
																													FONT_END, 148, 331,
																													FONT_BEGIN, 136, 589,
																													FONT_NEXT, 148, 331,
																													FONT_END, 129, 78,
																													FONT_BEGIN, 136, 589,
																													FONT_NEXT, 129, 78,
																													FONT_NEXT, 94, 540,
																													FONT_NEXT, 89, 127,
																													FONT_NEXT, 62, 481,
																													FONT_NEXT, 59, 186,
																													FONT_NEXT, 41, 410,
																													FONT_NEXT, 40, 254,
																													FONT_END, 34, 330,
																													FONT_BEGIN, 688, 330,
																													FONT_NEXT, 681, 254,
																													FONT_NEXT, 680, 410,
																													FONT_NEXT, 662, 186,
																													FONT_NEXT, 659, 481,
																													FONT_NEXT, 632, 127,
																													FONT_NEXT, 627, 540,
																													FONT_NEXT, 592, 78,
																													FONT_END, 586, 589,
																													FONT_ADVANCE, 722, 0
																											},
																												{
																													80,
																														FONT_BEGIN, 16, 643,
																														FONT_NEXT, 16, 662,
																														FONT_NEXT, 59, 635,
																														FONT_END, 85, 622,
																														FONT_BEGIN, 85, 622,
																														FONT_NEXT, 16, 662,
																														FONT_END, 202, 591,
																														FONT_BEGIN, 85, 622,
																														FONT_NEXT, 202, 591,
																														FONT_NEXT, 97, 596,
																														FONT_NEXT, 202, 331,
																														FONT_END, 202, 291,
																														FONT_BEGIN, 97, 596,
																														FONT_NEXT, 202, 291,
																														FONT_END, 202, 109,
																														FONT_BEGIN, 97, 596,
																														FONT_NEXT, 202, 109,
																														FONT_NEXT, 100, 553,
																														FONT_END, 100, 120,
																														FONT_BEGIN, 100, 120,
																														FONT_NEXT, 202, 109,
																														FONT_NEXT, 98, 74,
																														FONT_END, 89, 44,
																														FONT_BEGIN, 89, 44,
																														FONT_NEXT, 202, 109,
																														FONT_NEXT, 64, 27,
																														FONT_NEXT, 205, 64,
																														FONT_END, 16, 0,
																														FONT_BEGIN, 64, 27,
																														FONT_NEXT, 16, 0,
																														FONT_END, 16, 19,
																														FONT_BEGIN, 542, 482,
																														FONT_NEXT, 534, 430,
																														FONT_NEXT, 536, 526,
																														FONT_END, 520, 563,
																														FONT_BEGIN, 520, 563,
																														FONT_NEXT, 534, 430,
																														FONT_END, 517, 391,
																														FONT_BEGIN, 520, 563,
																														FONT_NEXT, 517, 391,
																														FONT_NEXT, 495, 594,
																														FONT_NEXT, 497, 363,
																														FONT_END, 481, 347,
																														FONT_BEGIN, 495, 594,
																														FONT_NEXT, 481, 347,
																														FONT_NEXT, 462, 619,
																														FONT_NEXT, 444, 320,
																														FONT_END, 433, 480,
																														FONT_BEGIN, 462, 619,
																														FONT_NEXT, 433, 480,
																														FONT_END, 426, 527,
																														FONT_BEGIN, 462, 619,
																														FONT_NEXT, 426, 527,
																														FONT_NEXT, 379, 651,
																														FONT_NEXT, 409, 562,
																														FONT_END, 384, 588,
																														FONT_BEGIN, 379, 651,
																														FONT_NEXT, 384, 588,
																														FONT_END, 354, 606,
																														FONT_BEGIN, 379, 651,
																														FONT_NEXT, 354, 606,
																														FONT_NEXT, 280, 662,
																														FONT_NEXT, 291, 622,
																														FONT_END, 243, 625,
																														FONT_BEGIN, 280, 662,
																														FONT_NEXT, 243, 625,
																														FONT_NEXT, 16, 662,
																														FONT_NEXT, 211, 620,
																														FONT_END, 204, 610,
																														FONT_BEGIN, 16, 662,
																														FONT_NEXT, 204, 610,
																														FONT_END, 202, 591,
																														FONT_BEGIN, 202, 331,
																														FONT_NEXT, 262, 328,
																														FONT_NEXT, 202, 291,
																														FONT_END, 269, 288,
																														FONT_BEGIN, 269, 288,
																														FONT_NEXT, 262, 328,
																														FONT_END, 304, 330,
																														FONT_BEGIN, 269, 288,
																														FONT_NEXT, 304, 330,
																														FONT_NEXT, 340, 291,
																														FONT_NEXT, 361, 347,
																														FONT_END, 388, 365,
																														FONT_BEGIN, 340, 291,
																														FONT_NEXT, 388, 365,
																														FONT_NEXT, 398, 302,
																														FONT_NEXT, 411, 392,
																														FONT_END, 427, 429,
																														FONT_BEGIN, 398, 302,
																														FONT_NEXT, 427, 429,
																														FONT_NEXT, 444, 320,
																														FONT_END, 433, 480,
																														FONT_BEGIN, 296, 19,
																														FONT_NEXT, 296, 0,
																														FONT_NEXT, 248, 23,
																														FONT_NEXT, 16, 0,
																														FONT_NEXT, 220, 37,
																														FONT_END, 205, 64,
																														FONT_ADVANCE, 556, 0
																												},
																												{
																													81,
																														FONT_BEGIN, 34, 330,
																														FONT_NEXT, 41, 410,
																														FONT_NEXT, 40, 255,
																														FONT_END, 57, 191,
																														FONT_BEGIN, 57, 191,
																														FONT_NEXT, 41, 410,
																														FONT_END, 62, 480,
																														FONT_BEGIN, 57, 191,
																														FONT_NEXT, 62, 480,
																														FONT_NEXT, 82, 138,
																														FONT_NEXT, 94, 540,
																														FONT_NEXT, 114, 94,
																														FONT_NEXT, 136, 588,
																														FONT_END, 148, 330,
																														FONT_BEGIN, 114, 94,
																														FONT_NEXT, 148, 330,
																														FONT_NEXT, 151, 58,
																														FONT_NEXT, 152, 253,
																														FONT_END, 165, 189,
																														FONT_BEGIN, 151, 58,
																														FONT_NEXT, 165, 189,
																														FONT_END, 186, 136,
																														FONT_BEGIN, 151, 58,
																														FONT_NEXT, 186, 136,
																														FONT_NEXT, 189, 31,
																														FONT_NEXT, 212, 93,
																														FONT_END, 244, 61,
																														FONT_BEGIN, 189, 31,
																														FONT_NEXT, 244, 61,
																														FONT_NEXT, 265, -1,
																														FONT_NEXT, 280, 39,
																														FONT_NEXT, 312, -56,
																														FONT_NEXT, 319, 26,
																														FONT_END, 361, 22,
																														FONT_BEGIN, 312, -56,
																														FONT_NEXT, 361, 22,
																														FONT_NEXT, 379, -113,
																														FONT_NEXT, 402, 26,
																														FONT_NEXT, 426, -7,
																														FONT_NEXT, 441, 39,
																														FONT_NEXT, 468, 2,
																														FONT_NEXT, 477, 61,
																														FONT_END, 509, 93,
																														FONT_BEGIN, 468, 2,
																														FONT_NEXT, 509, 93,
																														FONT_NEXT, 512, 20,
																														FONT_NEXT, 535, 136,
																														FONT_NEXT, 556, 47,
																														FONT_NEXT, 556, 189,
																														FONT_END, 569, 253,
																														FONT_BEGIN, 556, 47,
																														FONT_NEXT, 569, 253,
																														FONT_END, 574, 330,
																														FONT_BEGIN, 556, 47,
																														FONT_NEXT, 574, 330,
																														FONT_NEXT, 597, 83,
																														FONT_NEXT, 586, 589,
																														FONT_END, 627, 540,
																														FONT_BEGIN, 597, 83,
																														FONT_NEXT, 627, 540,
																														FONT_NEXT, 633, 130,
																														FONT_NEXT, 659, 481,
																														FONT_NEXT, 662, 186,
																														FONT_NEXT, 680, 410,
																														FONT_NEXT, 681, 252,
																														FONT_END, 688, 330,
																														FONT_BEGIN, 574, 330,
																														FONT_NEXT, 569, 409,
																														FONT_NEXT, 586, 589,
																														FONT_END, 536, 627,
																														FONT_BEGIN, 536, 627,
																														FONT_NEXT, 569, 409,
																														FONT_END, 555, 475,
																														FONT_BEGIN, 536, 627,
																														FONT_NEXT, 555, 475,
																														FONT_END, 533, 528,
																														FONT_BEGIN, 536, 627,
																														FONT_NEXT, 533, 528,
																														FONT_NEXT, 481, 654,
																														FONT_NEXT, 505, 570,
																														FONT_END, 473, 602,
																														FONT_BEGIN, 481, 654,
																														FONT_NEXT, 473, 602,
																														FONT_NEXT, 422, 670,
																														FONT_NEXT, 437, 623,
																														FONT_END, 399, 636,
																														FONT_BEGIN, 422, 670,
																														FONT_NEXT, 399, 636,
																														FONT_NEXT, 361, 676,
																														FONT_NEXT, 361, 640,
																														FONT_NEXT, 299, 670,
																														FONT_NEXT, 322, 636,
																														FONT_END, 284, 623,
																														FONT_BEGIN, 299, 670,
																														FONT_NEXT, 284, 623,
																														FONT_NEXT, 240, 654,
																														FONT_NEXT, 248, 602,
																														FONT_END, 216, 570,
																														FONT_BEGIN, 240, 654,
																														FONT_NEXT, 216, 570,
																														FONT_NEXT, 185, 626,
																														FONT_NEXT, 188, 528,
																														FONT_END, 166, 475,
																														FONT_BEGIN, 185, 626,
																														FONT_NEXT, 166, 475,
																														FONT_NEXT, 136, 588,
																														FONT_NEXT, 152, 409,
																														FONT_END, 148, 330,
																														FONT_BEGIN, 701, -159,
																														FONT_NEXT, 701, -177,
																														FONT_NEXT, 638, -153,
																														FONT_NEXT, 654, -178,
																														FONT_END, 555, -172,
																														FONT_BEGIN, 638, -153,
																														FONT_NEXT, 555, -172,
																														FONT_NEXT, 586, -138,
																														FONT_END, 505, -90,
																														FONT_BEGIN, 505, -90,
																														FONT_NEXT, 555, -172,
																														FONT_END, 462, -151,
																														FONT_BEGIN, 505, -90,
																														FONT_NEXT, 462, -151,
																														FONT_NEXT, 426, -7,
																														FONT_END, 379, -113,
																														FONT_ADVANCE, 722, 0
																												},
																													{
																														82,
																															FONT_BEGIN, 17, 643,
																															FONT_NEXT, 17, 662,
																															FONT_NEXT, 59, 636,
																															FONT_END, 85, 623,
																															FONT_BEGIN, 85, 623,
																															FONT_NEXT, 17, 662,
																															FONT_END, 204, 589,
																															FONT_BEGIN, 85, 623,
																															FONT_NEXT, 204, 589,
																															FONT_NEXT, 98, 597,
																															FONT_NEXT, 204, 343,
																															FONT_END, 204, 306,
																															FONT_BEGIN, 98, 597,
																															FONT_NEXT, 204, 306,
																															FONT_END, 204, 109,
																															FONT_BEGIN, 98, 597,
																															FONT_NEXT, 204, 109,
																															FONT_NEXT, 102, 553,
																															FONT_END, 102, 120,
																															FONT_BEGIN, 102, 120,
																															FONT_NEXT, 204, 109,
																															FONT_NEXT, 100, 73,
																															FONT_END, 89, 43,
																															FONT_BEGIN, 89, 43,
																															FONT_NEXT, 204, 109,
																															FONT_NEXT, 64, 26,
																															FONT_NEXT, 206, 67,
																															FONT_END, 17, 0,
																															FONT_BEGIN, 64, 26,
																															FONT_NEXT, 17, 0,
																															FONT_END, 17, 19,
																															FONT_BEGIN, 204, 306,
																															FONT_NEXT, 204, 343,
																															FONT_NEXT, 260, 308,
																															FONT_NEXT, 276, 346,
																															FONT_END, 352, 362,
																															FONT_BEGIN, 260, 308,
																															FONT_NEXT, 352, 362,
																															FONT_NEXT, 366, 319,
																															FONT_NEXT, 386, 380,
																															FONT_END, 413, 406,
																															FONT_BEGIN, 366, 319,
																															FONT_NEXT, 413, 406,
																															FONT_NEXT, 415, 330,
																															FONT_NEXT, 431, 441,
																															FONT_END, 438, 489,
																															FONT_BEGIN, 415, 330,
																															FONT_NEXT, 438, 489,
																															FONT_NEXT, 475, 356,
																															FONT_END, 453, 632,
																															FONT_BEGIN, 453, 632,
																															FONT_NEXT, 438, 489,
																															FONT_END, 432, 533,
																															FONT_BEGIN, 453, 632,
																															FONT_NEXT, 432, 533,
																															FONT_END, 416, 566,
																															FONT_BEGIN, 453, 632,
																															FONT_NEXT, 416, 566,
																															FONT_NEXT, 370, 656,
																															FONT_NEXT, 394, 590,
																															FONT_END, 367, 606,
																															FONT_BEGIN, 370, 656,
																															FONT_NEXT, 367, 606,
																															FONT_NEXT, 293, 662,
																															FONT_NEXT, 310, 622,
																															FONT_END, 266, 625,
																															FONT_BEGIN, 293, 662,
																															FONT_NEXT, 266, 625,
																															FONT_NEXT, 17, 662,
																															FONT_NEXT, 233, 623,
																															FONT_END, 215, 617,
																															FONT_BEGIN, 17, 662,
																															FONT_NEXT, 215, 617,
																															FONT_END, 206, 606,
																															FONT_BEGIN, 17, 662,
																															FONT_NEXT, 206, 606,
																															FONT_END, 204, 589,
																															FONT_BEGIN, 659, 19,
																															FONT_NEXT, 659, 0,
																															FONT_NEXT, 608, 33,
																															FONT_NEXT, 498, 0,
																															FONT_NEXT, 572, 66,
																															FONT_END, 366, 319,
																															FONT_BEGIN, 366, 319,
																															FONT_NEXT, 498, 0,
																															FONT_END, 260, 308,
																															FONT_BEGIN, 366, 319,
																															FONT_END, 260, 308,
																															FONT_BEGIN, 547, 487,
																															FONT_NEXT, 541, 441,
																															FONT_NEXT, 539, 538,
																															FONT_NEXT, 525, 405,
																															FONT_NEXT, 519, 579,
																															FONT_NEXT, 503, 377,
																															FONT_NEXT, 490, 610,
																															FONT_NEXT, 475, 356,
																															FONT_END, 453, 632,
																															FONT_BEGIN, 294, 19,
																															FONT_NEXT, 294, 0,
																															FONT_NEXT, 246, 25,
																															FONT_NEXT, 17, 0,
																															FONT_NEXT, 219, 40,
																															FONT_END, 206, 67,
																															FONT_ADVANCE, 667, 0
																													},
																													{
																														83,
																															FONT_BEGIN, 71, 506,
																															FONT_NEXT, 75, 546,
																															FONT_NEXT, 82, 441,
																															FONT_NEXT, 86, 580,
																															FONT_NEXT, 114, 390,
																															FONT_NEXT, 127, 633,
																															FONT_END, 157, 542,
																															FONT_BEGIN, 114, 390,
																															FONT_NEXT, 157, 542,
																															FONT_NEXT, 163, 349,
																															FONT_NEXT, 171, 491,
																															FONT_END, 209, 448,
																															FONT_BEGIN, 163, 349,
																															FONT_NEXT, 209, 448,
																															FONT_NEXT, 227, 310,
																															FONT_END, 276, 280,
																															FONT_BEGIN, 276, 280,
																															FONT_NEXT, 209, 448,
																															FONT_NEXT, 314, 254,
																															FONT_NEXT, 324, 375,
																															FONT_NEXT, 343, 231,
																															FONT_END, 363, 210,
																															FONT_BEGIN, 363, 210,
																															FONT_NEXT, 324, 375,
																															FONT_NEXT, 385, 172,
																															FONT_NEXT, 385, 336,
																															FONT_NEXT, 390, 133,
																															FONT_NEXT, 409, 20,
																															FONT_END, 373, 2,
																															FONT_BEGIN, 390, 133,
																															FONT_NEXT, 373, 2,
																															FONT_NEXT, 382, 92,
																															FONT_END, 360, 57,
																															FONT_BEGIN, 360, 57,
																															FONT_NEXT, 373, 2,
																															FONT_END, 330, -10,
																															FONT_BEGIN, 360, 57,
																															FONT_NEXT, 330, -10,
																															FONT_NEXT, 322, 31,
																															FONT_NEXT, 280, -14,
																															FONT_NEXT, 270, 22,
																															FONT_NEXT, 227, -9,
																															FONT_NEXT, 223, 27,
																															FONT_NEXT, 185, 3,
																															FONT_NEXT, 183, 43,
																															FONT_NEXT, 151, 14,
																															FONT_NEXT, 124, 92,
																															FONT_NEXT, 125, 20,
																															FONT_END, 101, 9,
																															FONT_BEGIN, 124, 92,
																															FONT_NEXT, 101, 9,
																															FONT_NEXT, 86, 151,
																															FONT_NEXT, 94, -13,
																															FONT_END, 72, -13,
																															FONT_BEGIN, 86, 151,
																															FONT_NEXT, 72, -13,
																															FONT_NEXT, 65, 199,
																															FONT_END, 42, 199,
																															FONT_BEGIN, 469, 463,
																															FONT_NEXT, 444, 463,
																															FONT_NEXT, 447, 676,
																															FONT_END, 426, 676,
																															FONT_BEGIN, 426, 676,
																															FONT_NEXT, 444, 463,
																															FONT_END, 422, 524,
																															FONT_BEGIN, 426, 676,
																															FONT_NEXT, 422, 524,
																															FONT_NEXT, 418, 654,
																															FONT_NEXT, 384, 579,
																															FONT_NEXT, 408, 645,
																															FONT_END, 391, 642,
																															FONT_BEGIN, 391, 642,
																															FONT_NEXT, 384, 579,
																															FONT_NEXT, 367, 647,
																															FONT_NEXT, 328, 619,
																															FONT_NEXT, 334, 659,
																															FONT_END, 252, 676,
																															FONT_BEGIN, 252, 676,
																															FONT_NEXT, 328, 619,
																															FONT_END, 295, 630,
																															FONT_BEGIN, 252, 676,
																															FONT_NEXT, 295, 630,
																															FONT_END, 258, 635,
																															FONT_BEGIN, 252, 676,
																															FONT_NEXT, 258, 635,
																															FONT_END, 221, 629,
																															FONT_BEGIN, 252, 676,
																															FONT_NEXT, 221, 629,
																															FONT_NEXT, 185, 665,
																															FONT_NEXT, 188, 611,
																															FONT_END, 165, 582,
																															FONT_BEGIN, 185, 665,
																															FONT_NEXT, 165, 582,
																															FONT_NEXT, 127, 633,
																															FONT_END, 157, 542,
																															FONT_BEGIN, 491, 167,
																															FONT_NEXT, 478, 102,
																															FONT_NEXT, 476, 235,
																															FONT_NEXT, 439, 44,
																															FONT_NEXT, 438, 290,
																															FONT_NEXT, 409, 20,
																															FONT_END, 385, 336,
																															FONT_ADVANCE, 556, 0
																													},
																														{
																															84,
																																FONT_BEGIN, 17, 492,
																																FONT_NEXT, 23, 662,
																																FONT_NEXT, 41, 492,
																																FONT_END, 59, 557,
																																FONT_BEGIN, 59, 557,
																																FONT_NEXT, 23, 662,
																																FONT_NEXT, 87, 596,
																																FONT_END, 131, 615,
																																FONT_BEGIN, 131, 615,
																																FONT_NEXT, 23, 662,
																																FONT_NEXT, 200, 620,
																																FONT_END, 254, 620,
																																FONT_BEGIN, 254, 620,
																																FONT_NEXT, 23, 662,
																																FONT_END, 356, 620,
																																FONT_BEGIN, 254, 620,
																																FONT_NEXT, 356, 620,
																																FONT_END, 356, 109,
																																FONT_BEGIN, 254, 620,
																																FONT_NEXT, 356, 109,
																																FONT_NEXT, 254, 120,
																																FONT_END, 252, 73,
																																FONT_BEGIN, 252, 73,
																																FONT_NEXT, 356, 109,
																																FONT_NEXT, 241, 43,
																																FONT_END, 213, 26,
																																FONT_BEGIN, 213, 26,
																																FONT_NEXT, 356, 109,
																																FONT_END, 359, 66,
																																FONT_BEGIN, 213, 26,
																																FONT_NEXT, 359, 66,
																																FONT_END, 160, 0,
																																FONT_BEGIN, 213, 26,
																																FONT_NEXT, 160, 0,
																																FONT_END, 160, 19,
																																FONT_BEGIN, 593, 492,
																																FONT_NEXT, 569, 492,
																																FONT_NEXT, 587, 662,
																																FONT_NEXT, 550, 558,
																																FONT_END, 522, 597,
																																FONT_BEGIN, 587, 662,
																																FONT_NEXT, 522, 597,
																																FONT_END, 478, 615,
																																FONT_BEGIN, 587, 662,
																																FONT_NEXT, 478, 615,
																																FONT_NEXT, 23, 662,
																																FONT_NEXT, 410, 620,
																																FONT_END, 356, 620,
																																FONT_BEGIN, 452, 19,
																																FONT_NEXT, 452, 0,
																																FONT_NEXT, 401, 24,
																																FONT_NEXT, 160, 0,
																																FONT_NEXT, 372, 39,
																																FONT_END, 359, 66,
																																FONT_ADVANCE, 611, 0
																														},
																														{
																															85,
																																FONT_BEGIN, 14, 643,
																																FONT_NEXT, 14, 662,
																																FONT_NEXT, 60, 637,
																																FONT_END, 87, 623,
																																FONT_BEGIN, 87, 623,
																																FONT_NEXT, 14, 662,
																																FONT_END, 209, 596,
																																FONT_BEGIN, 87, 623,
																																FONT_NEXT, 209, 596,
																																FONT_END, 206, 553,
																																FONT_BEGIN, 87, 623,
																																FONT_NEXT, 206, 553,
																																FONT_NEXT, 100, 597,
																																FONT_NEXT, 206, 233,
																																FONT_END, 164, 50,
																																FONT_BEGIN, 100, 597,
																																FONT_NEXT, 164, 50,
																																FONT_NEXT, 104, 553,
																																FONT_NEXT, 135, 91,
																																FONT_END, 117, 133,
																																FONT_BEGIN, 104, 553,
																																FONT_NEXT, 117, 133,
																																FONT_END, 107, 175,
																																FONT_BEGIN, 104, 553,
																																FONT_NEXT, 107, 175,
																																FONT_END, 104, 212,
																																FONT_BEGIN, 104, 553,
																																FONT_NEXT, 104, 212,
																																FONT_END, 104, 241,
																																FONT_BEGIN, 297, 662,
																																FONT_NEXT, 297, 643,
																																FONT_NEXT, 14, 662,
																																FONT_NEXT, 249, 636,
																																FONT_END, 221, 622,
																																FONT_BEGIN, 14, 662,
																																FONT_NEXT, 221, 622,
																																FONT_END, 209, 596,
																																FONT_BEGIN, 473, 643,
																																FONT_NEXT, 473, 662,
																																FONT_NEXT, 517, 636,
																																FONT_NEXT, 629, 615,
																																FONT_END, 614, 578,
																																FONT_BEGIN, 517, 636,
																																FONT_NEXT, 614, 578,
																																FONT_NEXT, 546, 617,
																																FONT_NEXT, 611, 515,
																																FONT_NEXT, 562, 579,
																																FONT_NEXT, 611, 254,
																																FONT_END, 606, 178,
																																FONT_BEGIN, 562, 579,
																																FONT_NEXT, 606, 178,
																																FONT_END, 597, 134,
																																FONT_BEGIN, 562, 579,
																																FONT_NEXT, 597, 134,
																																FONT_NEXT, 567, 515,
																																FONT_NEXT, 578, 90,
																																FONT_NEXT, 567, 245,
																																FONT_END, 565, 192,
																																FONT_BEGIN, 565, 192,
																																FONT_NEXT, 578, 90,
																																FONT_END, 547, 49,
																																FONT_BEGIN, 565, 192,
																																FONT_NEXT, 547, 49,
																																FONT_NEXT, 560, 154,
																																FONT_END, 544, 109,
																																FONT_BEGIN, 544, 109,
																																FONT_NEXT, 547, 49,
																																FONT_END, 501, 16,
																																FONT_BEGIN, 544, 109,
																																FONT_NEXT, 501, 16,
																																FONT_NEXT, 517, 76,
																																FONT_END, 480, 51,
																																FONT_BEGIN, 480, 51,
																																FONT_NEXT, 501, 16,
																																FONT_END, 437, -6,
																																FONT_BEGIN, 480, 51,
																																FONT_NEXT, 437, -6,
																																FONT_NEXT, 434, 35,
																																FONT_NEXT, 352, -14,
																																FONT_NEXT, 380, 30,
																																FONT_END, 322, 36,
																																FONT_BEGIN, 322, 36,
																																FONT_NEXT, 352, -14,
																																FONT_END, 270, -6,
																																FONT_BEGIN, 322, 36,
																																FONT_NEXT, 270, -6,
																																FONT_NEXT, 279, 52,
																																FONT_END, 248, 77,
																																FONT_BEGIN, 248, 77,
																																FONT_NEXT, 270, -6,
																																FONT_END, 208, 17,
																																FONT_BEGIN, 248, 77,
																																FONT_NEXT, 208, 17,
																																FONT_NEXT, 227, 107,
																																FONT_END, 208, 174,
																																FONT_BEGIN, 208, 174,
																																FONT_NEXT, 208, 17,
																																FONT_NEXT, 206, 233,
																																FONT_END, 164, 50,
																																FONT_BEGIN, 705, 662,
																																FONT_NEXT, 705, 643,
																																FONT_NEXT, 473, 662,
																																FONT_NEXT, 657, 634,
																																FONT_END, 629, 615,
																																FONT_ADVANCE, 722, 0
																														},
																															{
																																86,
																																	FONT_BEGIN, 282, 662,
																																	FONT_NEXT, 282, 643,
																																	FONT_NEXT, 16, 662,
																																	FONT_NEXT, 246, 640,
																																	FONT_END, 223, 635,
																																	FONT_BEGIN, 16, 662,
																																	FONT_NEXT, 223, 635,
																																	FONT_END, 210, 624,
																																	FONT_BEGIN, 16, 662,
																																	FONT_NEXT, 210, 624,
																																	FONT_END, 207, 606,
																																	FONT_BEGIN, 16, 662,
																																	FONT_NEXT, 207, 606,
																																	FONT_END, 71, 623,
																																	FONT_BEGIN, 16, 662,
																																	FONT_NEXT, 71, 623,
																																	FONT_END, 46, 638,
																																	FONT_BEGIN, 16, 662,
																																	FONT_NEXT, 46, 638,
																																	FONT_END, 16, 643,
																																	FONT_BEGIN, 549, 538,
																																	FONT_NEXT, 556, 556,
																																	FONT_NEXT, 546, 528,
																																	FONT_NEXT, 383, -11,
																																	FONT_NEXT, 399, 161,
																																	FONT_END, 248, 499,
																																	FONT_BEGIN, 248, 499,
																																	FONT_NEXT, 383, -11,
																																	FONT_END, 368, -11,
																																	FONT_BEGIN, 248, 499,
																																	FONT_NEXT, 368, -11,
																																	FONT_END, 122, 538,
																																	FONT_BEGIN, 248, 499,
																																	FONT_NEXT, 122, 538,
																																	FONT_NEXT, 218, 566,
																																	FONT_END, 210, 590,
																																	FONT_BEGIN, 210, 590,
																																	FONT_NEXT, 122, 538,
																																	FONT_NEXT, 207, 606,
																																	FONT_NEXT, 94, 592,
																																	FONT_END, 71, 623,
																																	FONT_BEGIN, 492, 643,
																																	FONT_NEXT, 492, 662,
																																	FONT_NEXT, 542, 634,
																																	FONT_NEXT, 625, 596,
																																	FONT_NEXT, 558, 621,
																																	FONT_NEXT, 605, 550,
																																	FONT_NEXT, 565, 597,
																																	FONT_END, 556, 556,
																																	FONT_BEGIN, 556, 556,
																																	FONT_NEXT, 605, 550,
																																	FONT_END, 383, -11,
																																	FONT_BEGIN, 556, 556,
																																	FONT_END, 383, -11,
																																	FONT_BEGIN, 697, 662,
																																	FONT_NEXT, 697, 643,
																																	FONT_NEXT, 492, 662,
																																	FONT_NEXT, 666, 637,
																																	FONT_END, 644, 623,
																																	FONT_BEGIN, 492, 662,
																																	FONT_NEXT, 644, 623,
																																	FONT_END, 625, 596,
																																	FONT_ADVANCE, 722, 0
																															},
																															{
																																87,
																																	FONT_BEGIN, 125, 480,
																																	FONT_NEXT, 108, 526,
																																	FONT_NEXT, 147, 421,
																																	FONT_END, 185, 609,
																																	FONT_BEGIN, 185, 609,
																																	FONT_NEXT, 108, 526,
																																	FONT_END, 86, 578,
																																	FONT_BEGIN, 185, 609,
																																	FONT_NEXT, 86, 578,
																																	FONT_END, 65, 613,
																																	FONT_BEGIN, 185, 609,
																																	FONT_NEXT, 65, 613,
																																	FONT_END, 40, 634,
																																	FONT_BEGIN, 185, 609,
																																	FONT_NEXT, 40, 634,
																																	FONT_NEXT, 5, 662,
																																	FONT_END, 5, 643,
																																	FONT_BEGIN, 313, 643,
																																	FONT_NEXT, 313, 662,
																																	FONT_NEXT, 346, 640,
																																	FONT_END, 371, 628,
																																	FONT_BEGIN, 371, 628,
																																	FONT_NEXT, 313, 662,
																																	FONT_END, 503, 605,
																																	FONT_BEGIN, 371, 628,
																																	FONT_NEXT, 503, 605,
																																	FONT_NEXT, 391, 601,
																																	FONT_END, 414, 553,
																																	FONT_BEGIN, 414, 553,
																																	FONT_NEXT, 503, 605,
																																	FONT_NEXT, 447, 471,
																																	FONT_NEXT, 470, 412,
																																	FONT_END, 316, -11,
																																	FONT_BEGIN, 447, 471,
																																	FONT_NEXT, 316, -11,
																																	FONT_NEXT, 340, 189,
																																	FONT_END, 196, 565,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 316, -11,
																																	FONT_END, 301, -11,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 301, -11,
																																	FONT_END, 279, 51,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 279, 51,
																																	FONT_END, 254, 123,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 254, 123,
																																	FONT_END, 199, 277,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 199, 277,
																																	FONT_END, 147, 421,
																																	FONT_BEGIN, 196, 565,
																																	FONT_NEXT, 147, 421,
																																	FONT_END, 185, 609,
																																	FONT_BEGIN, 580, 662,
																																	FONT_NEXT, 580, 643,
																																	FONT_NEXT, 313, 662,
																																	FONT_NEXT, 529, 637,
																																	FONT_END, 510, 626,
																																	FONT_BEGIN, 313, 662,
																																	FONT_NEXT, 510, 626,
																																	FONT_END, 503, 605,
																																	FONT_BEGIN, 734, 643,
																																	FONT_NEXT, 734, 662,
																																	FONT_NEXT, 781, 633,
																																	FONT_NEXT, 864, 606,
																																	FONT_END, 853, 582,
																																	FONT_BEGIN, 781, 633,
																																	FONT_NEXT, 853, 582,
																																	FONT_NEXT, 797, 620,
																																	FONT_END, 803, 597,
																																	FONT_BEGIN, 803, 597,
																																	FONT_NEXT, 853, 582,
																																	FONT_END, 745, 288,
																																	FONT_BEGIN, 803, 597,
																																	FONT_NEXT, 745, 288,
																																	FONT_NEXT, 787, 525,
																																	FONT_END, 662, 186,
																																	FONT_BEGIN, 662, 186,
																																	FONT_NEXT, 745, 288,
																																	FONT_END, 645, -11,
																																	FONT_BEGIN, 662, 186,
																																	FONT_NEXT, 645, -11,
																																	FONT_NEXT, 530, 527,
																																	FONT_NEXT, 630, -11,
																																	FONT_END, 470, 412,
																																	FONT_BEGIN, 530, 527,
																																	FONT_NEXT, 470, 412,
																																	FONT_NEXT, 510, 574,
																																	FONT_END, 503, 605,
																																	FONT_BEGIN, 932, 662,
																																	FONT_NEXT, 932, 643,
																																	FONT_NEXT, 734, 662,
																																	FONT_NEXT, 900, 635,
																																	FONT_END, 879, 624,
																																	FONT_BEGIN, 734, 662,
																																	FONT_NEXT, 879, 624,
																																	FONT_END, 864, 606,
																																	FONT_BEGIN, 250, 662,
																																	FONT_NEXT, 250, 643,
																																	FONT_NEXT, 5, 662,
																																	FONT_NEXT, 220, 640,
																																	FONT_END, 200, 635,
																																	FONT_BEGIN, 5, 662,
																																	FONT_NEXT, 200, 635,
																																	FONT_END, 188, 625,
																																	FONT_BEGIN, 5, 662,
																																	FONT_NEXT, 188, 625,
																																	FONT_END, 185, 609,
																																	FONT_ADVANCE, 944, 0
																															},
																																{
																																	88,
																																		FONT_BEGIN, 22, 643,
																																		FONT_NEXT, 22, 662,
																																		FONT_NEXT, 60, 637,
																																		FONT_END, 96, 618,
																																		FONT_BEGIN, 96, 618,
																																		FONT_NEXT, 22, 662,
																																		FONT_END, 248, 611,
																																		FONT_BEGIN, 96, 618,
																																		FONT_NEXT, 248, 611,
																																		FONT_NEXT, 116, 600,
																																		FONT_END, 139, 573,
																																		FONT_BEGIN, 139, 573,
																																		FONT_NEXT, 248, 611,
																																		FONT_NEXT, 168, 535,
																																		FONT_END, 203, 486,
																																		FONT_BEGIN, 203, 486,
																																		FONT_NEXT, 248, 611,
																																		FONT_END, 251, 592,
																																		FONT_BEGIN, 203, 486,
																																		FONT_NEXT, 251, 592,
																																		FONT_END, 265, 565,
																																		FONT_BEGIN, 203, 486,
																																		FONT_NEXT, 265, 565,
																																		FONT_END, 291, 523,
																																		FONT_BEGIN, 203, 486,
																																		FONT_NEXT, 291, 523,
																																		FONT_NEXT, 312, 326,
																																		FONT_NEXT, 333, 463,
																																		FONT_END, 338, 288,
																																		FONT_BEGIN, 312, 326,
																																		FONT_NEXT, 338, 288,
																																		FONT_END, 219, 140,
																																		FONT_BEGIN, 312, 326,
																																		FONT_NEXT, 219, 140,
																																		FONT_NEXT, 155, 133,
																																		FONT_NEXT, 185, 95,
																																		FONT_END, 172, 73,
																																		FONT_BEGIN, 155, 133,
																																		FONT_NEXT, 172, 73,
																																		FONT_END, 167, 56,
																																		FONT_BEGIN, 155, 133,
																																		FONT_NEXT, 167, 56,
																																		FONT_NEXT, 128, 100,
																																		FONT_END, 106, 74,
																																		FONT_BEGIN, 106, 74,
																																		FONT_NEXT, 167, 56,
																																		FONT_NEXT, 73, 41,
																																		FONT_NEXT, 10, 0,
																																		FONT_NEXT, 44, 24,
																																		FONT_END, 10, 19,
																																		FONT_BEGIN, 324, 662,
																																		FONT_NEXT, 324, 643,
																																		FONT_NEXT, 22, 662,
																																		FONT_NEXT, 290, 641,
																																		FONT_END, 266, 637,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 266, 637,
																																		FONT_END, 252, 628,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 252, 628,
																																		FONT_END, 248, 611,
																																		FONT_BEGIN, 458, 643,
																																		FONT_NEXT, 458, 662,
																																		FONT_NEXT, 505, 637,
																																		FONT_NEXT, 592, 599,
																																		FONT_NEXT, 521, 628,
																																		FONT_END, 528, 610,
																																		FONT_BEGIN, 528, 610,
																																		FONT_NEXT, 592, 599,
																																		FONT_END, 547, 549,
																																		FONT_BEGIN, 528, 610,
																																		FONT_NEXT, 547, 549,
																																		FONT_NEXT, 516, 579,
																																		FONT_NEXT, 401, 367,
																																		FONT_NEXT, 488, 542,
																																		FONT_END, 375, 404,
																																		FONT_BEGIN, 375, 404,
																																		FONT_NEXT, 401, 367,
																																		FONT_END, 338, 288,
																																		FONT_BEGIN, 375, 404,
																																		FONT_NEXT, 338, 288,
																																		FONT_END, 333, 463,
																																		FONT_BEGIN, 696, 662,
																																		FONT_NEXT, 696, 643,
																																		FONT_NEXT, 458, 662,
																																		FONT_NEXT, 660, 638,
																																		FONT_END, 627, 627,
																																		FONT_BEGIN, 458, 662,
																																		FONT_NEXT, 627, 627,
																																		FONT_END, 592, 599,
																																		FONT_BEGIN, 704, 19,
																																		FONT_NEXT, 704, 0,
																																		FONT_NEXT, 670, 24,
																																		FONT_NEXT, 407, 0,
																																		FONT_NEXT, 643, 37,
																																		FONT_END, 619, 58,
																																		FONT_BEGIN, 619, 58,
																																		FONT_NEXT, 407, 0,
																																		FONT_END, 464, 25,
																																		FONT_BEGIN, 619, 58,
																																		FONT_NEXT, 464, 25,
																																		FONT_NEXT, 593, 93,
																																		FONT_NEXT, 479, 35,
																																		FONT_END, 484, 53,
																																		FONT_BEGIN, 593, 93,
																																		FONT_NEXT, 484, 53,
																																		FONT_NEXT, 401, 367,
																																		FONT_NEXT, 478, 72,
																																		FONT_END, 464, 98,
																																		FONT_BEGIN, 401, 367,
																																		FONT_NEXT, 464, 98,
																																		FONT_END, 433, 148,
																																		FONT_BEGIN, 401, 367,
																																		FONT_NEXT, 433, 148,
																																		FONT_END, 338, 288,
																																		FONT_BEGIN, 407, 19,
																																		FONT_NEXT, 440, 21,
																																		FONT_NEXT, 407, 0,
																																		FONT_END, 464, 25,
																																		FONT_BEGIN, 243, 19,
																																		FONT_NEXT, 243, 0,
																																		FONT_NEXT, 190, 26,
																																		FONT_NEXT, 10, 0,
																																		FONT_NEXT, 173, 37,
																																		FONT_END, 167, 56,
																																		FONT_ADVANCE, 722, 0
																																},
																																{
																																	89,
																																		FONT_BEGIN, 302, 662,
																																		FONT_NEXT, 302, 643,
																																		FONT_NEXT, 22, 662,
																																		FONT_NEXT, 271, 641,
																																		FONT_END, 249, 637,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 249, 637,
																																		FONT_END, 235, 628,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 235, 628,
																																		FONT_END, 231, 612,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 231, 612,
																																		FONT_END, 81, 619,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 81, 619,
																																		FONT_END, 51, 637,
																																		FONT_BEGIN, 22, 662,
																																		FONT_NEXT, 51, 637,
																																		FONT_END, 22, 643,
																																		FONT_BEGIN, 484, 643,
																																		FONT_NEXT, 484, 662,
																																		FONT_NEXT, 530, 638,
																																		FONT_NEXT, 646, 624,
																																		FONT_END, 610, 590,
																																		FONT_BEGIN, 530, 638,
																																		FONT_NEXT, 610, 590,
																																		FONT_NEXT, 546, 628,
																																		FONT_NEXT, 589, 564,
																																		FONT_NEXT, 553, 610,
																																		FONT_NEXT, 565, 529,
																																		FONT_NEXT, 539, 573,
																																		FONT_NEXT, 417, 303,
																																		FONT_NEXT, 396, 347,
																																		FONT_NEXT, 417, 109,
																																		FONT_END, 313, 72,
																																		FONT_BEGIN, 396, 347,
																																		FONT_NEXT, 313, 72,
																																		FONT_END, 315, 120,
																																		FONT_BEGIN, 396, 347,
																																		FONT_NEXT, 315, 120,
																																		FONT_END, 315, 294,
																																		FONT_BEGIN, 396, 347,
																																		FONT_NEXT, 315, 294,
																																		FONT_NEXT, 248, 569,
																																		FONT_NEXT, 184, 486,
																																		FONT_NEXT, 231, 612,
																																		FONT_NEXT, 149, 535,
																																		FONT_END, 121, 573,
																																		FONT_BEGIN, 231, 612,
																																		FONT_NEXT, 121, 573,
																																		FONT_END, 99, 600,
																																		FONT_BEGIN, 231, 612,
																																		FONT_NEXT, 99, 600,
																																		FONT_END, 81, 619,
																																		FONT_BEGIN, 703, 662,
																																		FONT_NEXT, 703, 643,
																																		FONT_NEXT, 484, 662,
																																		FONT_NEXT, 675, 638,
																																		FONT_END, 646, 624,
																																		FONT_BEGIN, 520, 19,
																																		FONT_NEXT, 520, 0,
																																		FONT_NEXT, 465, 23,
																																		FONT_NEXT, 214, 0,
																																		FONT_NEXT, 434, 37,
																																		FONT_END, 420, 64,
																																		FONT_BEGIN, 420, 64,
																																		FONT_NEXT, 214, 0,
																																		FONT_END, 271, 24,
																																		FONT_BEGIN, 420, 64,
																																		FONT_NEXT, 271, 24,
																																		FONT_END, 301, 41,
																																		FONT_BEGIN, 420, 64,
																																		FONT_NEXT, 301, 41,
																																		FONT_NEXT, 417, 109,
																																		FONT_END, 313, 72,
																																		FONT_BEGIN, 214, 19,
																																		FONT_NEXT, 246, 20,
																																		FONT_NEXT, 214, 0,
																																		FONT_END, 271, 24,
																																		FONT_ADVANCE, 722, 0
																																},
																																	{
																																		90,
																																			FONT_BEGIN, 577, 662,
																																			FONT_NEXT, 577, 647,
																																			FONT_NEXT, 51, 662,
																																			FONT_NEXT, 446, 624,
																																			FONT_END, 225, 624,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 225, 624,
																																			FONT_END, 177, 621,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 177, 621,
																																			FONT_END, 140, 612,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 140, 612,
																																			FONT_END, 112, 599,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 112, 599,
																																			FONT_END, 92, 582,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 92, 582,
																																			FONT_END, 68, 540,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 68, 540,
																																			FONT_END, 57, 491,
																																			FONT_BEGIN, 51, 662,
																																			FONT_NEXT, 57, 491,
																																			FONT_END, 31, 491,
																																			FONT_BEGIN, 597, 176,
																																			FONT_NEXT, 573, 0,
																																			FONT_NEXT, 574, 176,
																																			FONT_END, 556, 123,
																																			FONT_BEGIN, 556, 123,
																																			FONT_NEXT, 573, 0,
																																			FONT_NEXT, 528, 79,
																																			FONT_END, 481, 49,
																																			FONT_BEGIN, 481, 49,
																																			FONT_NEXT, 573, 0,
																																			FONT_NEXT, 446, 40,
																																			FONT_NEXT, 9, 0,
																																			FONT_NEXT, 402, 38,
																																			FONT_END, 145, 38,
																																			FONT_BEGIN, 145, 38,
																																			FONT_NEXT, 9, 0,
																																			FONT_END, 9, 15,
																																			FONT_BEGIN, 145, 38,
																																			FONT_NEXT, 9, 15,
																																			FONT_END, 446, 624,
																																			FONT_BEGIN, 145, 38,
																																			FONT_NEXT, 446, 624,
																																			FONT_END, 577, 647,
																																			FONT_ADVANCE, 611, 0
																																	},
																																	{
																																		91,
																																			FONT_BEGIN, 299, 662,
																																			FONT_NEXT, 299, 637,
																																			FONT_NEXT, 88, 662,
																																			FONT_NEXT, 209, 637,
																																			FONT_END, 182, 632,
																																			FONT_BEGIN, 88, 662,
																																			FONT_NEXT, 182, 632,
																																			FONT_END, 169, 620,
																																			FONT_BEGIN, 88, 662,
																																			FONT_NEXT, 169, 620,
																																			FONT_END, 164, 593,
																																			FONT_BEGIN, 88, 662,
																																			FONT_NEXT, 164, 593,
																																			FONT_END, 164, -79,
																																			FONT_BEGIN, 88, 662,
																																			FONT_NEXT, 164, -79,
																																			FONT_NEXT, 88, -156,
																																			FONT_NEXT, 170, -110,
																																			FONT_END, 184, -125,
																																			FONT_BEGIN, 88, -156,
																																			FONT_NEXT, 184, -125,
																																			FONT_END, 213, -131,
																																			FONT_BEGIN, 88, -156,
																																			FONT_NEXT, 213, -131,
																																			FONT_NEXT, 299, -156,
																																			FONT_END, 299, -131,
																																			FONT_ADVANCE, 333, 0
																																	},
																																		{
																																			92,
																																				FONT_BEGIN, 287, -14,
																																				FONT_NEXT, 219, -14,
																																				FONT_NEXT, 58, 676,
																																				FONT_END, -9, 676,
																																				FONT_ADVANCE, 278, 0
																																		},
																																		{
																																			93,
																																				FONT_BEGIN, 34, 637,
																																				FONT_NEXT, 34, 662,
																																				FONT_NEXT, 120, 637,
																																				FONT_END, 148, 630,
																																				FONT_BEGIN, 148, 630,
																																				FONT_NEXT, 34, 662,
																																				FONT_END, 245, 662,
																																				FONT_BEGIN, 148, 630,
																																				FONT_NEXT, 245, 662,
																																				FONT_NEXT, 162, 615,
																																				FONT_END, 169, 585,
																																				FONT_BEGIN, 169, 585,
																																				FONT_NEXT, 245, 662,
																																				FONT_END, 245, -156,
																																				FONT_BEGIN, 169, 585,
																																				FONT_NEXT, 245, -156,
																																				FONT_NEXT, 169, -87,
																																				FONT_END, 163, -115,
																																				FONT_BEGIN, 163, -115,
																																				FONT_NEXT, 245, -156,
																																				FONT_NEXT, 150, -127,
																																				FONT_NEXT, 34, -156,
																																				FONT_NEXT, 124, -131,
																																				FONT_END, 34, -131,
																																				FONT_ADVANCE, 333, 0
																																		},
																																			{
																																				94,
																																					FONT_BEGIN, 446, 297,
																																					FONT_NEXT, 378, 297,
																																					FONT_NEXT, 265, 662,
																																					FONT_NEXT, 235, 586,
																																					FONT_NEXT, 205, 662,
																																					FONT_NEXT, 92, 297,
																																					FONT_END, 24, 297,
																																					FONT_ADVANCE, 469, 0
																																			},
																																			{
																																				95,
																																					FONT_BEGIN, 500, -75,
																																					FONT_NEXT, 500, -125,
																																					FONT_NEXT, 0, -75,
																																					FONT_END, 0, -125,
																																					FONT_ADVANCE, 500, 0
																																			},
																																				{
																																					96,
																																						FONT_BEGIN, 236, 657,
																																						FONT_NEXT, 195, 625,
																																						FONT_NEXT, 227, 676,
																																						FONT_END, 196, 658,
																																						FONT_BEGIN, 196, 658,
																																						FONT_NEXT, 195, 625,
																																						FONT_NEXT, 159, 625,
																																						FONT_NEXT, 170, 595,
																																						FONT_END, 157, 569,
																																						FONT_BEGIN, 159, 625,
																																						FONT_NEXT, 157, 569,
																																						FONT_NEXT, 128, 580,
																																						FONT_NEXT, 154, 551,
																																						FONT_END, 146, 449,
																																						FONT_BEGIN, 128, 580,
																																						FONT_NEXT, 146, 449,
																																						FONT_END, 124, 478,
																																						FONT_BEGIN, 128, 580,
																																						FONT_NEXT, 124, 478,
																																						FONT_END, 115, 529,
																																						FONT_BEGIN, 254, 491,
																																						FONT_NEXT, 248, 463,
																																						FONT_NEXT, 247, 515,
																																						FONT_NEXT, 234, 445,
																																						FONT_NEXT, 232, 531,
																																						FONT_NEXT, 196, 433,
																																						FONT_NEXT, 192, 541,
																																						FONT_NEXT, 173, 436,
																																						FONT_NEXT, 177, 539,
																																						FONT_END, 168, 537,
																																						FONT_BEGIN, 168, 537,
																																						FONT_NEXT, 173, 436,
																																						FONT_END, 146, 449,
																																						FONT_BEGIN, 168, 537,
																																						FONT_NEXT, 146, 449,
																																						FONT_NEXT, 158, 540,
																																						FONT_END, 154, 551,
																																						FONT_ADVANCE, 333, 0
																																				},
																																				{
																																					97,
																																						FONT_BEGIN, 56, 349,
																																						FONT_NEXT, 65, 383,
																																						FONT_NEXT, 65, 321,
																																						FONT_END, 79, 309,
																																						FONT_BEGIN, 79, 309,
																																						FONT_NEXT, 65, 383,
																																						FONT_END, 94, 419,
																																						FONT_BEGIN, 79, 309,
																																						FONT_NEXT, 94, 419,
																																						FONT_NEXT, 99, 305,
																																						FONT_END, 131, 318,
																																						FONT_BEGIN, 131, 318,
																																						FONT_NEXT, 94, 419,
																																						FONT_END, 139, 387,
																																						FONT_BEGIN, 131, 318,
																																						FONT_NEXT, 139, 387,
																																						FONT_END, 144, 348,
																																						FONT_BEGIN, 37, 94,
																																						FONT_NEXT, 41, 128,
																																						FONT_NEXT, 45, 49,
																																						FONT_NEXT, 53, 158,
																																						FONT_NEXT, 69, 16,
																																						FONT_NEXT, 73, 183,
																																						FONT_END, 100, 206,
																																						FONT_BEGIN, 69, 16,
																																						FONT_NEXT, 100, 206,
																																						FONT_NEXT, 103, -4,
																																						FONT_END, 125, 126,
																																						FONT_BEGIN, 125, 126,
																																						FONT_NEXT, 100, 206,
																																						FONT_NEXT, 125, 128,
																																						FONT_END, 129, 157,
																																						FONT_BEGIN, 129, 157,
																																						FONT_NEXT, 100, 206,
																																						FONT_NEXT, 149, 192,
																																						FONT_END, 169, 210,
																																						FONT_BEGIN, 169, 210,
																																						FONT_NEXT, 100, 206,
																																						FONT_END, 179, 248,
																																						FONT_BEGIN, 169, 210,
																																						FONT_NEXT, 179, 248,
																																						FONT_NEXT, 197, 229,
																																						FONT_END, 236, 249,
																																						FONT_BEGIN, 236, 249,
																																						FONT_NEXT, 179, 248,
																																						FONT_NEXT, 287, 268,
																																						FONT_NEXT, 287, 292,
																																						FONT_NEXT, 287, 123,
																																						FONT_NEXT, 287, 353,
																																						FONT_NEXT, 288, 63,
																																						FONT_END, 297, 21,
																																						FONT_BEGIN, 297, 21,
																																						FONT_NEXT, 287, 353,
																																						FONT_NEXT, 305, 443,
																																						FONT_NEXT, 279, 396,
																																						FONT_NEXT, 268, 455,
																																						FONT_NEXT, 259, 421,
																																						FONT_NEXT, 219, 460,
																																						FONT_NEXT, 234, 433,
																																						FONT_END, 210, 436,
																																						FONT_BEGIN, 219, 460,
																																						FONT_NEXT, 210, 436,
																																						FONT_NEXT, 178, 456,
																																						FONT_NEXT, 162, 423,
																																						FONT_NEXT, 144, 448,
																																						FONT_NEXT, 145, 407,
																																						FONT_END, 139, 387,
																																						FONT_BEGIN, 144, 448,
																																						FONT_NEXT, 139, 387,
																																						FONT_END, 94, 419,
																																						FONT_BEGIN, 287, 123,
																																						FONT_NEXT, 288, 63,
																																						FONT_NEXT, 281, 88,
																																						FONT_NEXT, 222, 14,
																																						FONT_NEXT, 271, 77,
																																						FONT_END, 253, 66,
																																						FONT_BEGIN, 253, 66,
																																						FONT_NEXT, 222, 14,
																																						FONT_NEXT, 226, 55,
																																						FONT_END, 191, 48,
																																						FONT_BEGIN, 191, 48,
																																						FONT_NEXT, 222, 14,
																																						FONT_END, 183, -4,
																																						FONT_BEGIN, 191, 48,
																																						FONT_NEXT, 183, -4,
																																						FONT_NEXT, 165, 53,
																																						FONT_NEXT, 143, -10,
																																						FONT_NEXT, 144, 69,
																																						FONT_END, 130, 93,
																																						FONT_BEGIN, 130, 93,
																																						FONT_NEXT, 143, -10,
																																						FONT_END, 103, -4,
																																						FONT_BEGIN, 130, 93,
																																						FONT_NEXT, 103, -4,
																																						FONT_END, 125, 126,
																																						FONT_BEGIN, 368, 300,
																																						FONT_NEXT, 368, 105,
																																						FONT_NEXT, 365, 350,
																																						FONT_NEXT, 353, -10,
																																						FONT_NEXT, 349, 402,
																																						FONT_NEXT, 315, -1,
																																						FONT_NEXT, 331, 425,
																																						FONT_END, 305, 443,
																																						FONT_BEGIN, 305, 443,
																																						FONT_NEXT, 315, -1,
																																						FONT_END, 297, 21,
																																						FONT_BEGIN, 305, 443,
																																						FONT_END, 297, 21,
																																						FONT_BEGIN, 442, 66,
																																						FONT_NEXT, 442, 40,
																																						FONT_NEXT, 423, 53,
																																						FONT_NEXT, 412, 11,
																																						FONT_NEXT, 397, 47,
																																						FONT_NEXT, 387, -4,
																																						FONT_NEXT, 380, 51,
																																						FONT_NEXT, 353, -10,
																																						FONT_NEXT, 371, 65,
																																						FONT_END, 368, 105,
																																						FONT_ADVANCE, 444, 0
																																				},
																																					{
																																						98,
																																							FONT_BEGIN, 3, 623,
																																							FONT_NEXT, 3, 639,
																																							FONT_NEXT, 25, 624,
																																							FONT_END, 50, 620,
																																							FONT_BEGIN, 50, 620,
																																							FONT_NEXT, 3, 639,
																																							FONT_END, 148, 683,
																																							FONT_BEGIN, 50, 620,
																																							FONT_NEXT, 148, 683,
																																							FONT_NEXT, 63, 609,
																																							FONT_END, 69, 573,
																																							FONT_BEGIN, 69, 573,
																																							FONT_NEXT, 148, 683,
																																							FONT_END, 127, 8,
																																							FONT_BEGIN, 69, 573,
																																							FONT_NEXT, 127, 8,
																																							FONT_END, 86, 28,
																																							FONT_BEGIN, 69, 573,
																																							FONT_NEXT, 86, 28,
																																							FONT_END, 73, 40,
																																							FONT_BEGIN, 69, 573,
																																							FONT_NEXT, 73, 40,
																																							FONT_END, 69, 54,
																																							FONT_BEGIN, 153, 379,
																																							FONT_NEXT, 155, 379,
																																							FONT_NEXT, 153, 322,
																																							FONT_END, 165, 357,
																																							FONT_BEGIN, 165, 357,
																																							FONT_NEXT, 155, 379,
																																							FONT_END, 167, 401,
																																							FONT_BEGIN, 165, 357,
																																							FONT_NEXT, 167, 401,
																																							FONT_NEXT, 192, 380,
																																							FONT_NEXT, 196, 428,
																																							FONT_NEXT, 224, 393,
																																							FONT_NEXT, 237, 450,
																																							FONT_NEXT, 253, 397,
																																							FONT_END, 284, 392,
																																							FONT_BEGIN, 284, 392,
																																							FONT_NEXT, 237, 450,
																																							FONT_END, 290, 460,
																																							FONT_BEGIN, 284, 392,
																																							FONT_NEXT, 290, 460,
																																							FONT_NEXT, 311, 379,
																																							FONT_NEXT, 332, 454,
																																							FONT_NEXT, 350, 333,
																																							FONT_NEXT, 368, 440,
																																							FONT_NEXT, 372, 270,
																																							FONT_END, 380, 201,
																																							FONT_BEGIN, 380, 201,
																																							FONT_NEXT, 368, 440,
																																							FONT_END, 399, 417,
																																							FONT_BEGIN, 380, 201,
																																							FONT_NEXT, 399, 417,
																																							FONT_END, 382, 44,
																																							FONT_BEGIN, 380, 201,
																																							FONT_NEXT, 382, 44,
																																							FONT_NEXT, 375, 145,
																																							FONT_NEXT, 340, 16,
																																							FONT_NEXT, 357, 86,
																																							FONT_END, 340, 61,
																																							FONT_BEGIN, 340, 61,
																																							FONT_NEXT, 340, 16,
																																							FONT_NEXT, 317, 40,
																																							FONT_NEXT, 287, -4,
																																							FONT_NEXT, 287, 26,
																																							FONT_END, 249, 22,
																																							FONT_BEGIN, 249, 22,
																																							FONT_NEXT, 287, -4,
																																							FONT_END, 224, -10,
																																							FONT_BEGIN, 249, 22,
																																							FONT_NEXT, 224, -10,
																																							FONT_NEXT, 223, 24,
																																							FONT_NEXT, 178, -6,
																																							FONT_NEXT, 191, 32,
																																							FONT_END, 164, 47,
																																							FONT_BEGIN, 164, 47,
																																							FONT_NEXT, 178, -6,
																																							FONT_END, 127, 8,
																																							FONT_BEGIN, 164, 47,
																																							FONT_NEXT, 127, 8,
																																							FONT_NEXT, 153, 70,
																																							FONT_NEXT, 148, 683,
																																							FONT_NEXT, 153, 322,
																																							FONT_END, 153, 379,
																																							FONT_BEGIN, 153, 379,
																																							FONT_NEXT, 148, 683,
																																							FONT_END, 153, 681,
																																							FONT_BEGIN, 153, 379,
																																							FONT_END, 153, 681,
																																							FONT_BEGIN, 468, 238,
																																							FONT_NEXT, 455, 161,
																																							FONT_NEXT, 457, 317,
																																							FONT_END, 424, 389,
																																							FONT_BEGIN, 424, 389,
																																							FONT_NEXT, 455, 161,
																																							FONT_END, 439, 119,
																																							FONT_BEGIN, 424, 389,
																																							FONT_NEXT, 439, 119,
																																							FONT_END, 415, 80,
																																							FONT_BEGIN, 424, 389,
																																							FONT_NEXT, 415, 80,
																																							FONT_NEXT, 399, 417,
																																							FONT_END, 382, 44,
																																							FONT_ADVANCE, 500, 0
																																					},
																																					{
																																						99,
																																							FONT_BEGIN, 25, 213,
																																							FONT_NEXT, 30, 270,
																																							FONT_NEXT, 29, 158,
																																							FONT_END, 42, 111,
																																							FONT_BEGIN, 42, 111,
																																							FONT_NEXT, 30, 270,
																																							FONT_END, 44, 320,
																																							FONT_BEGIN, 42, 111,
																																							FONT_NEXT, 44, 320,
																																							FONT_NEXT, 63, 73,
																																							FONT_NEXT, 67, 363,
																																							FONT_NEXT, 88, 42,
																																							FONT_NEXT, 96, 398,
																																							FONT_NEXT, 102, 253,
																																							FONT_END, 106, 302,
																																							FONT_BEGIN, 106, 302,
																																							FONT_NEXT, 96, 398,
																																							FONT_NEXT, 116, 342,
																																							FONT_NEXT, 130, 425,
																																							FONT_NEXT, 133, 373,
																																							FONT_END, 152, 397,
																																							FONT_BEGIN, 152, 397,
																																							FONT_NEXT, 130, 425,
																																							FONT_END, 168, 444,
																																							FONT_BEGIN, 152, 397,
																																							FONT_NEXT, 168, 444,
																																							FONT_NEXT, 195, 423,
																																							FONT_END, 231, 431,
																																							FONT_BEGIN, 231, 431,
																																							FONT_NEXT, 168, 444,
																																							FONT_END, 249, 460,
																																							FONT_BEGIN, 231, 431,
																																							FONT_NEXT, 249, 460,
																																							FONT_NEXT, 262, 427,
																																							FONT_END, 280, 418,
																																							FONT_BEGIN, 280, 418,
																																							FONT_NEXT, 249, 460,
																																							FONT_END, 306, 450,
																																							FONT_BEGIN, 280, 418,
																																							FONT_NEXT, 306, 450,
																																							FONT_NEXT, 297, 383,
																																							FONT_END, 303, 361,
																																							FONT_BEGIN, 303, 361,
																																							FONT_NEXT, 306, 450,
																																							FONT_NEXT, 320, 329,
																																							FONT_END, 334, 318,
																																							FONT_BEGIN, 334, 318,
																																							FONT_NEXT, 306, 450,
																																							FONT_NEXT, 352, 315,
																																							FONT_NEXT, 355, 429,
																																							FONT_NEXT, 386, 329,
																																							FONT_NEXT, 387, 392,
																																							FONT_END, 398, 359,
																																							FONT_BEGIN, 412, 147,
																																							FONT_NEXT, 369, 74,
																																							FONT_NEXT, 398, 156,
																																							FONT_END, 340, 89,
																																							FONT_BEGIN, 340, 89,
																																							FONT_NEXT, 369, 74,
																																							FONT_END, 322, 25,
																																							FONT_BEGIN, 340, 89,
																																							FONT_NEXT, 322, 25,
																																							FONT_NEXT, 302, 69,
																																							FONT_NEXT, 269, -2,
																																							FONT_NEXT, 255, 62,
																																							FONT_NEXT, 212, -10,
																																							FONT_NEXT, 195, 75,
																																							FONT_NEXT, 149, 2,
																																							FONT_NEXT, 147, 114,
																																							FONT_NEXT, 88, 42,
																																							FONT_NEXT, 114, 174,
																																							FONT_END, 102, 253,
																																							FONT_ADVANCE, 444, 0
																																					},
																																						{
																																							100,
																																								FONT_BEGIN, 113, 247,
																																								FONT_NEXT, 116, 195,
																																								FONT_NEXT, 103, 25,
																																								FONT_NEXT, 125, 152,
																																								FONT_END, 139, 116,
																																								FONT_BEGIN, 103, 25,
																																								FONT_NEXT, 139, 116,
																																								FONT_END, 157, 88,
																																								FONT_BEGIN, 103, 25,
																																								FONT_NEXT, 157, 88,
																																								FONT_NEXT, 164, -4,
																																								FONT_NEXT, 201, 53,
																																								FONT_NEXT, 209, -10,
																																								FONT_NEXT, 251, 42,
																																								FONT_NEXT, 257, -4,
																																								FONT_NEXT, 281, 46,
																																								FONT_NEXT, 294, 11,
																																								FONT_NEXT, 308, 58,
																																								FONT_NEXT, 320, 32,
																																								FONT_NEXT, 328, 77,
																																								FONT_NEXT, 338, 54,
																																								FONT_END, 340, 54,
																																								FONT_BEGIN, 340, 54,
																																								FONT_NEXT, 328, 77,
																																								FONT_END, 340, 102,
																																								FONT_BEGIN, 340, 54,
																																								FONT_NEXT, 340, 102,
																																								FONT_NEXT, 340, -7,
																																								FONT_NEXT, 340, 332,
																																								FONT_NEXT, 340, 417,
																																								FONT_NEXT, 327, 376,
																																								FONT_NEXT, 290, 449,
																																								FONT_NEXT, 303, 407,
																																								FONT_END, 271, 425,
																																								FONT_BEGIN, 290, 449,
																																								FONT_NEXT, 271, 425,
																																								FONT_NEXT, 234, 460,
																																								FONT_NEXT, 237, 432,
																																								FONT_END, 194, 422,
																																								FONT_BEGIN, 234, 460,
																																								FONT_NEXT, 194, 422,
																																								FONT_NEXT, 192, 454,
																																								FONT_NEXT, 154, 390,
																																								FONT_NEXT, 154, 439,
																																								FONT_END, 119, 416,
																																								FONT_BEGIN, 119, 416,
																																								FONT_NEXT, 154, 390,
																																								FONT_END, 124, 333,
																																								FONT_BEGIN, 119, 416,
																																								FONT_NEXT, 124, 333,
																																								FONT_END, 116, 293,
																																								FONT_BEGIN, 119, 416,
																																								FONT_NEXT, 116, 293,
																																								FONT_NEXT, 88, 386,
																																								FONT_NEXT, 113, 247,
																																								FONT_END, 103, 25,
																																								FONT_BEGIN, 88, 386,
																																								FONT_NEXT, 103, 25,
																																								FONT_END, 74, 53,
																																								FONT_BEGIN, 88, 386,
																																								FONT_NEXT, 74, 53,
																																								FONT_NEXT, 43, 307,
																																								FONT_NEXT, 49, 92,
																																								FONT_END, 33, 145,
																																								FONT_BEGIN, 43, 307,
																																								FONT_NEXT, 33, 145,
																																								FONT_NEXT, 31, 262,
																																								FONT_END, 27, 214,
																																								FONT_BEGIN, 272, 623,
																																								FONT_NEXT, 272, 639,
																																								FONT_NEXT, 296, 624,
																																								FONT_END, 321, 620,
																																								FONT_BEGIN, 321, 620,
																																								FONT_NEXT, 272, 639,
																																								FONT_END, 419, 683,
																																								FONT_BEGIN, 321, 620,
																																								FONT_NEXT, 419, 683,
																																								FONT_NEXT, 334, 609,
																																								FONT_END, 340, 573,
																																								FONT_BEGIN, 340, 573,
																																								FONT_NEXT, 419, 683,
																																								FONT_END, 344, -10,
																																								FONT_BEGIN, 340, 573,
																																								FONT_NEXT, 344, -10,
																																								FONT_END, 340, -7,
																																								FONT_BEGIN, 340, 573,
																																								FONT_NEXT, 340, -7,
																																								FONT_END, 340, 417,
																																								FONT_BEGIN, 491, 58,
																																								FONT_NEXT, 491, 42,
																																								FONT_NEXT, 452, 58,
																																								FONT_NEXT, 344, -10,
																																								FONT_NEXT, 432, 68,
																																								FONT_END, 425, 87,
																																								FONT_BEGIN, 425, 87,
																																								FONT_NEXT, 344, -10,
																																								FONT_NEXT, 424, 114,
																																								FONT_NEXT, 419, 683,
																																								FONT_END, 424, 681,
																																								FONT_ADVANCE, 500, 0
																																						},
																																						{
																																							101,
																																								FONT_BEGIN, 424, 157,
																																								FONT_NEXT, 417, 140,
																																								FONT_NEXT, 408, 164,
																																								FONT_NEXT, 406, 117,
																																								FONT_NEXT, 386, 131,
																																								FONT_NEXT, 368, 61,
																																								FONT_NEXT, 355, 97,
																																								FONT_NEXT, 304, 11,
																																								FONT_NEXT, 311, 70,
																																								FONT_END, 253, 59,
																																								FONT_BEGIN, 253, 59,
																																								FONT_NEXT, 304, 11,
																																								FONT_END, 262, -5,
																																								FONT_BEGIN, 253, 59,
																																								FONT_NEXT, 262, -5,
																																								FONT_END, 212, -10,
																																								FONT_BEGIN, 253, 59,
																																								FONT_NEXT, 212, -10,
																																								FONT_NEXT, 202, 68,
																																								FONT_NEXT, 172, -7,
																																								FONT_NEXT, 153, 101,
																																								FONT_NEXT, 135, 5,
																																								FONT_NEXT, 132, 130,
																																								FONT_NEXT, 104, 23,
																																								FONT_NEXT, 114, 168,
																																								FONT_END, 101, 216,
																																								FONT_BEGIN, 101, 216,
																																								FONT_NEXT, 104, 23,
																																								FONT_END, 76, 48,
																																								FONT_BEGIN, 101, 216,
																																								FONT_NEXT, 76, 48,
																																								FONT_NEXT, 94, 277,
																																								FONT_END, 68, 373,
																																								FONT_BEGIN, 68, 373,
																																								FONT_NEXT, 76, 48,
																																								FONT_END, 54, 80,
																																								FONT_BEGIN, 68, 373,
																																								FONT_NEXT, 54, 80,
																																								FONT_NEXT, 45, 330,
																																								FONT_NEXT, 38, 119,
																																								FONT_NEXT, 30, 277,
																																								FONT_NEXT, 28, 163,
																																								FONT_END, 25, 214,
																																								FONT_BEGIN, 303, 309,
																																								FONT_NEXT, 94, 277,
																																								FONT_NEXT, 97, 309,
																																								FONT_NEXT, 68, 373,
																																								FONT_END, 97, 406,
																																								FONT_BEGIN, 97, 309,
																																								FONT_NEXT, 97, 406,
																																								FONT_NEXT, 113, 364,
																																								FONT_NEXT, 129, 430,
																																								FONT_NEXT, 137, 399,
																																								FONT_NEXT, 163, 447,
																																								FONT_NEXT, 168, 418,
																																								FONT_END, 207, 424,
																																								FONT_BEGIN, 207, 424,
																																								FONT_NEXT, 163, 447,
																																								FONT_END, 229, 460,
																																								FONT_BEGIN, 207, 424,
																																								FONT_NEXT, 229, 460,
																																								FONT_NEXT, 254, 412,
																																								FONT_END, 282, 384,
																																								FONT_BEGIN, 282, 384,
																																								FONT_NEXT, 229, 460,
																																								FONT_END, 294, 449,
																																								FONT_BEGIN, 282, 384,
																																								FONT_NEXT, 294, 449,
																																								FONT_NEXT, 303, 309,
																																								FONT_NEXT, 348, 417,
																																								FONT_END, 386, 360,
																																								FONT_BEGIN, 303, 309,
																																								FONT_NEXT, 386, 360,
																																								FONT_END, 398, 322,
																																								FONT_BEGIN, 303, 309,
																																								FONT_NEXT, 398, 322,
																																								FONT_NEXT, 94, 277,
																																								FONT_END, 405, 277,
																																								FONT_ADVANCE, 444, 0
																																						},
																																							{
																																								102,
																																									FONT_BEGIN, 383, 622,
																																									FONT_NEXT, 379, 603,
																																									FONT_NEXT, 375, 645,
																																									FONT_NEXT, 369, 590,
																																									FONT_NEXT, 354, 665,
																																									FONT_NEXT, 340, 580,
																																									FONT_NEXT, 322, 678,
																																									FONT_NEXT, 246, 655,
																																									FONT_NEXT, 283, 683,
																																									FONT_END, 223, 675,
																																									FONT_BEGIN, 223, 675,
																																									FONT_NEXT, 246, 655,
																																									FONT_END, 213, 646,
																																									FONT_BEGIN, 223, 675,
																																									FONT_NEXT, 213, 646,
																																									FONT_NEXT, 178, 654,
																																									FONT_NEXT, 195, 624,
																																									FONT_END, 186, 566,
																																									FONT_BEGIN, 178, 654,
																																									FONT_NEXT, 186, 566,
																																									FONT_END, 186, 450,
																																									FONT_BEGIN, 178, 654,
																																									FONT_NEXT, 186, 450,
																																									FONT_END, 99, 58,
																																									FONT_BEGIN, 178, 654,
																																									FONT_NEXT, 99, 58,
																																									FONT_END, 103, 104,
																																									FONT_BEGIN, 178, 654,
																																									FONT_NEXT, 103, 104,
																																									FONT_NEXT, 146, 624,
																																									FONT_END, 125, 587,
																																									FONT_BEGIN, 125, 587,
																																									FONT_NEXT, 103, 104,
																																									FONT_NEXT, 105, 510,
																																									FONT_END, 103, 476,
																																									FONT_BEGIN, 103, 476,
																																									FONT_NEXT, 103, 104,
																																									FONT_NEXT, 103, 450,
																																									FONT_NEXT, 103, 418,
																																									FONT_NEXT, 21, 450,
																																									FONT_END, 21, 418,
																																									FONT_BEGIN, 309, 450,
																																									FONT_NEXT, 309, 418,
																																									FONT_NEXT, 186, 450,
																																									FONT_NEXT, 187, 418,
																																									FONT_END, 187, 104,
																																									FONT_BEGIN, 186, 450,
																																									FONT_NEXT, 187, 104,
																																									FONT_NEXT, 99, 58,
																																									FONT_END, 86, 31,
																																									FONT_BEGIN, 86, 31,
																																									FONT_NEXT, 187, 104,
																																									FONT_END, 188, 64,
																																									FONT_BEGIN, 86, 31,
																																									FONT_NEXT, 188, 64,
																																									FONT_END, 20, 0,
																																									FONT_BEGIN, 86, 31,
																																									FONT_NEXT, 20, 0,
																																									FONT_NEXT, 60, 19,
																																									FONT_END, 20, 15,
																																									FONT_BEGIN, 280, 15,
																																									FONT_NEXT, 280, 0,
																																									FONT_NEXT, 226, 21,
																																									FONT_NEXT, 20, 0,
																																									FONT_NEXT, 199, 37,
																																									FONT_END, 188, 64,
																																									FONT_ADVANCE, 333, 0
																																							},
																																							{
																																								103,
																																									FONT_BEGIN, 69, 293,
																																									FONT_NEXT, 73, 334,
																																									FONT_NEXT, 75, 251,
																																									FONT_NEXT, 85, 369,
																																									FONT_NEXT, 93, 216,
																																									FONT_END, 122, 186,
																																									FONT_BEGIN, 122, 186,
																																									FONT_NEXT, 85, 369,
																																									FONT_END, 126, 421,
																																									FONT_BEGIN, 122, 186,
																																									FONT_NEXT, 126, 421,
																																									FONT_END, 152, 342,
																																									FONT_BEGIN, 122, 186,
																																									FONT_NEXT, 152, 342,
																																									FONT_END, 157, 282,
																																									FONT_BEGIN, 122, 186,
																																									FONT_NEXT, 157, 282,
																																									FONT_NEXT, 162, 163,
																																									FONT_NEXT, 174, 228,
																																									FONT_END, 175, 146,
																																									FONT_BEGIN, 162, 163,
																																									FONT_NEXT, 175, 146,
																																									FONT_END, 155, 129,
																																									FONT_BEGIN, 162, 163,
																																									FONT_NEXT, 155, 129,
																																									FONT_NEXT, 132, 136,
																																									FONT_NEXT, 139, 109,
																																									FONT_END, 133, 91,
																																									FONT_BEGIN, 132, 136,
																																									FONT_NEXT, 133, 91,
																																									FONT_END, 126, 1,
																																									FONT_BEGIN, 132, 136,
																																									FONT_NEXT, 126, 1,
																																									FONT_NEXT, 110, 115,
																																									FONT_NEXT, 89, 21,
																																									FONT_NEXT, 84, 85,
																																									FONT_NEXT, 77, 35,
																																									FONT_NEXT, 74, 67,
																																									FONT_END, 73, 54,
																																									FONT_BEGIN, 175, 146,
																																									FONT_NEXT, 174, 228,
																																									FONT_NEXT, 193, 154,
																																									FONT_NEXT, 206, 189,
																																									FONT_END, 229, 177,
																																									FONT_BEGIN, 193, 154,
																																									FONT_NEXT, 229, 177,
																																									FONT_NEXT, 247, 149,
																																									FONT_NEXT, 256, 174,
																																									FONT_NEXT, 295, 156,
																																									FONT_NEXT, 296, 185,
																																									FONT_END, 318, 213,
																																									FONT_BEGIN, 295, 156,
																																									FONT_NEXT, 318, 213,
																																									FONT_NEXT, 347, 182,
																																									FONT_NEXT, 327, 245,
																																									FONT_END, 329, 270,
																																									FONT_BEGIN, 347, 182,
																																									FONT_NEXT, 329, 270,
																																									FONT_NEXT, 338, 437,
																																									FONT_NEXT, 325, 307,
																																									FONT_NEXT, 287, 454,
																																									FONT_NEXT, 310, 361,
																																									FONT_END, 279, 410,
																																									FONT_BEGIN, 287, 454,
																																									FONT_NEXT, 279, 410,
																																									FONT_NEXT, 241, 460,
																																									FONT_NEXT, 256, 426,
																																									FONT_END, 227, 432,
																																									FONT_BEGIN, 241, 460,
																																									FONT_NEXT, 227, 432,
																																									FONT_NEXT, 182, 450,
																																									FONT_NEXT, 200, 427,
																																									FONT_END, 176, 412,
																																									FONT_BEGIN, 182, 450,
																																									FONT_NEXT, 176, 412,
																																									FONT_NEXT, 126, 421,
																																									FONT_NEXT, 158, 384,
																																									FONT_END, 152, 342,
																																									FONT_BEGIN, 28, -121,
																																									FONT_NEXT, 36, -89,
																																									FONT_NEXT, 41, -159,
																																									FONT_NEXT, 57, -57,
																																									FONT_NEXT, 78, -190,
																																									FONT_NEXT, 98, -90,
																																									FONT_END, 105, -115,
																																									FONT_BEGIN, 78, -190,
																																									FONT_NEXT, 105, -115,
																																									FONT_END, 130, -138,
																																									FONT_BEGIN, 78, -190,
																																									FONT_NEXT, 130, -138,
																																									FONT_NEXT, 134, -211,
																																									FONT_NEXT, 176, -155,
																																									FONT_NEXT, 205, -218,
																																									FONT_NEXT, 248, -161,
																																									FONT_NEXT, 282, -208,
																																									FONT_NEXT, 326, -154,
																																									FONT_NEXT, 366, -176,
																																									FONT_NEXT, 384, -133,
																																									FONT_NEXT, 403, -153,
																																									FONT_NEXT, 420, -102,
																																									FONT_NEXT, 433, -124,
																																									FONT_NEXT, 433, -64,
																																									FONT_NEXT, 440, 13,
																																									FONT_END, 404, 43,
																																									FONT_BEGIN, 404, 43,
																																									FONT_NEXT, 433, -64,
																																									FONT_END, 424, -38,
																																									FONT_BEGIN, 404, 43,
																																									FONT_NEXT, 424, -38,
																																									FONT_END, 399, -24,
																																									FONT_BEGIN, 404, 43,
																																									FONT_NEXT, 399, -24,
																																									FONT_NEXT, 375, 53,
																																									FONT_NEXT, 360, -18,
																																									FONT_NEXT, 340, 58,
																																									FONT_NEXT, 310, -15,
																																									FONT_NEXT, 211, 64,
																																									FONT_NEXT, 251, -14,
																																									FONT_END, 210, -12,
																																									FONT_BEGIN, 211, 64,
																																									FONT_NEXT, 210, -12,
																																									FONT_NEXT, 160, 69,
																																									FONT_NEXT, 147, -2,
																																									FONT_NEXT, 140, 77,
																																									FONT_NEXT, 126, 1,
																																									FONT_END, 133, 91,
																																									FONT_BEGIN, 147, -2,
																																									FONT_NEXT, 110, -49,
																																									FONT_NEXT, 126, 1,
																																									FONT_END, 57, -57,
																																									FONT_BEGIN, 57, -57,
																																									FONT_NEXT, 110, -49,
																																									FONT_END, 98, -90,
																																									FONT_BEGIN, 57, -57,
																																									FONT_END, 98, -90,
																																									FONT_BEGIN, 461, -49,
																																									FONT_NEXT, 453, -89,
																																									FONT_NEXT, 457, -22,
																																									FONT_END, 440, 13,
																																									FONT_BEGIN, 440, 13,
																																									FONT_NEXT, 453, -89,
																																									FONT_END, 433, -124,
																																									FONT_BEGIN, 440, 13,
																																									FONT_END, 433, -124,
																																									FONT_BEGIN, 470, 427,
																																									FONT_NEXT, 470, 388,
																																									FONT_NEXT, 393, 427,
																																									FONT_NEXT, 387, 388,
																																									FONT_NEXT, 338, 437,
																																									FONT_END, 347, 182,
																																									FONT_BEGIN, 347, 182,
																																									FONT_NEXT, 387, 388,
																																									FONT_NEXT, 389, 227,
																																									FONT_NEXT, 400, 352,
																																									FONT_NEXT, 401, 258,
																																									FONT_NEXT, 404, 328,
																																									FONT_END, 406, 296,
																																									FONT_ADVANCE, 500, 0
																																							},
																																								{
																																									104,
																																										FONT_BEGIN, 157, 680,
																																										FONT_NEXT, 157, 378,
																																										FONT_NEXT, 152, 683,
																																										FONT_NEXT, 157, 343,
																																										FONT_END, 157, 102,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 157, 102,
																																										FONT_END, 70, 58,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 70, 58,
																																										FONT_END, 73, 102,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 73, 102,
																																										FONT_END, 73, 573,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 73, 573,
																																										FONT_END, 67, 609,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 67, 609,
																																										FONT_END, 54, 620,
																																										FONT_BEGIN, 152, 683,
																																										FONT_NEXT, 54, 620,
																																										FONT_NEXT, 10, 639,
																																										FONT_NEXT, 29, 624,
																																										FONT_END, 10, 623,
																																										FONT_BEGIN, 427, 301,
																																										FONT_NEXT, 427, 102,
																																										FONT_NEXT, 424, 342,
																																										FONT_NEXT, 339, 56,
																																										FONT_NEXT, 411, 395,
																																										FONT_END, 396, 420,
																																										FONT_BEGIN, 396, 420,
																																										FONT_NEXT, 339, 56,
																																										FONT_NEXT, 374, 440,
																																										FONT_NEXT, 343, 102,
																																										FONT_NEXT, 343, 454,
																																										FONT_NEXT, 343, 300,
																																										FONT_END, 338, 344,
																																										FONT_BEGIN, 343, 454,
																																										FONT_NEXT, 338, 344,
																																										FONT_NEXT, 303, 460,
																																										FONT_NEXT, 325, 378,
																																										FONT_END, 301, 398,
																																										FONT_BEGIN, 303, 460,
																																										FONT_NEXT, 301, 398,
																																										FONT_NEXT, 260, 454,
																																										FONT_NEXT, 266, 406,
																																										FONT_END, 216, 392,
																																										FONT_BEGIN, 260, 454,
																																										FONT_NEXT, 216, 392,
																																										FONT_NEXT, 221, 437,
																																										FONT_END, 187, 411,
																																										FONT_BEGIN, 187, 411,
																																										FONT_NEXT, 216, 392,
																																										FONT_END, 187, 373,
																																										FONT_BEGIN, 187, 411,
																																										FONT_NEXT, 187, 373,
																																										FONT_NEXT, 159, 378,
																																										FONT_NEXT, 157, 343,
																																										FONT_END, 157, 378,
																																										FONT_BEGIN, 487, 15,
																																										FONT_NEXT, 487, 0,
																																										FONT_NEXT, 456, 22,
																																										FONT_NEXT, 275, 0,
																																										FONT_NEXT, 438, 34,
																																										FONT_END, 429, 58,
																																										FONT_BEGIN, 429, 58,
																																										FONT_NEXT, 275, 0,
																																										FONT_END, 327, 31,
																																										FONT_BEGIN, 429, 58,
																																										FONT_NEXT, 327, 31,
																																										FONT_NEXT, 427, 102,
																																										FONT_END, 339, 56,
																																										FONT_BEGIN, 275, 15,
																																										FONT_NEXT, 306, 19,
																																										FONT_NEXT, 275, 0,
																																										FONT_END, 327, 31,
																																										FONT_BEGIN, 225, 15,
																																										FONT_NEXT, 225, 0,
																																										FONT_NEXT, 193, 19,
																																										FONT_NEXT, 9, 0,
																																										FONT_NEXT, 172, 31,
																																										FONT_END, 160, 56,
																																										FONT_BEGIN, 160, 56,
																																										FONT_NEXT, 9, 0,
																																										FONT_END, 61, 33,
																																										FONT_BEGIN, 160, 56,
																																										FONT_NEXT, 61, 33,
																																										FONT_NEXT, 157, 102,
																																										FONT_END, 70, 58,
																																										FONT_BEGIN, 9, 15,
																																										FONT_NEXT, 41, 21,
																																										FONT_NEXT, 9, 0,
																																										FONT_END, 61, 33,
																																										FONT_ADVANCE, 500, 0
																																								},
																																								{
																																									105,
																																										FONT_BEGIN, 179, 457,
																																										FONT_NEXT, 179, 102,
																																										FONT_NEXT, 175, 460,
																																										FONT_NEXT, 91, 55,
																																										FONT_END, 95, 102,
																																										FONT_BEGIN, 175, 460,
																																										FONT_NEXT, 95, 102,
																																										FONT_END, 95, 334,
																																										FONT_BEGIN, 175, 460,
																																										FONT_NEXT, 95, 334,
																																										FONT_END, 91, 368,
																																										FONT_BEGIN, 175, 460,
																																										FONT_NEXT, 91, 368,
																																										FONT_END, 83, 386,
																																										FONT_BEGIN, 175, 460,
																																										FONT_NEXT, 83, 386,
																																										FONT_END, 60, 394,
																																										FONT_BEGIN, 175, 460,
																																										FONT_NEXT, 60, 394,
																																										FONT_NEXT, 20, 405,
																																										FONT_END, 20, 390,
																																										FONT_BEGIN, 16, 15,
																																										FONT_NEXT, 54, 18,
																																										FONT_NEXT, 16, 0,
																																										FONT_END, 180, 61,
																																										FONT_BEGIN, 180, 61,
																																										FONT_NEXT, 54, 18,
																																										FONT_END, 78, 30,
																																										FONT_BEGIN, 180, 61,
																																										FONT_NEXT, 78, 30,
																																										FONT_NEXT, 179, 102,
																																										FONT_END, 91, 55,
																																										FONT_BEGIN, 253, 15,
																																										FONT_NEXT, 253, 0,
																																										FONT_NEXT, 212, 21,
																																										FONT_NEXT, 16, 0,
																																										FONT_NEXT, 190, 35,
																																										FONT_END, 180, 61,
																																										FONT_BEGIN, 180, 632,
																																										FONT_NEXT, 176, 611,
																																										FONT_NEXT, 165, 668,
																																										FONT_NEXT, 165, 595,
																																										FONT_NEXT, 148, 678,
																																										FONT_NEXT, 148, 584,
																																										FONT_NEXT, 128, 683,
																																										FONT_NEXT, 128, 581,
																																										FONT_NEXT, 105, 677,
																																										FONT_NEXT, 105, 586,
																																										FONT_NEXT, 89, 664,
																																										FONT_NEXT, 89, 599,
																																										FONT_END, 78, 632,
																																										FONT_ADVANCE, 278, 0
																																								},
																																									{
																																										106,
																																											FONT_BEGIN, 32, 390,
																																											FONT_NEXT, 32, 406,
																																											FONT_NEXT, 74, 394,
																																											FONT_NEXT, 188, 460,
																																											FONT_NEXT, 97, 386,
																																											FONT_END, 105, 368,
																																											FONT_BEGIN, 105, 368,
																																											FONT_NEXT, 188, 460,
																																											FONT_END, 184, -84,
																																											FONT_BEGIN, 105, 368,
																																											FONT_NEXT, 184, -84,
																																											FONT_NEXT, 109, 334,
																																											FONT_NEXT, 156, -153,
																																											FONT_END, 133, -180,
																																											FONT_BEGIN, 109, 334,
																																											FONT_NEXT, 133, -180,
																																											FONT_NEXT, 109, -45,
																																											FONT_END, 108, -102,
																																											FONT_BEGIN, 108, -102,
																																											FONT_NEXT, 133, -180,
																																											FONT_END, 103, -201,
																																											FONT_BEGIN, 108, -102,
																																											FONT_NEXT, 103, -201,
																																											FONT_NEXT, 102, -146,
																																											FONT_NEXT, 67, -214,
																																											FONT_NEXT, 87, -174,
																																											FONT_END, 59, -184,
																																											FONT_BEGIN, 59, -184,
																																											FONT_NEXT, 67, -214,
																																											FONT_END, 23, -218,
																																											FONT_BEGIN, 59, -184,
																																											FONT_NEXT, 23, -218,
																																											FONT_NEXT, -30, -124,
																																											FONT_NEXT, -12, -215,
																																											FONT_END, -42, -204,
																																											FONT_BEGIN, -30, -124,
																																											FONT_NEXT, -42, -204,
																																											FONT_NEXT, -58, -135,
																																											FONT_NEXT, -63, -187,
																																											FONT_END, -70, -162,
																																											FONT_BEGIN, 193, 457,
																																											FONT_NEXT, 193, 0,
																																											FONT_NEXT, 188, 460,
																																											FONT_END, 184, -84,
																																											FONT_BEGIN, 194, 632,
																																											FONT_NEXT, 190, 611,
																																											FONT_NEXT, 179, 668,
																																											FONT_NEXT, 179, 595,
																																											FONT_NEXT, 162, 678,
																																											FONT_NEXT, 162, 584,
																																											FONT_NEXT, 142, 683,
																																											FONT_NEXT, 142, 581,
																																											FONT_NEXT, 119, 677,
																																											FONT_NEXT, 119, 586,
																																											FONT_NEXT, 103, 664,
																																											FONT_NEXT, 103, 599,
																																											FONT_END, 92, 632,
																																											FONT_ADVANCE, 278, 0
																																									},
																																									{
																																										107,
																																											FONT_BEGIN, 166, 681,
																																											FONT_NEXT, 166, 265,
																																											FONT_NEXT, 162, 683,
																																											FONT_NEXT, 166, 248,
																																											FONT_END, 166, 67,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 166, 67,
																																											FONT_END, 79, 50,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 79, 50,
																																											FONT_END, 82, 82,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 82, 82,
																																											FONT_END, 82, 564,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 82, 564,
																																											FONT_END, 80, 592,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 80, 592,
																																											FONT_END, 74, 611,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 74, 611,
																																											FONT_END, 60, 621,
																																											FONT_BEGIN, 162, 683,
																																											FONT_NEXT, 60, 621,
																																											FONT_NEXT, 7, 639,
																																											FONT_NEXT, 37, 625,
																																											FONT_END, 7, 623,
																																											FONT_BEGIN, 276, 436,
																																											FONT_NEXT, 276, 450,
																																											FONT_NEXT, 301, 434,
																																											FONT_END, 316, 430,
																																											FONT_BEGIN, 316, 430,
																																											FONT_NEXT, 276, 450,
																																											FONT_END, 377, 404,
																																											FONT_BEGIN, 316, 430,
																																											FONT_NEXT, 377, 404,
																																											FONT_NEXT, 326, 417,
																																											FONT_NEXT, 347, 381,
																																											FONT_END, 309, 350,
																																											FONT_BEGIN, 326, 417,
																																											FONT_NEXT, 309, 350,
																																											FONT_NEXT, 320, 401,
																																											FONT_END, 303, 383,
																																											FONT_BEGIN, 303, 383,
																																											FONT_NEXT, 309, 350,
																																											FONT_END, 264, 309,
																																											FONT_BEGIN, 303, 383,
																																											FONT_NEXT, 264, 309,
																																											FONT_NEXT, 168, 263,
																																											FONT_NEXT, 235, 282,
																																											FONT_END, 168, 248,
																																											FONT_BEGIN, 168, 263,
																																											FONT_NEXT, 168, 248,
																																											FONT_NEXT, 166, 265,
																																											FONT_END, 166, 248,
																																											FONT_BEGIN, 480, 450,
																																											FONT_NEXT, 480, 435,
																																											FONT_NEXT, 276, 450,
																																											FONT_NEXT, 442, 433,
																																											FONT_END, 402, 419,
																																											FONT_BEGIN, 276, 450,
																																											FONT_NEXT, 402, 419,
																																											FONT_END, 377, 404,
																																											FONT_BEGIN, 505, 15,
																																											FONT_NEXT, 505, 0,
																																											FONT_NEXT, 482, 18,
																																											FONT_NEXT, 287, 0,
																																											FONT_NEXT, 455, 28,
																																											FONT_END, 424, 49,
																																											FONT_BEGIN, 424, 49,
																																											FONT_NEXT, 287, 0,
																																											FONT_END, 319, 18,
																																											FONT_BEGIN, 424, 49,
																																											FONT_NEXT, 319, 18,
																																											FONT_NEXT, 388, 88,
																																											FONT_NEXT, 327, 31,
																																											FONT_NEXT, 235, 282,
																																											FONT_NEXT, 321, 43,
																																											FONT_END, 306, 64,
																																											FONT_BEGIN, 235, 282,
																																											FONT_NEXT, 306, 64,
																																											FONT_END, 168, 248,
																																											FONT_BEGIN, 287, 15,
																																											FONT_NEXT, 306, 15,
																																											FONT_NEXT, 287, 0,
																																											FONT_END, 319, 18,
																																											FONT_BEGIN, 241, 15,
																																											FONT_NEXT, 241, 0,
																																											FONT_NEXT, 221, 16,
																																											FONT_NEXT, 7, 0,
																																											FONT_NEXT, 181, 26,
																																											FONT_END, 169, 42,
																																											FONT_BEGIN, 169, 42,
																																											FONT_NEXT, 7, 0,
																																											FONT_NEXT, 166, 67,
																																											FONT_NEXT, 69, 32,
																																											FONT_END, 79, 50,
																																											FONT_BEGIN, 7, 15,
																																											FONT_NEXT, 46, 22,
																																											FONT_NEXT, 7, 0,
																																											FONT_END, 69, 32,
																																											FONT_ADVANCE, 500, 0
																																									},
																																										{
																																											108,
																																												FONT_BEGIN, 182, 681,
																																												FONT_NEXT, 182, 84,
																																												FONT_NEXT, 178, 683,
																																												FONT_NEXT, 94, 53,
																																												FONT_END, 98, 87,
																																												FONT_BEGIN, 178, 683,
																																												FONT_NEXT, 98, 87,
																																												FONT_END, 98, 564,
																																												FONT_BEGIN, 178, 683,
																																												FONT_NEXT, 98, 564,
																																												FONT_END, 96, 592,
																																												FONT_BEGIN, 178, 683,
																																												FONT_NEXT, 96, 592,
																																												FONT_END, 90, 611,
																																												FONT_BEGIN, 178, 683,
																																												FONT_NEXT, 90, 611,
																																												FONT_END, 76, 621,
																																												FONT_BEGIN, 178, 683,
																																												FONT_NEXT, 76, 621,
																																												FONT_NEXT, 19, 639,
																																												FONT_NEXT, 53, 625,
																																												FONT_END, 19, 623,
																																												FONT_BEGIN, 257, 15,
																																												FONT_NEXT, 257, 0,
																																												FONT_NEXT, 220, 18,
																																												FONT_NEXT, 21, 0,
																																												FONT_NEXT, 197, 28,
																																												FONT_END, 185, 49,
																																												FONT_BEGIN, 185, 49,
																																												FONT_NEXT, 21, 0,
																																												FONT_END, 81, 31,
																																												FONT_BEGIN, 185, 49,
																																												FONT_NEXT, 81, 31,
																																												FONT_NEXT, 182, 84,
																																												FONT_END, 94, 53,
																																												FONT_BEGIN, 21, 15,
																																												FONT_NEXT, 57, 20,
																																												FONT_NEXT, 21, 0,
																																												FONT_END, 81, 31,
																																												FONT_ADVANCE, 278, 0
																																										},
																																										{
																																											109,
																																												FONT_BEGIN, 166, 458,
																																												FONT_NEXT, 166, 383,
																																												FONT_NEXT, 159, 460,
																																												FONT_NEXT, 80, 45,
																																												FONT_END, 86, 85,
																																												FONT_BEGIN, 159, 460,
																																												FONT_NEXT, 86, 85,
																																												FONT_END, 86, 338,
																																												FONT_BEGIN, 159, 460,
																																												FONT_NEXT, 86, 338,
																																												FONT_END, 82, 374,
																																												FONT_BEGIN, 159, 460,
																																												FONT_NEXT, 82, 374,
																																												FONT_END, 74, 393,
																																												FONT_BEGIN, 159, 460,
																																												FONT_NEXT, 74, 393,
																																												FONT_NEXT, 19, 415,
																																												FONT_NEXT, 51, 402,
																																												FONT_END, 19, 398,
																																												FONT_BEGIN, 706, 282,
																																												FONT_NEXT, 706, 76,
																																												FONT_NEXT, 702, 341,
																																												FONT_NEXT, 618, 49,
																																												FONT_NEXT, 686, 398,
																																												FONT_NEXT, 622, 87,
																																												FONT_NEXT, 650, 442,
																																												FONT_END, 622, 455,
																																												FONT_BEGIN, 622, 455,
																																												FONT_NEXT, 622, 87,
																																												FONT_END, 622, 298,
																																												FONT_BEGIN, 622, 455,
																																												FONT_NEXT, 622, 298,
																																												FONT_END, 612, 372,
																																												FONT_BEGIN, 622, 455,
																																												FONT_NEXT, 612, 372,
																																												FONT_NEXT, 588, 460,
																																												FONT_NEXT, 590, 397,
																																												FONT_END, 549, 408,
																																												FONT_BEGIN, 588, 460,
																																												FONT_NEXT, 549, 408,
																																												FONT_NEXT, 532, 450,
																																												FONT_NEXT, 511, 403,
																																												FONT_NEXT, 485, 427,
																																												FONT_NEXT, 481, 390,
																																												FONT_NEXT, 449, 399,
																																												FONT_NEXT, 438, 347,
																																												FONT_NEXT, 427, 376,
																																												FONT_NEXT, 438, 95,
																																												FONT_END, 349, 47,
																																												FONT_BEGIN, 427, 376,
																																												FONT_NEXT, 349, 47,
																																												FONT_NEXT, 414, 406,
																																												FONT_NEXT, 354, 86,
																																												FONT_NEXT, 393, 433,
																																												FONT_END, 362, 452,
																																												FONT_BEGIN, 362, 452,
																																												FONT_NEXT, 354, 86,
																																												FONT_END, 354, 303,
																																												FONT_BEGIN, 362, 452,
																																												FONT_NEXT, 354, 303,
																																												FONT_END, 350, 347,
																																												FONT_BEGIN, 362, 452,
																																												FONT_NEXT, 350, 347,
																																												FONT_NEXT, 320, 460,
																																												FONT_NEXT, 339, 380,
																																												FONT_END, 317, 400,
																																												FONT_BEGIN, 320, 460,
																																												FONT_NEXT, 317, 400,
																																												FONT_NEXT, 281, 454,
																																												FONT_NEXT, 285, 408,
																																												FONT_END, 237, 399,
																																												FONT_BEGIN, 281, 454,
																																												FONT_NEXT, 237, 399,
																																												FONT_NEXT, 244, 438,
																																												FONT_END, 166, 383,
																																												FONT_BEGIN, 166, 383,
																																												FONT_NEXT, 237, 399,
																																												FONT_END, 201, 380,
																																												FONT_BEGIN, 166, 383,
																																												FONT_NEXT, 201, 380,
																																												FONT_END, 178, 360,
																																												FONT_BEGIN, 166, 383,
																																												FONT_NEXT, 178, 360,
																																												FONT_END, 170, 349,
																																												FONT_BEGIN, 166, 383,
																																												FONT_NEXT, 170, 349,
																																												FONT_END, 170, 67,
																																												FONT_BEGIN, 166, 383,
																																												FONT_NEXT, 170, 67,
																																												FONT_NEXT, 80, 45,
																																												FONT_END, 66, 25,
																																												FONT_BEGIN, 66, 25,
																																												FONT_NEXT, 170, 67,
																																												FONT_END, 16, 0,
																																												FONT_BEGIN, 66, 25,
																																												FONT_NEXT, 16, 0,
																																												FONT_NEXT, 43, 16,
																																												FONT_END, 16, 15,
																																												FONT_BEGIN, 775, 15,
																																												FONT_NEXT, 775, 0,
																																												FONT_NEXT, 749, 17,
																																												FONT_NEXT, 556, 0,
																																												FONT_NEXT, 723, 26,
																																												FONT_END, 711, 40,
																																												FONT_BEGIN, 711, 40,
																																												FONT_NEXT, 556, 0,
																																												FONT_NEXT, 706, 76,
																																												FONT_NEXT, 607, 29,
																																												FONT_END, 618, 49,
																																												FONT_BEGIN, 556, 15,
																																												FONT_NEXT, 587, 19,
																																												FONT_NEXT, 556, 0,
																																												FONT_END, 607, 29,
																																												FONT_BEGIN, 510, 15,
																																												FONT_NEXT, 510, 0,
																																												FONT_NEXT, 481, 17,
																																												FONT_NEXT, 286, 0,
																																												FONT_NEXT, 458, 26,
																																												FONT_END, 443, 50,
																																												FONT_BEGIN, 443, 50,
																																												FONT_NEXT, 286, 0,
																																												FONT_END, 336, 27,
																																												FONT_BEGIN, 443, 50,
																																												FONT_NEXT, 336, 27,
																																												FONT_NEXT, 438, 95,
																																												FONT_END, 349, 47,
																																												FONT_BEGIN, 286, 15,
																																												FONT_NEXT, 315, 18,
																																												FONT_NEXT, 286, 0,
																																												FONT_END, 336, 27,
																																												FONT_BEGIN, 238, 15,
																																												FONT_NEXT, 238, 0,
																																												FONT_NEXT, 190, 23,
																																												FONT_NEXT, 16, 0,
																																												FONT_NEXT, 175, 38,
																																												FONT_END, 170, 67,
																																												FONT_ADVANCE, 778, 0
																																										},
																																											{
																																												110,
																																													FONT_BEGIN, 161, 458,
																																													FONT_NEXT, 161, 379,
																																													FONT_NEXT, 154, 460,
																																													FONT_NEXT, 76, 53,
																																													FONT_END, 80, 90,
																																													FONT_BEGIN, 154, 460,
																																													FONT_NEXT, 80, 90,
																																													FONT_END, 80, 338,
																																													FONT_BEGIN, 154, 460,
																																													FONT_NEXT, 80, 338,
																																													FONT_END, 76, 374,
																																													FONT_BEGIN, 154, 460,
																																													FONT_NEXT, 76, 374,
																																													FONT_END, 68, 393,
																																													FONT_BEGIN, 154, 460,
																																													FONT_NEXT, 68, 393,
																																													FONT_NEXT, 16, 415,
																																													FONT_NEXT, 45, 402,
																																													FONT_END, 16, 398,
																																													FONT_BEGIN, 424, 310,
																																													FONT_NEXT, 424, 81,
																																													FONT_NEXT, 420, 351,
																																													FONT_NEXT, 335, 54,
																																													FONT_NEXT, 412, 385,
																																													FONT_END, 382, 431,
																																													FONT_BEGIN, 382, 431,
																																													FONT_NEXT, 335, 54,
																																													FONT_END, 340, 99,
																																													FONT_BEGIN, 382, 431,
																																													FONT_NEXT, 340, 99,
																																													FONT_NEXT, 344, 453,
																																													FONT_NEXT, 340, 308,
																																													FONT_END, 336, 348,
																																													FONT_BEGIN, 344, 453,
																																													FONT_NEXT, 336, 348,
																																													FONT_NEXT, 307, 460,
																																													FONT_NEXT, 323, 379,
																																													FONT_END, 299, 398,
																																													FONT_BEGIN, 307, 460,
																																													FONT_NEXT, 299, 398,
																																													FONT_NEXT, 266, 453,
																																													FONT_NEXT, 263, 405,
																																													FONT_NEXT, 230, 436,
																																													FONT_NEXT, 216, 390,
																																													FONT_NEXT, 161, 379,
																																													FONT_NEXT, 164, 348,
																																													FONT_END, 164, 67,
																																													FONT_BEGIN, 161, 379,
																																													FONT_NEXT, 164, 67,
																																													FONT_NEXT, 76, 53,
																																													FONT_END, 66, 31,
																																													FONT_BEGIN, 66, 31,
																																													FONT_NEXT, 164, 67,
																																													FONT_END, 18, 0,
																																													FONT_BEGIN, 66, 31,
																																													FONT_NEXT, 18, 0,
																																													FONT_NEXT, 47, 19,
																																													FONT_END, 18, 15,
																																													FONT_BEGIN, 485, 15,
																																													FONT_NEXT, 485, 0,
																																													FONT_NEXT, 454, 20,
																																													FONT_NEXT, 277, 0,
																																													FONT_NEXT, 436, 31,
																																													FONT_END, 426, 51,
																																													FONT_BEGIN, 426, 51,
																																													FONT_NEXT, 277, 0,
																																													FONT_NEXT, 424, 81,
																																													FONT_NEXT, 322, 29,
																																													FONT_END, 335, 54,
																																													FONT_BEGIN, 277, 15,
																																													FONT_NEXT, 302, 18,
																																													FONT_NEXT, 277, 0,
																																													FONT_END, 322, 29,
																																													FONT_BEGIN, 230, 15,
																																													FONT_NEXT, 230, 0,
																																													FONT_NEXT, 197, 19,
																																													FONT_NEXT, 18, 0,
																																													FONT_NEXT, 177, 29,
																																													FONT_END, 167, 45,
																																													FONT_BEGIN, 167, 45,
																																													FONT_NEXT, 18, 0,
																																													FONT_END, 164, 67,
																																													FONT_BEGIN, 167, 45,
																																													FONT_END, 164, 67,
																																													FONT_ADVANCE, 500, 0
																																											},
																																											{
																																												111,
																																													FONT_BEGIN, 470, 228,
																																													FONT_NEXT, 456, 149,
																																													FONT_NEXT, 465, 279,
																																													FONT_END, 453, 325,
																																													FONT_BEGIN, 453, 325,
																																													FONT_NEXT, 456, 149,
																																													FONT_END, 416, 71,
																																													FONT_BEGIN, 453, 325,
																																													FONT_NEXT, 416, 71,
																																													FONT_NEXT, 433, 365,
																																													FONT_END, 406, 398,
																																													FONT_BEGIN, 406, 398,
																																													FONT_NEXT, 416, 71,
																																													FONT_END, 385, 39,
																																													FONT_BEGIN, 406, 398,
																																													FONT_NEXT, 385, 39,
																																													FONT_NEXT, 373, 424,
																																													FONT_NEXT, 380, 199,
																																													FONT_END, 370, 287,
																																													FONT_BEGIN, 373, 424,
																																													FONT_NEXT, 370, 287,
																																													FONT_NEXT, 335, 444,
																																													FONT_NEXT, 343, 362,
																																													FONT_END, 322, 390,
																																													FONT_BEGIN, 335, 444,
																																													FONT_NEXT, 322, 390,
																																													FONT_NEXT, 292, 455,
																																													FONT_NEXT, 298, 413,
																																													FONT_END, 269, 427,
																																													FONT_BEGIN, 292, 455,
																																													FONT_NEXT, 269, 427,
																																													FONT_NEXT, 245, 460,
																																													FONT_NEXT, 237, 432,
																																													FONT_NEXT, 197, 455,
																																													FONT_NEXT, 191, 422,
																																													FONT_NEXT, 154, 441,
																																													FONT_NEXT, 154, 393,
																																													FONT_NEXT, 117, 419,
																																													FONT_NEXT, 128, 344,
																																													FONT_END, 119, 276,
																																													FONT_BEGIN, 117, 419,
																																													FONT_NEXT, 119, 276,
																																													FONT_END, 90, 58,
																																													FONT_BEGIN, 117, 419,
																																													FONT_NEXT, 90, 58,
																																													FONT_NEXT, 86, 390,
																																													FONT_NEXT, 64, 93,
																																													FONT_NEXT, 62, 355,
																																													FONT_NEXT, 45, 133,
																																													FONT_NEXT, 43, 316,
																																													FONT_NEXT, 33, 179,
																																													FONT_END, 29, 228,
																																													FONT_BEGIN, 90, 58,
																																													FONT_NEXT, 119, 276,
																																													FONT_NEXT, 122, 29,
																																													FONT_NEXT, 124, 204,
																																													FONT_END, 145, 119,
																																													FONT_BEGIN, 122, 29,
																																													FONT_NEXT, 145, 119,
																																													FONT_NEXT, 159, 8,
																																													FONT_NEXT, 164, 80,
																																													FONT_END, 189, 47,
																																													FONT_BEGIN, 159, 8,
																																													FONT_NEXT, 189, 47,
																																													FONT_NEXT, 201, -6,
																																													FONT_NEXT, 221, 26,
																																													FONT_NEXT, 248, -10,
																																													FONT_NEXT, 262, 18,
																																													FONT_NEXT, 301, -4,
																																													FONT_NEXT, 301, 26,
																																													FONT_END, 340, 54,
																																													FONT_BEGIN, 301, -4,
																																													FONT_NEXT, 340, 54,
																																													FONT_NEXT, 346, 13,
																																													FONT_NEXT, 356, 78,
																																													FONT_END, 368, 110,
																																													FONT_BEGIN, 346, 13,
																																													FONT_NEXT, 368, 110,
																																													FONT_NEXT, 385, 39,
																																													FONT_NEXT, 377, 150,
																																													FONT_END, 380, 199,
																																													FONT_ADVANCE, 500, 0
																																											},
																																												{
																																													112,
																																														FONT_BEGIN, 159, 458,
																																														FONT_NEXT, 159, 385,
																																														FONT_NEXT, 153, 460,
																																														FONT_NEXT, 159, 334,
																																														FONT_END, 159, 88,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 159, 88,
																																														FONT_END, 159, 33,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 159, 33,
																																														FONT_END, 159, -124,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 159, -124,
																																														FONT_END, 73, -160,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 73, -160,
																																														FONT_END, 75, -131,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 75, -131,
																																														FONT_END, 75, 337,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 75, 337,
																																														FONT_END, 73, 362,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 73, 362,
																																														FONT_END, 69, 380,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 69, 380,
																																														FONT_END, 57, 390,
																																														FONT_BEGIN, 153, 460,
																																														FONT_NEXT, 57, 390,
																																														FONT_NEXT, 9, 409,
																																														FONT_NEXT, 37, 394,
																																														FONT_END, 9, 393,
																																														FONT_BEGIN, 470, 245,
																																														FONT_NEXT, 456, 156,
																																														FONT_NEXT, 466, 292,
																																														FONT_END, 456, 334,
																																														FONT_BEGIN, 456, 334,
																																														FONT_NEXT, 456, 156,
																																														FONT_NEXT, 420, 402,
																																														FONT_NEXT, 417, 74,
																																														FONT_NEXT, 368, 445,
																																														FONT_NEXT, 384, 215,
																																														FONT_END, 383, 229,
																																														FONT_BEGIN, 368, 445,
																																														FONT_NEXT, 383, 229,
																																														FONT_END, 381, 253,
																																														FONT_BEGIN, 368, 445,
																																														FONT_NEXT, 381, 253,
																																														FONT_END, 366, 316,
																																														FONT_BEGIN, 368, 445,
																																														FONT_NEXT, 366, 316,
																																														FONT_NEXT, 305, 460,
																																														FONT_NEXT, 350, 347,
																																														FONT_END, 328, 374,
																																														FONT_BEGIN, 305, 460,
																																														FONT_NEXT, 328, 374,
																																														FONT_END, 297, 392,
																																														FONT_BEGIN, 305, 460,
																																														FONT_NEXT, 297, 392,
																																														FONT_NEXT, 252, 451,
																																														FONT_NEXT, 257, 400,
																																														FONT_END, 227, 395,
																																														FONT_BEGIN, 252, 451,
																																														FONT_NEXT, 227, 395,
																																														FONT_NEXT, 212, 432,
																																														FONT_NEXT, 196, 381,
																																														FONT_NEXT, 181, 406,
																																														FONT_NEXT, 171, 360,
																																														FONT_NEXT, 161, 383,
																																														FONT_NEXT, 159, 334,
																																														FONT_END, 159, 385,
																																														FONT_BEGIN, 159, 88,
																																														FONT_NEXT, 167, 67,
																																														FONT_NEXT, 159, 33,
																																														FONT_NEXT, 189, 46,
																																														FONT_NEXT, 206, -1,
																																														FONT_NEXT, 221, 29,
																																														FONT_NEXT, 260, -10,
																																														FONT_NEXT, 261, 22,
																																														FONT_END, 294, 27,
																																														FONT_BEGIN, 260, -10,
																																														FONT_NEXT, 294, 27,
																																														FONT_NEXT, 308, -4,
																																														FONT_NEXT, 334, 53,
																																														FONT_NEXT, 351, 13,
																																														FONT_NEXT, 353, 77,
																																														FONT_END, 369, 111,
																																														FONT_BEGIN, 351, 13,
																																														FONT_NEXT, 369, 111,
																																														FONT_END, 380, 156,
																																														FONT_BEGIN, 351, 13,
																																														FONT_NEXT, 380, 156,
																																														FONT_NEXT, 387, 40,
																																														FONT_NEXT, 384, 215,
																																														FONT_END, 417, 74,
																																														FONT_BEGIN, 5, -200,
																																														FONT_NEXT, 43, -194,
																																														FONT_NEXT, 5, -217,
																																														FONT_END, 159, -124,
																																														FONT_BEGIN, 159, -124,
																																														FONT_NEXT, 43, -194,
																																														FONT_END, 64, -180,
																																														FONT_BEGIN, 159, -124,
																																														FONT_NEXT, 64, -180,
																																														FONT_END, 73, -160,
																																														FONT_BEGIN, 247, -199,
																																														FONT_NEXT, 247, -217,
																																														FONT_NEXT, 202, -197,
																																														FONT_NEXT, 5, -217,
																																														FONT_NEXT, 176, -186,
																																														FONT_END, 162, -163,
																																														FONT_BEGIN, 162, -163,
																																														FONT_NEXT, 5, -217,
																																														FONT_END, 159, -124,
																																														FONT_BEGIN, 162, -163,
																																														FONT_END, 159, -124,
																																														FONT_ADVANCE, 500, 0
																																												},
																																												{
																																													113,
																																														FONT_BEGIN, 110, 238,
																																														FONT_NEXT, 117, 169,
																																														FONT_NEXT, 72, 44,
																																														FONT_END, 124, 3,
																																														FONT_BEGIN, 124, 3,
																																														FONT_NEXT, 117, 169,
																																														FONT_END, 140, 110,
																																														FONT_BEGIN, 124, 3,
																																														FONT_NEXT, 140, 110,
																																														FONT_END, 180, 67,
																																														FONT_BEGIN, 124, 3,
																																														FONT_NEXT, 180, 67,
																																														FONT_NEXT, 188, -10,
																																														FONT_NEXT, 206, 55,
																																														FONT_NEXT, 234, -3,
																																														FONT_NEXT, 238, 51,
																																														FONT_NEXT, 278, 14,
																																														FONT_NEXT, 288, 60,
																																														FONT_NEXT, 314, 38,
																																														FONT_NEXT, 318, 73,
																																														FONT_END, 333, 88,
																																														FONT_BEGIN, 314, 38,
																																														FONT_NEXT, 333, 88,
																																														FONT_NEXT, 336, 62,
																																														FONT_END, 341, 62,
																																														FONT_BEGIN, 341, 62,
																																														FONT_NEXT, 333, 88,
																																														FONT_END, 341, 127,
																																														FONT_BEGIN, 341, 62,
																																														FONT_NEXT, 341, 127,
																																														FONT_NEXT, 341, -124,
																																														FONT_NEXT, 341, 333,
																																														FONT_NEXT, 360, 425,
																																														FONT_NEXT, 339, 360,
																																														FONT_END, 328, 393,
																																														FONT_BEGIN, 360, 425,
																																														FONT_NEXT, 328, 393,
																																														FONT_NEXT, 308, 450,
																																														FONT_NEXT, 300, 420,
																																														FONT_NEXT, 247, 460,
																																														FONT_NEXT, 276, 428,
																																														FONT_END, 245, 432,
																																														FONT_BEGIN, 247, 460,
																																														FONT_NEXT, 245, 432,
																																														FONT_NEXT, 201, 455,
																																														FONT_NEXT, 189, 418,
																																														FONT_NEXT, 159, 440,
																																														FONT_NEXT, 147, 380,
																																														FONT_NEXT, 121, 417,
																																														FONT_NEXT, 119, 319,
																																														FONT_NEXT, 88, 387,
																																														FONT_NEXT, 110, 238,
																																														FONT_END, 72, 44,
																																														FONT_BEGIN, 88, 387,
																																														FONT_NEXT, 72, 44,
																																														FONT_NEXT, 61, 351,
																																														FONT_NEXT, 52, 75,
																																														FONT_NEXT, 41, 308,
																																														FONT_NEXT, 37, 113,
																																														FONT_NEXT, 28, 262,
																																														FONT_NEXT, 27, 159,
																																														FONT_END, 24, 212,
																																														FONT_BEGIN, 425, 456,
																																														FONT_NEXT, 425, -141,
																																														FONT_NEXT, 421, 460,
																																														FONT_NEXT, 337, -157,
																																														FONT_NEXT, 414, 457,
																																														FONT_NEXT, 341, -124,
																																														FONT_END, 360, 425,
																																														FONT_BEGIN, 252, -200,
																																														FONT_NEXT, 297, -194,
																																														FONT_NEXT, 252, -217,
																																														FONT_END, 425, -141,
																																														FONT_BEGIN, 425, -141,
																																														FONT_NEXT, 297, -194,
																																														FONT_END, 324, -180,
																																														FONT_BEGIN, 425, -141,
																																														FONT_NEXT, 324, -180,
																																														FONT_END, 337, -157,
																																														FONT_BEGIN, 488, -203,
																																														FONT_NEXT, 488, -217,
																																														FONT_NEXT, 459, -196,
																																														FONT_NEXT, 252, -217,
																																														FONT_NEXT, 439, -187,
																																														FONT_END, 428, -170,
																																														FONT_BEGIN, 428, -170,
																																														FONT_NEXT, 252, -217,
																																														FONT_END, 425, -141,
																																														FONT_BEGIN, 428, -170,
																																														FONT_END, 425, -141,
																																														FONT_ADVANCE, 500, 0
																																												},
																																													{
																																														114,
																																															FONT_BEGIN, 160, 458,
																																															FONT_NEXT, 160, 369,
																																															FONT_NEXT, 155, 460,
																																															FONT_NEXT, 160, 315,
																																															FONT_END, 160, 90,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 160, 90,
																																															FONT_END, 72, 51,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 72, 51,
																																															FONT_END, 76, 84,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 76, 84,
																																															FONT_END, 76, 334,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 76, 334,
																																															FONT_END, 72, 368,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 72, 368,
																																															FONT_END, 64, 386,
																																															FONT_BEGIN, 155, 460,
																																															FONT_NEXT, 64, 386,
																																															FONT_NEXT, 7, 406,
																																															FONT_NEXT, 41, 394,
																																															FONT_END, 7, 390,
																																															FONT_BEGIN, 335, 406,
																																															FONT_NEXT, 325, 374,
																																															FONT_NEXT, 331, 428,
																																															FONT_END, 320, 445,
																																															FONT_BEGIN, 320, 445,
																																															FONT_NEXT, 325, 374,
																																															FONT_END, 297, 362,
																																															FONT_BEGIN, 320, 445,
																																															FONT_NEXT, 297, 362,
																																															FONT_NEXT, 303, 456,
																																															FONT_END, 281, 460,
																																															FONT_BEGIN, 281, 460,
																																															FONT_NEXT, 297, 362,
																																															FONT_END, 274, 367,
																																															FONT_BEGIN, 281, 460,
																																															FONT_NEXT, 274, 367,
																																															FONT_NEXT, 252, 455,
																																															FONT_NEXT, 258, 379,
																																															FONT_END, 228, 397,
																																															FONT_BEGIN, 252, 455,
																																															FONT_NEXT, 228, 397,
																																															FONT_NEXT, 224, 441,
																																															FONT_NEXT, 207, 388,
																																															FONT_NEXT, 195, 413,
																																															FONT_NEXT, 185, 368,
																																															FONT_NEXT, 162, 369,
																																															FONT_NEXT, 167, 341,
																																															FONT_END, 160, 315,
																																															FONT_BEGIN, 162, 369,
																																															FONT_NEXT, 160, 315,
																																															FONT_END, 160, 369,
																																															FONT_BEGIN, 245, 15,
																																															FONT_NEXT, 245, 0,
																																															FONT_NEXT, 210, 17,
																																															FONT_NEXT, 5, 0,
																																															FONT_NEXT, 183, 27,
																																															FONT_END, 166, 49,
																																															FONT_BEGIN, 166, 49,
																																															FONT_NEXT, 5, 0,
																																															FONT_END, 61, 32,
																																															FONT_BEGIN, 166, 49,
																																															FONT_NEXT, 61, 32,
																																															FONT_NEXT, 160, 90,
																																															FONT_END, 72, 51,
																																															FONT_BEGIN, 5, 15,
																																															FONT_NEXT, 39, 22,
																																															FONT_NEXT, 5, 0,
																																															FONT_END, 61, 32,
																																															FONT_ADVANCE, 333, 0
																																													},
																																													{
																																														115,
																																															FONT_BEGIN, 51, 338,
																																															FONT_NEXT, 59, 380,
																																															FONT_NEXT, 60, 288,
																																															FONT_NEXT, 84, 420,
																																															FONT_NEXT, 86, 250,
																																															FONT_NEXT, 113, 372,
																																															FONT_END, 120, 337,
																																															FONT_BEGIN, 86, 250,
																																															FONT_NEXT, 120, 337,
																																															FONT_NEXT, 123, 219,
																																															FONT_NEXT, 133, 318,
																																															FONT_END, 156, 301,
																																															FONT_BEGIN, 123, 219,
																																															FONT_NEXT, 156, 301,
																																															FONT_NEXT, 168, 192,
																																															FONT_END, 226, 159,
																																															FONT_BEGIN, 226, 159,
																																															FONT_NEXT, 156, 301,
																																															FONT_NEXT, 264, 127,
																																															FONT_NEXT, 264, 237,
																																															FONT_NEXT, 278, 85,
																																															FONT_NEXT, 299, 21,
																																															FONT_NEXT, 270, 49,
																																															FONT_NEXT, 252, -3,
																																															FONT_NEXT, 250, 27,
																																															FONT_NEXT, 203, -10,
																																															FONT_NEXT, 224, 15,
																																															FONT_END, 197, 12,
																																															FONT_BEGIN, 197, 12,
																																															FONT_NEXT, 203, -10,
																																															FONT_END, 138, -1,
																																															FONT_BEGIN, 197, 12,
																																															FONT_NEXT, 138, -1,
																																															FONT_NEXT, 163, 15,
																																															FONT_END, 137, 26,
																																															FONT_BEGIN, 137, 26,
																																															FONT_NEXT, 138, -1,
																																															FONT_END, 87, 8,
																																															FONT_BEGIN, 137, 26,
																																															FONT_NEXT, 87, 8,
																																															FONT_NEXT, 100, 62,
																																															FONT_END, 79, 107,
																																															FONT_BEGIN, 79, 107,
																																															FONT_NEXT, 87, 8,
																																															FONT_END, 73, 5,
																																															FONT_BEGIN, 79, 107,
																																															FONT_NEXT, 73, 5,
																																															FONT_NEXT, 68, 152,
																																															FONT_NEXT, 65, -4,
																																															FONT_NEXT, 52, 152,
																																															FONT_END, 52, -4,
																																															FONT_BEGIN, 315, 314,
																																															FONT_NEXT, 300, 314,
																																															FONT_NEXT, 311, 451,
																																															FONT_END, 300, 451,
																																															FONT_BEGIN, 300, 451,
																																															FONT_NEXT, 300, 314,
																																															FONT_NEXT, 283, 440,
																																															FONT_NEXT, 279, 376,
																																															FONT_NEXT, 247, 450,
																																															FONT_NEXT, 252, 413,
																																															FONT_END, 221, 432,
																																															FONT_BEGIN, 247, 450,
																																															FONT_NEXT, 221, 432,
																																															FONT_NEXT, 191, 460,
																																															FONT_NEXT, 189, 437,
																																															FONT_NEXT, 128, 448,
																																															FONT_NEXT, 153, 430,
																																															FONT_END, 130, 414,
																																															FONT_BEGIN, 128, 448,
																																															FONT_NEXT, 130, 414,
																																															FONT_END, 117, 393,
																																															FONT_BEGIN, 128, 448,
																																															FONT_NEXT, 117, 393,
																																															FONT_NEXT, 84, 420,
																																															FONT_END, 113, 372,
																																															FONT_BEGIN, 348, 119,
																																															FONT_NEXT, 334, 62,
																																															FONT_NEXT, 341, 158,
																																															FONT_END, 324, 189,
																																															FONT_BEGIN, 324, 189,
																																															FONT_NEXT, 334, 62,
																																															FONT_END, 299, 21,
																																															FONT_BEGIN, 324, 189,
																																															FONT_NEXT, 299, 21,
																																															FONT_END, 264, 237,
																																															FONT_ADVANCE, 389, 0
																																													},
																																														{
																																															116,
																																																FONT_BEGIN, 13, 425,
																																																FONT_NEXT, 26, 441,
																																																FONT_NEXT, 17, 418,
																																																FONT_NEXT, 57, 465,
																																																FONT_NEXT, 70, 418,
																																																FONT_NEXT, 102, 516,
																																																FONT_END, 96, 16,
																																																FONT_BEGIN, 70, 418,
																																																FONT_NEXT, 96, 16,
																																																FONT_END, 77, 54,
																																																FONT_BEGIN, 70, 418,
																																																FONT_NEXT, 77, 54,
																																																FONT_END, 70, 117,
																																																FONT_BEGIN, 154, 566,
																																																FONT_NEXT, 154, 450,
																																																FONT_NEXT, 147, 579,
																																																FONT_NEXT, 154, 418,
																																																FONT_END, 154, 132,
																																																FONT_BEGIN, 147, 579,
																																																FONT_NEXT, 154, 132,
																																																FONT_END, 126, -5,
																																																FONT_BEGIN, 147, 579,
																																																FONT_NEXT, 126, -5,
																																																FONT_NEXT, 122, 544,
																																																FONT_NEXT, 96, 16,
																																																FONT_END, 102, 516,
																																																FONT_BEGIN, 255, 450,
																																																FONT_NEXT, 255, 418,
																																																FONT_NEXT, 154, 450,
																																																FONT_END, 154, 418,
																																																FONT_BEGIN, 279, 66,
																																																FONT_NEXT, 257, 35,
																																																FONT_NEXT, 266, 77,
																																																FONT_END, 243, 54,
																																																FONT_BEGIN, 243, 54,
																																																FONT_NEXT, 257, 35,
																																																FONT_END, 228, 11,
																																																FONT_BEGIN, 243, 54,
																																																FONT_NEXT, 228, 11,
																																																FONT_NEXT, 205, 42,
																																																FONT_NEXT, 196, -5,
																																																FONT_NEXT, 175, 51,
																																																FONT_NEXT, 162, -10,
																																																FONT_NEXT, 160, 75,
																																																FONT_NEXT, 126, -5,
																																																FONT_END, 154, 132,
																																																FONT_ADVANCE, 278, 0
																																														},
																																														{
																																															117,
																																																FONT_BEGIN, 9, 436,
																																																FONT_NEXT, 9, 450,
																																																FONT_NEXT, 51, 425,
																																																FONT_NEXT, 155, 450,
																																																FONT_NEXT, 65, 407,
																																																FONT_NEXT, 155, 124,
																																																FONT_END, 154, -5,
																																																FONT_BEGIN, 65, 407,
																																																FONT_NEXT, 154, -5,
																																																FONT_NEXT, 71, 372,
																																																FONT_NEXT, 115, 15,
																																																FONT_END, 83, 54,
																																																FONT_BEGIN, 71, 372,
																																																FONT_NEXT, 83, 54,
																																																FONT_END, 74, 83,
																																																FONT_BEGIN, 71, 372,
																																																FONT_NEXT, 74, 83,
																																																FONT_END, 71, 120,
																																																FONT_BEGIN, 338, -7,
																																																FONT_NEXT, 338, 76,
																																																FONT_NEXT, 340, -9,
																																																FONT_END, 333, 370,
																																																FONT_BEGIN, 333, 370,
																																																FONT_NEXT, 338, 76,
																																																FONT_NEXT, 333, 135,
																																																FONT_END, 328, 100,
																																																FONT_BEGIN, 328, 100,
																																																FONT_NEXT, 338, 76,
																																																FONT_END, 295, 33,
																																																FONT_BEGIN, 328, 100,
																																																FONT_NEXT, 295, 33,
																																																FONT_NEXT, 317, 84,
																																																FONT_END, 274, 57,
																																																FONT_BEGIN, 274, 57,
																																																FONT_NEXT, 295, 33,
																																																FONT_END, 264, 9,
																																																FONT_BEGIN, 274, 57,
																																																FONT_NEXT, 264, 9,
																																																FONT_NEXT, 230, 48,
																																																FONT_NEXT, 234, -4,
																																																FONT_END, 208, -9,
																																																FONT_BEGIN, 230, 48,
																																																FONT_NEXT, 208, -9,
																																																FONT_NEXT, 190, 57,
																																																FONT_NEXT, 190, -10,
																																																FONT_NEXT, 167, 80,
																																																FONT_NEXT, 154, -5,
																																																FONT_NEXT, 157, 106,
																																																FONT_END, 155, 124,
																																																FONT_BEGIN, 479, 50,
																																																FONT_NEXT, 479, 36,
																																																FONT_NEXT, 444, 53,
																																																FONT_NEXT, 340, -9,
																																																FONT_NEXT, 425, 64,
																																																FONT_END, 418, 82,
																																																FONT_BEGIN, 418, 82,
																																																FONT_NEXT, 340, -9,
																																																FONT_NEXT, 417, 107,
																																																FONT_NEXT, 333, 370,
																																																FONT_END, 331, 396,
																																																FONT_BEGIN, 417, 107,
																																																FONT_NEXT, 331, 396,
																																																FONT_NEXT, 417, 450,
																																																FONT_NEXT, 322, 415,
																																																FONT_NEXT, 259, 450,
																																																FONT_NEXT, 300, 427,
																																																FONT_END, 259, 433,
																																																FONT_ADVANCE, 500, 0
																																														},
																																															{
																																																118,
																																																	FONT_BEGIN, 215, 450,
																																																	FONT_NEXT, 215, 435,
																																																	FONT_NEXT, 19, 450,
																																																	FONT_NEXT, 183, 428,
																																																	FONT_END, 172, 419,
																																																	FONT_BEGIN, 19, 450,
																																																	FONT_NEXT, 172, 419,
																																																	FONT_END, 169, 405,
																																																	FONT_BEGIN, 19, 450,
																																																	FONT_NEXT, 169, 405,
																																																	FONT_END, 48, 427,
																																																	FONT_BEGIN, 19, 450,
																																																	FONT_NEXT, 48, 427,
																																																	FONT_END, 19, 435,
																																																	FONT_BEGIN, 338, 435,
																																																	FONT_NEXT, 338, 450,
																																																	FONT_NEXT, 370, 426,
																																																	FONT_NEXT, 427, 393,
																																																	FONT_NEXT, 385, 402,
																																																	FONT_NEXT, 412, 357,
																																																	FONT_NEXT, 381, 383,
																																																	FONT_NEXT, 284, 36,
																																																	FONT_NEXT, 373, 355,
																																																	FONT_END, 345, 280,
																																																	FONT_BEGIN, 345, 280,
																																																	FONT_NEXT, 284, 36,
																																																	FONT_NEXT, 280, 114,
																																																	FONT_NEXT, 269, 0,
																																																	FONT_NEXT, 178, 370,
																																																	FONT_NEXT, 257, -14,
																																																	FONT_END, 246, -7,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 246, -7,
																																																	FONT_END, 239, 7,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 239, 7,
																																																	FONT_END, 230, 33,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 230, 33,
																																																	FONT_END, 217, 63,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 217, 63,
																																																	FONT_END, 200, 106,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 200, 106,
																																																	FONT_END, 156, 212,
																																																	FONT_BEGIN, 178, 370,
																																																	FONT_NEXT, 156, 212,
																																																	FONT_NEXT, 169, 405,
																																																	FONT_NEXT, 110, 319,
																																																	FONT_END, 90, 364,
																																																	FONT_BEGIN, 169, 405,
																																																	FONT_NEXT, 90, 364,
																																																	FONT_END, 74, 398,
																																																	FONT_BEGIN, 169, 405,
																																																	FONT_NEXT, 74, 398,
																																																	FONT_END, 48, 427,
																																																	FONT_BEGIN, 477, 450,
																																																	FONT_NEXT, 477, 435,
																																																	FONT_NEXT, 338, 450,
																																																	FONT_NEXT, 455, 429,
																																																	FONT_END, 440, 417,
																																																	FONT_BEGIN, 338, 450,
																																																	FONT_NEXT, 440, 417,
																																																	FONT_END, 427, 393,
																																																	FONT_ADVANCE, 500, 0
																																															},
																																															{
																																																119,
																																																	FONT_BEGIN, 201, 450,
																																																	FONT_NEXT, 201, 435,
																																																	FONT_NEXT, 21, 450,
																																																	FONT_NEXT, 169, 427,
																																																	FONT_END, 158, 417,
																																																	FONT_BEGIN, 21, 450,
																																																	FONT_NEXT, 158, 417,
																																																	FONT_END, 155, 401,
																																																	FONT_BEGIN, 21, 450,
																																																	FONT_NEXT, 155, 401,
																																																	FONT_END, 49, 420,
																																																	FONT_BEGIN, 21, 450,
																																																	FONT_NEXT, 49, 420,
																																																	FONT_END, 21, 435,
																																																	FONT_BEGIN, 262, 435,
																																																	FONT_NEXT, 262, 450,
																																																	FONT_NEXT, 288, 429,
																																																	FONT_END, 308, 413,
																																																	FONT_BEGIN, 308, 413,
																																																	FONT_NEXT, 262, 450,
																																																	FONT_END, 407, 398,
																																																	FONT_BEGIN, 308, 413,
																																																	FONT_NEXT, 407, 398,
																																																	FONT_NEXT, 327, 376,
																																																	FONT_NEXT, 372, 265,
																																																	FONT_NEXT, 338, 347,
																																																	FONT_END, 351, 310,
																																																	FONT_BEGIN, 351, 310,
																																																	FONT_NEXT, 372, 265,
																																																	FONT_END, 260, 25,
																																																	FONT_BEGIN, 351, 310,
																																																	FONT_NEXT, 260, 25,
																																																	FONT_NEXT, 260, 111,
																																																	FONT_END, 224, 205,
																																																	FONT_BEGIN, 224, 205,
																																																	FONT_NEXT, 260, 25,
																																																	FONT_END, 244, -5,
																																																	FONT_BEGIN, 224, 205,
																																																	FONT_NEXT, 244, -5,
																																																	FONT_END, 235, -14,
																																																	FONT_BEGIN, 224, 205,
																																																	FONT_NEXT, 235, -14,
																																																	FONT_END, 224, -5,
																																																	FONT_BEGIN, 224, 205,
																																																	FONT_NEXT, 224, -5,
																																																	FONT_NEXT, 190, 290,
																																																	FONT_NEXT, 217, 8,
																																																	FONT_END, 209, 30,
																																																	FONT_BEGIN, 190, 290,
																																																	FONT_NEXT, 209, 30,
																																																	FONT_END, 74, 372,
																																																	FONT_BEGIN, 190, 290,
																																																	FONT_NEXT, 74, 372,
																																																	FONT_NEXT, 164, 358,
																																																	FONT_END, 157, 383,
																																																	FONT_BEGIN, 157, 383,
																																																	FONT_NEXT, 74, 372,
																																																	FONT_NEXT, 155, 401,
																																																	FONT_NEXT, 60, 401,
																																																	FONT_END, 49, 420,
																																																	FONT_BEGIN, 465, 450,
																																																	FONT_NEXT, 465, 435,
																																																	FONT_NEXT, 262, 450,
																																																	FONT_NEXT, 423, 425,
																																																	FONT_END, 411, 415,
																																																	FONT_BEGIN, 262, 450,
																																																	FONT_NEXT, 411, 415,
																																																	FONT_END, 407, 398,
																																																	FONT_BEGIN, 571, 435,
																																																	FONT_NEXT, 571, 450,
																																																	FONT_NEXT, 602, 425,
																																																	FONT_NEXT, 671, 419,
																																																	FONT_END, 653, 381,
																																																	FONT_BEGIN, 602, 425,
																																																	FONT_NEXT, 653, 381,
																																																	FONT_NEXT, 611, 416,
																																																	FONT_END, 615, 402,
																																																	FONT_BEGIN, 615, 402,
																																																	FONT_NEXT, 653, 381,
																																																	FONT_NEXT, 610, 374,
																																																	FONT_NEXT, 515, 35,
																																																	FONT_NEXT, 598, 338,
																																																	FONT_END, 508, 116,
																																																	FONT_BEGIN, 508, 116,
																																																	FONT_NEXT, 515, 35,
																																																	FONT_END, 505, 10,
																																																	FONT_BEGIN, 508, 116,
																																																	FONT_NEXT, 505, 10,
																																																	FONT_NEXT, 428, 330,
																																																	FONT_NEXT, 498, -5,
																																																	FONT_END, 487, -14,
																																																	FONT_BEGIN, 428, 330,
																																																	FONT_NEXT, 487, -14,
																																																	FONT_END, 477, -6,
																																																	FONT_BEGIN, 428, 330,
																																																	FONT_NEXT, 477, -6,
																																																	FONT_END, 471, 7,
																																																	FONT_BEGIN, 428, 330,
																																																	FONT_NEXT, 471, 7,
																																																	FONT_END, 463, 29,
																																																	FONT_BEGIN, 428, 330,
																																																	FONT_NEXT, 463, 29,
																																																	FONT_END, 372, 265,
																																																	FONT_BEGIN, 428, 330,
																																																	FONT_NEXT, 372, 265,
																																																	FONT_NEXT, 411, 371,
																																																	FONT_END, 407, 398,
																																																	FONT_BEGIN, 694, 450,
																																																	FONT_NEXT, 694, 435,
																																																	FONT_NEXT, 571, 450,
																																																	FONT_END, 671, 419,
																																																	FONT_ADVANCE, 722, 0
																																															},
																																																{
																																																	120,
																																																		FONT_BEGIN, 24, 435,
																																																		FONT_NEXT, 24, 450,
																																																		FONT_NEXT, 64, 428,
																																																		FONT_NEXT, 188, 412,
																																																		FONT_NEXT, 83, 410,
																																																		FONT_END, 110, 375,
																																																		FONT_BEGIN, 110, 375,
																																																		FONT_NEXT, 188, 412,
																																																		FONT_END, 194, 391,
																																																		FONT_BEGIN, 110, 375,
																																																		FONT_NEXT, 194, 391,
																																																		FONT_NEXT, 204, 231,
																																																		FONT_NEXT, 210, 362,
																																																		FONT_END, 221, 197,
																																																		FONT_BEGIN, 204, 231,
																																																		FONT_NEXT, 221, 197,
																																																		FONT_END, 142, 74,
																																																		FONT_BEGIN, 204, 231,
																																																		FONT_NEXT, 142, 74,
																																																		FONT_NEXT, 90, 66,
																																																		FONT_NEXT, 122, 33,
																																																		FONT_NEXT, 68, 39,
																																																		FONT_END, 51, 24,
																																																		FONT_BEGIN, 51, 24,
																																																		FONT_NEXT, 122, 33,
																																																		FONT_END, 17, 0,
																																																		FONT_BEGIN, 51, 24,
																																																		FONT_NEXT, 17, 0,
																																																		FONT_END, 17, 15,
																																																		FONT_BEGIN, 231, 450,
																																																		FONT_NEXT, 231, 435,
																																																		FONT_NEXT, 24, 450,
																																																		FONT_NEXT, 202, 431,
																																																		FONT_END, 192, 424,
																																																		FONT_BEGIN, 24, 450,
																																																		FONT_NEXT, 192, 424,
																																																		FONT_END, 188, 412,
																																																		FONT_BEGIN, 275, 435,
																																																		FONT_NEXT, 275, 450,
																																																		FONT_NEXT, 301, 431,
																																																		FONT_END, 311, 424,
																																																		FONT_BEGIN, 311, 424,
																																																		FONT_NEXT, 275, 450,
																																																		FONT_END, 379, 417,
																																																		FONT_BEGIN, 311, 424,
																																																		FONT_NEXT, 379, 417,
																																																		FONT_END, 352, 391,
																																																		FONT_BEGIN, 311, 424,
																																																		FONT_NEXT, 352, 391,
																																																		FONT_NEXT, 315, 412,
																																																		FONT_END, 313, 402,
																																																		FONT_BEGIN, 313, 402,
																																																		FONT_NEXT, 352, 391,
																																																		FONT_END, 269, 271,
																																																		FONT_BEGIN, 313, 402,
																																																		FONT_NEXT, 269, 271,
																																																		FONT_NEXT, 304, 386,
																																																		FONT_END, 284, 356,
																																																		FONT_BEGIN, 284, 356,
																																																		FONT_NEXT, 269, 271,
																																																		FONT_NEXT, 248, 304,
																																																		FONT_NEXT, 221, 197,
																																																		FONT_END, 210, 362,
																																																		FONT_BEGIN, 433, 450,
																																																		FONT_NEXT, 433, 435,
																																																		FONT_NEXT, 275, 450,
																																																		FONT_NEXT, 403, 429,
																																																		FONT_END, 379, 417,
																																																		FONT_BEGIN, 278, 15,
																																																		FONT_NEXT, 309, 20,
																																																		FONT_NEXT, 278, 0,
																																																		FONT_END, 397, 75,
																																																		FONT_BEGIN, 397, 75,
																																																		FONT_NEXT, 309, 20,
																																																		FONT_END, 318, 41,
																																																		FONT_BEGIN, 397, 75,
																																																		FONT_NEXT, 318, 41,
																																																		FONT_NEXT, 269, 271,
																																																		FONT_NEXT, 311, 60,
																																																		FONT_END, 302, 73,
																																																		FONT_BEGIN, 269, 271,
																																																		FONT_NEXT, 302, 73,
																																																		FONT_END, 221, 197,
																																																		FONT_BEGIN, 479, 15,
																																																		FONT_NEXT, 479, 0,
																																																		FONT_NEXT, 445, 23,
																																																		FONT_NEXT, 278, 0,
																																																		FONT_NEXT, 423, 41,
																																																		FONT_END, 397, 75,
																																																		FONT_BEGIN, 162, 15,
																																																		FONT_NEXT, 162, 0,
																																																		FONT_NEXT, 136, 17,
																																																		FONT_NEXT, 17, 0,
																																																		FONT_END, 122, 33,
																																																		FONT_ADVANCE, 500, 0
																																																},
																																																{
																																																	121,
																																																		FONT_BEGIN, 220, 450,
																																																		FONT_NEXT, 220, 435,
																																																		FONT_NEXT, 14, 450,
																																																		FONT_NEXT, 182, 430,
																																																		FONT_END, 167, 421,
																																																		FONT_BEGIN, 14, 450,
																																																		FONT_NEXT, 167, 421,
																																																		FONT_END, 162, 406,
																																																		FONT_BEGIN, 14, 450,
																																																		FONT_NEXT, 162, 406,
																																																		FONT_END, 39, 428,
																																																		FONT_BEGIN, 14, 450,
																																																		FONT_NEXT, 39, 428,
																																																		FONT_END, 14, 436,
																																																		FONT_BEGIN, 340, 435,
																																																		FONT_NEXT, 340, 450,
																																																		FONT_NEXT, 376, 428,
																																																		FONT_NEXT, 439, 414,
																																																		FONT_END, 427, 390,
																																																		FONT_BEGIN, 376, 428,
																																																		FONT_NEXT, 427, 390,
																																																		FONT_NEXT, 388, 407,
																																																		FONT_NEXT, 273, -18,
																																																		FONT_NEXT, 380, 379,
																																																		FONT_END, 365, 339,
																																																		FONT_BEGIN, 365, 339,
																																																		FONT_NEXT, 273, -18,
																																																		FONT_NEXT, 287, 117,
																																																		FONT_END, 172, 370,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 273, -18,
																																																		FONT_END, 241, 20,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 241, 20,
																																																		FONT_END, 233, 43,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 233, 43,
																																																		FONT_END, 215, 83,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 215, 83,
																																																		FONT_END, 194, 125,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 194, 125,
																																																		FONT_END, 179, 158,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 179, 158,
																																																		FONT_END, 65, 404,
																																																		FONT_BEGIN, 172, 370,
																																																		FONT_NEXT, 65, 404,
																																																		FONT_NEXT, 162, 406,
																																																		FONT_END, 39, 428,
																																																		FONT_BEGIN, 475, 450,
																																																		FONT_NEXT, 475, 435,
																																																		FONT_NEXT, 340, 450,
																																																		FONT_NEXT, 451, 427,
																																																		FONT_END, 439, 414,
																																																		FONT_BEGIN, 30, -160,
																																																		FONT_NEXT, 36, -136,
																																																		FONT_NEXT, 34, -181,
																																																		FONT_END, 47, -199,
																																																		FONT_BEGIN, 47, -199,
																																																		FONT_NEXT, 36, -136,
																																																		FONT_END, 51, -123,
																																																		FONT_BEGIN, 47, -199,
																																																		FONT_NEXT, 51, -123,
																																																		FONT_NEXT, 70, -213,
																																																		FONT_NEXT, 78, -116,
																																																		FONT_NEXT, 103, -218,
																																																		FONT_NEXT, 118, -125,
																																																		FONT_END, 149, -134,
																																																		FONT_BEGIN, 103, -218,
																																																		FONT_NEXT, 149, -134,
																																																		FONT_NEXT, 154, -206,
																																																		FONT_NEXT, 167, -129,
																																																		FONT_END, 185, -113,
																																																		FONT_BEGIN, 154, -206,
																																																		FONT_NEXT, 185, -113,
																																																		FONT_NEXT, 197, -169,
																																																		FONT_NEXT, 214, -63,
																																																		FONT_END, 233, -11,
																																																		FONT_BEGIN, 197, -169,
																																																		FONT_NEXT, 233, -11,
																																																		FONT_NEXT, 235, -106,
																																																		FONT_NEXT, 239, 8,
																																																		FONT_END, 241, 20,
																																																		FONT_BEGIN, 235, -106,
																																																		FONT_NEXT, 241, 20,
																																																		FONT_END, 273, -18,
																																																		FONT_ADVANCE, 500, 0
																																																},
																																																	{
																																																		122,
																																																			FONT_BEGIN, 403, 450,
																																																			FONT_NEXT, 403, 435,
																																																			FONT_NEXT, 56, 450,
																																																			FONT_NEXT, 293, 420,
																																																			FONT_END, 155, 420,
																																																			FONT_BEGIN, 56, 450,
																																																			FONT_NEXT, 155, 420,
																																																			FONT_END, 112, 413,
																																																			FONT_BEGIN, 56, 450,
																																																			FONT_NEXT, 112, 413,
																																																			FONT_END, 88, 396,
																																																			FONT_BEGIN, 56, 450,
																																																			FONT_NEXT, 88, 396,
																																																			FONT_END, 76, 368,
																																																			FONT_BEGIN, 56, 450,
																																																			FONT_NEXT, 76, 368,
																																																			FONT_END, 71, 332,
																																																			FONT_BEGIN, 56, 450,
																																																			FONT_NEXT, 71, 332,
																																																			FONT_END, 53, 332,
																																																			FONT_BEGIN, 418, 135,
																																																			FONT_NEXT, 404, 0,
																																																			FONT_NEXT, 400, 139,
																																																			FONT_END, 388, 89,
																																																			FONT_BEGIN, 388, 89,
																																																			FONT_NEXT, 404, 0,
																																																			FONT_NEXT, 369, 55,
																																																			FONT_END, 334, 36,
																																																			FONT_BEGIN, 334, 36,
																																																			FONT_NEXT, 404, 0,
																																																			FONT_NEXT, 307, 31,
																																																			FONT_NEXT, 27, 0,
																																																			FONT_NEXT, 272, 30,
																																																			FONT_END, 134, 30,
																																																			FONT_BEGIN, 134, 30,
																																																			FONT_NEXT, 27, 0,
																																																			FONT_END, 27, 15,
																																																			FONT_BEGIN, 134, 30,
																																																			FONT_NEXT, 27, 15,
																																																			FONT_END, 293, 420,
																																																			FONT_BEGIN, 134, 30,
																																																			FONT_NEXT, 293, 420,
																																																			FONT_END, 403, 435,
																																																			FONT_ADVANCE, 444, 0
																																																	},
																																																	{
																																																		123,
																																																			FONT_BEGIN, 100, 250,
																																																			FONT_NEXT, 142, 266,
																																																			FONT_NEXT, 142, 233,
																																																			FONT_END, 168, 209,
																																																			FONT_BEGIN, 168, 209,
																																																			FONT_NEXT, 142, 266,
																																																			FONT_END, 168, 248,
																																																			FONT_BEGIN, 168, 209,
																																																			FONT_NEXT, 168, 248,
																																																			FONT_NEXT, 181, 178,
																																																			FONT_NEXT, 215, 230,
																																																			FONT_NEXT, 185, 136,
																																																			FONT_NEXT, 210, -139,
																																																			FONT_END, 196, -114,
																																																			FONT_BEGIN, 185, 136,
																																																			FONT_NEXT, 196, -114,
																																																			FONT_END, 187, -82,
																																																			FONT_BEGIN, 185, 136,
																																																			FONT_NEXT, 187, -82,
																																																			FONT_END, 185, -41,
																																																			FONT_BEGIN, 185, 541,
																																																			FONT_NEXT, 187, 581,
																																																			FONT_NEXT, 185, 363,
																																																			FONT_NEXT, 196, 612,
																																																			FONT_END, 210, 637,
																																																			FONT_BEGIN, 185, 363,
																																																			FONT_NEXT, 210, 637,
																																																			FONT_END, 215, 267,
																																																			FONT_BEGIN, 185, 363,
																																																			FONT_NEXT, 215, 267,
																																																			FONT_NEXT, 181, 320,
																																																			FONT_NEXT, 168, 250,
																																																			FONT_NEXT, 168, 289,
																																																			FONT_END, 142, 266,
																																																			FONT_BEGIN, 142, 266,
																																																			FONT_NEXT, 168, 250,
																																																			FONT_END, 168, 248,
																																																			FONT_BEGIN, 142, 266,
																																																			FONT_END, 168, 248,
																																																			FONT_BEGIN, 350, 680,
																																																			FONT_NEXT, 350, 669,
																																																			FONT_NEXT, 280, 675,
																																																			FONT_NEXT, 305, 652,
																																																			FONT_END, 277, 627,
																																																			FONT_BEGIN, 280, 675,
																																																			FONT_NEXT, 277, 627,
																																																			FONT_NEXT, 228, 655,
																																																			FONT_NEXT, 263, 592,
																																																			FONT_END, 259, 546,
																																																			FONT_BEGIN, 228, 655,
																																																			FONT_NEXT, 259, 546,
																																																			FONT_END, 259, 378,
																																																			FONT_BEGIN, 228, 655,
																																																			FONT_NEXT, 259, 378,
																																																			FONT_END, 255, 330,
																																																			FONT_BEGIN, 228, 655,
																																																			FONT_NEXT, 255, 330,
																																																			FONT_END, 242, 294,
																																																			FONT_BEGIN, 228, 655,
																																																			FONT_NEXT, 242, 294,
																																																			FONT_END, 215, 267,
																																																			FONT_BEGIN, 228, 655,
																																																			FONT_NEXT, 215, 267,
																																																			FONT_END, 210, 637,
																																																			FONT_BEGIN, 259, 121,
																																																			FONT_NEXT, 259, -47,
																																																			FONT_NEXT, 255, 168,
																																																			FONT_NEXT, 228, -156,
																																																			FONT_NEXT, 242, 204,
																																																			FONT_END, 215, 230,
																																																			FONT_BEGIN, 215, 230,
																																																			FONT_NEXT, 228, -156,
																																																			FONT_END, 210, -139,
																																																			FONT_BEGIN, 215, 230,
																																																			FONT_END, 210, -139,
																																																			FONT_BEGIN, 350, -170,
																																																			FONT_NEXT, 350, -181,
																																																			FONT_NEXT, 305, -154,
																																																			FONT_NEXT, 280, -176,
																																																			FONT_NEXT, 277, -129,
																																																			FONT_NEXT, 228, -156,
																																																			FONT_NEXT, 263, -94,
																																																			FONT_END, 259, -47,
																																																			FONT_ADVANCE, 480, 0
																																																	},
																																																		{
																																																			124,
																																																				FONT_BEGIN, 133, 676,
																																																				FONT_NEXT, 133, -14,
																																																				FONT_NEXT, 67, 676,
																																																				FONT_END, 67, -14,
																																																				FONT_ADVANCE, 200, 0
																																																		},
																																																		{
																																																			125,
																																																				FONT_BEGIN, 130, 669,
																																																				FONT_NEXT, 130, 680,
																																																				FONT_NEXT, 174, 652,
																																																				FONT_NEXT, 199, 675,
																																																				FONT_NEXT, 202, 627,
																																																				FONT_END, 216, 592,
																																																				FONT_BEGIN, 216, 592,
																																																				FONT_NEXT, 199, 675,
																																																				FONT_END, 251, 655,
																																																				FONT_BEGIN, 216, 592,
																																																				FONT_NEXT, 251, 655,
																																																				FONT_NEXT, 221, 546,
																																																				FONT_NEXT, 237, 294,
																																																				FONT_END, 224, 330,
																																																				FONT_BEGIN, 221, 546,
																																																				FONT_NEXT, 224, 330,
																																																				FONT_END, 221, 378,
																																																				FONT_BEGIN, 295, 540,
																																																				FONT_NEXT, 295, 363,
																																																				FONT_NEXT, 292, 580,
																																																				FONT_NEXT, 264, 268,
																																																				FONT_NEXT, 283, 612,
																																																				FONT_END, 269, 637,
																																																				FONT_BEGIN, 269, 637,
																																																				FONT_NEXT, 264, 268,
																																																				FONT_NEXT, 251, 655,
																																																				FONT_END, 237, 294,
																																																				FONT_BEGIN, 380, 250,
																																																				FONT_NEXT, 337, 233,
																																																				FONT_NEXT, 337, 266,
																																																				FONT_END, 311, 289,
																																																				FONT_BEGIN, 311, 289,
																																																				FONT_NEXT, 337, 233,
																																																				FONT_END, 312, 251,
																																																				FONT_BEGIN, 311, 289,
																																																				FONT_NEXT, 312, 251,
																																																				FONT_END, 264, 268,
																																																				FONT_BEGIN, 311, 289,
																																																				FONT_NEXT, 264, 268,
																																																				FONT_NEXT, 298, 320,
																																																				FONT_END, 295, 363,
																																																				FONT_BEGIN, 221, 121,
																																																				FONT_NEXT, 224, 168,
																																																				FONT_NEXT, 221, -47,
																																																				FONT_NEXT, 237, 204,
																																																				FONT_NEXT, 251, -157,
																																																				FONT_NEXT, 264, 231,
																																																				FONT_NEXT, 269, -139,
																																																				FONT_END, 283, -114,
																																																				FONT_BEGIN, 283, -114,
																																																				FONT_NEXT, 264, 231,
																																																				FONT_NEXT, 292, -83,
																																																				FONT_END, 295, -42,
																																																				FONT_BEGIN, 295, -42,
																																																				FONT_NEXT, 264, 231,
																																																				FONT_NEXT, 295, 136,
																																																				FONT_END, 298, 178,
																																																				FONT_BEGIN, 298, 178,
																																																				FONT_NEXT, 264, 231,
																																																				FONT_NEXT, 311, 209,
																																																				FONT_NEXT, 312, 249,
																																																				FONT_NEXT, 337, 233,
																																																				FONT_END, 312, 251,
																																																				FONT_BEGIN, 130, -170,
																																																				FONT_NEXT, 174, -154,
																																																				FONT_NEXT, 130, -181,
																																																				FONT_END, 199, -177,
																																																				FONT_BEGIN, 199, -177,
																																																				FONT_NEXT, 174, -154,
																																																				FONT_END, 202, -129,
																																																				FONT_BEGIN, 199, -177,
																																																				FONT_NEXT, 202, -129,
																																																				FONT_END, 216, -94,
																																																				FONT_BEGIN, 199, -177,
																																																				FONT_NEXT, 216, -94,
																																																				FONT_NEXT, 251, -157,
																																																				FONT_END, 221, -47,
																																																				FONT_ADVANCE, 480, 0
																																																		},
																																																			{
																																																				126,
																																																					FONT_BEGIN, 502, 273,
																																																					FONT_NEXT, 451, 214,
																																																					FONT_NEXT, 466, 323,
																																																					FONT_END, 428, 273,
																																																					FONT_BEGIN, 428, 273,
																																																					FONT_NEXT, 451, 214,
																																																					FONT_END, 418, 194,
																																																					FONT_BEGIN, 428, 273,
																																																					FONT_NEXT, 418, 194,
																																																					FONT_NEXT, 405, 257,
																																																					FONT_NEXT, 377, 187,
																																																					FONT_NEXT, 378, 251,
																																																					FONT_END, 330, 263,
																																																					FONT_BEGIN, 330, 263,
																																																					FONT_NEXT, 377, 187,
																																																					FONT_END, 324, 196,
																																																					FONT_BEGIN, 330, 263,
																																																					FONT_NEXT, 324, 196,
																																																					FONT_NEXT, 276, 287,
																																																					FONT_NEXT, 275, 220,
																																																					FONT_NEXT, 219, 309,
																																																					FONT_NEXT, 222, 244,
																																																					FONT_END, 160, 255,
																																																					FONT_BEGIN, 219, 309,
																																																					FONT_NEXT, 160, 255,
																																																					FONT_NEXT, 165, 319,
																																																					FONT_END, 120, 311,
																																																					FONT_BEGIN, 120, 311,
																																																					FONT_NEXT, 160, 255,
																																																					FONT_END, 130, 248,
																																																					FONT_BEGIN, 120, 311,
																																																					FONT_NEXT, 130, 248,
																																																					FONT_END, 107, 232,
																																																					FONT_BEGIN, 120, 311,
																																																					FONT_NEXT, 107, 232,
																																																					FONT_NEXT, 85, 291,
																																																					FONT_NEXT, 76, 183,
																																																					FONT_END, 40, 233,
																																																					FONT_ADVANCE, 541, 0
																																																			},
																																																			{
																																																				END_OF_LIST
																																																			}
			};
static GLubyte bitmapFont[][1 + 13] = {
				{
					32,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00
				},
				{
					33,
						0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x18,
						0x18, 0x18, 0x18, 0x18, 0x18, 0x18
				},
					{
						34,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x00, 0x36, 0x36, 0x36, 0x36
					},
					{
						35,
							0x00, 0x00, 0x00, 0x66, 0x66, 0xff, 0x66,
							0x66, 0xff, 0x66, 0x66, 0x00, 0x00
					},
						{
							36,
								0x00, 0x00, 0x18, 0x7e, 0xff, 0x1b, 0x1f,
								0x7e, 0xf8, 0xd8, 0xff, 0x7e, 0x18
						},
						{
							37,
								0x00, 0x00, 0x0e, 0x1b, 0xdb, 0x6e, 0x30,
								0x18, 0x0c, 0x76, 0xdb, 0xd8, 0x70
						},
							{
								38,
									0x00, 0x00, 0x7f, 0xc6, 0xcf, 0xd8, 0x70,
									0x70, 0xd8, 0xcc, 0xcc, 0x6c, 0x38
							},
							{
								39,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x18, 0x1c, 0x0c, 0x0e
							},
								{
									40,
										0x00, 0x00, 0x0c, 0x18, 0x30, 0x30, 0x30,
										0x30, 0x30, 0x30, 0x30, 0x18, 0x0c
								},
								{
									41,
										0x00, 0x00, 0x30, 0x18, 0x0c, 0x0c, 0x0c,
										0x0c, 0x0c, 0x0c, 0x0c, 0x18, 0x30
								},
									{
										42,
											0x00, 0x00, 0x00, 0x00, 0x99, 0x5a, 0x3c,
											0xff, 0x3c, 0x5a, 0x99, 0x00, 0x00
									},
									{
										43,
											0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0xff,
											0xff, 0x18, 0x18, 0x18, 0x00, 0x00
									},
										{
											44,
												0x00, 0x00, 0x30, 0x18, 0x1c, 0x1c, 0x00,
												0x00, 0x00, 0x00, 0x00, 0x00, 0x00
										},
										{
											45,
												0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
												0xff, 0x00, 0x00, 0x00, 0x00, 0x00
										},
											{
												46,
													0x00, 0x00, 0x00, 0x38, 0x38, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00
											},
											{
												47,
													0x00, 0x60, 0x60, 0x30, 0x30, 0x18, 0x18,
													0x0c, 0x0c, 0x06, 0x06, 0x03, 0x03
											},
												{
													48,
														0x00, 0x00, 0x3c, 0x66, 0xc3, 0xe3, 0xf3,
														0xdb, 0xcf, 0xc7, 0xc3, 0x66, 0x3c
												},
												{
													49,
														0x00, 0x00, 0x7e, 0x18, 0x18, 0x18, 0x18,
														0x18, 0x18, 0x18, 0x78, 0x38, 0x18
												},
													{
														50,
															0x00, 0x00, 0xff, 0xc0, 0xc0, 0x60, 0x30,
															0x18, 0x0c, 0x06, 0x03, 0xe7, 0x7e
													},
													{
														51,
															0x00, 0x00, 0x7e, 0xe7, 0x03, 0x03, 0x07,
															0x7e, 0x07, 0x03, 0x03, 0xe7, 0x7e
													},
														{
															52,
																0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
																0xff, 0xcc, 0x6c, 0x3c, 0x1c, 0x0c
														},
														{
															53,
																0x00, 0x00, 0x7e, 0xe7, 0x03, 0x03, 0x07,
																0xfe, 0xc0, 0xc0, 0xc0, 0xc0, 0xff
														},
															{
																54,
																	0x00, 0x00, 0x7e, 0xe7, 0xc3, 0xc3, 0xc7,
																	0xfe, 0xc0, 0xc0, 0xc0, 0xe7, 0x7e
															},
															{
																55,
																	0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x18,
																	0x0c, 0x06, 0x03, 0x03, 0x03, 0xff
															},
																{
																	56,
																		0x00, 0x00, 0x7e, 0xe7, 0xc3, 0xc3, 0xe7,
																		0x7e, 0xe7, 0xc3, 0xc3, 0xe7, 0x7e
																},
																{
																	57,
																		0x00, 0x00, 0x7e, 0xe7, 0x03, 0x03, 0x03,
																		0x7f, 0xe7, 0xc3, 0xc3, 0xe7, 0x7e
																},
																	{
																		58,
																			0x00, 0x00, 0x00, 0x38, 0x38, 0x00, 0x00,
																			0x38, 0x38, 0x00, 0x00, 0x00, 0x00
																	},
																	{
																		59,
																			0x00, 0x00, 0x30, 0x18, 0x1c, 0x1c, 0x00,
																			0x00, 0x1c, 0x1c, 0x00, 0x00, 0x00
																	},
																		{
																			60,
																				0x00, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60,
																				0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06
																		},
																		{
																			61,
																				0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00,
																				0xff, 0xff, 0x00, 0x00, 0x00, 0x00
																		},
																			{
																				62,
																					0x00, 0x00, 0x60, 0x30, 0x18, 0x0c, 0x06,
																					0x03, 0x06, 0x0c, 0x18, 0x30, 0x60
																			},
																			{
																				63,
																					0x00, 0x00, 0x18, 0x00, 0x00, 0x18, 0x18,
																					0x0c, 0x06, 0x03, 0xc3, 0xc3, 0x7e
																			},
																				{
																					64,
																						0x00, 0x00, 0x3f, 0x60, 0xcf, 0xdb, 0xd3,
																						0xdd, 0xc3, 0x7e, 0x00, 0x00, 0x00
																				},
																				{
																					65,
																						0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,
																						0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18
																				},
																					{
																						66,
																							0x00, 0x00, 0xfe, 0xc7, 0xc3, 0xc3, 0xc7,
																							0xfe, 0xc7, 0xc3, 0xc3, 0xc7, 0xfe
																					},
																					{
																						67,
																							0x00, 0x00, 0x7e, 0xe7, 0xc0, 0xc0, 0xc0,
																							0xc0, 0xc0, 0xc0, 0xc0, 0xe7, 0x7e
																					},
																						{
																							68,
																								0x00, 0x00, 0xfc, 0xce, 0xc7, 0xc3, 0xc3,
																								0xc3, 0xc3, 0xc3, 0xc7, 0xce, 0xfc
																						},
																						{
																							69,
																								0x00, 0x00, 0xff, 0xc0, 0xc0, 0xc0, 0xc0,
																								0xfc, 0xc0, 0xc0, 0xc0, 0xc0, 0xff
																						},
																							{
																								70,
																									0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
																									0xc0, 0xfc, 0xc0, 0xc0, 0xc0, 0xff
																							},
																							{
																								71,
																									0x00, 0x00, 0x7e, 0xe7, 0xc3, 0xc3, 0xcf,
																									0xc0, 0xc0, 0xc0, 0xc0, 0xe7, 0x7e
																							},
																								{
																									72,
																										0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
																										0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3
																								},
																								{
																									73,
																										0x00, 0x00, 0x7e, 0x18, 0x18, 0x18, 0x18,
																										0x18, 0x18, 0x18, 0x18, 0x18, 0x7e
																								},
																									{
																										74,
																											0x00, 0x00, 0x7c, 0xee, 0xc6, 0x06, 0x06,
																											0x06, 0x06, 0x06, 0x06, 0x06, 0x06
																									},
																									{
																										75,
																											0x00, 0x00, 0xc3, 0xc6, 0xcc, 0xd8, 0xf0,
																											0xe0, 0xf0, 0xd8, 0xcc, 0xc6, 0xc3
																									},
																										{
																											76,
																												0x00, 0x00, 0xff, 0xc0, 0xc0, 0xc0, 0xc0,
																												0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0
																										},
																										{
																											77,
																												0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
																												0xc3, 0xdb, 0xff, 0xff, 0xe7, 0xc3
																										},
																											{
																												78,
																													0x00, 0x00, 0xc7, 0xc7, 0xcf, 0xcf, 0xdf,
																													0xdb, 0xfb, 0xf3, 0xf3, 0xe3, 0xe3
																											},
																											{
																												79,
																													0x00, 0x00, 0x7e, 0xe7, 0xc3, 0xc3, 0xc3,
																													0xc3, 0xc3, 0xc3, 0xc3, 0xe7, 0x7e
																											},
																												{
																													80,
																														0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
																														0xfe, 0xc7, 0xc3, 0xc3, 0xc7, 0xfe
																												},
																												{
																													81,
																														0x00, 0x00, 0x3f, 0x6e, 0xdf, 0xdb, 0xc3,
																														0xc3, 0xc3, 0xc3, 0xc3, 0x66, 0x3c
																												},
																													{
																														82,
																															0x00, 0x00, 0xc3, 0xc6, 0xcc, 0xd8, 0xf0,
																															0xfe, 0xc7, 0xc3, 0xc3, 0xc7, 0xfe
																													},
																													{
																														83,
																															0x00, 0x00, 0x7e, 0xe7, 0x03, 0x03, 0x07,
																															0x7e, 0xe0, 0xc0, 0xc0, 0xe7, 0x7e
																													},
																														{
																															84,
																																0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18,
																																0x18, 0x18, 0x18, 0x18, 0x18, 0xff
																														},
																														{
																															85,
																																0x00, 0x00, 0x7e, 0xe7, 0xc3, 0xc3, 0xc3,
																																0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3
																														},
																															{
																																86,
																																	0x00, 0x00, 0x18, 0x3c, 0x3c, 0x66, 0x66,
																																	0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3
																															},
																															{
																																87,
																																	0x00, 0x00, 0xc3, 0xe7, 0xff, 0xff, 0xdb,
																																	0xdb, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3
																															},
																																{
																																	88,
																																		0x00, 0x00, 0xc3, 0x66, 0x66, 0x3c, 0x3c,
																																		0x18, 0x3c, 0x3c, 0x66, 0x66, 0xc3
																																},
																																{
																																	89,
																																		0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18,
																																		0x18, 0x3c, 0x3c, 0x66, 0x66, 0xc3
																																},
																																	{
																																		90,
																																			0x00, 0x00, 0xff, 0xc0, 0xc0, 0x60, 0x30,
																																			0x7e, 0x0c, 0x06, 0x03, 0x03, 0xff
																																	},
																																	{
																																		91,
																																			0x00, 0x00, 0x3c, 0x30, 0x30, 0x30, 0x30,
																																			0x30, 0x30, 0x30, 0x30, 0x30, 0x3c
																																	},
																																		{
																																			92,
																																				0x00, 0x03, 0x03, 0x06, 0x06, 0x0c, 0x0c,
																																				0x18, 0x18, 0x30, 0x30, 0x60, 0x60
																																		},
																																		{
																																			93,
																																				0x00, 0x00, 0x3c, 0x0c, 0x0c, 0x0c, 0x0c,
																																				0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x3c
																																		},
																																			{
																																				94,
																																					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
																																					0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18
																																			},
																																			{
																																				95,
																																					0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
																																					0x00, 0x00, 0x00, 0x00, 0x00, 0x00
																																			},
																																				{
																																					96,
																																						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
																																						0x00, 0x00, 0x18, 0x38, 0x30, 0x70
																																				},
																																				{
																																					97,
																																						0x00, 0x00, 0x7f, 0xc3, 0xc3, 0x7f, 0x03,
																																						0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00
																																				},
																																					{
																																						98,
																																							0x00, 0x00, 0xfe, 0xc3, 0xc3, 0xc3, 0xc3,
																																							0xfe, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0
																																					},
																																					{
																																						99,
																																							0x00, 0x00, 0x7e, 0xc3, 0xc0, 0xc0, 0xc0,
																																							0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00
																																					},
																																						{
																																							100,
																																								0x00, 0x00, 0x7f, 0xc3, 0xc3, 0xc3, 0xc3,
																																								0x7f, 0x03, 0x03, 0x03, 0x03, 0x03
																																						},
																																						{
																																							101,
																																								0x00, 0x00, 0x7f, 0xc0, 0xc0, 0xfe, 0xc3,
																																								0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00
																																						},
																																							{
																																								102,
																																									0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x30,
																																									0xfc, 0x30, 0x30, 0x30, 0x33, 0x1e
																																							},
																																							{
																																								103,
																																									0x7e, 0xc3, 0x03, 0x03, 0x7f, 0xc3, 0xc3,
																																									0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00
																																							},
																																								{
																																									104,
																																										0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
																																										0xc3, 0xfe, 0xc0, 0xc0, 0xc0, 0xc0
																																								},
																																								{
																																									105,
																																										0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18,
																																										0x18, 0x18, 0x00, 0x00, 0x18, 0x00
																																								},
																																									{
																																										106,
																																											0x38, 0x6c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
																																											0x0c, 0x0c, 0x00, 0x00, 0x0c, 0x00
																																									},
																																									{
																																										107,
																																											0x00, 0x00, 0xc6, 0xcc, 0xf8, 0xf0, 0xd8,
																																											0xcc, 0xc6, 0xc0, 0xc0, 0xc0, 0xc0
																																									},
																																										{
																																											108,
																																												0x00, 0x00, 0x7e, 0x18, 0x18, 0x18, 0x18,
																																												0x18, 0x18, 0x18, 0x18, 0x18, 0x78
																																										},
																																										{
																																											109,
																																												0x00, 0x00, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb,
																																												0xdb, 0xfe, 0x00, 0x00, 0x00, 0x00
																																										},
																																											{
																																												110,
																																													0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6,
																																													0xc6, 0xfc, 0x00, 0x00, 0x00, 0x00
																																											},
																																											{
																																												111,
																																													0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6,
																																													0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00
																																											},
																																												{
																																													112,
																																														0xc0, 0xc0, 0xc0, 0xfe, 0xc3, 0xc3, 0xc3,
																																														0xc3, 0xfe, 0x00, 0x00, 0x00, 0x00
																																												},
																																												{
																																													113,
																																														0x03, 0x03, 0x03, 0x7f, 0xc3, 0xc3, 0xc3,
																																														0xc3, 0x7f, 0x00, 0x00, 0x00, 0x00
																																												},
																																													{
																																														114,
																																															0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
																																															0xe0, 0xfe, 0x00, 0x00, 0x00, 0x00
																																													},
																																													{
																																														115,
																																															0x00, 0x00, 0xfe, 0x03, 0x03, 0x7e, 0xc0,
																																															0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00
																																													},
																																														{
																																															116,
																																																0x00, 0x00, 0x1c, 0x36, 0x30, 0x30, 0x30,
																																																0x30, 0xfc, 0x30, 0x30, 0x30, 0x00
																																														},
																																														{
																																															117,
																																																0x00, 0x00, 0x7e, 0xc6, 0xc6, 0xc6, 0xc6,
																																																0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00
																																														},
																																															{
																																																118,
																																																	0x00, 0x00, 0x18, 0x3c, 0x3c, 0x66, 0x66,
																																																	0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00
																																															},
																																															{
																																																119,
																																																	0x00, 0x00, 0xc3, 0xe7, 0xff, 0xdb, 0xc3,
																																																	0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00
																																															},
																																																{
																																																	120,
																																																		0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18, 0x3c,
																																																		0x66, 0xc3, 0x00, 0x00, 0x00, 0x00
																																																},
																																																{
																																																	121,
																																																		0xc0, 0x60, 0x60, 0x30, 0x18, 0x3c, 0x66,
																																																		0x66, 0xc3, 0x00, 0x00, 0x00, 0x00
																																																},
																																																	{
																																																		122,
																																																			0x00, 0x00, 0xff, 0x60, 0x30, 0x18, 0x0c,
																																																			0x06, 0xff, 0x00, 0x00, 0x00, 0x00
																																																	},
																																																	{
																																																		123,
																																																			0x00, 0x00, 0x0f, 0x18, 0x18, 0x18, 0x38,
																																																			0xf0, 0x38, 0x18, 0x18, 0x18, 0x0f
																																																	},
																																																		{
																																																			124,
																																																				0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
																																																				0x18, 0x18, 0x18, 0x18, 0x18, 0x18
																																																		},
																																																		{
																																																			125,
																																																				0x00, 0x00, 0xf0, 0x18, 0x18, 0x18, 0x1c,
																																																				0x0f, 0x1c, 0x18, 0x18, 0x18, 0xf0
																																																		},
																																																			{
																																																				126,
																																																					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
																																																					0x8f, 0xf1, 0x60, 0x00, 0x00, 0x00
																																																			},
																																																			{
																																																				END_OF_LIST
																																																			}
			};
#pragma endregion
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
			Camera(osgViewer::Viewer* camGroup)
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
#define THREADING_MODEL osgViewer::ViewerBase::AutomaticSelection

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
				m_camGroup->frame(currTime_s);
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
					ThrottleFrame();  // Wait a bit if we need to
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
				ASSERT(s_mainWindow == this);
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
			void TryClose()
			{
				if (!m_quitting)
				{
					m_quitting = true;
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
	public:
		Entity_Properties();
		virtual ~Entity_Properties() {}
		//The prep script takes care of the outer layer global table setup
		//override to search the appropriate global table
		//virtual const char *SetUpGlobalTable(Scripting::Script& script) { return script.GetGlobalTable(m_EntityName.c_str()); }
		//virtual void LoadFromScript(Scripting::Script& script);
		void Initialize(Entity2D *NewEntity) const;
	protected:
		std::string m_EntityName;  //derived classes can let base class know what type to read
	private:
		//std::string m_NAME;  <-do not need this
		//Stuff needed for physics
		double m_Mass;
		double m_Dimensions[2]; //Dimensions- Length Width

		//Note: this is in Transmitted entity, but I don't think it belongs here as it doesn't describe what, but rather where
		//! Positions in meters, rotation in degrees
		//double m_X, m_Y, m_Heading;
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

#if 1
#if 1
const double c_Scene_XRes_InPixels = 1280.0;
const double c_Scene_YRes_InPixels = 1024.0;
#else
const double c_Scene_XRes_InPixels = 1680.0;
const double c_Scene_YRes_InPixels = 1050.0;
#endif
#else
const double c_Scene_XRes_InPixels = 1600.0;
const double c_Scene_YRes_InPixels = 1200.0;
#endif

class Actor
{
protected:
	//Gives life to this shell dynamically
	EntityPropertiesInterface *m_EntityProperties_Interface;
	//osg::Node &m_Node;
public:
	Actor(); //osg::Node &node <--may not need this
	virtual void SetEntityProperties_Interface(EntityPropertiesInterface *entity) { m_EntityProperties_Interface = entity; }
	EntityPropertiesInterface *GetEntityProperties_Interface() const { return m_EntityProperties_Interface; }
};

class UI_GameClient;
class Actor_Text : public Actor, public osg::Drawable::UpdateCallback
{
private:
	UI_GameClient *m_pParent;
	std::string m_TextImage;
	std::string m_TeamName; //cache team name to avoid flooding
	osg::ref_ptr<osgText::Text> m_Text;
	osg::ref_ptr<osgText::Text> m_IntendedOrientation;
	//Here is a quick reference on the character layout, used to determine size against the real dimensions, and also to center the intended orientation
	//caret for ships
	osg::Vec2d m_CharacterDimensions;
	double m_FontSize; //cache the last size setting to avoid flooding
protected:
	virtual void update(osg::NodeVisitor *nv, osg::Drawable *draw);
public:
	~Actor_Text();
	Actor_Text(UI_GameClient *parent, const char TextImage[] = "X");
	//call this if you want intended orientation graphics (after setting up the character dimensions)
	void Init_IntendedOrientation();
	osg::ref_ptr<osgText::Text> GetText() { return m_Text; }
	osg::ref_ptr<osgText::Text> GetIntendedOrientation() { return m_IntendedOrientation; }
	std::string &GetTextImage() { return m_TextImage; }
	//For now just have the client code write these
	osg::Vec2d &GetCharacterDimensions() { return m_CharacterDimensions; }
	double GetFontSize() const { return m_FontSize; }
	virtual void SetEntityProperties_Interface(EntityPropertiesInterface *entity);
	virtual void UpdateScene_Additional(osg::Geode *geode, bool AddOrRemove);
	UI_GameClient *GetParent() { return m_pParent; }
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
	GG_Framework::UI::MainWindow *m_MainWin;
	//Exposing our node to render text
	osg::ref_ptr<osg::Group> m_RootNode;
	osg::ref_ptr<osg::Geode> m_Geode;
	Viewer_Callback_Interface *m_Callback;

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
			traverse(node, nv);
		}
	public:
		ViewerCallback(Viewer *parent) : m_pParent(parent) {}
	};
	osg::ref_ptr <ViewerCallback> m_ViewerCallback;
	//This will make all time deltas the same length (ideal for debugging)
	bool m_UseSyntheticTimeDeltas;
	bool m_UseUserPrefs;
public:
	Viewer(bool useUserPrefs = true) :m_Callback(NULL), m_UseSyntheticTimeDeltas(false), m_UseUserPrefs(useUserPrefs) {}
	void SetCallbackInterface(Viewer_Callback_Interface *callback) { m_Callback = callback; }
	void Start()
	{
		using namespace Robot_Tester;
		using namespace GG_Framework::Base;
		using namespace GG_Framework::UI;

		// Content Directory
		//char contentDIR[512];
		//_getcwd(contentDIR, 512);
		//printf("Content Directory: %s\n", contentDIR);

		//Audio::ISoundSystem* sound = new Audio::SoundSystem_Mock();

		// Create a new scope, so all auto-variables will be deleted when they fall out
		{
			// Create the singletons, These must be instantiated before the scene manager too - Rick
			m_MainWin = new MainWindow(true, 0.0, 0, 0, m_UseUserPrefs);
			MainWindow &mainWin = *m_MainWin;

			// We are going to use this single timer to fire against
			OSG::OSG_Timer timer("OSGV Timer Log.csv");
			//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ToggleFrameLog", osgGA::GUIEventAdapter::KEY_F7);
			//timer.Logger.ListenForToggleEvent(mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ToggleFrameLog"]);

			// Create the scene manager with each of the files
			osg::Group* mainGroup = NULL;

			// Here is a good example of setting up an easy text PDCB
			Framerate_PDCB* fpdcb = new Framerate_PDCB(mainWin);
			//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ShowFR", osgGA::GUIEventAdapter::KEY_F2);
			mainWin.GetMainCamera()->addPostDrawCallback(*fpdcb);
			//mainWin.GetKeyboard_Mouse().GlobalEventMap.Event_Map["ShowFR"].Subscribe(
			//	fpdcb->ehl, (Text_PDCB&)(*fpdcb), &Text_PDCB::ToggleEnabled);

			// And a debug one
			DebugOut_PDCB* dpdcb = new DebugOut_PDCB(mainWin);
			//mainWin.GetKeyboard_Mouse().AddKeyBindingR(false, "ShowDebug", osgGA::GUIEventAdapter::KEY_F9);
			mainWin.GetMainCamera()->addPostDrawCallback(*dpdcb);
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
			// make sure the root node is group so we can add extra nodes to it.
			osg::Group* group = new osg::Group;
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

			// Connect the argParser to the camera, in case it wants to handle stats (I do not know that I like this here)
			//argParser.AttatchCamera(&mainWin, &timer);

			// Loop while we are waiting for the Connection to be made and all of the scenes
			double dTime_s = 0.0;
			double currTime = 0.0;
			do
			{
				if (!m_UseSyntheticTimeDeltas)
					dTime_s = timer.FireTimer();
				else
				{
					//dTime_s = 0.016;  //hard code a typical 60 fps
					dTime_s = 0.010;  //Testing robot autonomous loop
				}

				currTime = timer.GetCurrTime_s();
				if (m_Callback)
				{
					m_Callback->UpdateData(dTime_s);
					m_Callback->UpdateScene(m_RootNode, m_Geode);
				}
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
		m_Viewer->Start();
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

#pragma region _Test Text_
void createHUDText(osg::Group *rootNode, osg::Geode* geode)
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

class OSG_Viewer_Internal
{
private:
	#pragma region _members_
	std::shared_ptr<GUIThread> m_UI_thread = nullptr;
	#pragma endregion
	void init()
	{
		if (!m_UI_thread)
			m_UI_thread = std::make_shared<GUIThread>();
	}
public:
	OSG_Viewer_Internal()
	{
	}
	void Test(size_t index)
	{
		switch (index)
		{
		case 0:
		{
			using namespace GG_Framework::Base;
			init();
			TestCallback test;
			m_UI_thread->Init(&test);
			assert(m_UI_thread);
			size_t TimeOut = 0;
			while (!test.GetIsSetup() && TimeOut++ < 100)
				ThreadSleep(200);
			m_UI_thread->GetUI()->SetCallbackInterface(NULL);
		}
			break;
		}
	}
};

#pragma region _wrapper methods_
OSG_Viewer::OSG_Viewer()
{
	m_OSG_Viewer = std::make_shared<OSG_Viewer_Internal>();
}
void OSG_Viewer::Test(size_t index)
{
	m_OSG_Viewer->Test(index);
}

#pragma endregion

}
#pragma endregion

