#pragma region _includes_
#include "stdafx.h"
#include <list>
#include <conio.h>

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


#pragma endregion
//Robot Tester.h
#pragma region _GG_FrameWork_UI_
#pragma region _Useful Constants_
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
#include <OpenThreads/Mutex>
#include <OpenThreads/Thread>

namespace GG_Framework
{
	namespace Base
	{
		FRAMEWORK_BASE_API void ThreadSleep(unsigned sleepMS);

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
			virtual void run();
			static bool ERROR_STATE;

		protected:
			virtual void tryRun() = 0;
		};
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

		FRAMEWORK_BASE_API std::string BuildString(const char *format, ...);
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
			FrameLogger(std::string logFileName);
			~FrameLogger();
			void WriteLog();

			//! Called between time updates, with prior time and current time, ONLY when active
			Event2<double, double> TimerEvent;

			//! Write out your title, using a comma before 
			Event1<FILE*> WriteTitles;
			Event3<FILE*, double, double> WriteValues;

			void ToggleActive() { m_active = !m_active; }
			bool IsActive() { return m_active; }

			void ListenForToggleEvent(Event0& ev);
			void ListenForTimerUpdate(Event1<double>& timerEv);

		private:
			void TimeChanged(double t);
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
			ProfilingLogger(std::string itemName);

			// Call these to write on the next frame
			void Start();
			void End();

		private:
			__int64 begC, endC, freq;
		};
		//////////////////////////////////////////////////////////////////////////

		class FRAMEWORK_BASE_API Timer
		{
		public:
			Timer(std::string logFileName);
			virtual ~Timer();

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
			double SetCurrTime_s(double t_s);
			double IncTime_s(double i_s);

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
				GLuint loadFont(Font font);

				bool createStrokeFont(GLuint fontBase);
				bool createOutlineFont(GLuint fontBase);
				bool createFilledFont(GLuint fontBase);
				bool createBitmapFont(GLuint fontBase);
			public:
				Text();
				void drawString(Font, std::string);
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

				virtual void SetCameraManipulator(ICameraManipulator* cameraManip);
				Event3<ICamera*, ICameraManipulator*, ICameraManipulator*> CamManipChanged; // (this, old, new)
				ICameraManipulator* GetCameraManipulator() { return m_camManip; }

				virtual void Update(double dTime_s);
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
				OSG_Timer(std::string logFileName);
				virtual ~OSG_Timer();
				virtual double FireTimer();

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
				VectorDerivativeOverTimeAverager(unsigned numSamples);
				~VectorDerivativeOverTimeAverager();
				osg::Vec3 GetVectorDerivative(osg::Vec3 vec, double dTime_s);
				void Reset() { m_currIndex = (unsigned)-1; }
			};
		}
	}
}
#pragma endregion
//#include "LoadStatus.h"

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

			virtual void operator () (const osg::Camera &cam) const;

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

		class Camera : public GG_Framework::UI::OSG::ICamera
		{
		private:
			osg::ref_ptr<osgViewer::Viewer> m_camGroup;
			class CompositeDrawCallback : public osg::Camera::DrawCallback
			{
			protected:
				virtual ~CompositeDrawCallback();
			public:
				Ref_List_impl CallbackList;
				mutable OpenThreads::Mutex MyMutex;
				virtual void operator () (const osg::Camera& camera) const;
			};
			osg::ref_ptr<CompositeDrawCallback> m_compositePostDrawCallback;
			DistortionSubgraph* m_distortion;

		public:
			Camera(osgViewer::Viewer* camGroup);
			virtual void setClearColor(const osg::Vec4& c);
			virtual void getClearColor(osg::Vec4& c);

			virtual void addPostDrawCallback(osg::Camera::DrawCallback& cb);
			virtual void removePostDrawCallback(osg::Camera::DrawCallback& cb);

			virtual void setFinalDrawCallback(osg::Camera::DrawCallback* cb);

			virtual void SetMatrix(const osg::Matrix& camMatrix, float distortionAmt);
			virtual osg::Matrix GetCameraMatrix() const;
			virtual osg::Node* GetSceneNode();
			virtual void SetSceneNode(osg::Node* node, float distortionPCNT);
		};
	}
}
#pragma endregion
//#include "ConfigurationManager.h"
//#include "JoystickBinder.h"
//#include "KeyboardMouse_CB.h"
//#include "MainWindow.h"
#pragma region _Main Window_
namespace GG_Framework
{
	namespace UI
	{
		class FRAMEWORK_UI_API Window
		{
		public:
			// Use this block for all threading updates
			static OpenThreads::Mutex UpdateMutex;
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

			Window(bool useAntiAlias, unsigned screenWidth, unsigned screenHeight, bool useUserPrefs);
			virtual ~Window();

			void SetWindowRectangle(int x, int y, int w, int h, bool resize);
			void GetWindowRectangle(int& x, int& y, unsigned& w, unsigned& h);
			void SetFullScreen(bool fs);
			bool IsFullScreen();
			void SetWindowText(const char* windowTitle);
			bool IsAntiAliased() { return m_useAntiAlias; }
			virtual void Realize();
			virtual bool Update(double currTime_s, double dTick_s);
			virtual double GetThrottleFPS() { return 0.0; }

			double GetFrameRate() { return m_frameRate; }
			double GetAverageFramerate() { return m_frameRateAvg; }
			int GetPerformanceIndex() { return m_performanceIndex; }
			Event2<int, int> PerformanceIndexChange; //! provides old and new values, smaller means struggling
			GG_Framework::UI::OSG::ICamera* GetMainCamera();

			// Used for performance indexing
			static double PERFORMANCE_MIN;
			static double PERFORMANCE_MAX;
			static int PERFORMANCE_INIT_INDEX;

			// Work with the Cursor, we will eventually be able to manipulate the cursor itself
			void UseCursor(bool flag);

			// Call this just as we are starting the main loop to enable the camera and get one more frame in
			void EnableMouse();

			// Position the pointer explicitly
			void PositionPointer(float x, float y);

			/** compute, from normalized mouse coords (x,y) the,  for the specified
			* RenderSurface, the pixel coordinates (pixel_x,pixel_y). return true
			* if pixel_x and pixel_y have been successful computed, otherwise return
			* false with pixel_x and pixel_y left unchanged.*/
			bool ComputePixelCoords(float x, float y, float& pixel_x, float& pixel_y);

			ThreadSafeViewer* GetViewer() { return m_camGroup.get(); }

			//Note: we strip out input
			//KeyboardMouse_CB &GetKeyboard_Mouse() const { return *m_Keyboard_Mouse; }
			//JoyStick_Binder &GetJoystick() const { return *m_Joystick; }

		protected:
			osgViewer::GraphicsWindow* GetGraphicsWindow();
			bool m_useAntiAlias;
			osg::ref_ptr<ThreadSafeViewer>	m_camGroup;
			bool m_isFullScreen;
			unsigned m_origScreenWidth, m_origScreenHeight;
			unsigned m_newScreenWidth, m_newScreenHeight;
			int m_lastX, m_lastY;
			int m_lastWidth, m_lastHeight;

			Camera m_mainCam;

			// Used for calculating frame-rate
			void UpdateFrameRate(double currTime_s, double lastDrawnFrameDur);
			double m_frameRate;
			double m_frameRateAvg;
			Averager<double, 30> m_framerateAverager;
			double m_lastFrameRateCheckTime_s;
			int m_frameNum;
			int m_performanceIndex;
			double m_waitForPerformanceIndex_s;
			int m_performanceStrikes;

			bool UpdateCameraGroup(double currTime_s);
			void UpdateSound(double dTick_s);
			GG_Framework::UI::OSG::VectorDerivativeOverTimeAverager m_velocityAvg;

		private:
			// Attach to events with this, also change where the callbacks go
			//osg::ref_ptr<KeyboardMouse_CB> refKBM;
			//KeyboardMouse_CB *m_Keyboard_Mouse;
			//JoyStick_Binder *m_Joystick;					// Scoped pointer.
			//AudioVolumeControls_PlusBLS *m_VolumeControls;	// Scoped pointer.

			//TODO see if we need this
			//ConfigurationManager m_ConfigurationManager;

		};


		class FRAMEWORK_UI_API MainWindow : public Window
		{
		public:
			MainWindow(bool useAntiAlias, double throttle_fps, unsigned screenWidth, unsigned screenHeight, bool useUserPrefs);
			virtual bool Update(double currTime_s, double dTick_s);
			virtual ~MainWindow();
			virtual void Realize();
			bool IsQuitting() { return m_quitting; }
			void ToggleFullScreen() { SetFullScreen(!IsFullScreen()); }
			void TryClose();

			Event2<MainWindow*, bool&> Closing;
			Event2<MainWindow*, bool&> EscapeQuit;
			Event1<MainWindow*> Close;

			// Here is the singleton
			static MainWindow* GetMainWindow() { return s_mainWindow; }

			// This is public so we can tie events to it
			IEvent::HandlerList ehl;

			virtual double GetThrottleFPS() { return m_throttleFPS; }
			void SetThrottleFPS(double throttleFPS);

			GG_Framework::Base::ProfilingLogger UpdateTimerLogger;

		private:
			void OnEscape();
			static MainWindow* s_mainWindow;
			bool m_quitting;
			osg::Timer m_throttleTimer;
			double m_throttleFPS;

			void ThrottleFrame();

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
		};
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

			virtual std::string GetText() const;
			virtual osg::Vec2 GetPosition(double winWidth, double winHeight) const { return osg::Vec2(10.0, winHeight - 20.0); }
			virtual void operator () (const osg::Camera &cam) const;

			// To populate this with a formatted string, Use
			// GG_Framework::UI::DebugOut_PDCB::TEXT1 = GG_Framework::Base::BuildString(format, ...);
			static std::string TEXT1;
			static std::string TEXT2;
			static std::string TEXT3;
			static std::string TEXT4;
			static std::string TEXT5;
		};
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
