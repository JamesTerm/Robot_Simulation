#pragma once
#include <math.h>
#include <stdlib.h>
#include <map>
#include <queue>

typedef std::map<std::string, std::string, std::greater<std::string> > StringMap;

//TODO fix
#if 0
#define ASSERT(cond) assert(cond);
#define ASSERT_MSG(cond, msg) if (!(cond)){printf((msg)); assert(cond);}
#else
#define ASSERT(cond)
#define ASSERT_MSG(cond, msg)
#endif

#ifndef _countof
#define _countof(x) sizeof(x)/sizeof(*x)
#endif

#ifndef _Win32
char *itoa(int value,char *string,int radix);
#endif

namespace Framework
{
	namespace Base
	{

		std::string BuildString(const char *format, ... );
		void DebugOutput(const char *format, ... );
		char* GetLastSlash(char* fn, char* before=NULL);
		std::string GetContentDir_FromFile(const char* fn);

		// Parses name=value pairs, stripping whitespace and comments starting with '#'
		// Returns true if file was opened properly
		bool ReadStringMapFromIniFile(std::string filename, StringMap& resMap);
		void StripCommentsAndTrailingWhiteSpace(char* line);
		std::string TrimString( const std::string& StrToTrim );

		//! Returns false iff c == [ 'f', 'F', 'n', 'N', '0', 0 ]
		bool ParseBooleanFromChar(char c);

		//! Trying to keep the Win32 FindFirstFile stuff wrapped
		//! -4 for neither file exists
		//! -3 the first file does not exist but the second does
		//! -2 the second file does not exist but the first does
		//! -1 First file time is earlier than second file time.
		//! 0 The times are the same
		//! 1 the second file time is earlier than the first
		int CompareFileLastWriteTimes(const char* f1, const char* f2);

		// Internal methods
		struct track_memory_impl
		{	static void add   ( void* p_data, const char* p_name );
			static void remove( void* p_data, const char* p_name );
		};
	}
}

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
	virtual ~Averager() {if (m_array) delete[] m_array;}
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
			int arrayIndex = (m_currIndex*-1)-1;
			m_array[arrayIndex] = newItem;

			// Still counting backwards unless we have filled all of the array
			if (arrayIndex == (NUMELEMENTS-1))	// This means we have filled the array
				m_currIndex = 0;				// Start taking from the array next time
			else
				--m_currIndex;

			// Return the average based on what we have counted so far
			return (m_sum / (arrayIndex+1));
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

	void Reset(){m_currIndex=-1;}

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
		if (m_Count==(size_t)-1)
		{
			m_CurrentItem=m_LastRequestedItem=newItem;
			m_Count=0;
		}
		//T ret=m_CurrentItem;   hmmm not used
		if (newItem!=m_CurrentItem)
		{
			if (newItem==m_LastRequestedItem)
			{
				m_Count++;
				if (m_Count>THRESHOLD)
					m_CurrentItem=newItem,m_Count=0;
			}
			else
				m_Count=0;
			m_LastRequestedItem=newItem;
		}
		return m_CurrentItem;
	}
private:
	size_t m_Count;
	T m_CurrentItem,m_LastRequestedItem;
};

//Here is a very light weight averager that uses the blend technique to obtain an average.  Designed to easily replace Averager.
template<class T>
class Blend_Averager
{
public:
	/// \param SmoothingValue a range from 0.0 - 1.0 representing percentage of new item to factor in  
	Blend_Averager(double SmoothingValue=0.5) : m_SmoothingValue(-1.0),m_DefaultSmoothingValue(SmoothingValue) {}
	//get and set smoothing value  (optional)
	//exposing this allows for dynamic smoothing
	double &GetSmoothingValue() {return m_SmoothingValue;}
	T operator()(T newItem)
	{
		if (m_SmoothingValue!=-1.0)
			m_CurrentValue=((newItem * m_SmoothingValue ) + (m_CurrentValue  * (1.0-m_SmoothingValue)));
		else
		{
			m_SmoothingValue=m_DefaultSmoothingValue;
			m_CurrentValue=newItem;
		}
		return m_CurrentValue;
	}
	void Reset(){m_SmoothingValue=-1;}
private:
	double m_SmoothingValue,m_DefaultSmoothingValue;
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
	Priority_Averager(size_t SampleSize, double PurgePercent) : m_SampleSize(SampleSize),m_PurgePercent(PurgePercent),
		m_CurrentBadApple_Percentage(0.0),m_Iteration_Counter(0)
	{
	}
	double operator()(double newItem)
	{
		m_queue.push(newItem);
		double ret=m_queue.top();
		if (m_queue.size()>m_SampleSize)
			m_queue.pop();
		//Now to manage when to purge the bad apples
		m_Iteration_Counter++;
		if ((m_Iteration_Counter % m_SampleSize)==0)
		{
			m_CurrentBadApple_Percentage+=m_PurgePercent;
			//printf(" p=%.2f ",m_CurrentBadApple_Percentage);
			if (m_CurrentBadApple_Percentage >= 1.0)
			{
				//Time to purge all the bad apples
				flush();
				m_queue.push(ret);  //put one good apple back in to start the cycle over
				m_CurrentBadApple_Percentage-=1.0;
				//printf(" p=%.2f ",m_CurrentBadApple_Percentage);
			}
		}
		return ret;
	}
};

template<class T>
__inline T Enum_GetValue(const char *value,const char * const Table[],size_t NoItems)
{
	//assert(value);  //If this fails... somebody forgot to enter a value 
	T ret=(T) -1;
	for (size_t i=0;i<NoItems;i++)
	{
		if (strcmp(value,Table[i])==0)
			ret=(T)i;
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

#define SCRIPT_INIT_SIZET_NoDefault(x,y) err=script.GetField(y, NULL, NULL, &fValue); \
	if (!err) x=(size_t)fValue; \
	else if (!NoDefaults) x=0;

#define SCRIPT_INIT_DOUBLE2_NoDefault(x,y,z) err=script.GetField(y, NULL, NULL, &fValue); \
	if (!err) x=fValue; \
	else if (!NoDefaults) x=z;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double Pi=M_PI;
const double Pi2=M_PI*2.0;
const double PI_2 = 1.57079632679489661923;

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
	double tol = (d2>0.0) ? d2*0.000001 : -d2*0.000001;
	double delta = (d1>d2) ? d1-d2 : d2-d1;
	return (delta < tol);
}
inline bool Equals(float d1, float d2)
{
	double tol = (d2>0.0f) ? d2*0.000001f : -d2*0.000001f;
	double delta = (d1>d2) ? d1-d2 : d2-d1;
	return (delta < tol);
}

//Use this to check for zero if your value is going to be used as a denominator in division
inline bool IsZero(double value,double tolerance=1e-5)
{
	return fabs(value)<tolerance;
}

inline double RAND_GEN(double minR = 0.0, double maxR = 1.0)
{
	return minR + (maxR-minR) * ((double)(rand()) / (double)(RAND_MAX));
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

namespace Base=Framework::Base;
namespace Entity2D_Kind=Framework::Base;
#define COMMON_API

