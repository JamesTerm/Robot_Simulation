#ifdef _Win32
#include <windows.h>
#else
#include "frc/Timer.h"
using namespace frc;
#endif

#include "Base_Includes.h"
#include "Time_Type.h"

//using namespace FrameWork;

  /*******************************************************************************************************/
 /*												time_type												*/
/*******************************************************************************************************/

time_type::time_type()
{
	m_Time=0xcdcdcdcd;
}
time_type::time_type(double NewValue)
{
	*this=NewValue;
}
time_type::time_type(__int64 NewValue)
{
	*this=NewValue;
}

time_type::time_type(const time_type &NewValue)
{
	m_Time=NewValue.m_Time;
}

time_type::operator double ( void ) const
{
	return m_Time/10000000.0;
}
time_type::operator __int64 ( void ) const
{
	return m_Time;
}
time_type::operator __int64* ( void )
{
	return &m_Time;
}

void time_type::operator= (double NewValue)
{
	m_Time=(__int64)(NewValue*10000000.0);
}
void time_type::operator= (__int64 NewValue)
{
	m_Time=NewValue;
}
time_type& time_type::operator= (const time_type & NewValue)
{
	m_Time=NewValue.m_Time;
	return *this;
}
time_type time_type::operator- (const time_type &NewValue) const
{
	time_type Temp(*this);
	Temp-=NewValue;
	return Temp;
	//return m_Time-NewValue.m_Time;
}

time_type time_type::operator+ (const time_type &NewValue) const
{
	time_type Temp(*this);
	Temp+=NewValue;
	return Temp;
	//return m_Time+NewValue.m_Time;
}
void time_type::operator+= (const __int64 &NewValue)
{
	m_Time+=NewValue;
}
void time_type::operator+= (double NewValue)
{
	m_Time+=(__int64)(NewValue*10000000.0);
}
void time_type::operator+= (const time_type &NewValue)
{
	m_Time+=(__int64)NewValue;
}
void time_type::operator-= (const __int64 &NewValue)
{
	m_Time-=NewValue;
}
void time_type::operator-= (double NewValue)
{
	m_Time-=(__int64)(NewValue*10000000.0);
}
void time_type::operator-= (const time_type &NewValue)
{
	m_Time-=(__int64)NewValue;
}
bool time_type::operator> (const time_type &Value) const
{
	return (__int64)m_Time>(__int64)Value;
}
bool time_type::operator>= (const time_type &Value) const
{
	return (__int64)m_Time>=(__int64)Value;
}

bool time_type::operator< (const time_type &Value) const
{
	return (__int64)m_Time<(__int64)Value;
}
bool time_type::operator<= (const time_type &Value) const
{
	return (__int64)m_Time<=(__int64)Value;
}

#ifdef _Win32
time_type time_type::get_current_time()
{
	LARGE_INTEGER freq, current;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&current);

	//Using const helps the compiler pull the remainder from the same IDIV instruction (for 64 bit)
	const __int64 TimeStamp_Divisor = (__int64)current.QuadPart;
	const __int64 TimeBase_Dividend = (__int64)freq.QuadPart;

	__int64 Quotient = (TimeStamp_Divisor / TimeBase_Dividend);
	const __int64 Remainder = (TimeStamp_Divisor%TimeBase_Dividend);

	//Now to scale the integer and remainder to 10 microsecond units
	const __int64 TimeTypeUnitBase = 10000000;
	Quotient *= TimeTypeUnitBase;
	__int64 RemainderScaled = (Remainder*TimeTypeUnitBase) / TimeBase_Dividend;
	Quotient += RemainderScaled;
	return Quotient;
}

#else
time_type time_type::get_current_time()
{
	const double CurrentTime=GetTime();
	return CurrentTime;
}
#endif