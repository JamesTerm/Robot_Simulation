#pragma once

/* /// Here is a quick handy function to get a time stamp using the QueryPerformanceCounter
double FRAMEWORK_BASE_API get_current_time( void );*/

///Here is a container which houses time particularly for time stamping operations.  This natively stores the value as a fixed precision of
///10,000,000 to 1 to a 64bit integer (i.e. __int64).  Overloaded operators have been provided for ease of conversion between various types
///The two primary types are __int64 and double.  \note This fixed precision standard is native to Direct Show sometimes in the form of
///LARGE_INTEGER
struct time_type
{
	public:
		/// \note The default does not do anything except initialize the internal time to 0xcdcdcdcd
		time_type();
		time_type(double NewValue);
		time_type(__int64 NewValue);
		time_type(const time_type &NewValue);
		/// \todo add a format spec for timecode
		//time_type(wchar_t *format=NULL)
		operator double ( void ) const;
		operator __int64 ( void ) const;
		operator __int64* ( void );
		//TODO timecode
		//operator const wchar_t*( void ) const;
		//The copy constructor
		time_type& operator= (const time_type & NewValue);
		void operator= (double NewValue);
		void operator= (__int64 NewValue);

		time_type operator- (const time_type &NewValue) const;
		time_type operator+ (const time_type &NewValue) const;
		void operator+= (const __int64 &NewValue);
		void operator+= (double NewValue);
		void operator+= (const time_type &NewValue);
		void operator-= (const __int64 &NewValue);
		void operator-= (double NewValue);
		void operator-= (const time_type &NewValue);
		bool operator>  (const time_type &Value) const;
		bool operator>= (const time_type &Value) const;
		bool operator<  (const time_type &Value) const;
		bool operator<= (const time_type &Value) const;

		///This will produce a time stamp of system time (i.e. QueryPerformanceCounter) and offers it in a form that is compliant to this type
		///In a way that minimizes precision loss (int64 division + remainder)
		static time_type get_current_time();
	private:
		///There are 10,000,000 (10 million) units for one second 
		__int64 m_Time;
};
