#include <string>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _Win32
// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#define _CRT_SECURE_NO_WARNINGS
#endif

namespace Framework
{
namespace Base
{

void DebugOutput(const char *format, ... )
{
	va_list marker;
	va_start(marker,format);
		static char Temp[2048];
		vsprintf(Temp,format,marker);
		//OutputDebugString(Temp);
	va_end(marker); 
}

std::string BuildString(const char *format, ... )
{
	char Temp[2048];
	va_list marker;
	va_start(marker,format);
	vsprintf(Temp,format,marker);
	va_end(marker); 
	std::string ret(Temp);
	return ret;
}
//////////////////////////////////////////////////////////////////////////

char* GetLastSlash(char* fn, char* before)
{
	if (!fn) return NULL;
	char* lastSlash = before ? before-1 : fn+strlen(fn);

	while (lastSlash > fn)
	{
		if ((*lastSlash == '/') || (*lastSlash == '\\'))
			return lastSlash;
		--lastSlash;
	}

	return NULL;
}
//////////////////////////////////////////////////////////////////////////

//! Returns false iff c == [ 'f', 'F', 'n', 'N', '0', 0 ]
bool ParseBooleanFromChar(char c)
{
	c = toupper(c);
	if ((c == 'F') || (c == 'N') || (c == '0') || (c == 0))
		return false;
	else
		return true;
}
//////////////////////////////////////////////////////////////////////////

void StripCommentsAndTrailingWhiteSpace(char* line)
{
	for (char* eol = line; ; ++eol)
	{
		if ((eol[0] == '\n') || (eol[0] == '\r') || (eol[0] == '#') || (eol[0] == '\0'))
		{
			eol[0] = '\0';
			--eol;
			while ((eol >= line) && ((eol[0]==' ') || (eol[0]=='\t')))
			{
				eol[0] = '\0';
				--eol;
			}
			return;
		}
	}
}

	}
}


#ifndef _Win32
//from http://www.koders.com/c/fid5F9B1CF12E947E5030A132D309A367C5CCB671CE.aspx
char *itoa (int value, char *string, int radix)
{
  char tmp[33];
  char *tp = tmp;
  int i;
  unsigned v;
  int sign;
  char *sp;

  if (radix > 36 || radix <= 1)
  {
    //__set_errno(EDOM);
    return 0;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
    v = -value;
  else
    v = (unsigned)value;
  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  if (string == 0)
    string = (char *)malloc((tp-tmp)+sign+1);
  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;
  return string;
}
#endif