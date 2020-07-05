#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers


typedef   signed    char  int8_t;
typedef unsigned    char uint8_t;
typedef            short  int16_t;
typedef unsigned   short uint16_t;
typedef              int  int32_t;
typedef unsigned     int uint32_t;
typedef          __int64  int64_t;
typedef unsigned __int64 uint64_t;
typedef int                 BOOL;
typedef int 		(*FUNCPTR) (...);     /* ptr to function returning int */
#define NULL    0
typedef	int		STATUS;

// TODO: reference additional headers your program requires here
#include <intsafe.h>
#include <assert.h>

// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
//using 'this' in the base member's initializer list
#pragma warning ( disable : 4355 )
//I normally would not disable this... but for now I just need their stuff to build
#pragma warning ( disable : 4101 )
