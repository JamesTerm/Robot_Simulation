// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

typedef int(*FUNCPTR) (...);     /* ptr to function returning int */
typedef	int		STATUS;

#include <intsafe.h>
#include <assert.h>

// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
//using 'this' in the base member's initializer list
#pragma warning ( disable : 4355 )
//I normally would not disable this... but for now I just need their stuff to build
#pragma warning ( disable : 4101 )
