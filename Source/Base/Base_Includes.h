#pragma once

#ifndef _Win32
typedef long long __int64;
#else
// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#define _CRT_SECURE_NO_WARNINGS
#endif

//typedef unsigned long size_t;
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <assert.h>
#include <string.h>
