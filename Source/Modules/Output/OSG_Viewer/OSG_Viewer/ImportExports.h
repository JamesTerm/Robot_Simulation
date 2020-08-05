#pragma once

// DLL export warning
#pragma warning ( disable : 4251 ) 
#pragma warning ( disable : 4273 ) 
#pragma warning ( disable : 4275 ) 

#ifndef OSGVIEW_LIB
#ifdef OSGVIEW_EXPORTS
#define OSG_View_API __declspec(dllexport)
#else
#define OSG_View_API __declspec(dllimport)
#endif
#else
#define OSG_View_API
#endif
