#pragma once
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#define Robot_TesterCode
__inline bool Auton_Smart_GetSingleValue_Bool(const char* SmartName, bool default_value)
{
	bool result = default_value;
	//Can't use try catch on cRIO since Thunder RIO has issue with using catch(...)
	//RoboRio uses SetDefault*() to accomplish same effect
	//Simulation can use try catch method, but we could modify smart dashboard to allow using the new method
#if defined Robot_TesterCode
	try
	{
		result = SmartDashboard::GetBoolean(SmartName);
	}
	catch (...)
	{
		//set up some good defaults for a small box
		SmartDashboard::PutBoolean(SmartName, default_value);
	}
#else
#if !defined __USE_LEGACY_WPI_LIBRARIES__
	SmartDashboard::SetDefaultBoolean(SmartName, default_value);
	result = SmartDashboard::GetBoolean(SmartName);
#else
//for cRIO checked in using zero in lua (default) to prompt the variable and then change to -1 to use it
	if (!SmartDashboard::GetBoolean("TestVariables_set"))
		SmartDashboard::PutBoolean(SmartName, default_value);
	else
		result = SmartDashboard::GetBoolean(SmartName);
#endif
#endif
	return result;
}

__inline void Auton_Smart_GetMultiValue_Bool(size_t NoItems, const char* const SmartNames[], bool* const SmartVariables[])
{
	//Remember can't do this on cRIO since Thunder RIO has issue with using catch(...)
#if defined Robot_TesterCode
	for (size_t i = 0; i < NoItems; i++)
	{
		try
		{
			*(SmartVariables[i]) = SmartDashboard::GetBoolean(SmartNames[i]);
		}
		catch (...)
		{
			//I may need to prime the pump here
			SmartDashboard::PutBoolean(SmartNames[i], *(SmartVariables[i]));
		}
	}
#else
#if !defined __USE_LEGACY_WPI_LIBRARIES__
	for (size_t i = 0; i < NoItems; i++)
	{
		SmartDashboard::SetDefaultBoolean(SmartNames[i], *(SmartVariables[i]));
		*(SmartVariables[i]) = SmartDashboard::GetBoolean(SmartNames[i]);
	}
#else
	for (size_t i = 0; i < NoItems; i++)
	{
		if (SmartDashboard::GetBoolean("TestVariables_set"))
			*(SmartVariables[i]) = SmartDashboard::GetBoolean(SmartNames[i]);
		else
			SmartDashboard::PutBoolean(SmartNames[i], *(SmartVariables[i]));
	}
#endif
#endif
}


__inline double Auton_Smart_GetSingleValue(const char* SmartName, double default_value)
{
	double result = default_value;
	//Can't use try catch on cRIO since Thunder RIO has issue with using catch(...)
	//RoboRio uses SetDefault*() to accomplish same effect
	//Simulation can use try catch method, but we could modify smart dashboard to allow using the new method
#if defined Robot_TesterCode
	try
	{
		result = SmartDashboard::GetNumber(SmartName);
	}
	catch (...)
	{
		//set up some good defaults for a small box
		SmartDashboard::PutNumber(SmartName, default_value);
	}
#else
#if !defined __USE_LEGACY_WPI_LIBRARIES__
	SmartDashboard::SetDefaultNumber(SmartName, default_value);
	result = SmartDashboard::GetNumber(SmartName);
#else
//for cRIO checked in using zero in lua (default) to prompt the variable and then change to -1 to use it
	if (!SmartDashboard::GetBoolean("TestVariables_set"))
		SmartDashboard::PutNumber(SmartName, default_value);
	else
		result = SmartDashboard::GetNumber(SmartName);
#endif
#endif
	return result;
}

__inline void Auton_Smart_GetMultiValue(size_t NoItems, const char* const SmartNames[], double* const SmartVariables[])
{
	//Remember can't do this on cRIO since Thunder RIO has issue with using catch(...)
#if defined Robot_TesterCode
	for (size_t i = 0; i < NoItems; i++)
	{
		try
		{
			*(SmartVariables[i]) = SmartDashboard::GetNumber(SmartNames[i]);
		}
		catch (...)
		{
			//I may need to prime the pump here
			SmartDashboard::PutNumber(SmartNames[i], *(SmartVariables[i]));
		}
	}
#else
#if !defined __USE_LEGACY_WPI_LIBRARIES__
	for (size_t i = 0; i < NoItems; i++)
	{
		SmartDashboard::SetDefaultNumber(SmartNames[i], *(SmartVariables[i]));
		*(SmartVariables[i]) = SmartDashboard::GetNumber(SmartNames[i]);
	}
#else
	for (size_t i = 0; i < NoItems; i++)
	{
		if (SmartDashboard::GetBoolean("TestVariables_set"))
			*(SmartVariables[i]) = SmartDashboard::GetNumber(SmartNames[i]);
		else
			SmartDashboard::PutNumber(SmartNames[i], *(SmartVariables[i]));
	}
#endif
#endif
}

#undef Robot_TesterCode
