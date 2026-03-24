#pragma once
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "DirectAutonChainLog.h"

#define Robot_TesterCode
__inline bool Auton_Smart_GetSingleValue_Bool(const char* SmartName, bool default_value)
{
	bool result = default_value;
	//Can't use try catch on cRIO since Thunder RIO has issue with using catch(...)
	//RoboRio uses SetDefault*() to accomplish same effect
	//Simulation can use try catch method, but we could modify smart dashboard to allow using the new method
#if defined Robot_TesterCode
	auto tryGetBool = [&](const char* key, bool& out) -> bool
	{
		try
		{
			out = SmartDashboard::GetBoolean(key);
			return true;
		}
		catch (...)
		{
		}

		try
		{
			const std::string str_value = SmartDashboard::GetString(key);
			if (str_value == "1" || str_value == "true" || str_value == "TRUE" || str_value == "True")
			{
				out = true;
				return true;
			}
			if (str_value == "0" || str_value == "false" || str_value == "FALSE" || str_value == "False")
			{
				out = false;
				return true;
			}
		}
		catch (...)
		{
		}

		return false;
	};

	const std::string scoped_name = std::string("Test/") + SmartName;
	try
	{
		result = SmartDashboard::GetBoolean(SmartName);
	}
	catch (...)
	{
		bool temp = default_value;
		if (tryGetBool(scoped_name.c_str(), temp) || tryGetBool(SmartName, temp))
			result = temp;
		else
			result = default_value;
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
	auto tryGetNumber = [&](const char* key, double& out) -> bool
	{
		// Ian: In Direct mode, a missing number must stay "missing" instead of
		// looking like a valid zero. Use TryGet* first so auton commands do not
		// silently collapse to 0-distance moves.
		if (::SmartDashboard::TryGetNumber(key, out))
			return true;

		std::string str_value;
		if (::SmartDashboard::TryGetString(key, str_value))
		{
			out = atof(str_value.c_str());
			return true;
		}

		return false;
	};

	const std::string scoped_name = std::string("Test/") + SmartName;
	// Ian: Prefer the scoped key first because SmartDashboard control widgets in
	// simulator layouts often publish as `Test/<key>`, but keep the flat alias
	// for legacy compatibility.
	if (SmartName && std::string(SmartName) == "TestMove")
	{
		char dbg[256] = {};
		sprintf_s(dbg, "[Auton_Smart_GetSingleValue] reading '%s' scoped='%s'", SmartName, scoped_name.c_str());
		Module::Input::AppendDirectAutonChainLog(dbg);
	}
	bool hasValue = false;
	hasValue = tryGetNumber(scoped_name.c_str(), result) || tryGetNumber(SmartName, result);
	if (SmartName && std::string(SmartName) == "TestMove")
	{
		char dbg[256] = {};
		sprintf_s(dbg, "[Auton_Smart_GetSingleValue] result '%s' has=%d value=%g", SmartName, hasValue ? 1 : 0, result);
		Module::Input::AppendDirectAutonChainLog(dbg);
	}
	if (!hasValue)
		result = default_value;
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
	auto tryGetNumber = [&](const char* key, double& out) -> bool
	{
		if (::SmartDashboard::TryGetNumber(key, out))
			return true;

		std::string str_value;
		if (::SmartDashboard::TryGetString(key, str_value))
		{
			out = atof(str_value.c_str());
			return true;
		}

		return false;
	};

	for (size_t i = 0; i < NoItems; i++)
	{
		const std::string scoped_name = std::string("Test/") + SmartNames[i];
		double temp = *(SmartVariables[i]);
		if (tryGetNumber(scoped_name.c_str(), temp) || tryGetNumber(SmartNames[i], temp))
			*(SmartVariables[i]) = temp;
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
