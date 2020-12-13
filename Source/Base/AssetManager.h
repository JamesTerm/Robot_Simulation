#pragma once

#pragma region _includes macros_
#include "Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "Vec2D.h"
#include "Misc.h"

// Standard C++ support
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <bitset>
#include <map>
#include <algorithm>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>
#pragma endregion

#pragma region _Description_
// This is a light weight "mail man" that allows one module to communicated assets to another module in a way that is easy,
// encapsulated, and simple.  To accomplish this there are redundant methods to make it easy to tailor to the clients current
// setup.  Since the underlying tasks are simple and we restrict parameters to primitive types it makes it possible to have
// several redundant methods which are similar to the methods in SmartDashboard.

#pragma endregion

namespace Framework
{
	namespace Base
	{

class asset_manager
{
private:
	std::map<std::string,double> m_number_database;
	std::map<std::string,bool> m_bool_database;
public:
	#pragma region _bool methods_
	void put_bool(const char* keyName, bool value)
	{
		m_bool_database[keyName] = value;
	}
	bool get_bool_native(const char* keyName, bool& value) const
	{
		bool ret = false;
		auto element = m_bool_database.find(keyName);
		if (element != m_bool_database.end())
		{
			value = (*element).second;
			ret = true;
		}
		return ret;
	}
	bool get_bool(const char* keyName, bool default_value=false) const
	{
		bool test;
		bool ret = get_bool_native(keyName , test) ? test : default_value;
		return ret;
	}
	#pragma endregion
	#pragma region _number methods_
	void put_number(const char* keyName, double value)
	{
		m_number_database[keyName] = value;
	}
	bool get_number_native(const char* keyName, double& value) const
	{
		bool ret = false;
		auto element = m_number_database.find(keyName);
		if (element != m_number_database.end())
		{
			value = (*element).second;
			ret = true;
		}
		return ret;
	}
	double get_number(const char* keyName, double default_value = 0.0) const
	{
		double test;
		double ret = get_number_native(keyName, test) ? test : default_value;
		return ret;
	}
	#pragma region _other primative number types to cast_
	//Note: we can add put casts here if needed as well
	int get_number_int(const char* keyName, int default_value = 0) const
	{
		return (int)get_number(keyName,(double)default_value);
	}
	size_t get_number_size_t(const char* keyName, size_t default_value = 0) const
	{
		return (size_t)get_number(keyName, (double)default_value);
	}
	#pragma endregion
	#pragma endregion
	#pragma region _string methods_
	//TODO
	void put_string(const char* keyName, const char* value)
	{}
	const char* get_string(const char* keyName) const
	{
		return nullptr;
	}
	const char* get_string(const char* keyName, const char* defaultValue) const
	{
		return nullptr;
	}
	#pragma endregion
};

	}
}
