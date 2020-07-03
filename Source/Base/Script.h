
#pragma once

struct lua_State;

namespace Framework
{
	namespace Scripting
	{


//This class abstracts away some of the interesting LUA syntax and makes LUA easier to use. 
class Script
{
public:
	Script();
	~Script();

	//! Returns an error string if there was a problem
	//! ASSERTS that the script is not already valid
	//! Read from a file or a buffer
	const char* LoadScript(const char* fileName_or_buff, bool file);

	const char* GetFilename(){return m_filename.c_str();}

	//! Close it, however it was read
	void CloseScript();

	bool IsValid(){return (m_lua_state != NULL);}

	//! Pass in a pointer to the type of variable you want.
	//! if the global exists and casts to that type, the return value will be
	//! populated.  You CAN have more than one return type, as long as the values
	//! cast to the appropriate value.  Everything can cast to a String.
	//! any numeric type can cast to a float, etc.  Use NULL for the positions
	//! where you are not interested in the result.  If there is a not a global value,
	//! or if the value cannot be cast to one of the non-null positions, the
	//! function will return false.  In the case of multiple values, and there is
	//! a global and it can cast to SOME of the values, but not all of them,
	//! the function will return false, and NONE of the values will be set.
	const char* GetGlobal(std::string globalName, 
		std::string* t_string=NULL, bool* t_boolean=NULL, double* t_number=NULL);

	const char* GetGlobalTable(std::string globalTableName);

	// Note that all of these functions assume that a Table is at pos index
	// Most of the time the table is at the top of the stack, at pos -1.
	const char* GetField(std::string fieldName, 
		std::string* t_string=NULL, bool* t_boolean=NULL, double* t_number=NULL);
	const char* GetFieldTable(std::string fieldTableName);
	const char* GetIndex(unsigned index, 
		std::string* t_string=NULL, bool* t_boolean=NULL, double* t_number=NULL);
	const char* GetIndexTable(unsigned index);

	//! Call a function using a varg list
	//! e.g.: double z; CallFunction("f", "dd>d", 1.0, 2.0, &z);
	//! For params: d=double, i=int, s=char*
	//! For returns: d=double*, i=int*, s=char**, t=(Event1<Script*>*
	const char* CallFunction(const char *func, const char *sig, ...);

	//! Call this when you are done with a Table.  It pops the Table off the stack
	void Pop();

	//! Use this to map the names of items in the script
	//! e.g. NameMap["EXISTING_ENTITIES"] = "EXISTING_SHIPS"
	std::map<std::string,std::string,std::greater<std::string> > NameMap;
	std::string GetMappedName(std::string name)
	{
		if (NameMap[name].empty())
			NameMap[name] = name;
		return NameMap[name];
	}

private:
	std::string m_filename;
	const char* GetStackValue(
		std::string* t_string=NULL, bool* t_boolean=NULL, double* t_number=NULL);
	lua_State* m_lua_state;
	std::vector<std::string*> m_errBuffs;
	const char* CacheError(std::string err, int pop);
};

	}
}

namespace Scripting=Framework::Scripting;
