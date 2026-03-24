#if 1
#include "Base_Includes.h"
#include "Script.h"
#include "Misc.h"
#include "LUA.h"


  /***********************************************************************************************************/
 /*													Script													*/
/***********************************************************************************************************/

using namespace Framework::Scripting;

Script::Script() : m_filename("UN-INITIALIZED"),m_lua_state(NULL) {}
Script::~Script()
{
	CloseScript();
	unsigned numErrs = (unsigned)m_errBuffs.size();
	for (unsigned i = 0; i < numErrs; ++i)
		delete m_errBuffs[i];
}
//////////////////////////////////////////////////////////////////////////

void Script::CloseScript()
{
	if (m_lua_state)
	{
		lua_close(m_lua_state);
		m_lua_state = NULL;
	}
}
//////////////////////////////////////////////////////////////////////////


//! This function makes sure we are not using stuff off the LUA stack or our other buffers
const char* Script::CacheError(std::string err, int pop)
{
	std::string* errStr = new std::string(err);
	m_errBuffs.push_back(errStr);
	if (pop)
	{
		ASSERT(m_lua_state);
		lua_pop(m_lua_state, pop);
	}
	return errStr->c_str();
}
//////////////////////////////////////////////////////////////////////////



//! Returns an error string if there was a problem
//! ASSERTS that the script is not already valid
const char* Script::LoadScript(const char* fileName_or_buff, bool file)
{
	ASSERT(!IsValid());
	ASSERT(fileName_or_buff && fileName_or_buff[0]);

	m_filename = file ? fileName_or_buff : "BUFFER";

	const char* ret = NULL;
	m_lua_state = lua_open();
	luaL_openlibs(m_lua_state);
	//I shouldn't need this for wind river
	//Register_C_Function::RegisterAllFunctions(m_lua_state);

	// Load the script file and execute the globals
	int errLoading = file ? luaL_loadfile(m_lua_state, fileName_or_buff) : 
		luaL_loadbuffer(m_lua_state, fileName_or_buff, strlen(fileName_or_buff), "buffer");
	if (errLoading != 0)
		ret = CacheError(lua_tostring(m_lua_state, -1), 1);
	else if (lua_pcall(m_lua_state, 0, 0, 0) != 0)
		ret = CacheError(lua_tostring(m_lua_state, -1), 1);

	// If Something did not go well, shut it all down
	if (ret)
		CloseScript();
		
	return ret;
}


const char* Script::GetStackValue(std::string* t_string, bool* t_boolean, double* t_number)
{
	ASSERT(t_string || t_boolean || t_number);

	// These might hold the temporary values until we know we can return true.
	std::string l_string;
	bool l_boolean;
	double l_number;

	if (t_string)
	{
		if (lua_isstring(m_lua_state, -1))
			l_string = lua_tostring(m_lua_state, -1);
		else return "The value on the stack does not cast to a string";
	}

	if (t_boolean)
	{
		if (lua_isboolean(m_lua_state, -1))
			l_boolean = (lua_toboolean(m_lua_state, -1) != 0);
		else return "The value on the stack does not cast to a bool(int)";
	}

	if (t_number)
	{
		if (lua_isnumber(m_lua_state, -1))
			l_number = lua_tonumber(m_lua_state, -1);
		else return "The value on the stack does not cast to a number(double)";
	}

	// So far we know that we have gotten what we asked for
	if (t_string)
		*t_string = l_string;
	if (t_boolean)
		*t_boolean = l_boolean;
	if (t_number)
		*t_number = l_number;

	return NULL;
}

const char* Script::GetGlobal(std::string globalName,std::string* t_string, bool* t_boolean, double* t_number)
{
	ASSERT(IsValid());
	std::string mappedName = GetMappedName(globalName);
	lua_getglobal(m_lua_state, mappedName.c_str());
	const char* err = GetStackValue(t_string, t_boolean, t_number);
	if (err)
	{
		lua_getglobal(m_lua_state, globalName.c_str());
		err = GetStackValue(t_string, t_boolean, t_number);
	}
	
	if (err)
	{
		std::string errMsg(err);
		return CacheError(Framework::Base::BuildString("LUA Script[%s] failed to read %s, {%s}\n", 
			GetFilename(), mappedName.c_str(), errMsg.c_str()), false);
	}
	else
		return NULL;
}

//////////////////////////////////////////////////////////////////////////

const char* Script::GetGlobalTable(std::string globalTableName)
{
	ASSERT(IsValid());
	lua_getglobal(m_lua_state, GetMappedName(globalTableName).c_str());
	if (!lua_istable(m_lua_state, -1))
	{
		return CacheError(Framework::Base::BuildString(
			"LUA Script[%s] could Not load Table from the Global %s\n", 
			GetFilename(), globalTableName.c_str()), false);
	}
	else return NULL;	// All is ok
}
//////////////////////////////////////////////////////////////////////////

const char* Script::GetFieldTable(std::string fieldTableName)
{
	ASSERT(IsValid());
	ASSERT(lua_istable(m_lua_state, -1));	// We should already have a stack
	lua_pushstring(m_lua_state, GetMappedName(fieldTableName).c_str());
	lua_gettable(m_lua_state, -2);
	if (!lua_istable(m_lua_state, -1))
	{
		// This is not a good table name, Pop off the stack now
		lua_pop(m_lua_state, 1);  // Pop the name back off the stack
		return CacheError(Framework::Base::BuildString(
			"LUA Script[%s] could Not load Table from the field name %s\n", 
			GetFilename(), GetMappedName(fieldTableName).c_str()), false);
	}
	else
		return NULL;
}
//////////////////////////////////////////////////////////////////////////
const char* Script::GetField(std::string fieldName,std::string* t_string, bool* t_boolean, double* t_number)
{
	ASSERT(IsValid());
	ASSERT(lua_istable(m_lua_state, -1));	// We should already have a stack
	
	lua_pushstring(m_lua_state, GetMappedName(fieldName).c_str());
	lua_gettable(m_lua_state, -2);
	const char* err = GetStackValue(t_string, t_boolean, t_number);
	lua_pop(m_lua_state, 1);  // Pop the name back off the stack
	
	if (err)
	{
		std::string errMsg(err);
		return CacheError(Framework::Base::BuildString("LUA Script[%s] failed to read field name %s, {%s}\n", 
			GetFilename(), GetMappedName(fieldName).c_str(), errMsg.c_str()), false);
	}
	else
		return NULL;
}
//////////////////////////////////////////////////////////////////////////

void Script::Pop()
{
	lua_pop(m_lua_state, 1);
}
//////////////////////////////////////////////////////////////////////////

const char* Script::GetIndex(unsigned index, 
		std::string* t_string, bool* t_boolean, double* t_number)
{
	ASSERT(IsValid());
	ASSERT(lua_istable(m_lua_state, -1));	// We should already have a stack
	lua_rawgeti(m_lua_state, -1, index);
	const char* err = GetStackValue(t_string, t_boolean, t_number);
	lua_pop(m_lua_state, 1);  // Pop the name back off the stack
	
	if (err)
	{
		std::string errMsg(err);
		return CacheError(Framework::Base::BuildString("LUA Script[%s] failed to read field index %i, {%s}\n", 
			GetFilename(), index, errMsg.c_str()), false);
	}
	else
		return NULL;
}
//////////////////////////////////////////////////////////////////////////

const char* Script::GetIndexTable(unsigned index)
{
	ASSERT(IsValid());
	ASSERT(lua_istable(m_lua_state, -1));	// We should already have a stack
	lua_rawgeti(m_lua_state, -1, index);
	if (!lua_istable(m_lua_state, -1))
		return CacheError(Framework::Base::BuildString(
		"LUA Script[%s] could Not load Table from the index %i", GetFilename(), index), false);
	else
		return NULL;
}
//////////////////////////////////////////////////////////////////////////

//! e.g. CallFunction("f", "dd>d", x, y, &z);
//! Note that this was copied from the examples, and I do not necessarily like the style,
//! but it works and should get us going, and it is quite robust.
//! For params: d=double, i=int, s=char*
//! For returns: d=double*, i=int*, s=char**, t=(Event1<Script*>*
const char* Script::CallFunction(const char *func, const char *sig, ...) 
{
	va_list vl;
	int narg, nres;  /* number of arguments and results */

	va_start(vl, sig);
	lua_getglobal(m_lua_state, GetMappedName(func).c_str());  /* get function */

	/* push arguments */
	narg = 0;
	while (*sig) {  /* push arguments */
		switch (*sig++) {

		  case 'd':  /* double argument */
			  lua_pushnumber(m_lua_state, va_arg(vl, double));
			  break;

		  case 'i':  /* int argument */
			  lua_pushnumber(m_lua_state, va_arg(vl, int));
			  break;

		  case 's':  /* string argument */
			  lua_pushstring(m_lua_state, va_arg(vl, char *));
			  break;

		  case '>':
			  goto endwhile;

		  default:
			  va_end(vl);
			  return CacheError(Framework::Base::BuildString("invalid option (%c)\n", *(sig - 1)), narg);
		}
		narg++;
		luaL_checkstack(m_lua_state, 1, "too many arguments");
	} endwhile:

	/* do the call */
	nres = (int)strlen(sig);  /* number of expected results */
	if (lua_pcall(m_lua_state, narg, nres, 0) != 0)  /* do the call */
	{
		va_end(vl);
		return CacheError(Framework::Base::BuildString("error running function `%s': %s\n",
			func, lua_tostring(m_lua_state, -1)), false);
	}

	/* retrieve results */
	int resIndex = -nres;  /* stack index of first result */
	while (*sig) {  /* get results */
		switch (*sig++) {

		  case 'd':  /* double result */
			  if (!lua_isnumber(m_lua_state, resIndex))
			  {
				  va_end(vl);
				  return CacheError(Framework::Base::BuildString("Invalid return type for %s at pos %i, expected number\n",
					  func, -resIndex), nres);
			  }
			  *va_arg(vl, double *) = lua_tonumber(m_lua_state, resIndex);
			  break;

		  case 'i':  /* int result */
			  if (!lua_isnumber(m_lua_state, resIndex))
			  {
				  va_end(vl);
				  return CacheError(Framework::Base::BuildString("Invalid return type for %s at pos %i, expected int\n",
					  func, -resIndex), nres);
			  }
			  *va_arg(vl, int *) = (int)lua_tonumber(m_lua_state, resIndex);
			  break;

		  case 'b':  /* bool result */
			  if (!lua_isboolean(m_lua_state, resIndex))
			  {
				  va_end(vl);
				  return CacheError(Framework::Base::BuildString("Invalid return type for %s at pos %i, expected bool(int)\n",
					  func, -resIndex), nres);
			  }
			  *va_arg(vl, bool *) = (int)lua_toboolean(m_lua_state, resIndex) != 0;
			  break;

		  case 's':  /* string result */
			  if (!lua_isstring(m_lua_state, resIndex))
			  {
				  va_end(vl);
				  return CacheError(Framework::Base::BuildString("Invalid return type for %s at pos %i, expected string\n",
					  func, -resIndex), nres);
			  }
			  *va_arg(vl, const char **) = lua_tostring(m_lua_state, resIndex);
			  break;

		  case 't':	/* Table result */
			  if (!lua_istable(m_lua_state, resIndex))
			  {
				  va_end(vl);
				  return CacheError(Framework::Base::BuildString("Invalid return type for %s at pos %i, expected table\n",
					  func, -resIndex), nres);
			  }
			  // Push this table to the TOP of the stack
			  lua_pushvalue(m_lua_state, resIndex);

				//TODO see if I need this

			  // OOH! Ugly void* cast, but it was the only way I could work out how to do it.
			  //((Event1<Script*>*)(va_arg(vl, void*)))->Fire(this);

			  // Pop the table back off to get the stack back the way it was
				lua_pop(m_lua_state, 1);
			  break;

		  default:
			  va_end(vl);
			  return CacheError(Framework::Base::BuildString("invalid option (%c)\n", *(sig - 1)), nres);
		}
		resIndex++;
	}
	va_end(vl);

	// Pop the results
	lua_pop(m_lua_state, nres);
	return NULL;
}
//////////////////////////////////////////////////////////////////////////
#endif

