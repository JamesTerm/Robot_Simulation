#pragma once

#ifndef _Win32
typedef long long __int64;
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <assert.h>
#include <limits.h>
#include <errno.h>
#include <stddef.h>

#define LUA_API
#define LUALIB_API
#define LUAI_FUNC
//#define ASSERT(x)


//#include "luaconf.h"------------------------------------------------------------------------------------------------------
/*
@@ LUA_ANSI controls the use of non-ansi features.
** CHANGE it (define it) if you want Lua to avoid the use of any
** non-ansi feature or library.
*/
#if defined(__STRICT_ANSI__)
#define LUA_ANSI
#endif


#if !defined(LUA_ANSI) && defined(_WIN32)
#define LUA_WIN
#endif

#if defined(LUA_USE_LINUX)
#define LUA_USE_POSIX
#define LUA_USE_DLOPEN		/* needs an extra library: -ldl */
#define LUA_USE_READLINE	/* needs some extra libraries */
#endif

#if defined(LUA_USE_MACOSX)
#define LUA_USE_POSIX
#define LUA_DL_DYLD		/* does not need extra library */
#endif



/*
@@ LUA_USE_POSIX includes all functionallity listed as X/Open System
@* Interfaces Extension (XSI).
** CHANGE it (define it) if your system is XSI compatible.
*/
#if defined(LUA_USE_POSIX)
#define LUA_USE_MKSTEMP
#define LUA_USE_ISATTY
#define LUA_USE_POPEN
#define LUA_USE_ULONGJMP
#endif


/*
@@ LUA_PATH and LUA_CPATH are the names of the environment variables that
@* Lua check to set its paths.
@@ LUA_INIT is the name of the environment variable that Lua
@* checks for initialization code.
** CHANGE them if you want different names.
*/
#define LUA_PATH        "LUA_PATH"
#define LUA_CPATH       "LUA_CPATH"
#define LUA_INIT	"LUA_INIT"


/*
@@ LUA_PATH_DEFAULT is the default path that Lua uses to look for
@* Lua libraries.
@@ LUA_CPATH_DEFAULT is the default path that Lua uses to look for
@* C libraries.
** CHANGE them if your machine has a non-conventional directory
** hierarchy or if you want to install your libraries in
** non-conventional directories.
*/
#if defined(_WIN32)
/*
** In Windows, any exclamation mark ('!') in the path is replaced by the
** path of the directory of the executable file of the current process.
*/
#define LUA_LDIR	"!\\lua\\"
#define LUA_CDIR	"!\\"
#define LUA_PATH_DEFAULT  \
	".\\?.lua;"  LUA_LDIR"?.lua;"  LUA_LDIR"?\\init.lua;" \
	LUA_CDIR"?.lua;"  LUA_CDIR"?\\init.lua"
#define LUA_CPATH_DEFAULT \
	".\\?.dll;"  LUA_CDIR"?.dll;" LUA_CDIR"loadall.dll"

#else
#define LUA_ROOT	"/usr/local/"
#define LUA_LDIR	LUA_ROOT "share/lua/5.1/"
#define LUA_CDIR	LUA_ROOT "lib/lua/5.1/"
#define LUA_PATH_DEFAULT  \
	"./?.lua;"  LUA_LDIR"?.lua;"  LUA_LDIR"?/init.lua;" \
	LUA_CDIR"?.lua;"  LUA_CDIR"?/init.lua"
#define LUA_CPATH_DEFAULT \
	"./?.so;"  LUA_CDIR"?.so;" LUA_CDIR"loadall.so"
#endif


/*
@@ LUA_DIRSEP is the directory separator (for submodules).
** CHANGE it if your machine does not use "/" as the directory separator
** and is not Windows. (On Windows Lua automatically uses "\".)
*/
#if defined(_WIN32)
#define LUA_DIRSEP	"\\"
#else
#define LUA_DIRSEP	"/"
#endif


/*
@@ LUA_PATHSEP is the character that separates templates in a path.
@@ LUA_PATH_MARK is the string that marks the substitution points in a
@* template.
@@ LUA_EXECDIR in a Windows path is replaced by the executable's
@* directory.
@@ LUA_IGMARK is a mark to ignore all before it when bulding the
@* luaopen_ function name.
** CHANGE them if for some reason your system cannot use those
** characters. (E.g., if one of those characters is a common character
** in file/directory names.) Probably you do not need to change them.
*/
#define LUA_PATHSEP	";"
#define LUA_PATH_MARK	"?"
#define LUA_EXECDIR	"!"
#define LUA_IGMARK	"-"


/*
@@ LUA_INTEGER is the integral type used by lua_pushinteger/lua_tointeger.
** CHANGE that if ptrdiff_t is not adequate on your machine. (On most
** machines, ptrdiff_t gives a good choice between int or long.)
*/
#define LUA_INTEGER	ptrdiff_t


/*
@@ LUA_API is a mark for all core API functions.
@@ LUALIB_API is a mark for all standard library functions.
** CHANGE them if you need to define those functions in some special way.
** For instance, if you want to create one Windows DLL with the core and
** the libraries, you may want to use the following definition (define
** LUA_BUILD_AS_DLL to get it).
*/
#if defined(LUA_BUILD_AS_DLL)

#if defined(LUA_CORE) || defined(LUA_LIB)
#define LUA_API __declspec(dllexport)
#else
#define LUA_API __declspec(dllimport)
#endif

#else

//#define LUA_API		extern

#endif

/* more often than not the libs go together with the core */
//#define LUALIB_API	LUA_API


/*
@@ LUAI_FUNC is a mark for all extern functions that are not to be
@* exported to outside modules.
@@ LUAI_DATA is a mark for all extern (const) variables that are not to
@* be exported to outside modules.
** CHANGE them if you need to mark them in some special way. Elf/gcc
** (versions 3.2 and later) mark them as "hidden" to optimize access
** when Lua is compiled as a shared library.
*/
#if defined(luaall_c)
#define LUAI_FUNC	static
#define LUAI_DATA	/* empty */

#elif defined(__GNUC__) && ((__GNUC__*100 + __GNUC_MINOR__) >= 302) && \
	defined(__ELF__)
//#define LUAI_FUNC	__attribute__((visibility("hidden"))) extern
#define LUAI_DATA	LUAI_FUNC

//#else
//#define LUAI_FUNC	extern
//#define LUAI_DATA	extern
#endif



/*
@@ LUA_QL describes how error messages quote program elements.
** CHANGE it if you want a different appearance.
*/
#define LUA_QL(x)	"'" x "'"
#define LUA_QS		LUA_QL("%s")


/*
@@ LUA_IDSIZE gives the maximum size for the description of the source
@* of a function in debug information.
** CHANGE it if you want a different size.
*/
#define LUA_IDSIZE	60


/*
** {==================================================================
** Stand-alone configuration
** ===================================================================
*/

#if defined(lua_c) || defined(luaall_c)

/*
@@ lua_stdin_is_tty detects whether the standard input is a 'tty' (that
@* is, whether we're running lua interactively).
** CHANGE it if you have a better definition for non-POSIX/non-Windows
** systems.
*/
#if defined(LUA_USE_ISATTY)
#include <unistd.h>
#define lua_stdin_is_tty()	isatty(0)
#elif defined(LUA_WIN)
#include <io.h>
#include <stdio.h>
#define lua_stdin_is_tty()	_isatty(_fileno(stdin))
#else
#define lua_stdin_is_tty()	1  /* assume stdin is a tty */
#endif


/*
@@ LUA_PROMPT is the default prompt used by stand-alone Lua.
@@ LUA_PROMPT2 is the default continuation prompt used by stand-alone Lua.
** CHANGE them if you want different prompts. (You can also change the
** prompts dynamically, assigning to globals _PROMPT/_PROMPT2.)
*/
#define LUA_PROMPT		"> "
#define LUA_PROMPT2		">> "


/*
@@ LUA_PROGNAME is the default name for the stand-alone Lua program.
** CHANGE it if your stand-alone interpreter has a different name and
** your system is not able to detect that name automatically.
*/
#define LUA_PROGNAME		"lua"


/*
@@ LUA_MAXINPUT is the maximum length for an input line in the
@* stand-alone interpreter.
** CHANGE it if you need longer lines.
*/
#define LUA_MAXINPUT	512


/*
@@ lua_readline defines how to show a prompt and then read a line from
@* the standard input.
@@ lua_saveline defines how to "save" a read line in a "history".
@@ lua_freeline defines how to free a line read by lua_readline.
** CHANGE them if you want to improve this functionality (e.g., by using
** GNU readline and history facilities).
*/
#if defined(LUA_USE_READLINE)
#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#define lua_readline(L,b,p)	((void)L, ((b)=readline(p)) != NULL)
#define lua_saveline(L,idx) \
	if (lua_strlen(L,idx) > 0)  /* non-empty line? */ \
	add_history(lua_tostring(L, idx));  /* add it to history */
#define lua_freeline(L,b)	((void)L, free(b))
#else
#define lua_readline(L,b,p)	\
	((void)L, fputs(p, stdout), fflush(stdout),  /* show prompt */ \
	fgets(b, LUA_MAXINPUT, stdin) != NULL)  /* get line */
#define lua_saveline(L,idx)	{ (void)L; (void)idx; }
#define lua_freeline(L,b)	{ (void)L; (void)b; }
#endif

#endif

/* }================================================================== */


/*
@@ LUAI_GCPAUSE defines the default pause between garbage-collector cycles
@* as a percentage.
** CHANGE it if you want the GC to run faster or slower (higher values
** mean larger pauses which mean slower collection.) You can also change
** this value dynamically.
*/
#define LUAI_GCPAUSE	200  /* 200% (wait memory to double before next GC) */


/*
@@ LUAI_GCMUL defines the default speed of garbage collection relative to
@* memory allocation as a percentage.
** CHANGE it if you want to change the granularity of the garbage
** collection. (Higher values mean coarser collections. 0 represents
** infinity, where each step performs a full collection.) You can also
** change this value dynamically.
*/
#define LUAI_GCMUL	200 /* GC runs 'twice the speed' of memory allocation */



/*
@@ LUA_COMPAT_GETN controls compatibility with old getn behavior.
** CHANGE it (define it) if you want exact compatibility with the
** behavior of setn/getn in Lua 5.0.
*/
#undef LUA_COMPAT_GETN

/*
@@ LUA_COMPAT_LOADLIB controls compatibility about global loadlib.
** CHANGE it to undefined as soon as you do not need a global 'loadlib'
** function (the function is still available as 'package.loadlib').
*/
#undef LUA_COMPAT_LOADLIB

/*
@@ LUA_COMPAT_VARARG controls compatibility with old vararg feature.
** CHANGE it to undefined as soon as your programs use only '...' to
** access vararg parameters (instead of the old 'arg' table).
*/
#define LUA_COMPAT_VARARG

/*
@@ LUA_COMPAT_MOD controls compatibility with old math.mod function.
** CHANGE it to undefined as soon as your programs use 'math.fmod' or
** the new '%' operator instead of 'math.mod'.
*/
#define LUA_COMPAT_MOD

/*
@@ LUA_COMPAT_LSTR controls compatibility with old long string nesting
@* facility.
** CHANGE it to 2 if you want the old behaviour, or undefine it to turn
** off the advisory error when nesting [[...]].
*/
#define LUA_COMPAT_LSTR		1

/*
@@ LUA_COMPAT_GFIND controls compatibility with old 'string.gfind' name.
** CHANGE it to undefined as soon as you rename 'string.gfind' to
** 'string.gmatch'.
*/
#define LUA_COMPAT_GFIND

/*
@@ LUA_COMPAT_OPENLIB controls compatibility with old 'luaL_openlib'
@* behavior.
** CHANGE it to undefined as soon as you replace to 'luaL_register'
** your uses of 'luaL_openlib'
*/
#define LUA_COMPAT_OPENLIB



/*
@@ luai_apicheck is the assert macro used by the Lua-C API.
** CHANGE luai_apicheck if you want Lua to perform some checks in the
** parameters it gets from API calls. This may slow down the interpreter
** a bit, but may be quite useful when debugging C code that interfaces
** with Lua. A useful redefinition is to use assert.h.
*/
#if defined(LUA_USE_APICHECK)
#include <assert.h>
#define luai_apicheck(L,o)	{ (void)L; assert(o); }
#else
#define luai_apicheck(L,o)	{ (void)L; }
#endif


/*
@@ LUAI_BITSINT defines the number of bits in an int.
** CHANGE here if Lua cannot automatically detect the number of bits of
** your machine. Probably you do not need to change this.
*/
/* avoid overflows in comparison */
#if INT_MAX-20 < 32760
#define LUAI_BITSINT	16
#elif INT_MAX > 2147483640L
/* int has at least 32 bits */
#define LUAI_BITSINT	32
#else
#error "you must define LUA_BITSINT with number of bits in an integer"
#endif


/*
@@ LUAI_UINT32 is an unsigned integer with at least 32 bits.
@@ LUAI_INT32 is an signed integer with at least 32 bits.
@@ LUAI_UMEM is an unsigned integer big enough to count the total
@* memory used by Lua.
@@ LUAI_MEM is a signed integer big enough to count the total memory
@* used by Lua.
** CHANGE here if for some weird reason the default definitions are not
** good enough for your machine. (The definitions in the 'else'
** part always works, but may waste space on machines with 64-bit
** longs.) Probably you do not need to change this.
*/
#if LUAI_BITSINT >= 32
#define LUAI_UINT32	unsigned int
#define LUAI_INT32	int
#define LUAI_MAXINT32	INT_MAX
#define LUAI_UMEM	size_t
#define LUAI_MEM	ptrdiff_t
#else
/* 16-bit ints */
#define LUAI_UINT32	unsigned long
#define LUAI_INT32	long
#define LUAI_MAXINT32	LONG_MAX
#define LUAI_UMEM	unsigned long
#define LUAI_MEM	long
#endif


/*
@@ LUAI_MAXCALLS limits the number of nested calls.
** CHANGE it if you need really deep recursive calls. This limit is
** arbitrary; its only purpose is to stop infinite recursion before
** exhausting memory.
*/
#define LUAI_MAXCALLS	20000


/*
@@ LUAI_MAXCSTACK limits the number of Lua stack slots that a C function
@* can use.
** CHANGE it if you need lots of (Lua) stack space for your C
** functions. This limit is arbitrary; its only purpose is to stop C
** functions to consume unlimited stack space.
*/
#define LUAI_MAXCSTACK	2048



/*
** {==================================================================
** CHANGE (to smaller values) the following definitions if your system
** has a small C stack. (Or you may want to change them to larger
** values if your system has a large C stack and these limits are
** too rigid for you.) Some of these constants control the size of
** stack-allocated arrays used by the compiler or the interpreter, while
** others limit the maximum number of recursive calls that the compiler
** or the interpreter can perform. Values too large may cause a C stack
** overflow for some forms of deep constructs.
** ===================================================================
*/


/*
@@ LUAI_MAXCCALLS is the maximum depth for nested C calls (short) and
@* syntactical nested non-terminals in a program.
*/
#define LUAI_MAXCCALLS		200


/*
@@ LUAI_MAXVARS is the maximum number of local variables per function
@* (must be smaller than 250).
*/
#define LUAI_MAXVARS		200


/*
@@ LUAI_MAXUPVALUES is the maximum number of upvalues per function
@* (must be smaller than 250).
*/
#define LUAI_MAXUPVALUES	60


/*
@@ LUAL_BUFFERSIZE is the buffer size used by the lauxlib buffer system.
*/
#define LUAL_BUFFERSIZE		BUFSIZ

/* }================================================================== */




/*
** {==================================================================
@@ LUA_NUMBER is the type of numbers in Lua.
** CHANGE the following definitions only if you want to build Lua
** with a number type different from double. You may also need to
** change lua_number2int & lua_number2integer.
** ===================================================================
*/

#define LUA_NUMBER_DOUBLE
#define LUA_NUMBER	double

/*
@@ LUAI_UACNUMBER is the result of an 'usual argument conversion'
@* over a number.
*/
#define LUAI_UACNUMBER	double


/*
@@ LUA_NUMBER_SCAN is the format for reading numbers.
@@ LUA_NUMBER_FMT is the format for writing numbers.
@@ lua_number2str converts a number to a string.
@@ LUAI_MAXNUMBER2STR is maximum size of previous conversion.
@@ lua_str2number converts a string to a number.
*/
#define LUA_NUMBER_SCAN		"%lf"
#define LUA_NUMBER_FMT		"%.14g"
#define lua_number2str(s,n)	sprintf((s), LUA_NUMBER_FMT, (n))
#define LUAI_MAXNUMBER2STR	32 /* 16 digits, sign, point, and \0 */
#define lua_str2number(s,p)	strtod((s), (p))


/*
@@ The luai_num* macros define the primitive operations over numbers.
*/
//#if defined(LUA_CORE)
#if 1
#include <math.h>
#define luai_numadd(a,b)	((a)+(b))
#define luai_numsub(a,b)	((a)-(b))
#define luai_nummul(a,b)	((a)*(b))
#define luai_numdiv(a,b)	((a)/(b))
#define luai_nummod(a,b)	((a) - floor((a)/(b))*(b))
#define luai_numpow(a,b)	(pow(a,b))
#define luai_numunm(a)		(-(a))
#define luai_numeq(a,b)		((a)==(b))
#define luai_numlt(a,b)		((a)<(b))
#define luai_numle(a,b)		((a)<=(b))
#define luai_numisnan(a)	(!luai_numeq((a), (a)))
#endif


/*
@@ lua_number2int is a macro to convert lua_Number to int.
@@ lua_number2integer is a macro to convert lua_Number to lua_Integer.
** CHANGE them if you know a faster way to convert a lua_Number to
** int (with any rounding method and without throwing errors) in your
** system. In Pentium machines, a naive typecast from double to int
** in C is extremely slow, so any alternative is worth trying.
*/

/* On a Pentium, resort to a trick */
#if defined(LUA_NUMBER_DOUBLE) && !defined(LUA_ANSI) && !defined(__SSE2__) && \
	(defined(__i386) || defined (_M_IX86) || defined(__i386__))

/* On a Microsoft compiler, use assembler */
#if defined(_MSC_VER)

#define lua_number2int(i,d)   __asm fld d   __asm fistp i
#define lua_number2integer(i,n)		lua_number2int(i, n)

/* the next trick should work on any Pentium, but sometimes clashes
with a DirectX idiosyncrasy */
#else

union luai_Cast { double l_d; long l_l; };
#define lua_number2int(i,d) \
{ volatile union luai_Cast u; u.l_d = (d) + 6755399441055744.0; (i) = u.l_l; }
#define lua_number2integer(i,n)		lua_number2int(i, n)

#endif


/* this option always works, but may be slow */
#else
#define lua_number2int(i,d)	((i)=(int)(d))
#define lua_number2integer(i,d)	((i)=(lua_Integer)(d))

#endif

/* }================================================================== */


/*
@@ LUAI_USER_ALIGNMENT_T is a type that requires maximum alignment.
** CHANGE it if your system requires alignments larger than double. (For
** instance, if your system supports long doubles and they must be
** aligned in 16-byte boundaries, then you should add long double in the
** union.) Probably you do not need to change this.
*/
#define LUAI_USER_ALIGNMENT_T	union { double u; void *s; long l; }


/*
@@ LUAI_THROW/LUAI_TRY define how Lua does exception handling.
** CHANGE them if you prefer to use longjmp/setjmp even with C++
** or if want/don't to use _longjmp/_setjmp instead of regular
** longjmp/setjmp. By default, Lua handles errors with exceptions when
** compiling as C++ code, with _longjmp/_setjmp when asked to use them,
** and with longjmp/setjmp otherwise.
*/
#if defined(__cplusplus)
/* C++ exceptions */
#define LUAI_THROW(L,c)	throw(c)
#define LUAI_TRY(L,c,a)	try { a } catch(...) \
	{ if ((c)->status == 0) (c)->status = -1; }
#define luai_jmpbuf	int  /* dummy variable */

#elif defined(LUA_USE_ULONGJMP)
/* in Unix, try _longjmp/_setjmp (more efficient) */
#define LUAI_THROW(L,c)	_longjmp((c)->b, 1)
#define LUAI_TRY(L,c,a)	if (_setjmp((c)->b) == 0) { a }
#define luai_jmpbuf	jmp_buf

#else
/* default handling with long jumps */
#define LUAI_THROW(L,c)	longjmp((c)->b, 1)
#define LUAI_TRY(L,c,a)	if (setjmp((c)->b) == 0) { a }
#define luai_jmpbuf	jmp_buf

#endif


/*
@@ LUA_MAXCAPTURES is the maximum number of captures that a pattern
@* can do during pattern-matching.
** CHANGE it if you need more captures. This limit is arbitrary.
*/
#define LUA_MAXCAPTURES		32


/*
@@ lua_tmpnam is the function that the OS library uses to create a
@* temporary name.
@@ LUA_TMPNAMBUFSIZE is the maximum size of a name created by lua_tmpnam.
** CHANGE them if you have an alternative to tmpnam (which is considered
** insecure) or if you want the original tmpnam anyway.  By default, Lua
** uses tmpnam except when POSIX is available, where it uses mkstemp.
*/
#if defined(loslib_c) || defined(luaall_c)

#if defined(LUA_USE_MKSTEMP)
#include <unistd.h>
#define LUA_TMPNAMBUFSIZE	32
#define lua_tmpnam(b,e)	{ \
	strcpy(b, "/tmp/lua_XXXXXX"); \
	e = mkstemp(b); \
	if (e != -1) close(e); \
	e = (e == -1); }

#else
#define LUA_TMPNAMBUFSIZE	L_tmpnam
#define lua_tmpnam(b,e)		{ e = (tmpnam(b) == NULL); }
#endif

#endif


/*
@@ lua_popen spawns a new process connected to the current one through
@* the file streams.
** CHANGE it if you have a way to implement it in your system.
*/
#if defined(LUA_USE_POPEN)

#define lua_popen(L,c,m)	((void)L, popen(c,m))
#define lua_pclose(L,file)	((void)L, (pclose(file) != -1))

#elif defined(LUA_WIN)

#define lua_popen(L,c,m)	((void)L, _popen(c,m))
#define lua_pclose(L,file)	((void)L, (_pclose(file) != -1))

#else

#define lua_popen(L,c,m)	((void)((void)c, m),  \
	luaL_error(L, LUA_QL("popen") " not supported"), (FILE*)0)
#define lua_pclose(L,file)		((void)((void)L, file), 0)

#endif

/*
@@ LUA_DL_* define which dynamic-library system Lua should use.
** CHANGE here if Lua has problems choosing the appropriate
** dynamic-library system for your platform (either Windows' DLL, Mac's
** dyld, or Unix's dlopen). If your system is some kind of Unix, there
** is a good chance that it has dlopen, so LUA_DL_DLOPEN will work for
** it.  To use dlopen you also need to adapt the src/Makefile (probably
** adding -ldl to the linker options), so Lua does not select it
** automatically.  (When you change the makefile to add -ldl, you must
** also add -DLUA_USE_DLOPEN.)
** If you do not want any kind of dynamic library, undefine all these
** options.
** By default, _WIN32 gets LUA_DL_DLL and MAC OS X gets LUA_DL_DYLD.
*/
#if defined(LUA_USE_DLOPEN)
#define LUA_DL_DLOPEN
#endif

#if defined(LUA_WIN)
#define LUA_DL_DLL
#endif


/*
@@ LUAI_EXTRASPACE allows you to add user-specific data in a lua_State
@* (the data goes just *before* the lua_State pointer).
** CHANGE (define) this if you really need that. This value must be
** a multiple of the maximum alignment required for your machine.
*/
#define LUAI_EXTRASPACE		0


/*
@@ luai_userstate* allow user-specific actions on threads.
** CHANGE them if you defined LUAI_EXTRASPACE and need to do something
** extra when a thread is created/deleted/resumed/yielded.
*/
#define luai_userstateopen(L)		((void)L)
#define luai_userstateclose(L)		((void)L)
#define luai_userstatethread(L,L1)	((void)L)
#define luai_userstatefree(L)		((void)L)
#define luai_userstateresume(L,n)	((void)L)
#define luai_userstateyield(L,n)	((void)L)


/*
@@ LUA_INTFRMLEN is the length modifier for integer conversions
@* in 'string.format'.
@@ LUA_INTFRM_T is the integer type correspoding to the previous length
@* modifier.
** CHANGE them if your system supports long long or does not support long.
*/

#if defined(LUA_USELONGLONG)

#define LUA_INTFRMLEN		"ll"
#define LUA_INTFRM_T		long long

#else

#define LUA_INTFRMLEN		"l"
#define LUA_INTFRM_T		long

#endif



/* =================================================================== */

/*
** Local configuration. You can use this space to add your redefinitions
** without modifying the main part of the file.
*/


//#include "lua.h"----------------------------------------------------------------------------------------------------------
#define LUA_VERSION	"Lua 5.1"
#define LUA_RELEASE	"Lua 5.1.2"
#define LUA_VERSION_NUM	501
#define LUA_COPYRIGHT	"Copyright (C) 1994-2007 Lua.org, PUC-Rio"
#define LUA_AUTHORS 	"R. Ierusalimschy, L. H. de Figueiredo & W. Celes"

/* mark for precompiled code (`<esc>Lua') */
#define	LUA_SIGNATURE	"\033Lua"

/* option for multiple returns in `lua_pcall' and `lua_call' */
#define LUA_MULTRET	(-1)


/*
** pseudo-indices
*/
#define LUA_REGISTRYINDEX	(-10000)
#define LUA_ENVIRONINDEX	(-10001)
#define LUA_GLOBALSINDEX	(-10002)
#define lua_upvalueindex(i)	(LUA_GLOBALSINDEX-(i))


/* thread status; 0 is OK */
#define LUA_YIELD	1
#define LUA_ERRRUN	2
#define LUA_ERRSYNTAX	3
#define LUA_ERRMEM	4
#define LUA_ERRERR	5


typedef struct lua_State lua_State;

typedef int (*lua_CFunction) (lua_State *L);


/*
** functions that read/write blocks when loading/dumping Lua chunks
*/
typedef const char * (*lua_Reader) (lua_State *L, void *ud, size_t *sz);

typedef int (*lua_Writer) (lua_State *L, const void* p, size_t sz, void* ud);


/*
** prototype for memory-allocation functions
*/
typedef void * (*lua_Alloc) (void *ud, void *ptr, size_t osize, size_t nsize);

/*
** basic types
*/
#define LUA_TNONE		(-1)

#define LUA_TNIL		0
#define LUA_TBOOLEAN		1
#define LUA_TLIGHTUSERDATA	2
#define LUA_TNUMBER		3
#define LUA_TSTRING		4
#define LUA_TTABLE		5
#define LUA_TFUNCTION		6
#define LUA_TUSERDATA		7
#define LUA_TTHREAD		8



/* minimum Lua stack available to a C function */
#define LUA_MINSTACK	20

/*
** generic extra include file
*/
#if defined(LUA_USER_H)
#include LUA_USER_H
#endif


/* type of numbers in Lua */
typedef LUA_NUMBER lua_Number;


/* type for integer functions */
typedef LUA_INTEGER lua_Integer;

/* 
** ===============================================================
** some useful macros
** ===============================================================
*/

#define lua_pop(L,n)		lua_settop(L, -(n)-1)

#define lua_newtable(L)		lua_createtable(L, 0, 0)

#define lua_register(L,n,f) (lua_pushcfunction(L, (f)), lua_setglobal(L, (n)))

#define lua_pushcfunction(L,f)	lua_pushcclosure(L, (f), 0)

#define lua_strlen(L,i)		lua_objlen(L, (i))

#define lua_isfunction(L,n)	(lua_type(L, (n)) == LUA_TFUNCTION)
#define lua_istable(L,n)	(lua_type(L, (n)) == LUA_TTABLE)
#define lua_islightuserdata(L,n)	(lua_type(L, (n)) == LUA_TLIGHTUSERDATA)
#define lua_isnil(L,n)		(lua_type(L, (n)) == LUA_TNIL)
#define lua_isboolean(L,n)	(lua_type(L, (n)) == LUA_TBOOLEAN)
#define lua_isthread(L,n)	(lua_type(L, (n)) == LUA_TTHREAD)
#define lua_isnone(L,n)		(lua_type(L, (n)) == LUA_TNONE)
#define lua_isnoneornil(L, n)	(lua_type(L, (n)) <= 0)

#define lua_pushliteral(L, s)	\
	lua_pushlstring(L, "" s, (sizeof(s)/sizeof(char))-1)

#define lua_setglobal(L,s)	lua_setfield(L, LUA_GLOBALSINDEX, (s))
#define lua_getglobal(L,s)	lua_getfield(L, LUA_GLOBALSINDEX, (s))

#define lua_tostring(L,i)	lua_tolstring(L, (i), NULL)

/*
** compatibility macros and functions
*/

#define lua_open()	luaL_newstate()

#define lua_getregistry(L)	lua_pushvalue(L, LUA_REGISTRYINDEX)

#define lua_getgccount(L)	lua_gc(L, LUA_GCCOUNT, 0)

#define lua_Chunkreader		lua_Reader
#define lua_Chunkwriter		lua_Writer

/*
** {======================================================================
** Debug API
** =======================================================================
*/


/*
** Event codes
*/
#define LUA_HOOKCALL	0
#define LUA_HOOKRET	1
#define LUA_HOOKLINE	2
#define LUA_HOOKCOUNT	3
#define LUA_HOOKTAILRET 4


/*
** Event masks
*/
#define LUA_MASKCALL	(1 << LUA_HOOKCALL)
#define LUA_MASKRET	(1 << LUA_HOOKRET)
#define LUA_MASKLINE	(1 << LUA_HOOKLINE)
#define LUA_MASKCOUNT	(1 << LUA_HOOKCOUNT)

typedef struct lua_Debug lua_Debug;  /* activation record */


/* Functions to be called by the debuger in specific events */
typedef void (*lua_Hook) (lua_State *L, lua_Debug *ar);


LUA_API int lua_getstack (lua_State *L, int level, lua_Debug *ar);
LUA_API int lua_getinfo (lua_State *L, const char *what, lua_Debug *ar);
LUA_API const char *lua_getlocal (lua_State *L, const lua_Debug *ar, int n);
LUA_API const char *lua_setlocal (lua_State *L, const lua_Debug *ar, int n);
LUA_API const char *lua_getupvalue (lua_State *L, int funcindex, int n);
LUA_API const char *lua_setupvalue (lua_State *L, int funcindex, int n);

LUA_API int lua_sethook (lua_State *L, lua_Hook func, int mask, int count);
LUA_API lua_Hook lua_gethook (lua_State *L);
LUA_API int lua_gethookmask (lua_State *L);
LUA_API int lua_gethookcount (lua_State *L);


struct lua_Debug {
	int event;
	const char *name;	/* (n) */
	const char *namewhat;	/* (n) `global', `local', `field', `method' */
	const char *what;	/* (S) `Lua', `C', `main', `tail' */
	const char *source;	/* (S) */
	int currentline;	/* (l) */
	int nups;		/* (u) number of upvalues */
	int linedefined;	/* (S) */
	int lastlinedefined;	/* (S) */
	char short_src[LUA_IDSIZE]; /* (S) */
	/* private part */
	int i_ci;  /* active function */
};

//#include "lualib.h"-------------------------------------------------------------------------------------------------------
/* Key to file-handle type */
#define LUA_FILEHANDLE		"FILE*"


#define LUA_COLIBNAME	"coroutine"
LUALIB_API int (luaopen_base) (lua_State *L);

#define LUA_TABLIBNAME	"table"
LUALIB_API int (luaopen_table) (lua_State *L);

#define LUA_IOLIBNAME	"io"
LUALIB_API int (luaopen_io) (lua_State *L);

#define LUA_OSLIBNAME	"os"
LUALIB_API int (luaopen_os) (lua_State *L);

#define LUA_STRLIBNAME	"string"
LUALIB_API int (luaopen_string) (lua_State *L);

#define LUA_MATHLIBNAME	"math"
LUALIB_API int (luaopen_math) (lua_State *L);

#define LUA_DBLIBNAME	"debug"
LUALIB_API int (luaopen_debug) (lua_State *L);

#define LUA_LOADLIBNAME	"package"
LUALIB_API int (luaopen_package) (lua_State *L);


/* open all previous libraries */
LUALIB_API void (luaL_openlibs) (lua_State *L); 



//#ifndef lua_assert
//#define lua_assert(x)	((void)0)
//#endif

//#include "lapi.h"
//#include "ldebug.h"-------------------------------------------------------------------------------------------------------
#define pcRel(pc, p)	(cast(int, (pc) - (p)->code) - 1)
#define getline(f,pc)	(((f)->lineinfo) ? (f)->lineinfo[pc] : 0)
#define resethookcount(L)	(L->hookcount = L->basehookcount)

//#include "lmem.h"---------------------------------------------------------------------------------------------------------
#define MEMERRMSG	"not enough memory"

#define luaM_reallocv(L,b,on,n,e) \
	((cast(size_t, (n)+1) <= MAX_SIZET/(e)) ?  /* +1 to avoid warnings */ \
	luaM_realloc_(L, (b), (on)*(e), (n)*(e)) : \
	luaM_toobig(L))

#define luaM_freemem(L, b, s)	luaM_realloc_(L, (b), (s), 0)
#define luaM_free(L, b)		luaM_realloc_(L, (b), sizeof(*(b)), 0)
#define luaM_freearray(L, b, n, t)   luaM_reallocv(L, (b), n, 0, sizeof(t))

#define luaM_malloc(L,t)	luaM_realloc_(L, NULL, 0, (t))
#define luaM_new(L,t)		cast(t *, luaM_malloc(L, sizeof(t)))
#define luaM_newvector(L,n,t) \
	cast(t *, luaM_reallocv(L, NULL, 0, n, sizeof(t)))

#define luaM_growvector(L,v,nelems,size,t,limit,e) \
	if ((nelems)+1 > (size)) \
	((v)=cast(t *, luaM_growaux_(L,v,&(size),sizeof(t),limit,e)))

#define luaM_reallocvector(L, v,oldn,n,t) \
	((v)=cast(t *, luaM_reallocv(L, v, oldn, n, sizeof(t))))


LUAI_FUNC void *luaM_realloc_ (lua_State *L, void *block, size_t oldsize,
							   size_t size);
LUAI_FUNC void *luaM_toobig (lua_State *L);
LUAI_FUNC void *luaM_growaux_ (lua_State *L, void *block, int *size,
							   size_t size_elem, int limit,
							   const char *errormsg);

//#include "limits.h"-------------------------------------------------------------------------------------------------------
typedef LUAI_UINT32 lu_int32;

typedef LUAI_UMEM lu_mem;

typedef LUAI_MEM l_mem;



/* chars used as small naturals (so that `char' is reserved for characters) */
typedef unsigned char lu_byte;


#define MAX_SIZET	((size_t)(~(size_t)0)-2)

#define MAX_LUMEM	((lu_mem)(~(lu_mem)0)-2)


#define MAX_INT (INT_MAX-2)  /* maximum value of an int (-2 for safety) */

/*
** conversion of pointer to integer
** this is for hashing only; there is no problem if the integer
** cannot hold the whole pointer value
*/
#define IntPoint(p)  ((unsigned int)(lu_mem)(p))



/* type to ensure maximum alignment */
typedef LUAI_USER_ALIGNMENT_T L_Umaxalign;


/* result of a `usual argument conversion' over lua_Number */
typedef LUAI_UACNUMBER l_uacNumber;


/* internal assertions for in-house debugging */
//#ifdef lua_assert
//
//#define check_exp(c,e)		(lua_assert(c), (e))
//#define api_check(l,e)		lua_assert(e)
//
//#else

#define lua_assert(c)		((void)0)
#define check_exp(c,e)		(e)
#define api_check		luai_apicheck

//#endif


#ifndef UNUSED
#define UNUSED(x)	((void)(x))	/* to avoid warnings */
#endif


#ifndef cast
#define cast(t, exp)	((t)(exp))
#endif

#define cast_byte(i)	cast(lu_byte, (i))
#define cast_num(i)	cast(lua_Number, (i))
#define cast_int(i)	cast(int, (i))



/*
** type for virtual-machine instructions
** must be an unsigned with (at least) 4 bytes (see details in lopcodes.h)
*/
typedef lu_int32 Instruction;



/* maximum stack for a Lua function */
#define MAXSTACK	250



/* minimum size for the string table (must be power of 2) */
#ifndef MINSTRTABSIZE
#define MINSTRTABSIZE	32
#endif


/* minimum size for string buffer */
#ifndef LUA_MINBUFFER
#define LUA_MINBUFFER	32
#endif

#define lapi_c
#define LUA_CORE

#ifndef lua_lock
#define lua_lock(L)     ((void) 0) 
#define lua_unlock(L)   ((void) 0)
#endif

#ifndef luai_threadyield
#define luai_threadyield(L)     {lua_unlock(L); lua_lock(L);}
#endif


/*
** macro to control inclusion of some hard tests on stack reallocation
*/ 
#ifndef HARDSTACKTESTS
#define condhardstacktests(x)	((void)0)
#else
#define condhardstacktests(x)	x
#endif


//oops limits here?
#define luai_apicheck(L,o)	{ (void)L; }
//#define api_check		luai_apicheck


//#define CHAR_BIT      8         /* number of bits in a char */
//#define SCHAR_MIN   (-128)      /* minimum signed char value */
//#define SCHAR_MAX     127       /* maximum signed char value */
//#define UCHAR_MAX     0xff      /* maximum unsigned char value */

#ifndef _CHAR_UNSIGNED
//#define CHAR_MIN    SCHAR_MIN   /* mimimum char value */
//#define CHAR_MAX    SCHAR_MAX   /* maximum char value */
#else
#define CHAR_MIN      0
#define CHAR_MAX    UCHAR_MAX
#endif  /* _CHAR_UNSIGNED */

//#define MB_LEN_MAX    5             /* max. # bytes in multibyte char */
//#define SHRT_MIN    (-32768)        /* minimum (signed) short value */
//#define SHRT_MAX      32767         /* maximum (signed) short value */
//#define USHRT_MAX     0xffff        /* maximum unsigned short value */
//#define INT_MIN     (-2147483647 - 1) /* minimum (signed) int value */
//#define INT_MAX       2147483647    /* maximum (signed) int value */
//#define UINT_MAX      0xffffffff    /* maximum unsigned int value */
//#define LONG_MIN    (-2147483647L - 1) /* minimum (signed) long value */
//#define LONG_MAX      2147483647L   /* maximum (signed) long value */
//#define ULONG_MAX     0xffffffffUL  /* maximum unsigned long value */
//#define LLONG_MAX     9223372036854775807i64       /* maximum signed long long int value */
//#define LLONG_MIN   (-9223372036854775807i64 - 1)  /* minimum signed long long int value */
//#define ULLONG_MAX    0xffffffffffffffffui64       /* maximum unsigned long long int value */

#define _I8_MIN     (-127i8 - 1)    /* minimum signed 8 bit value */
#define _I8_MAX       127i8         /* maximum signed 8 bit value */
#define _UI8_MAX      0xffui8       /* maximum unsigned 8 bit value */

#define _I16_MIN    (-32767i16 - 1) /* minimum signed 16 bit value */
#define _I16_MAX      32767i16      /* maximum signed 16 bit value */
#define _UI16_MAX     0xffffui16    /* maximum unsigned 16 bit value */

#define _I32_MIN    (-2147483647i32 - 1) /* minimum signed 32 bit value */
#define _I32_MAX      2147483647i32 /* maximum signed 32 bit value */
#define _UI32_MAX     0xffffffffui32 /* maximum unsigned 32 bit value */

/* minimum signed 64 bit value */
#define _I64_MIN    (-9223372036854775807i64 - 1)
/* maximum signed 64 bit value */
#define _I64_MAX      9223372036854775807i64
/* maximum unsigned 64 bit value */
#define _UI64_MAX     0xffffffffffffffffui64

#if     _INTEGRAL_MAX_BITS >= 128
/* minimum signed 128 bit value */
#define _I128_MIN   (-170141183460469231731687303715884105727i128 - 1)
/* maximum signed 128 bit value */
#define _I128_MAX     170141183460469231731687303715884105727i128
/* maximum unsigned 128 bit value */
#define _UI128_MAX    0xffffffffffffffffffffffffffffffffui128
#endif

#ifndef SIZE_MAX
#ifdef _WIN64 
#define SIZE_MAX _UI64_MAX
#else
#define SIZE_MAX UINT_MAX
#endif
#endif

#if __STDC_WANT_SECURE_LIB__
/* While waiting to the C standard committee to finalize the decision on RSIZE_MAX and rsize_t,
* we define RSIZE_MAX as SIZE_MAX
*/
#ifndef RSIZE_MAX
#define RSIZE_MAX SIZE_MAX
#endif
#endif

//#include "lopcodes.h"-----------------------------------------------------------------------------------------------------
/*===========================================================================
We assume that instructions are unsigned numbers.
All instructions have an opcode in the first 6 bits.
Instructions can have the following fields:
`A' : 8 bits
`B' : 9 bits
`C' : 9 bits
`Bx' : 18 bits (`B' and `C' together)
`sBx' : signed Bx

A signed argument is represented in excess K; that is, the number
value is the unsigned value minus K. K is exactly the maximum value
for that argument (so that -max is represented by 0, and +max is
represented by 2*max), which is half the maximum for the corresponding
unsigned argument.
===========================================================================*/


enum OpMode {iABC, iABx, iAsBx};  /* basic instruction format */


/*
** size and position of opcode arguments.
*/
#define SIZE_C		9
#define SIZE_B		9
#define SIZE_Bx		(SIZE_C + SIZE_B)
#define SIZE_A		8

#define SIZE_OP		6

#define POS_OP		0
#define POS_A		(POS_OP + SIZE_OP)
#define POS_C		(POS_A + SIZE_A)
#define POS_B		(POS_C + SIZE_C)
#define POS_Bx		POS_C


/*
** limits for opcode arguments.
** we use (signed) int to manipulate most arguments,
** so they must fit in LUAI_BITSINT-1 bits (-1 for sign)
*/
#if SIZE_Bx < LUAI_BITSINT-1
#define MAXARG_Bx        ((1<<SIZE_Bx)-1)
#define MAXARG_sBx        (MAXARG_Bx>>1)         /* `sBx' is signed */
#else
#define MAXARG_Bx        MAX_INT
#define MAXARG_sBx        MAX_INT
#endif


#define MAXARG_A        ((1<<SIZE_A)-1)
#define MAXARG_B        ((1<<SIZE_B)-1)
#define MAXARG_C        ((1<<SIZE_C)-1)


/* creates a mask with `n' 1 bits at position `p' */
#define MASK1(n,p)	((~((~(Instruction)0)<<n))<<p)

/* creates a mask with `n' 0 bits at position `p' */
#define MASK0(n,p)	(~MASK1(n,p))

/*
** the following macros help to manipulate instructions
*/

#define GET_OPCODE(i)	(cast(OpCode, ((i)>>POS_OP) & MASK1(SIZE_OP,0)))
#define SET_OPCODE(i,o)	((i) = (((i)&MASK0(SIZE_OP,POS_OP)) | \
	((cast(Instruction, o)<<POS_OP)&MASK1(SIZE_OP,POS_OP))))

#define GETARG_A(i)	(cast(int, ((i)>>POS_A) & MASK1(SIZE_A,0)))
#define SETARG_A(i,u)	((i) = (((i)&MASK0(SIZE_A,POS_A)) | \
	((cast(Instruction, u)<<POS_A)&MASK1(SIZE_A,POS_A))))

#define GETARG_B(i)	(cast(int, ((i)>>POS_B) & MASK1(SIZE_B,0)))
#define SETARG_B(i,b)	((i) = (((i)&MASK0(SIZE_B,POS_B)) | \
	((cast(Instruction, b)<<POS_B)&MASK1(SIZE_B,POS_B))))

#define GETARG_C(i)	(cast(int, ((i)>>POS_C) & MASK1(SIZE_C,0)))
#define SETARG_C(i,b)	((i) = (((i)&MASK0(SIZE_C,POS_C)) | \
	((cast(Instruction, b)<<POS_C)&MASK1(SIZE_C,POS_C))))

#define GETARG_Bx(i)	(cast(int, ((i)>>POS_Bx) & MASK1(SIZE_Bx,0)))
#define SETARG_Bx(i,b)	((i) = (((i)&MASK0(SIZE_Bx,POS_Bx)) | \
	((cast(Instruction, b)<<POS_Bx)&MASK1(SIZE_Bx,POS_Bx))))

#define GETARG_sBx(i)	(GETARG_Bx(i)-MAXARG_sBx)
#define SETARG_sBx(i,b)	SETARG_Bx((i),cast(unsigned int, (b)+MAXARG_sBx))


#define CREATE_ABC(o,a,b,c)	((cast(Instruction, o)<<POS_OP) \
	| (cast(Instruction, a)<<POS_A) \
	| (cast(Instruction, b)<<POS_B) \
	| (cast(Instruction, c)<<POS_C))

#define CREATE_ABx(o,a,bc)	((cast(Instruction, o)<<POS_OP) \
	| (cast(Instruction, a)<<POS_A) \
	| (cast(Instruction, bc)<<POS_Bx))


/*
** Macros to operate RK indices
*/

/* this bit 1 means constant (0 means register) */
#define BITRK		(1 << (SIZE_B - 1))

/* test whether value is a constant */
#define ISK(x)		((x) & BITRK)

/* gets the index of the constant */
#define INDEXK(r)	((int)(r) & ~BITRK)

#define MAXINDEXRK	(BITRK - 1)

/* code a constant index as a RK value */
#define RKASK(x)	((x) | BITRK)


/*
** invalid register that fits in 8 bits
*/
#define NO_REG		MAXARG_A


/*
** R(x) - register
** Kst(x) - constant (in constant table)
** RK(x) == if ISK(x) then Kst(INDEXK(x)) else R(x)
*/


/*
** grep "ORDER OP" if you change these enums
*/

typedef enum {
	/*----------------------------------------------------------------------
	name		args	description
	------------------------------------------------------------------------*/
	OP_MOVE,/*	A B	R(A) := R(B)					*/
	OP_LOADK,/*	A Bx	R(A) := Kst(Bx)					*/
	OP_LOADBOOL,/*	A B C	R(A) := (Bool)B; if (C) pc++			*/
	OP_LOADNIL,/*	A B	R(A) := ... := R(B) := nil			*/
	OP_GETUPVAL,/*	A B	R(A) := UpValue[B]				*/

	OP_GETGLOBAL,/*	A Bx	R(A) := Gbl[Kst(Bx)]				*/
	OP_GETTABLE,/*	A B C	R(A) := R(B)[RK(C)]				*/

	OP_SETGLOBAL,/*	A Bx	Gbl[Kst(Bx)] := R(A)				*/
	OP_SETUPVAL,/*	A B	UpValue[B] := R(A)				*/
	OP_SETTABLE,/*	A B C	R(A)[RK(B)] := RK(C)				*/

	OP_NEWTABLE,/*	A B C	R(A) := {} (size = B,C)				*/

	OP_SELF,/*	A B C	R(A+1) := R(B); R(A) := R(B)[RK(C)]		*/

	OP_ADD,/*	A B C	R(A) := RK(B) + RK(C)				*/
	OP_SUB,/*	A B C	R(A) := RK(B) - RK(C)				*/
	OP_MUL,/*	A B C	R(A) := RK(B) * RK(C)				*/
	OP_DIV,/*	A B C	R(A) := RK(B) / RK(C)				*/
	OP_MOD,/*	A B C	R(A) := RK(B) % RK(C)				*/
	OP_POW,/*	A B C	R(A) := RK(B) ^ RK(C)				*/
	OP_UNM,/*	A B	R(A) := -R(B)					*/
	OP_NOT,/*	A B	R(A) := not R(B)				*/
	OP_LEN,/*	A B	R(A) := length of R(B)				*/

	OP_CONCAT,/*	A B C	R(A) := R(B).. ... ..R(C)			*/

	OP_JMP,/*	sBx	pc+=sBx					*/

	OP_EQ,/*	A B C	if ((RK(B) == RK(C)) ~= A) then pc++		*/
	OP_LT,/*	A B C	if ((RK(B) <  RK(C)) ~= A) then pc++  		*/
	OP_LE,/*	A B C	if ((RK(B) <= RK(C)) ~= A) then pc++  		*/

	OP_TEST,/*	A C	if not (R(A) <=> C) then pc++			*/ 
	OP_TESTSET,/*	A B C	if (R(B) <=> C) then R(A) := R(B) else pc++	*/ 

	OP_CALL,/*	A B C	R(A), ... ,R(A+C-2) := R(A)(R(A+1), ... ,R(A+B-1)) */
	OP_TAILCALL,/*	A B C	return R(A)(R(A+1), ... ,R(A+B-1))		*/
	OP_RETURN,/*	A B	return R(A), ... ,R(A+B-2)	(see note)	*/

	OP_FORLOOP,/*	A sBx	R(A)+=R(A+2);
			   if R(A) <?= R(A+1) then { pc+=sBx; R(A+3)=R(A) }*/
			   OP_FORPREP,/*	A sBx	R(A)-=R(A+2); pc+=sBx				*/

			   OP_TFORLOOP,/*	A C	R(A+3), ... ,R(A+2+C) := R(A)(R(A+1), R(A+2)); 
						   if R(A+3) ~= nil then R(A+2)=R(A+3) else pc++	*/ 
						   OP_SETLIST,/*	A B C	R(A)[(C-1)*FPF+i] := R(A+i), 1 <= i <= B	*/

						   OP_CLOSE,/*	A 	close all variables in the stack up to (>=) R(A)*/
						   OP_CLOSURE,/*	A Bx	R(A) := closure(KPROTO[Bx], R(A), ... ,R(A+n))	*/

						   OP_VARARG/*	A B	R(A), R(A+1), ..., R(A+B-1) = vararg		*/
} OpCode;


#define NUM_OPCODES	(cast(int, OP_VARARG) + 1)



/*===========================================================================
Notes:
(*) In OP_CALL, if (B == 0) then B = top. C is the number of returns - 1,
and can be 0: OP_CALL then sets `top' to last_result+1, so
next open instruction (OP_CALL, OP_RETURN, OP_SETLIST) may use `top'.

(*) In OP_VARARG, if (B == 0) then use actual number of varargs and
set top (like in OP_CALL with C == 0).

(*) In OP_RETURN, if (B == 0) then return up to `top'

(*) In OP_SETLIST, if (B == 0) then B = `top';
if (C == 0) then next `instruction' is real C

(*) For comparisons, A specifies what condition the test should accept
(true or false).

(*) All `skips' (pc++) assume that next instruction is a jump
===========================================================================*/


/*
** masks for instruction properties. The format is:
** bits 0-1: op mode
** bits 2-3: C arg mode
** bits 4-5: B arg mode
** bit 6: instruction set register A
** bit 7: operator is a test
*/  

enum OpArgMask {
	OpArgN,  /* argument is not used */
	OpArgU,  /* argument is used */
	OpArgR,  /* argument is a register or a jump offset */
	OpArgK   /* argument is a constant or register/constant */
};

extern const lu_byte luaP_opmodes[NUM_OPCODES];

#define getOpMode(m)	(cast(enum OpMode, luaP_opmodes[m] & 3))
#define getBMode(m)	(cast(enum OpArgMask, (luaP_opmodes[m] >> 4) & 3))
#define getCMode(m)	(cast(enum OpArgMask, (luaP_opmodes[m] >> 2) & 3))
#define testAMode(m)	(luaP_opmodes[m] & (1 << 6))
#define testTMode(m)	(luaP_opmodes[m] & (1 << 7))


extern const char *const luaP_opnames[NUM_OPCODES+1];  /* opcode names */


/* number of list items to accumulate before a SETLIST instruction */
#define LFIELDS_PER_FLUSH	50

//#include "lobject.h"------------------------------------------------------------------------------------------------------
/* tags for values visible from Lua */
#define LAST_TAG	LUA_TTHREAD

#define NUM_TAGS	(LAST_TAG+1)


/*
** Extra tags for non-values
*/
#define LUA_TPROTO	(LAST_TAG+1)
#define LUA_TUPVAL	(LAST_TAG+2)
#define LUA_TDEADKEY	(LAST_TAG+3)


/*
** Union of all collectable objects
*/
typedef union GCObject GCObject;


/*
** Common Header for all collectable objects (in macro form, to be
** included in other objects)
*/
#define CommonHeader	GCObject *next; lu_byte tt; lu_byte marked


/*
** Common header in struct form
*/
typedef struct GCheader {
	CommonHeader;
} GCheader;




/*
** Union of all Lua values
*/
typedef union {
	GCObject *gc;
	void *p;
	lua_Number n;
	int b;
} Value;


/*
** Tagged Values
*/

#define TValuefields	Value value; int tt

typedef struct lua_TValue {
	TValuefields;
} TValue;


/* Macros to test type */
#define ttisnil(o)	(ttype(o) == LUA_TNIL)
#define ttisnumber(o)	(ttype(o) == LUA_TNUMBER)
#define ttisstring(o)	(ttype(o) == LUA_TSTRING)
#define ttistable(o)	(ttype(o) == LUA_TTABLE)
#define ttisfunction(o)	(ttype(o) == LUA_TFUNCTION)
#define ttisboolean(o)	(ttype(o) == LUA_TBOOLEAN)
#define ttisuserdata(o)	(ttype(o) == LUA_TUSERDATA)
#define ttisthread(o)	(ttype(o) == LUA_TTHREAD)
#define ttislightuserdata(o)	(ttype(o) == LUA_TLIGHTUSERDATA)

/* Macros to access values */
#define ttype(o)	((o)->tt)
#define gcvalue(o)	check_exp(iscollectable(o), (o)->value.gc)
#define pvalue(o)	check_exp(ttislightuserdata(o), (o)->value.p)
#define nvalue(o)	check_exp(ttisnumber(o), (o)->value.n)
#define rawtsvalue(o)	check_exp(ttisstring(o), &(o)->value.gc->ts)
#define tsvalue(o)	(&rawtsvalue(o)->tsv)
#define rawuvalue(o)	check_exp(ttisuserdata(o), &(o)->value.gc->u)
#define uvalue(o)	(&rawuvalue(o)->uv)
#define clvalue(o)	check_exp(ttisfunction(o), &(o)->value.gc->cl)
#define hvalue(o)	check_exp(ttistable(o), &(o)->value.gc->h)
#define bvalue(o)	check_exp(ttisboolean(o), (o)->value.b)
#define thvalue(o)	check_exp(ttisthread(o), &(o)->value.gc->th)

#define l_isfalse(o)	(ttisnil(o) || (ttisboolean(o) && bvalue(o) == 0))

/*
** for internal debug only
*/
#define checkconsistency(obj) \
	lua_assert(!iscollectable(obj) || (ttype(obj) == (obj)->value.gc->gch.tt))

#define checkliveness(g,obj) \
	lua_assert(!iscollectable(obj) || \
	((ttype(obj) == (obj)->value.gc->gch.tt) && !isdead(g, (obj)->value.gc)))


/* Macros to set values */
#define setnilvalue(obj) ((obj)->tt=LUA_TNIL)

#define setnvalue(obj,x) \
{ TValue *i_o=(obj); i_o->value.n=(x); i_o->tt=LUA_TNUMBER; }

#define setpvalue(obj,x) \
{ TValue *i_o=(obj); i_o->value.p=(x); i_o->tt=LUA_TLIGHTUSERDATA; }

#define setbvalue(obj,x) \
{ TValue *i_o=(obj); i_o->value.b=(x); i_o->tt=LUA_TBOOLEAN; }

#define setsvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TSTRING; \
	checkliveness(G(L),i_o); }

#define setuvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TUSERDATA; \
	checkliveness(G(L),i_o); }

#define setthvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TTHREAD; \
	checkliveness(G(L),i_o); }

#define setclvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TFUNCTION; \
	checkliveness(G(L),i_o); }

#define sethvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TTABLE; \
	checkliveness(G(L),i_o); }

#define setptvalue(L,obj,x) \
{ TValue *i_o=(obj); \
	i_o->value.gc=cast(GCObject *, (x)); i_o->tt=LUA_TPROTO; \
	checkliveness(G(L),i_o); }




#define setobj(L,obj1,obj2) \
{ const TValue *o2=(obj2); TValue *o1=(obj1); \
	o1->value = o2->value; o1->tt=o2->tt; \
	checkliveness(G(L),o1); }


/*
** different types of sets, according to destination
*/

/* from stack to (same) stack */
#define setobjs2s	setobj
/* to stack (not from same stack) */
#define setobj2s	setobj
#define setsvalue2s	setsvalue
#define sethvalue2s	sethvalue
#define setptvalue2s	setptvalue
/* from table to same table */
#define setobjt2t	setobj
/* to table */
#define setobj2t	setobj
/* to new object */
#define setobj2n	setobj
#define setsvalue2n	setsvalue

#define setttype(obj, tt) (ttype(obj) = (tt))


#define iscollectable(o)	(ttype(o) >= LUA_TSTRING)



typedef TValue *StkId;  /* index to stack elements */


/*
** String headers for string table
*/
typedef union TString {
	L_Umaxalign dummy;  /* ensures maximum alignment for strings */
	struct {
		CommonHeader;
		lu_byte reserved;
		unsigned int hash;
		size_t len;
	} tsv;
} TString;


#define getstr(ts)	cast(const char *, (ts) + 1)
#define svalue(o)       getstr(tsvalue(o))



typedef union Udata {
	L_Umaxalign dummy;  /* ensures maximum alignment for `local' udata */
	struct {
		CommonHeader;
		struct Table *metatable;
		struct Table *env;
		size_t len;
	} uv;
} Udata;




/*
** Function Prototypes
*/
typedef struct Proto {
	CommonHeader;
	TValue *k;  /* constants used by the function */
	Instruction *code;
	struct Proto **p;  /* functions defined inside the function */
	int *lineinfo;  /* map from opcodes to source lines */
	struct LocVar *locvars;  /* information about local variables */
	TString **upvalues;  /* upvalue names */
	TString  *source;
	int sizeupvalues;
	int sizek;  /* size of `k' */
	int sizecode;
	int sizelineinfo;
	int sizep;  /* size of `p' */
	int sizelocvars;
	int linedefined;
	int lastlinedefined;
	GCObject *gclist;
	lu_byte nups;  /* number of upvalues */
	lu_byte numparams;
	lu_byte is_vararg;
	lu_byte maxstacksize;
} Proto;


/* masks for new-style vararg */
#define VARARG_HASARG		1
#define VARARG_ISVARARG		2
#define VARARG_NEEDSARG		4


typedef struct LocVar {
	TString *varname;
	int startpc;  /* first point where variable is active */
	int endpc;    /* first point where variable is dead */
} LocVar;



/*
** Upvalues
*/

typedef struct UpVal {
	CommonHeader;
	TValue *v;  /* points to stack or to its own value */
	union {
		TValue value;  /* the value (when closed) */
		struct {  /* double linked list (when open) */
			struct UpVal *prev;
			struct UpVal *next;
		} l;
	} u;
} UpVal;


/*
** Closures
*/

#define ClosureHeader \
	CommonHeader; lu_byte isC; lu_byte nupvalues; GCObject *gclist; \
struct Table *env

typedef struct CClosure {
	ClosureHeader;
	lua_CFunction f;
	TValue upvalue[1];
} CClosure;


typedef struct LClosure {
	ClosureHeader;
	struct Proto *p;
	UpVal *upvals[1];
} LClosure;


typedef union Closure {
	CClosure c;
	LClosure l;
} Closure;


#define iscfunction(o)	(ttype(o) == LUA_TFUNCTION && clvalue(o)->c.isC)
#define isLfunction(o)	(ttype(o) == LUA_TFUNCTION && !clvalue(o)->c.isC)


/*
** Tables
*/

typedef union TKey {
	struct {
		TValuefields;
		struct Node *next;  /* for chaining */
	} nk;
	TValue tvk;
} TKey;


typedef struct Node {
	TValue i_val;
	TKey i_key;
} Node;


typedef struct Table {
	CommonHeader;
	lu_byte flags;  /* 1<<p means tagmethod(p) is not present */ 
	lu_byte lsizenode;  /* log2 of size of `node' array */
	struct Table *metatable;
	TValue *array;  /* array part */
	Node *node;
	Node *lastfree;  /* any free position is before this position */
	GCObject *gclist;
	int sizearray;  /* size of `array' array */
} Table;



/*
** `module' operation for hashing (size is always a power of 2)
*/
#define lmod(s,size) \
	(check_exp((size&(size-1))==0, (cast(int, (s) & ((size)-1)))))

#ifdef _WIN64
#define twoto(x)	(1LL<<(x))
#else
#define twoto(x)	(1<<(x))
#endif

#define sizenode(t)	(twoto((t)->lsizenode))


#define luaO_nilobject		(&luaO_nilobject_)

extern const TValue luaO_nilobject_;

#define ceillog2(x)	(luaO_log2((x)-1) + 1)

//#include "lfunc.h"--------------------------------------------------------------------------------------------------------
#define sizeCclosure(n)	(cast(int, sizeof(CClosure)) + \
	cast(int, sizeof(TValue)*((n)-1)))

#define sizeLclosure(n)	(cast(int, sizeof(LClosure)) + \
	cast(int, sizeof(TValue *)*((n)-1)))

//#include "ltable.h"-------------------------------------------------------------------------------------------------------
#define gnode(t,i)	(&(t)->node[i])
#define gkey(n)		(&(n)->i_key.nk)
#define gval(n)		(&(n)->i_val)
#define gnext(n)	((n)->i_key.nk.next)

#define key2tval(n)	(&(n)->i_key.tvk)

//#include "lgc.h"----------------------------------------------------------------------------------------------------------
/*
** Possible states of the Garbage Collector
*/
#define GCSpause	0
#define GCSpropagate	1
#define GCSsweepstring	2
#define GCSsweep	3
#define GCSfinalize	4


/*
** some userful bit tricks
*/
#define resetbits(x,m)	((x) &= cast(lu_byte, ~(m)))
#define setbits(x,m)	((x) |= (m))
#define testbits(x,m)	((x) & (m))
#define bitmask(b)	(1<<(b))
#define bit2mask(b1,b2)	(bitmask(b1) | bitmask(b2))
#define l_setbit(x,b)	setbits(x, bitmask(b))
#define resetbit(x,b)	resetbits(x, bitmask(b))
#define testbit(x,b)	testbits(x, bitmask(b))
#define set2bits(x,b1,b2)	setbits(x, (bit2mask(b1, b2)))
#define reset2bits(x,b1,b2)	resetbits(x, (bit2mask(b1, b2)))
#define test2bits(x,b1,b2)	testbits(x, (bit2mask(b1, b2)))



/*
** Layout for bit use in `marked' field:
** bit 0 - object is white (type 0)
** bit 1 - object is white (type 1)
** bit 2 - object is black
** bit 3 - for userdata: has been finalized
** bit 3 - for tables: has weak keys
** bit 4 - for tables: has weak values
** bit 5 - object is fixed (should not be collected)
** bit 6 - object is "super" fixed (only the main thread)
*/


#define WHITE0BIT	0
#define WHITE1BIT	1
#define BLACKBIT	2
#define FINALIZEDBIT	3
#define KEYWEAKBIT	3
#define VALUEWEAKBIT	4
#define FIXEDBIT	5
#define SFIXEDBIT	6
#define WHITEBITS	bit2mask(WHITE0BIT, WHITE1BIT)


#define iswhite(x)      test2bits((x)->gch.marked, WHITE0BIT, WHITE1BIT)
#define isblack(x)      testbit((x)->gch.marked, BLACKBIT)
#define isgray(x)	(!isblack(x) && !iswhite(x))

#define otherwhite(g)	(g->currentwhite ^ WHITEBITS)
#define isdead(g,v)	((v)->gch.marked & otherwhite(g) & WHITEBITS)

#define changewhite(x)	((x)->gch.marked ^= WHITEBITS)
#define gray2black(x)	l_setbit((x)->gch.marked, BLACKBIT)

#define valiswhite(x)	(iscollectable(x) && iswhite(gcvalue(x)))

#define luaC_white(g)	cast(lu_byte, (g)->currentwhite & WHITEBITS)


#define luaC_checkGC(L) { \
	condhardstacktests(luaD_reallocstack(L, L->stacksize - EXTRA_STACK - 1)); \
	if (G(L)->totalbytes >= G(L)->GCthreshold) \
	luaC_step(L); }


#define luaC_barrier(L,p,v) { if (valiswhite(v) && isblack(obj2gco(p)))  \
	luaC_barrierf(L,obj2gco(p),gcvalue(v)); }

#define luaC_barriert(L,t,v) { if (valiswhite(v) && isblack(obj2gco(t)))  \
	luaC_barrierback(L,t); }

#define luaC_objbarrier(L,p,o)  \
{ if (iswhite(obj2gco(o)) && isblack(obj2gco(p))) \
	luaC_barrierf(L,obj2gco(p),obj2gco(o)); }

#define luaC_objbarriert(L,t,o)  \
{ if (iswhite(obj2gco(o)) && isblack(obj2gco(t))) luaC_barrierback(L,t); }

//#include "ltm.h"----------------------------------------------------------------------------------------------------------
/*
* WARNING: if you change the order of this enumeration,
* grep "ORDER TM"
*/
typedef enum {
	TM_INDEX,
	TM_NEWINDEX,
	TM_GC,
	TM_MODE,
	TM_EQ,  /* last tag method with `fast' access */
	TM_ADD,
	TM_SUB,
	TM_MUL,
	TM_DIV,
	TM_MOD,
	TM_POW,
	TM_UNM,
	TM_LEN,
	TM_LT,
	TM_LE,
	TM_CONCAT,
	TM_CALL,
	TM_N		/* number of elements in the enum */
} TMS;



#define gfasttm(g,et,e) ((et) == NULL ? NULL : \
	((et)->flags & (1u<<(e))) ? NULL : luaT_gettm(et, e, (g)->tmname[e]))

#define fasttm(l,et,e)	gfasttm(G(l), et, e)

extern const char *const luaT_typenames[];


LUAI_FUNC const TValue *luaT_gettm (Table *events, TMS event, TString *ename);
LUAI_FUNC const TValue *luaT_gettmbyobj (lua_State *L, const TValue *o,
										 TMS event);
LUAI_FUNC void luaT_init (lua_State *L);

//#include "lzio.h"---------------------------------------------------------------------------------------------------------
#define EOZ	(-1)			/* end of stream */

typedef struct Zio ZIO;

#define char2int(c)	cast(int, cast(unsigned char, (c)))

#define zgetc(z)  (((z)->n--)>0 ?  char2int(*(z)->p++) : luaZ_fill(z))

typedef struct Mbuffer {
	char *buffer;
	size_t n;
	size_t buffsize;
} Mbuffer;

#define luaZ_initbuffer(L, buff) ((buff)->buffer = NULL, (buff)->buffsize = 0)

#define luaZ_buffer(buff)	((buff)->buffer)
#define luaZ_sizebuffer(buff)	((buff)->buffsize)
#define luaZ_bufflen(buff)	((buff)->n)

#define luaZ_resetbuffer(buff) ((buff)->n = 0)


#define luaZ_resizebuffer(L, buff, size) \
	(luaM_reallocvector(L, (buff)->buffer, (buff)->buffsize, size, char), \
	(buff)->buffsize = size)

#define luaZ_freebuffer(L, buff)	luaZ_resizebuffer(L, buff, 0)


LUAI_FUNC char *luaZ_openspace (lua_State *L, Mbuffer *buff, size_t n);
LUAI_FUNC void luaZ_init (lua_State *L, ZIO *z, lua_Reader reader,
						  void *data);
LUAI_FUNC size_t luaZ_read (ZIO* z, void* b, size_t n);	/* read next n bytes */
LUAI_FUNC int luaZ_lookahead (ZIO *z);



/* --------- Private Part ------------------ */

struct Zio {
	size_t n;			/* bytes still unread */
	const char *p;		/* current position in buffer */
	lua_Reader reader;
	void* data;			/* additional data */
	lua_State *L;			/* Lua state (for reader) */
};


LUAI_FUNC int luaZ_fill (ZIO *z);

//#include "lparser.h"------------------------------------------------------------------------------------------------------
/*
** Expression descriptor
*/

typedef enum {
	VVOID,	/* no value */
	VNIL,
	VTRUE,
	VFALSE,
	VK,		/* info = index of constant in `k' */
	VKNUM,	/* nval = numerical value */
	VLOCAL,	/* info = local register */
	VUPVAL,       /* info = index of upvalue in `upvalues' */
	VGLOBAL,	/* info = index of table; aux = index of global name in `k' */
	VINDEXED,	/* info = table register; aux = index register (or `k') */
	VJMP,		/* info = instruction pc */
	VRELOCABLE,	/* info = instruction pc */
	VNONRELOC,	/* info = result register */
	VCALL,	/* info = instruction pc */
	VVARARG	/* info = instruction pc */
} expkind;

typedef struct expdesc {
	expkind k;
	union {
		struct { int info, aux; } s;
		lua_Number nval;
	} u;
	int t;  /* patch list of `exit when true' */
	int f;  /* patch list of `exit when false' */
} expdesc;


typedef struct upvaldesc {
	lu_byte k;
	lu_byte info;
} upvaldesc;


struct BlockCnt;  /* defined in lparser.c */


/* state needed to generate code for a given function */
typedef struct FuncState {
	Proto *f;  /* current function header */
	Table *h;  /* table to find (and reuse) elements in `k' */
	struct FuncState *prev;  /* enclosing function */
	struct LexState *ls;  /* lexical state */
	struct lua_State *L;  /* copy of the Lua state */
	struct BlockCnt *bl;  /* chain of current blocks */
	int pc;  /* next position to code (equivalent to `ncode') */
	int lasttarget;   /* `pc' of last `jump target' */
	int jpc;  /* list of pending jumps to `pc' */
	int freereg;  /* first free register */
	int nk;  /* number of elements in `k' */
	int np;  /* number of elements in `p' */
	short nlocvars;  /* number of elements in `locvars' */
	lu_byte nactvar;  /* number of active local variables */
	upvaldesc upvalues[LUAI_MAXUPVALUES];  /* upvalues */
	unsigned short actvar[LUAI_MAXVARS];  /* declared-variable stack */
} FuncState;

//#include "lcode.h"--------------------------------------------------------------------------------------------------------
/*
** Marks the end of a patch list. It is an invalid value both as an absolute
** address, and as a list link (would link an element to itself).
*/
#define NO_JUMP (-1)


/*
** grep "ORDER OPR" if you change these enums
*/
typedef enum BinOpr {
	OPR_ADD, OPR_SUB, OPR_MUL, OPR_DIV, OPR_MOD, OPR_POW,
	OPR_CONCAT,
	OPR_NE, OPR_EQ,
	OPR_LT, OPR_LE, OPR_GT, OPR_GE,
	OPR_AND, OPR_OR,
	OPR_NOBINOPR
} BinOpr;


typedef enum UnOpr { OPR_MINUS, OPR_NOT, OPR_LEN, OPR_NOUNOPR } UnOpr;


#define getcode(fs,e)	((fs)->f->code[(e)->u.s.info])

#define luaK_codeAsBx(fs,o,A,sBx)	luaK_codeABx(fs,o,A,(sBx)+MAXARG_sBx)

#define luaK_setmultret(fs,e)	luaK_setreturns(fs, e, LUA_MULTRET)

//#include "llex.h"---------------------------------------------------------------------------------------------------------
#define FIRST_RESERVED	257

/* maximum length of a reserved word */
#define TOKEN_LEN	(sizeof("function")/sizeof(char))


/*
* WARNING: if you change the order of this enumeration,
* grep "ORDER RESERVED"
*/
enum RESERVED {
	/* terminal symbols denoted by reserved words */
	TK_AND = FIRST_RESERVED, TK_BREAK,
	TK_DO, TK_ELSE, TK_ELSEIF, TK_END, TK_FALSE, TK_FOR, TK_FUNCTION,
	TK_IF, TK_IN, TK_LOCAL, TK_NIL, TK_NOT, TK_OR, TK_REPEAT,
	TK_RETURN, TK_THEN, TK_TRUE, TK_UNTIL, TK_WHILE,
	/* other terminal symbols */
	TK_CONCAT, TK_DOTS, TK_EQ, TK_GE, TK_LE, TK_NE, TK_NUMBER,
	TK_NAME, TK_STRING, TK_EOS
};

/* number of reserved words */
#define NUM_RESERVED	(cast(int, TK_WHILE-FIRST_RESERVED+1))


/* array with token `names' */
extern const char *const luaX_tokens [];


typedef union {
	lua_Number r;
	TString *ts;
} SemInfo;  /* semantics information */


typedef struct Token {
	int token;
	SemInfo seminfo;
} Token;


typedef struct LexState {
	int current;  /* current character (charint) */
	int linenumber;  /* input line counter */
	int lastline;  /* line of last token `consumed' */
	Token t;  /* current token */
	Token lookahead;  /* look ahead token */
	struct FuncState *fs;  /* `FuncState' is private to the parser */
	struct lua_State *L;
	ZIO *z;  /* input stream */
	Mbuffer *buff;  /* buffer for tokens */
	TString *source;  /* current source name */
	char decpoint;  /* locale decimal point */
} LexState;

//#include "lstate.h"-------------------------------------------------------------------------------------------------------
struct lua_longjmp;  /* defined in ldo.c */


/* table of globals */
#define gt(L)	(&L->l_gt)

/* registry */
#define registry(L)	(&G(L)->l_registry)


/* extra stack space to handle TM calls and some other extras */
#define EXTRA_STACK   5


#define BASIC_CI_SIZE           8

#define BASIC_STACK_SIZE        (2*LUA_MINSTACK)



typedef struct stringtable {
  GCObject **hash;
  lu_int32 nuse;  /* number of elements */
  int size;
} stringtable;


/*
** informations about a call
*/
typedef struct CallInfo {
  StkId base;  /* base for this function */
  StkId func;  /* function index in the stack */
  StkId	top;  /* top for this function */
  const Instruction *savedpc;
  int nresults;  /* expected number of results from this function */
  int tailcalls;  /* number of tail calls lost under this entry */
} CallInfo;



#define curr_func(L)	(clvalue(L->ci->func))
#define ci_func(ci)	(clvalue((ci)->func))
#define f_isLua(ci)	(!ci_func(ci)->c.isC)
#define isLua(ci)	(ttisfunction((ci)->func) && f_isLua(ci))


/*
** `global state', shared by all threads of this state
*/
typedef struct global_State {
  stringtable strt;  /* hash table for strings */
  lua_Alloc frealloc;  /* function to reallocate memory */
  void *ud;         /* auxiliary data to `frealloc' */
  lu_byte currentwhite;
  lu_byte gcstate;  /* state of garbage collector */
  int sweepstrgc;  /* position of sweep in `strt' */
  GCObject *rootgc;  /* list of all collectable objects */
  GCObject **sweepgc;  /* position of sweep in `rootgc' */
  GCObject *gray;  /* list of gray objects */
  GCObject *grayagain;  /* list of objects to be traversed atomically */
  GCObject *weak;  /* list of weak tables (to be cleared) */
  GCObject *tmudata;  /* last element of list of userdata to be GC */
  Mbuffer buff;  /* temporary buffer for string concatentation */
  lu_mem GCthreshold;
  lu_mem totalbytes;  /* number of bytes currently allocated */
  lu_mem estimate;  /* an estimate of number of bytes actually in use */
  lu_mem gcdept;  /* how much GC is `behind schedule' */
  int gcpause;  /* size of pause between successive GCs */
  int gcstepmul;  /* GC `granularity' */
  lua_CFunction panic;  /* to be called in unprotected errors */
  TValue l_registry;
  struct lua_State *mainthread;
  UpVal uvhead;  /* head of double-linked list of all open upvalues */
  struct Table *mt[NUM_TAGS];  /* metatables for basic types */
  TString *tmname[TM_N];  /* array with tag-method names */
} global_State;

/*
** `per thread' state
*/
struct lua_State {
	CommonHeader;
	lu_byte status;
	StkId top;  /* first free slot in the stack */
	StkId base;  /* base of current function */
	global_State *l_G;
	CallInfo *ci;  /* call info for current function */
	const Instruction *savedpc;  /* `savedpc' of current function */
	StkId stack_last;  /* last free slot in the stack */
	StkId stack;  /* stack base */
	CallInfo *end_ci;  /* points after end of ci array*/
	CallInfo *base_ci;  /* array of CallInfo's */
	int stacksize;
	int size_ci;  /* size of array `base_ci' */
	unsigned short nCcalls;  /* number of nested C calls */
	lu_byte hookmask;
	lu_byte allowhook;
	int basehookcount;
	int hookcount;
	lua_Hook hook;
	TValue l_gt;  /* table of globals */
	TValue env;  /* temporary place for environments */
	GCObject *openupval;  /* list of open upvalues in this stack */
	GCObject *gclist;
	struct lua_longjmp *errorJmp;  /* current error recover point */
	ptrdiff_t errfunc;  /* current error handling function (stack index) */
};


#define G(L)	(L->l_G)


/*
** Union of all collectable objects
*/
union GCObject {
	GCheader gch;
	union TString ts;
	union Udata u;
	union Closure cl;
	struct Table h;
	struct Proto p;
	struct UpVal uv;
	struct lua_State th;  /* thread */
};


/* macros to convert a GCObject into a specific value */
#define rawgco2ts(o)	check_exp((o)->gch.tt == LUA_TSTRING, &((o)->ts))
#define gco2ts(o)	(&rawgco2ts(o)->tsv)
#define rawgco2u(o)	check_exp((o)->gch.tt == LUA_TUSERDATA, &((o)->u))
#define gco2u(o)	(&rawgco2u(o)->uv)
#define gco2cl(o)	check_exp((o)->gch.tt == LUA_TFUNCTION, &((o)->cl))
#define gco2h(o)	check_exp((o)->gch.tt == LUA_TTABLE, &((o)->h))
#define gco2p(o)	check_exp((o)->gch.tt == LUA_TPROTO, &((o)->p))
#define gco2uv(o)	check_exp((o)->gch.tt == LUA_TUPVAL, &((o)->uv))
#define ngcotouv(o) \
	check_exp((o) == NULL || (o)->gch.tt == LUA_TUPVAL, &((o)->uv))
#define gco2th(o)	check_exp((o)->gch.tt == LUA_TTHREAD, &((o)->th))

/* macro to convert any Lua object into a GCObject */
#define obj2gco(v)	(cast(GCObject *, (v)))

//#include "lstring.h"------------------------------------------------------------------------------------------------------
#define sizestring(s)	(sizeof(union TString)+((s)->len+1)*sizeof(char))

#define sizeudata(u)	(sizeof(union Udata)+(u)->len)

#define luaS_new(L, s)	(luaS_newlstr(L, s, strlen(s)))
#define luaS_newliteral(L, s)	(luaS_newlstr(L, "" s, \
	(sizeof(s)/sizeof(char))-1))

#define luaS_fix(s)	l_setbit((s)->tsv.marked, FIXEDBIT)

//#include "ldo.h"----------------------------------------------------------------------------------------------------------
#define luaD_checkstack(L,n)	\
	if ((char *)L->stack_last - (char *)L->top <= (n)*(int)sizeof(TValue)) \
	luaD_growstack(L, n); \
  else condhardstacktests(luaD_reallocstack(L, L->stacksize - EXTRA_STACK - 1));


#define incr_top(L) {luaD_checkstack(L,1); L->top++;}

#define savestack(L,p)		((char *)(p) - (char *)L->stack)
#define restorestack(L,n)	((TValue *)((char *)L->stack + (n)))

#define saveci(L,p)		((char *)(p) - (char *)L->base_ci)
#define restoreci(L,n)		((CallInfo *)((char *)L->base_ci + (n)))


/* results from luaD_precall */
#define PCRLUA		0	/* initiated a call to a Lua function */
#define PCRC		1	/* did a call to a C function */
#define PCRYIELD	2	/* C funtion yielded */


/* type of protected functions, to be ran by `runprotected' */
typedef void (*Pfunc) (lua_State *L, void *ud);

LUAI_FUNC int luaD_protectedparser (lua_State *L, ZIO *z, const char *name);
LUAI_FUNC void luaD_callhook (lua_State *L, int event, int line);
LUAI_FUNC int luaD_precall (lua_State *L, StkId func, int nresults);
LUAI_FUNC void luaD_call (lua_State *L, StkId func, int nResults);
LUAI_FUNC int luaD_pcall (lua_State *L, Pfunc func, void *u,
						  ptrdiff_t oldtop, ptrdiff_t ef);
LUAI_FUNC int luaD_poscall (lua_State *L, StkId firstResult);
LUAI_FUNC void luaD_reallocCI (lua_State *L, int newsize);
LUAI_FUNC void luaD_reallocstack (lua_State *L, int newsize);
LUAI_FUNC void luaD_growstack (lua_State *L, int n);

LUAI_FUNC void luaD_throw (lua_State *L, int errcode);
LUAI_FUNC int luaD_rawrunprotected (lua_State *L, Pfunc f, void *ud);

LUAI_FUNC void luaD_seterrorobj (lua_State *L, int errcode, StkId oldtop);


//#include "lundump.h"------------------------------------------------------------------------------------------------------
/* for header of binary files -- this is Lua 5.1 */
#define LUAC_VERSION		0x51
/* for header of binary files -- this is the official format */
#define LUAC_FORMAT		0
/* size of header of binary files */
#define LUAC_HEADERSIZE		12

//#include "lvm.h"----------------------------------------------------------------------------------------------------------
#define tostring(L,o) ((ttype(o) == LUA_TSTRING) || (luaV_tostring(L, o)))

#define tonumber(o,n)	(ttype(o) == LUA_TNUMBER || \
	(((o) = luaV_tonumber(o,n)) != NULL))

#define equalobj(L,o1,o2) \
	(ttype(o1) == ttype(o2) && luaV_equalval(L, o1, o2))




//------------------------export functions----------------------------------------------------------------------------------
//lua.h
/*
** state manipulation
*/
LUA_API lua_State *(lua_newstate) (lua_Alloc f, void *ud);
LUA_API void       (lua_close) (lua_State *L);
LUA_API lua_State *(lua_newthread) (lua_State *L);
LUA_API lua_CFunction (lua_atpanic) (lua_State *L, lua_CFunction panicf);
/*
** basic stack manipulation
*/
LUA_API int   (lua_gettop) (lua_State *L);
LUA_API void  (lua_settop) (lua_State *L, int idx);
LUA_API void  (lua_pushvalue) (lua_State *L, int idx);
LUA_API void  (lua_remove) (lua_State *L, int idx);
LUA_API void  (lua_insert) (lua_State *L, int idx);
LUA_API void  (lua_replace) (lua_State *L, int idx);
LUA_API int   (lua_checkstack) (lua_State *L, int sz);

LUA_API void  (lua_xmove) (lua_State *from, lua_State *to, int n);

/*
** access functions (stack -> C)
*/

LUA_API int             (lua_isnumber) (lua_State *L, int idx);
LUA_API int             (lua_isstring) (lua_State *L, int idx);
LUA_API int             (lua_iscfunction) (lua_State *L, int idx);
LUA_API int             (lua_isuserdata) (lua_State *L, int idx);
LUA_API int             (lua_type) (lua_State *L, int idx);
LUA_API const char     *(lua_typename) (lua_State *L, int tp);

LUA_API int            (lua_equal) (lua_State *L, int idx1, int idx2);
LUA_API int            (lua_rawequal) (lua_State *L, int idx1, int idx2);
LUA_API int            (lua_lessthan) (lua_State *L, int idx1, int idx2);

LUA_API lua_Number      (lua_tonumber) (lua_State *L, int idx);
LUA_API lua_Integer     (lua_tointeger) (lua_State *L, int idx);
LUA_API int             (lua_toboolean) (lua_State *L, int idx);
LUA_API const char     *(lua_tolstring) (lua_State *L, int idx, size_t *len);
LUA_API size_t          (lua_objlen) (lua_State *L, int idx);
LUA_API lua_CFunction   (lua_tocfunction) (lua_State *L, int idx);
LUA_API void	       *(lua_touserdata) (lua_State *L, int idx);
LUA_API lua_State      *(lua_tothread) (lua_State *L, int idx);
LUA_API const void     *(lua_topointer) (lua_State *L, int idx);
/*
** push functions (C -> stack)
*/
LUA_API void  (lua_pushnil) (lua_State *L);
LUA_API void  (lua_pushnumber) (lua_State *L, lua_Number n);
LUA_API void  (lua_pushinteger) (lua_State *L, lua_Integer n);
LUA_API void  (lua_pushlstring) (lua_State *L, const char *s, size_t l);
LUA_API void  (lua_pushstring) (lua_State *L, const char *s);
LUA_API const char *(lua_pushvfstring) (lua_State *L, const char *fmt,
										va_list argp);
LUA_API const char *(lua_pushfstring) (lua_State *L, const char *fmt, ...);
LUA_API void  (lua_pushcclosure) (lua_State *L, lua_CFunction fn, int n);
LUA_API void  (lua_pushboolean) (lua_State *L, int b);
LUA_API void  (lua_pushlightuserdata) (lua_State *L, void *p);
LUA_API int   (lua_pushthread) (lua_State *L);

/*
** get functions (Lua -> stack)
*/
LUA_API void  (lua_gettable) (lua_State *L, int idx);
LUA_API void  (lua_getfield) (lua_State *L, int idx, const char *k);
LUA_API void  (lua_rawget) (lua_State *L, int idx);
LUA_API void  (lua_rawgeti) (lua_State *L, int idx, int n);
LUA_API void  (lua_createtable) (lua_State *L, int narr, int nrec);
LUA_API void *(lua_newuserdata) (lua_State *L, size_t sz);
LUA_API int   (lua_getmetatable) (lua_State *L, int objindex);
LUA_API void  (lua_getfenv) (lua_State *L, int idx);


/*
** set functions (stack -> Lua)
*/
LUA_API void  (lua_settable) (lua_State *L, int idx);
LUA_API void  (lua_setfield) (lua_State *L, int idx, const char *k);
LUA_API void  (lua_rawset) (lua_State *L, int idx);
LUA_API void  (lua_rawseti) (lua_State *L, int idx, int n);
LUA_API int   (lua_setmetatable) (lua_State *L, int objindex);
LUA_API int   (lua_setfenv) (lua_State *L, int idx);

/*
** `load' and `call' functions (load and run Lua code)
*/
LUA_API void  (lua_call) (lua_State *L, int nargs, int nresults);
LUA_API int   (lua_pcall) (lua_State *L, int nargs, int nresults, int errfunc);
LUA_API int   (lua_cpcall) (lua_State *L, lua_CFunction func, void *ud);
LUA_API int   (lua_load) (lua_State *L, lua_Reader reader, void *dt,
						  const char *chunkname);

LUA_API int (lua_dump) (lua_State *L, lua_Writer writer, void *data);


/*
** coroutine functions
*/
LUA_API int  (lua_yield) (lua_State *L, int nresults);
LUA_API int  (lua_resume) (lua_State *L, int narg);
LUA_API int  (lua_status) (lua_State *L);


//lauxlib.h
LUALIB_API void (luaL_checkstack) (lua_State *L, int sz, const char *msg);
LUALIB_API int (luaL_loadfile) (lua_State *L, const char *filename);
LUALIB_API int (luaL_loadbuffer) (lua_State *L, const char *buff, size_t sz,const char *name);
LUALIB_API lua_State *(luaL_newstate) (void);
