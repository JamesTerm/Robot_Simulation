/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __NTTASK_H__
#define __NTTASK_H__

#if (defined __vxworks || defined _WINDOWS)

#include "ErrorBase.h"
#ifdef __vxworks
#include <vxWorks.h>
#endif

/**
 * WPI task is a wrapper for the native Task object.
 * All WPILib tasks are managed by a static task manager for simplified cleanup.
 **/
class NTTask : public ErrorBase
{
public:
	static const UINT32 kDefaultPriority = 101;
	static const INT32 kInvalidTaskID = -1;

	NTTask(const char* name, FUNCPTR function, INT32 priority = kDefaultPriority, UINT32 stackSize = 20000);
	virtual ~NTTask();

	bool Start(size_t arg0 = 0, size_t arg1 = 0, size_t arg2 = 0, size_t arg3 = 0, size_t arg4 = 0, 
			size_t arg5 = 0, size_t arg6 = 0, size_t arg7 = 0, size_t arg8 = 0, size_t arg9 = 0);
	bool Restart();
	bool Stop();

	bool IsReady();
	bool IsSuspended();

	bool Suspend();
	bool Resume();

	bool Verify();

	INT32 GetPriority();
	bool SetPriority(INT32 priority);
	const char* GetName();
	INT32 GetID();

	#ifdef _WINDOWS
	FUNCPTR m_function;
	size_t m_Arg[10];
	#endif
private:
	char* m_taskName;

	#ifdef _WINDOWS
	bool StartInternal();
	bool CloseThread();  //This will wait until thread procedure has finished
	HANDLE m_Handle;
	DWORD m_ID;
	#else
	FUNCPTR m_function;
	INT32 m_taskID;
	#endif

	UINT32 m_stackSize;
	INT32 m_priority;
	bool HandleError(STATUS results);
	DISALLOW_COPY_AND_ASSIGN(NTTask);
};

#endif // __vxworks
#endif // __TASK_H__
