#include <windows.h>
#include "stdafx.h"

#pragma once

//typedef struct semaphore *	SEM_ID;
typedef CRITICAL_SECTION *	SEM_ID;

/* semaphore options */

#define SEM_Q_FIFO		 0x00	/* first in first out queue */
#define SEM_Q_PRIORITY		 0x01	/* priority sorted queue */
#define SEM_DELETE_SAFE		 0x04	/* owner delete safe (mutex opt.) */
#define SEM_INVERSION_SAFE	 0x08	/* no priority inversion (mutex opt.) */
#define SEM_EVENTSEND_ERR_NOTIFY 0x10	/* notify when eventRsrcSend fails */
#define SEM_INTERRUPTIBLE        0x20   /* interruptible on RTP signal */

/* timeout defines */

#define NO_WAIT		0
#define WAIT_FOREVER	(-1)

typedef enum 		/* SEM_TYPE */
{
	SEM_TYPE_BINARY,         /* 0: binary semaphore */
	SEM_TYPE_MUTEX,          /* 1: mutual exclusion semaphore */
	SEM_TYPE_COUNTING,       /* 2: counting semaphore */
	SEM_TYPE_OLD,	     /* 3: 4.x style semaphore  (not in user space) */
	SEM_TYPE_RW,	     /* 4: read/write semaphore (not in user space) */

	/* 
	* Add new semaphore types above this one.  Only 8 types 
	* are allowed without making major modifications to the
	* kernel 
	*/
	SEM_TYPE_MAX = 8	
} SEM_TYPE;

/* binary semaphore initial state */

typedef enum		/* SEM_B_STATE */
{
	SEM_EMPTY,			/* 0: semaphore not available */
	SEM_FULL			/* 1: semaphore available */
} SEM_B_STATE;

/* 
* Information structure filled by semInfoGet.  The taskIdListMax
* and taskIdList information is not provided in user land.
*/

typedef struct			/* SEM_INFO */
{
	UINT	numTasks;	/* OUT: number of blocked tasks */
	SEM_TYPE 	semType;	/* OUT: semaphore type */
	int		options;	/* OUT: options with which sem was created */
	union
	{
		UINT	count;		/* OUT: semaphore count (couting sems) */
		BOOL	full;		/* OUT: binary semaphore FULL? */
		int	owner;		/* OUT: task ID of mutex semaphore owner */
	} state;
#ifdef _WRS_KERNEL
	int		taskIdListMax;	/* IN: max tasks to fill in taskIdList */
	int *	taskIdList;	/* PTR: array of pending task IDs */
#endif /* _WRS_KERNEL */
} SEM_INFO;


extern SEM_ID 	  semMCreate 	(int options);
extern SEM_ID 	  semBCreate 	(int options, SEM_B_STATE initialState);
extern SEM_ID 	  semCCreate 	(int options, int initialCount);
extern STATUS 	  semDelete 	(SEM_ID semId);
extern STATUS 	  semFlush 	(SEM_ID semId);
extern STATUS 	  semGive 	(SEM_ID semId);
extern STATUS 	  semTake 	(SEM_ID semId, int timeout);
extern SEM_ID	  semOpen	(const char * name, SEM_TYPE type, 
							 int initState, int options, int mode,
							 void * context);
extern STATUS	  semInfoGet	(SEM_ID semId, SEM_INFO *pInfo);
extern STATUS 	  semClose	(SEM_ID semId);
extern STATUS 	  semUnlink	(const char * name);

#if 0
extern STATUS 	semBLibInit 	(void);
extern STATUS 	semCLibInit 	(void);
extern STATUS 	semMLibInit 	(void);
extern void	semOpenInit	(void);
extern STATUS   semMGiveForce 	(SEM_ID semId);
extern SEM_ID   semBInitialize  (char * pSemMem, int options, 
								 SEM_B_STATE initialState);
extern SEM_ID   semCInitialize  (char * pSemMem, int options, 
								 int initialCount);
extern SEM_ID   semMInitialize  (char * pSemMem, int options);

/* workaround Diab handling of deprecated attribute in C++ */

//extern STATUS 	semOLibInit 	(void) _WRS_DEPRECATED ("please use semBLib instead");
//extern SEM_ID 	semCreate 	(void) _WRS_DEPRECATED ("please use semBLib instead");
extern void 	semShowInit 	(void);
extern STATUS 	semShow 	(SEM_ID semId, int level);
extern int      semInfo         (SEM_ID semId, int idList[], int maxTasks);
#endif