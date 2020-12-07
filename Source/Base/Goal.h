#pragma once

namespace Framework
{
	namespace Base
	{

class Goal
{
public:
	enum Goal_Status
	{
		eInactive,  //The goal is waiting to be activated
		eActive,    //The goal has been activated and will be processed each update step
		eCompleted, //The goal has completed and will be removed on the next update
		eFailed     //The goal has failed and will either re-plan or be removed on the next update
	};
protected:
	Goal_Status m_Status;
	//TODO see if Owner and Type are necessary
public:
	virtual ~Goal() {}
	///This contains initialization logic and represents the planning phase of the goal.  A Goal is able to call its activate method
	///any number of times to re-plan if the situation demands.
	virtual void Activate() = 0;

	//TODO we may want to pass in the delta time slice here
	/// This is executed during the update step
	virtual Goal_Status Process(double dTime_s) = 0;
	/// This undertakes any necessary tidying up before a goal is exited and is called just before a goal is destroyed.
	virtual void Terminate() = 0;
	//bool HandleMessage()  //TODO get event equivalent
	//TODO see if AddSubgoal really needs to be at this level 
	Goal_Status GetStatus() const { return m_Status; }
	//Here is a very common call to do in the first line of a process update
	inline void ActivateIfInactive() { if (m_Status == eInactive) Activate(); }
	inline void ReActivateIfFailed() { if (m_Status == eFailed) Activate(); }
};

	}
}