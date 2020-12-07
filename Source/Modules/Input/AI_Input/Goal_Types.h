#pragma once
#include <list>
#include "../../../Base/Goal.h"

//This is internal to provide a common goal foundation for various goals.  It is also possible to break down various components 
//of the same goal into multiple files by use of these goal types.  To be consistent with goals these use the same namespace as
//goals, but do not need to be a part of the framework because no other part of the robot need to know about them; however,
//that said, this could change once I have change to evaluate hybrid cases.


namespace Framework
{
	namespace Base
	{

class AtomicGoal : public Goal
{
	protected:  //from Goal
		virtual void Activate() {}
		virtual Goal_Status Process(double dTime_s) {return eCompleted;}
		virtual void Terminate() {}
		//bool HandleMessage()  //TODO get event equivalent

};

class CompositeGoal : public Goal
{
	protected:  //from Goal
		~CompositeGoal()
		{
			RemoveAllSubgoals();
		}
		virtual void Activate() {}
		virtual Goal_Status Process(double dTime_s) {return eCompleted;}
		virtual void Terminate() {}
		//bool HandleMessage()  //TODO get event equivalent
		//Subgoals are pushed in LIFO like a stack
		virtual void AddSubgoal(Goal *g) {m_SubGoals.push_front(g);}
		//Feel free to make this virtual if we find that necessary
		/// All composite goals call this method each update step to process their subgoals.  The method ensures that all completed and failed goals
		/// are removed from the list before processing the next subgoal in line and returning its status.  If the subgoal is empty eCompleted is
		/// returned.
		Goal_Status ProcessSubgoals(double dTime_s)
		{
			Goal_Status StatusOfSubGoals;
			//Remove all completed and failed goals from the front of the subgoal list
			while (!m_SubGoals.empty() && (m_SubGoals.front()->GetStatus() == eCompleted || m_SubGoals.front()->GetStatus() == eFailed))
			{
				m_SubGoals.front()->Terminate();
				delete m_SubGoals.front();
				m_SubGoals.pop_front();
			}
			//If any subgoals remain, process the one at the front of the list
			if (!m_SubGoals.empty())
			{
				//grab the status of the front-most subgoal
				StatusOfSubGoals = m_SubGoals.front()->Process(dTime_s);

				//we have to test for the special case where the front-most subgoal reports "completed" and the subgoal list contains additional goals.
				//When this is the case, to ensure the parent keeps processing its subgoal list, the "active" status is returned.
				if (StatusOfSubGoals == eCompleted && m_SubGoals.size() > 1)
					StatusOfSubGoals = eActive;
			}
			else
				StatusOfSubGoals = eCompleted;
			return StatusOfSubGoals;
		}
		void RemoveAllSubgoals()
		{
			for (SubgoalList::iterator it = m_SubGoals.begin(); it != m_SubGoals.end(); ++it)
			{
				(*it)->Terminate();
				delete* it;
			}
			m_SubGoals.clear();
		}
	private:
		typedef std::list<Goal *> SubgoalList;
		SubgoalList m_SubGoals;
};

//Similar to a Composite goal where it is composed of a list of goals, but this one will process all goals simultaneously
class MultitaskGoal : public Goal
{
public:
	/// \param WaitAll if true the goal is active when the state of all objects are no longer active; If false it will either be the first
	/// completed goal or when all goals are no longer active (whichever comes first).  The idea is similar to WaitForMultipleObjects in win32.
	MultitaskGoal(bool WaitAll=true) : m_WaitAll(WaitAll)
	{
		m_Status = eInactive;
	}
	~MultitaskGoal()
	{
		RemoveAllGoals();
	}
	///first add the goals here
	void AddGoal(Goal *g) {m_GoalsToProcess.push_back(g);}
	///Then call this to manually activate once all goals are added
	virtual void Activate()
	{
		for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
			(*it)->Activate();
	}
	Goal &AsGoal() {return *this;}
protected:  //from Goal
	virtual Goal_Status Process(double dTime_s)
	{
		ActivateIfInactive();
		Goal_Status status = eFailed;
		size_t NonActiveCount = 0;

		bool SuccessDetected = false;
		//To keep things simple we'll always run a complete iteration of all the goals for the given process cycle, and simply or in the success
		//detected... This way any success that happened will be reflected and dealt with below
		for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
		{
			status = (*it)->Process(dTime_s);
			//If any subgoal fails... bail
			if (status == eFailed)
				return eFailed;
			if (status != eActive)
			{
				NonActiveCount++;
				SuccessDetected |= (status == eCompleted);
			}
		}

		//Either we wait until no more active goals exist, or if the wait all option is false, we can complete when the first detected successful completion
		//has occurred.  So for that... if no successful completion then it would fall back to the wait all logic and then evaluate if it failed
		const bool IsCompleted = ((NonActiveCount >= m_GoalsToProcess.size()) || ((!m_WaitAll) && (SuccessDetected)));
		if (!IsCompleted)
			m_Status = eActive;
		else
		{
			status = eCompleted;
			//Check the final status it is completed unless any goal failed
			for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
			{
				if ((*it)->GetStatus() == eFailed)
					status = eFailed;
			}
			m_Status = status;
		}
		return status;
	}
	virtual void Terminate()
	{
		//ensure its all clean
		RemoveAllGoals();
		m_Status = eInactive; //make this inactive

	}
	void RemoveAllGoals()
	{
		for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
		{
			(*it)->Terminate();
			delete* it;
		}
		m_GoalsToProcess.clear();
	}
private:
	typedef std::list<Goal *> GoalList;
	GoalList m_GoalsToProcess;
	bool m_WaitAll;
};

//This class can be used as a stand-alone composite which does nothing special but goes through a list
class Generic_CompositeGoal : public CompositeGoal
{
private:
	bool m_AutoActivate;
public:
	Generic_CompositeGoal(bool AutoActivate=false) : m_AutoActivate(AutoActivate)
	{
		m_Status=eInactive;
	}
	//give public access for client to populate goals
	virtual void AddSubgoal(Goal *g) {__super::AddSubgoal(g);}
	//client activates manually when goals are added
	virtual void Activate()
	{
		m_Status=eActive; 
	}

	virtual Goal_Status Process(double dTime_s)
	{
		//Client will activate if m_AutoActivate is not enabled
		if (m_AutoActivate)
			ActivateIfInactive();
		if (m_Status==eInactive)
			return m_Status;

		if (m_Status==eActive)
			m_Status=ProcessSubgoals(dTime_s);

		return m_Status;
	}
	virtual void Terminate()
	{
		//ensure its all clean
		RemoveAllSubgoals();
		m_Status=eInactive; //make this inactive
	}
};

	}
}