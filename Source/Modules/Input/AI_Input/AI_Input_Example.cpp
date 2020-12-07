#include "StdAfx.h"
#include "AI_Input_Example.h"
#include "Goal_Types.h"

using namespace Framework::Base;

namespace Module
{
	namespace Input
	{

//now this goal can be whatever we want with access to robot resources
class AI_Example_internal : public AtomicGoal
{
private:
	AI_Input* m_pParent=nullptr;
public:
	AI_Example_internal(AI_Input* parent) : m_pParent(parent)
	{
		//Testing access
		//m_pParent->GetCurrentHeading();
	}
};

AI_Example::AI_Example()
{
	m_AI_Input = std::make_shared<AI_Example_internal>(this);
}

Goal& AI_Example::GetGoal()
{
	return *m_AI_Input;
}
	}
}