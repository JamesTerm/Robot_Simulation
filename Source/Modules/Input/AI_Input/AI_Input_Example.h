#pragma once

#include "AI_Input.h"

namespace Module
{
	namespace Input
	{
//Forward declare
class AI_Example_internal;

class AI_Example : public AI_Input
{
public:
	AI_Example();
	virtual Framework::Base::Goal& GetGoal() override;
private:
	std::shared_ptr<AI_Example_internal> m_AI_Input;
};

	}
}