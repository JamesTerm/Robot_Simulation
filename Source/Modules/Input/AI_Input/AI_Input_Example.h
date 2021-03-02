#pragma once

#include "AI_Input.h"
#include "../../../Base/AssetManager.h"

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
	//our autonomous really should have access to the properties, especially to make tweaks of the autonomous settings in script
	void Initialize(const Framework::Base::asset_manager* props = nullptr);
	virtual Framework::Base::Goal& GetGoal() override;
private:
	std::shared_ptr<AI_Example_internal> m_AI_Input;
};

	}
}