#include "stdafx.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "Robot_Tester.h"
#include "../../RobotAssembly/RobotAssembly/TeleOpV1.h"

class RobotTester_Internal
{
private:
	Application::TeleOp_V1 m_tele_v1;
public:
	RobotTester_Internal()
	{
		SmartDashboard::init();
	}
	void Shutdown()
	{
		StopStreaming();
		//TODO do we really need this, why is it crashing
		//SmartDashboard::shutdown();
	}
	void init()
	{
		m_tele_v1.init();
	}
	void StartStreaming()
	{
		m_tele_v1.Start();
	}
	void StopStreaming()
	{
		m_tele_v1.Stop();
	}
};


#pragma region _wrapper methods_
void RobotTester::RobotTester_create(void)
{
	m_p_RobotTester = std::make_shared<RobotTester_Internal>();
}

void RobotTester::RobotTester_init()
{
	m_p_RobotTester->init();  //go ahead and init
}

void RobotTester::Shutdown()
{
	m_p_RobotTester->Shutdown();
}

void RobotTester::Test(int test)
{
}

void RobotTester::SetGameMode(int mode)
{
	//m_p_RobotTester->SetGameMode(mode);
}

void RobotTester::StartStreaming()
{
	m_p_RobotTester->StartStreaming();
}
void RobotTester::StopStreaming()
{
	m_p_RobotTester->StopStreaming();
}

void RobotTester::ShowControls(bool show)
{
	//m_p_RobotTester->ShowControls(show);
}

void RobotTester::HookSampleGoals(bool hook)
{
	//m_p_RobotTester->HookSampleGoals(hook);
}

void RobotTester::RobotTester_SetParentBindCallback(std::function<void(RobotAssem *, bool)> callback)
{
	//m_p_RobotTester->RobotTester_SetParentBindCallback(callback);
}
#pragma endregion