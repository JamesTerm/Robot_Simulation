#include "stdafx.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"
#include "Robot_Tester.h"
#include "../../RobotAssembly/RobotAssembly/TeleOpV1.h"
#include "../../RobotAssembly/RobotAssembly/TeleOpV2.h"
#include "../../RobotAssembly/RobotAssembly/TeleOpV3.h"
#include "../../RobotAssembly/RobotAssembly/TeleAutonV1.h"


//I'm leaving it able to test older versions.
//version 1 is completely detached from the viewer and keyboard support, but is still useful to review and learn because it simpler
//Then you can compare how the viewer is integrated (which isn't much, except that it controls the threading internally)
//We'll keep this convention going so you can see how it builds as more features get integrated
class RobotTester_Internal
{
private:
	//Application::TeleOp_V1 m_tele;
	//Application::TeleOp_V3 m_tele;
	Application::TeleAuton_V1 m_tele;
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
		m_tele.init();
	}
	void StartStreaming()
	{
		m_tele.Start();
	}
	void StopStreaming()
	{
		m_tele.Stop();
	}
	void SetGameMode(int mode)
	{
		//Note: previous versions do not support this method
		m_tele.SetGameMode(mode);
	}
	void Test(int test)
	{
		//Note: previous versions do not support this method but should
		m_tele.Test(test);
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
	//Driver station can't really call this directly,  however the start with test mode could have a method for a test number
	//(e.g. SmartDashboard) and then call the tester's method, while the console can obtain a test number by command line
	m_p_RobotTester->Test(test);
}

void RobotTester::SetGameMode(int mode)
{
	m_p_RobotTester->SetGameMode(mode);
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