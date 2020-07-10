#pragma once

class RobotTester_Internal; //forward declare... 
class RobotAssem;

class RobotTester
{
public:
	void RobotTester_create(void); //separate creation and calling init() to allow setting hooks
	void RobotTester_init(void);  // allows the declaration to remain here
	void Shutdown();  //early call for some resources to close
	void Test(int test);
	void StartStreaming();
	void StopStreaming();
	void SetGameMode(int mode);  //currently 0=auton 1=tele and 2=test
	void ShowControls(bool show);  //off by default
	void HookSampleGoals(bool hook = true);  //Give UI access to manage this
	void RobotTester_SetParentBindCallback(std::function<void(RobotAssem *,bool)> callback);
private:
	std::shared_ptr<RobotTester_Internal> m_p_RobotTester; //a pimpl idiom (using shared_ptr allows declaration to be hidden from destructor)
};
