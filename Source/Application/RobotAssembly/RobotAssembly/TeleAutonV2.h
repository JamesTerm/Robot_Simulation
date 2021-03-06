#pragma once
#include <memory>

namespace Application
{
class Test_Swerve_Properties;
class TeleAuton_V2
{
public:
	void Reset();
	void init();
	void Start();
	void Stop();
	void Test(int test);
	void SetGameMode(int mode);  //currently 0=auton 1=tele and 2=test
private:
	std::shared_ptr<Test_Swerve_Properties> m_Tele_Internal;
};

}