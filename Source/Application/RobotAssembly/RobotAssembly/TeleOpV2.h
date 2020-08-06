#pragma once

namespace Application
{
class Test_Swerve_Viewer;
class TeleOp_V2
{
public:
	void Reset();
	void init();
	void Start();
	void Stop();
private:
	std::shared_ptr<Test_Swerve_Viewer> m_Tele_Internal;
};

}