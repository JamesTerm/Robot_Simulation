#pragma once

namespace Application
{
class Test_Swerve_Rotary;
class TeleOp_V3
{
public:
	void Reset();
	void init();
	void Start();
	void Stop();
private:
	std::shared_ptr<Test_Swerve_Rotary> m_Tele_Internal;
};

}