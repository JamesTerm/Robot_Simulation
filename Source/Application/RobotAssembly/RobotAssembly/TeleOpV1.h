#pragma once

namespace Application
{
class Test_Swerve_Entity_Joystick;
class TeleOp_V1
{
public:
	void Reset();
	void init();
	void Start();
	void Stop();
private:
	std::shared_ptr<Test_Swerve_Entity_Joystick> m_Tele_Internal;
};

}