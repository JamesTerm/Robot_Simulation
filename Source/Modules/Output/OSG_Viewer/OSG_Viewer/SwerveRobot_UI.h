#pragma once

namespace Robot_Tester
{
class Swerve_Robot_UI;

class SwerveRobot_UI
{
public:
	union uSwerveRobot_State
	{
		struct SwerveRobot_State
		{
			//Use a simple struct to keep methods of return
			struct Vector2D
			{
				double x, y;
			};
			Vector2D Pos_m;
			enum Swerve_Robot_VelocityIndex
			{
				eWheel_FL,
				eWheel_FR,
				eWheel_RL,
				eWheel_RR,
				eSwivel_FL,
				eSwivel_FR,
				eSwivel_RL,
				eSwivel_RR,
				eNoSwerveRobotSpeedControllerDevices
			};
			double SwerveVelocitiesFromIndex[8]; //shows wheels angles and their velocities see Swerve_Robot_VelocityIndex
			double Att_r; //heading in radians
			double IntendedOrientation;
			//These should remain static:
		} bits;
		//give as an array for ease of updates in a full state loop
		struct AsRaw
		{
			double element[12];
		} raw;
	};
	using SwerveRobot_State = uSwerveRobot_State::SwerveRobot_State;
	SwerveRobot_UI();
	//Be sure to set callback before initialize
	void SetSwerveRobot_Callback(std::function<SwerveRobot_State()> callback);
	//Call this from viewer's SetSceneCallback() to this for adding; we'll not need to worry about removing for now
	void UpdateScene(void *geode, bool AddOrRemove);
	void Initialize();
	//Call this from viewer's SetUpdateCallback(), also change the state while in this callback
	void TimeChange(double dTime_s);
private:
	std::shared_ptr<Swerve_Robot_UI> m_Robot;
};
}