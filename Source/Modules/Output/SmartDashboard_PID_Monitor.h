#pragma once
#include "../../Libraries/SmartDashboard/SmartDashboard_Import.h"
//#include "../Robot/SwerveRobot/SwerveRobot/RotarySystem.h"

//This simply transfers the PID variables to smart dashboard, as the rotary system should not be dependent on it

namespace Module
{
	namespace Output {


void Velocity_PID_Monitor(double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler)
{
	SmartDashboard::PutNumber("voltage", Voltage);
	SmartDashboard::PutNumber("desired_velocity", CurrentVelocity);
	SmartDashboard::PutNumber("actual_velocity", Encoder_Velocity);
	SmartDashboard::PutNumber("pid_error_offset", ErrorOffset);
	SmartDashboard::PutNumber("pid_scaler", ErrorOffset);
	//TODO we may want to have a check box check for this
	//Leave on for now because the SmartDashboard line plot doesn't really work that well
	printf("v=%.2f p=%.2f e=%.2f eo=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, ErrorOffset, CalibratedScaler);
}

void Position_PID_Monitor(double Voltage, double Position, double PredictedPosition, double CurrentVelocity, double Encoder_Velocity, double ErrorOffset)
{
	SmartDashboard::PutNumber("voltage", Voltage);
	SmartDashboard::PutNumber("actual_position", Position);
	SmartDashboard::PutNumber("desired_position", PredictedPosition);
	SmartDashboard::PutNumber("desired_velocity", CurrentVelocity);
	SmartDashboard::PutNumber("actual_velocity", Encoder_Velocity);
	SmartDashboard::PutNumber("pid_error_offset", ErrorOffset);
}

	}
}