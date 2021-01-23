#pragma once
#include <memory>
#include <functional>
#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Misc.h"
#include "../../../../Base/Poly.h"

//All rotary systems can fall into either a velocity controlled (like a shooter wheel, or drive)
//or position controlled like an arm or turret, both can be open or closed loop, which can be controlled by the client
//where open loop has no odometry callback.

namespace Module {
	namespace Robot {

class RotaryPosition_Internal;

struct rotary_properties
{
	//Optional: can provide good defaults to help client code to avoid managing all of them
	void Init();

	struct Entity1D_Props
	{
		//Stuff needed for physics
		double m_StartingPosition;  //the position used when reset position is called
		double m_Mass;
		//This can be used for the wheel diameter to work with RPS to linear conversions
		double m_Dimension; //Dimension- Length for linear and diameter for angular
		bool m_IsAngular;
	} entity_props;

	struct Ship_1D_Props
	{
		//Note there may be a difference between MAX_SPEED and MaxSpeed_Forward/MaxSpeed_Reverse, where MAX_SPEED represents the fastest speed something is capable of traveling, while
		// MaxSpeed_Forward/MaxSpeed_Reverse is the fastest desired speed the controller will want to manage this becomes more important in robotics where the gearing has some rotary
		// systems have a much faster max speed than what the desired speed would want to be.  These are also handy for button controlled operations to operate at a certain desired
		// max speed when held down
		double MAX_SPEED;
		double MaxSpeed_Forward, MaxSpeed_Reverse;
		double ACCEL, BRAKE;

		double MaxAccelForward, MaxAccelReverse;
		double MinRange, MaxRange;
		//This is used to avoid overshoot when trying to rotate to a heading
		double DistanceDegradeScaler;
		bool UsingRange;
	} ship_props;

	struct Rotary_Props
	{
		using PolynomialEquation_forth_Props = Framework::Base::PolynomialEquation_forth_Props;
		double VoltageScaler;		//Used to handle reversed voltage wiring
		//Note: EncoderToRS_Ratio is a place holder property that is implemented in the robot control
		//interface as needed for that control... it is not used in the rotary system code
		//The gear reduction used when multiplied by the encoder RPS will equal the *Rotary System's* RPS
		//This is typically the motor speed since this solves to apply voltage to it
		double EncoderToRS_Ratio;
		//Very similar to EncoderToRS_Ratio and is also a placeholder implemented in the robot control
			//to initialize the pulse count on the encoders (0 default implies 360)
		//While it ultimately solves the "gear reduction" it allows the script to specify the encoders specifications of the pulse count
		//while the EncoderToRS_Ratio can represent the actual gear reduction
		double EncoderPulsesPerRevolution;
		double PID[3]; //p,i,d
		double PrecisionTolerance;  //Used to manage voltage override and avoid oscillation
		//Currently supporting 4 terms in polynomial equation
		PolynomialEquation_forth_Props Voltage_Terms;  //Here is the curve fitting terms where 0th element is C, 1 = Cx^1, 2 = Cx^2, 3 = Cx^3 and so on...
		//This may be computed from stall torque and then torque at wheel (does not factor in traction) to linear in reciprocal form to avoid division
		//or alternatively solved empirically.  Using zero disables this feature
		double InverseMaxAccel;  //This is used to solve voltage at the acceleration level where the acceleration / max acceleration gets scaled down to voltage
		double InverseMaxDecel;  //used for deceleration case
		double Positive_DeadZone;
		double Negative_DeadZone;  //These must be in negative form
		double MinLimitRange, MaxLimitRange; //for position control these are the angles reset to when limit switches are triggered (only works for open loop)

		size_t Feedback_DiplayRow;  //Choose a row for display -1 for none (Only active if __DebugLUA__ is defined)
		enum LoopStates
		{
			eNone, //Will never read them (ideal for systems that do not have any encoders)
			eOpen,  //Will read them but never alter velocities
			eClosed, //Will attempt to match predicted velocity to actual velocity
			eClosed_ManualAssist //For position control this mode is also closed during manual assist
		} LoopState; //This should always be false once control is fully functional
		bool PID_Console_Dump;  //This will dump the console PID info (Only active if __DebugLUA__ is defined)

		//Only supported in Rotary_Velocity_Control
		bool UseAggressiveStop;  //If true, will use adverse force to assist in stopping.
		//Very similar to EncoderToRS_Ratio and is also a placeholder implemented in the robot control
		//This too is a method provided at startup to keep numbers positive
		bool EncoderReversed_Wheel;
		size_t AverageReadingsCount; //used to average readings great for latent reads or noise, but does introduce latency
		//Only supported in Rotary_Position_Control
		struct Rotary_Arm_GainAssist_Props
		{
			double PID_Up[3]; //p,i,d
			double PID_Down[3]; //p,i,d

			double InverseMaxAccel_Up;
			double InverseMaxDecel_Up;

			double InverseMaxAccel_Down;
			double InverseMaxDecel_Down;

			double SlowVelocityVoltage;  //Empirically solved as the max voltage to keep load just above steady state for worst case scenario
			double SlowVelocity;  //Rate at which the gain assist voltage gets blended out; This may be a bit more than the slow velocity used for SlowVelocityVoltage
			double GainAssistAngleScaler;  //Convert gear ratio into the readable ratio for cos() (i.e. GearToArmRatio)
			double ToleranceConsecutiveCount;
			//In milliseconds predict what the position will be by using the potentiometers velocity to help compensate for lag
			double VelocityPredictUp;
			double VelocityPredictDown;

			double PulseBurstTimeMs;  //Time in milliseconds for how long to enable pulse burst  (if zero this is disabled)
			double PulseBurstRange;  //Extended tolerance time to activate pulse burst
			bool UsePID_Up_Only;
		} ArmGainAssist;

		struct Voltage_Stall_Safety
		{
			//Note the on/off times will be shared resources of the gain assist
			double ErrorThreshold;  //solved by observing a run without obstacle and run with obstacle finding a level as close with some room for error
			double OnBurstLevel;  //the voltage level of the pulse
			size_t PulseBurstTimeOut; //specify the max number of pulses before it disengages the lock
			size_t StallCounterThreshold;  //specify the point when to activate pulse by counting stall time cycles
		} VoltageStallSafety;
	} rotary_props;
};

class RotarySystem_Position
{
public:
	RotarySystem_Position();
	// \param IntanceIndex optional to help diagnose which instance is being ran
	// \param props needs to be final and ready during init() call, uses default if null
	void Init(size_t InstanceIndex, rotary_properties *props=nullptr);
	void ShutDown();
	//Recommended units: radians or meters for linear
	//This allows setting the desired position
	//If this is angular it is always absolute, so relative can be managed by the caller
	//like written before where 0 is north, pi/2 east, etc.
	void SetPosition(double position);
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//set internal caching to a position, or PID variables as needed
	void Reset(double position = 0.0);
	//Output callback of new voltage to be set to output device (e.g. WPI, simulated odometry etc...)
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback);
	//provide access to the odometry for closed loops (e.g. potentiometer), use nullptr for open loops
	void SetOdometryCallback(std::function<double()> callback);
	//optional PID monitor for calibration
	using PID_Monitor_proto = void(double Voltage, double Position, double PredictedPosition, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset);
	void SetExternal_PID_Monitor_Callback(std::function<PID_Monitor_proto> callback);
private:
	std::shared_ptr<RotaryPosition_Internal> m_rotary_system;
};

class RotaryVelocity_Internal;
class RotarySystem_Velocity
{
public:
	RotarySystem_Velocity();
	// \param IntanceIndex optional to help diagnose which instance is being ran
	// \param props needs to be final and ready during init() call, uses default if null
	void Init(size_t InstanceIndex, rotary_properties *props = nullptr);
	void ShutDown();
	//Recommended units: radians per second
	//Note: For angular 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetVelocity(double rate);
	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//clear velocity caching, or PID variables as needed
	void Reset();
	//Output
	//Output callback of new voltage to be set to output device (e.g. WPI, simulated odometry etc...)
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback);
	//provide access to the odometry for closed loops (e.g. encoder), use nullptr for open loops
	void SetOdometryCallback(std::function<double()> callback);
	//optional PID monitor for calibration
	using PID_Monitor_proto = void(double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler);
	void SetExternal_PID_Monitor_Callback(std::function<PID_Monitor_proto> callback);
private:
	std::shared_ptr<RotaryVelocity_Internal> m_rotary_system;
};


	}
}