#pragma once
#include "Ship1D_Legacy.h"

namespace Module {
	namespace Robot {
		namespace Legacy {
#pragma region _Rotary System Properties_
#pragma region _Rotary_Properties_

struct Rotary_Props
{
	using PolynomialEquation_forth_Props = Framework::Base::PolynomialEquation_forth_Props;
	double VoltageScalar;		//Used to handle reversed voltage wiring
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
		double GainAssistAngleScalar;  //Convert gear ratio into the readable ratio for cos() (i.e. GearToArmRatio)
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
};

class COMMON_API Rotary_Properties : public Ship_1D_Properties
{
public:
	void Init()
	{
		Rotary_Props props;
		memset(&props, 0, sizeof(Rotary_Props));

		props.VoltageScalar = 1.0;
		props.EncoderToRS_Ratio = 1.0;
		//Late assign this to override the initial default
		props.PID[0] = 1.0; //set PIDs to a safe default of 1,0,0
		props.PrecisionTolerance = 0.01;  //It is really hard to say what the default should be
		props.Feedback_DiplayRow = (size_t)-1;  //Only assigned to a row during calibration of feedback sensor
		props.LoopState = Rotary_Props::eNone;  //Always false when control is fully functional
		props.PID_Console_Dump = false;  //Always false unless you want to analyze PID (only one system at a time!)
		props.UseAggressiveStop = false;  //This is only for angular so false is a good default (must be explicit in script otherwise)
		props.Voltage_Terms.Init();
		props.InverseMaxAccel = props.InverseMaxDecel = 0.0;
		props.Positive_DeadZone = props.Negative_DeadZone = 0.0;
		props.MaxLimitRange = props.MinLimitRange = 0.0;  //just zero out; these are optionally used and explicitly set when used
		props.EncoderPulsesPerRevolution = 0.0;
		props.EncoderReversed_Wheel = false;
		props.AverageReadingsCount = 0;
		Rotary_Props::Rotary_Arm_GainAssist_Props &arm = props.ArmGainAssist;
		arm.SlowVelocity = arm.SlowVelocityVoltage = 0.0;
		arm.GainAssistAngleScalar = 1.0;
		arm.InverseMaxAccel_Down = arm.InverseMaxAccel_Up = arm.InverseMaxDecel_Down = arm.InverseMaxDecel_Up = 0.0;
		for (size_t i = 0; i < 3; i++)
			arm.PID_Down[i] = arm.PID_Up[i] = 0.0;
		arm.ToleranceConsecutiveCount = 1;
		arm.VelocityPredictUp = arm.VelocityPredictDown = 0.0;
		arm.UsePID_Up_Only = false;
		arm.PulseBurstRange = 0.0;
		arm.PulseBurstTimeMs = 0.0;
		m_RotaryProps = props;
	}
	Rotary_Properties(const char EntityName[], double Mass, double Dimension,
		double MAX_SPEED, double ACCEL, double BRAKE, double MaxAccelForward, double MaxAccelReverse,
		//Ship_Type ShipType = Ship_1D_Props::eDefault,
		bool UsingRange = false, double MinRange = 0.0, double MaxRange = 0.0,
		bool IsAngular = false) : Ship_1D_Properties(EntityName, Mass, Dimension, MAX_SPEED, ACCEL, BRAKE, MaxAccelForward,
			MaxAccelReverse, //ShipType, 
			UsingRange, MinRange, MaxRange, IsAngular) {
		Init();
	}

	Rotary_Properties() { Init(); }
	//virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults = false);
	const Rotary_Props &GetRotaryProps() const { return m_RotaryProps; }
	//Get and Set the properties
	Rotary_Props &RotaryProps() { return m_RotaryProps; }
protected:
	Rotary_Props m_RotaryProps;
private:
};
#pragma endregion
#pragma region _Rotary_Pot_Props_
		struct COMMON_API Rotary_Pot_Props
		{
			//These are addition attributes for any generic potentiometer  (This implies used only for position control)

			using PolynomialEquation_forth_Props = Framework::Base::PolynomialEquation_forth_Props;
			// init to some meaning data
			Rotary_Pot_Props() : IsFlipped(false), PotMaxValue(1000.0), PotMinValue(0.0), PotentiometerOffset(0.0) {}
			bool IsFlipped;  // is the range flipped
			double PotMaxValue;  //highest you want the potentiometer to read (add some padding)
			double PotMinValue;
			//This allows adjustment of the potentiometer in software to avoid manual recalibration.
			double PotentiometerOffset;
			double PotLimitTolerance;  //Specify extra padding to avoid accidental trigger of the safety
			PolynomialEquation_forth_Props PotPolyTerms; //in some environments the potentiometer is not linear
		};
#pragma endregion
#pragma region _Rotary_Pot_Properties_
		class COMMON_API Rotary_Pot_Properties : public Rotary_Properties
		{
		public:
			void Pot_Init() { m_RotaryPotProps.PotPolyTerms.Init(); }
			Rotary_Pot_Properties(const char EntityName[], double Mass, double Dimension,
				double MAX_SPEED, double ACCEL, double BRAKE, double MaxAccelForward, double MaxAccelReverse,
				//Ship_Type ShipType = Ship_1D_Props::eDefault, 
				bool UsingRange = false, double MinRange = 0.0, double MaxRange = 0.0,
				bool IsAngular = false) : Rotary_Properties(EntityName, Mass, Dimension, MAX_SPEED, ACCEL, BRAKE, MaxAccelForward,
					MaxAccelReverse,
					//ShipType, 
					UsingRange, MinRange, MaxRange, IsAngular) {
				Pot_Init();
			}
			Rotary_Pot_Properties() { Pot_Init(); }
			//virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults = false);
			const Rotary_Pot_Props &GetRotary_Pot_Properties() const { return m_RotaryPotProps; }
		protected:
			Rotary_Pot_Props m_RotaryPotProps;
		private:
		};
#pragma endregion

#pragma endregion

		}
	}
}