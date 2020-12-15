#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../../Base/Base_Includes.h"
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"
#include "../../../../Base/Physics.h"
#include "../../../../Base/Poly.h"
#include "../../../../Base/PIDController.h"
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "RotarySystem.h"
#include "Ship1D_Legacy.h"

//Useful for diagnostics
//#define __UseBypass__
#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Rotary System Legacy_
namespace Legacy {

#pragma region _Rotary System Properties_
#pragma region _Rotary_Properties_
class COMMON_API Rotary_Properties : public Ship_1D_Properties
{
public:
	using Rotary_Props = rotary_properties::Rotary_Props;
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


#pragma region _CalibrationTesting_
//Note:  This section really belongs with the simulated odometry; however, given the legacy dependency on ship 1D and properties
//it is cleaner to keep the code intact in the order of dependencies, and rewrite a better updated simulation there
#pragma region _CT constants_
const double c_OptimalAngleUp_r = DEG_2_RAD(70.0);
const double c_OptimalAngleDn_r = DEG_2_RAD(50.0);
const double c_ArmToGearRatio = 72.0 / 28.0;
const double c_GearToArmRatio = 1.0 / c_ArmToGearRatio;
//const double c_PotentiometerToGearRatio=60.0/32.0;
//const double c_PotentiometerToArmRatio=c_PotentiometerToGearRatio * c_GearToArmRatio;
const double c_PotentiometerToArmRatio = 36.0 / 54.0;
const double c_PotentiometerToGearRatio = c_PotentiometerToArmRatio * c_ArmToGearRatio;
const double c_ArmToPotentiometerRatio = 1.0 / c_PotentiometerToArmRatio;
const double c_GearToPotentiometer = 1.0 / c_PotentiometerToGearRatio;
//const double c_TestRate=3.0;
//const double c_TestRate=6.0;
//const double c_Potentiometer_TestRate=18.0;
const double c_Potentiometer_TestRate = 24.0;
//const double c_Potentiometer_TestRate=48.0;
const double Polynomial[5] =
{
	//0.0,1.0   ,0.0    ,0.0   ,0.0
	0.0,2.4878,-2.2091,0.7134,0.0
};

const double c_DeadZone = 0.085;
const double c_MidRangeZone = .150;
const double c_MidDistance = c_MidRangeZone - c_DeadZone;
const double c_rMidDistance = 1.0 / c_MidDistance;
const double c_EndDistance = 1.0 - c_MidRangeZone;
const double c_rEndDistance = 1.0 / c_EndDistance;

#define ENCODER_TEST_RATE 1
#if ENCODER_TEST_RATE==0
const double c_Encoder_TestRate = 16.0;
const double c_Encoder_MaxAccel = 120.0;
#endif

#if ENCODER_TEST_RATE==1
const double c_Encoder_TestRate = 2.916;
const double c_Encoder_MaxAccel = 60.0;
#endif

#if ENCODER_TEST_RATE==2
const double c_Encoder_TestRate = 2.4;
const double c_Encoder_MaxAccel = 4.0;
#endif

#if ENCODER_TEST_RATE==3
const double c_Encoder_TestRate = 1.1;
const double c_Encoder_MaxAccel = 2.0;
#endif

const double c_OunceInchToNewton = 0.00706155183333;

const double c_CIM_Amp_To_Torque_oz = (1.0 / (133.0 - 2.7)) * 343.3;
const double c_CIM_Amp_To_Torque_nm = c_CIM_Amp_To_Torque_oz * c_OunceInchToNewton;
const double c_CIM_Vel_To_Torque_oz = (1.0 / (5310 / 60.0)) * 343.4;
const double c_CIM_Vel_To_Torque_nm = c_CIM_Vel_To_Torque_oz * c_OunceInchToNewton;
const double c_CIM_Torque_to_Vel_nm = 1.0 / c_CIM_Vel_To_Torque_nm;

#define __USE_PAYLOAD_MASS__
#pragma endregion
#pragma region _helper functions_
double GetTweakedVoltage(double Voltage)
{
	double VoltMag = fabs(Voltage);
	double sign = Voltage / VoltMag;

	//simulate stresses on the robot
	if (VoltMag > 0.0)
	{
		if (VoltMag < c_DeadZone)
		{
			//DebugOutput("Buzz %f\n", Voltage);   //simulate no movement but hearing the motor
			VoltMag = 0.0;
		}
		else if (VoltMag < c_MidRangeZone)
			VoltMag = (VoltMag - c_DeadZone) * c_rMidDistance * 0.5;
		else
			VoltMag = ((VoltMag - c_MidRangeZone) * c_rEndDistance * 0.5) + 0.5;

		Voltage = VoltMag * sign;
	}
	return Voltage;
}
static double AngleToHeight_m(double Angle_r)
{
	//TODO fix
	const double c_ArmToGearRatio = 72.0 / 28.0;
	const double c_GearToArmRatio = 1.0 / c_ArmToGearRatio;
	const double c_ArmLength_m = 1.8288;  //6 feet
	const double c_GearHeightOffset = 1.397;  //55 inches

	return (sin(Angle_r * c_GearToArmRatio) * c_ArmLength_m) + c_GearHeightOffset;
}
#pragma endregion
class COMMON_API Potentiometer_Tester : public Ship_1D
{
private:
	double m_Time_s=0.0;
	Ship_1D_Properties m_PotentiometerProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	bool m_Bypass;  //used for stress test
public:
	Potentiometer_Tester() : m_PotentiometerProps(
		"Potentiometer",
		2.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		c_Potentiometer_TestRate,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		c_Potentiometer_TestRate, c_Potentiometer_TestRate,
		//Ship_1D_Props::eRobotArm,
		true,	//Using the range
		DEG_2_RAD(-135.0), DEG_2_RAD(135.0)
	), Ship_1D("Potentiometer")
	{
		m_Bypass = false;
		Initialize(&m_PotentiometerProps);
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		Voltage = GetTweakedVoltage(Voltage);
		if (!m_Bypass)
			SetRequestedVelocity(Voltage * c_GearToPotentiometer * m_PotentiometerProps.GetMaxSpeed());
		else
			SetRequestedVelocity(0.0);
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		//Note this is in native potentiometer ratios
		double Pos_m = GetPos_m();
		double height = AngleToHeight_m(Pos_m);

		//DOUT5("Pot=%f Angle=%f %fft %fin",m_Physics.GetVelocity(),RAD_2_DEG(Pos_m*c_GearToArmRatio),height*3.2808399,height*39.3700787);

		return Pos_m;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		__super::TimeChange(m_Time_s);
	}
	void SetBypass(bool bypass) {m_Bypass=bypass;}
};
class COMMON_API Potentiometer_Tester2 : public Ship_1D
{
private:
	void SimulateOpposingForce(double Voltage)
	{
		const double OuterDistance = 3.8;
		const double CoreDistance = 4.0;

		const double Pos = GetPos_m();
		const double Velocity = m_Physics.GetVelocity();
		double ForceScalar = 0.0;
		if ((Pos > OuterDistance) && (Velocity >= 0.0))
		{
			//determine how much force is being applied
			const double Force = m_Physics.GetForceFromVelocity(Velocity + Velocity, m_Time_s);
			if (Pos < CoreDistance)
			{
				//dampen force depending on the current distance
				const double scale = 1.0 / (CoreDistance - OuterDistance);
				ForceScalar = (Pos - OuterDistance) * scale;
			}
			else
				ForceScalar = 1.0;
			m_Physics.ApplyFractionalForce(-Force * ForceScalar, m_Time_s);
		}
		SetRequestedVelocity(Voltage * (1.0 - ForceScalar) * m_PotentiometerProps.GetMaxSpeed());
	}
	double m_Time_s=0.0;
	Ship_1D_Properties m_PotentiometerProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	bool m_SimulateOpposingForce;  //used for stress test
public:
	Potentiometer_Tester2() : m_PotentiometerProps(
		"Potentiometer2",
		2.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		10.0,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		10.0, 10.0,
		//Ship_1D_Props::eSwivel,
		true,	//Using the range
		DEG_2_RAD(-180.0), DEG_2_RAD(180.0)
	), Ship_1D("Potentiometer2")
	{
		m_SimulateOpposingForce = false;
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		if (props)
			m_PotentiometerProps = *props;
		__super::Initialize(&m_PotentiometerProps);
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		//Voltage=GetTweakedVoltage(Voltage);
		if (!m_SimulateOpposingForce)
			SetRequestedVelocity(Voltage * m_PotentiometerProps.GetMaxSpeed());
		else
			SimulateOpposingForce(Voltage);
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		//Note this is in native potentiometer ratios
		double Pos_m = GetPos_m();
		return Pos_m;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		__super::TimeChange(m_Time_s);
	}
	void SetSimulateOpposingForce(bool Simulate) {m_SimulateOpposingForce=Simulate;}
};
class COMMON_API Encoder_Simulator : public Ship_1D
{
private:
	double m_Time_s;
	Ship_1D_Properties m_EncoderProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	Framework::Base::LatencyFilter m_Latency;
	double m_EncoderScalar; //used to implement reverse
	bool m_GetEncoderFirstCall;  //allows GetEncoderVelocity to know when a new set of calls occur within a time slice
public:
	Encoder_Simulator(const char *EntityName="EncSimulator") : m_Time_s(0.0), m_EncoderProps(
		EntityName,
		68.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		c_Encoder_TestRate,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		c_Encoder_MaxAccel, c_Encoder_MaxAccel,
		//Ship_1D_Props::eRobotArm,
		false	//Not using the range
	), Ship_1D(EntityName), m_Latency(0.300),
		m_EncoderScalar(1.0), m_GetEncoderFirstCall(false)
	{
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		if (props)
			m_EncoderProps = *props;
		__super::Initialize(&m_EncoderProps);
	}
	void UpdateEncoderVoltage(double Voltage)
	{
		//Here is some stress to emulate a bad curved victor
		#if 0
		//This is how it would be if the motor was set to non-coast
		double VoltageMag = pow(fabs(Voltage), 0.5);
		Voltage = (Voltage > 0.0) ? VoltageMag : -VoltageMag;
		#endif
		SetRequestedVelocity(Voltage * m_EncoderProps.GetMaxSpeed());
		#if 0
		//For coast it is more like applying force
		double Velocity = m_Physics.GetVelocity();
		double MaxSpeed = m_EncoderProps.GetMaxSpeed();
		double Accel = Voltage * 5000.0 * MaxSpeed * m_Time_s;  //determine current acceleration by applying the ratio with a scalar  
		double filter = 1.0 - (fabs(Velocity) / MaxSpeed);  //as there is more speed there is less torque
		//double friction=((Velocity>0.04)?-0.025 : (Velocity<-0.04)?+0.025 : 0.0 )* rTime ;  //There is a constant amount of friction opposite to current velocity
		double friction = ((Velocity > 0.04) ? -250 : (Velocity < -0.04) ? +250 : 0.0) * m_Time_s;  //There is a constant amount of friction opposite to current velocity
		SetCurrentLinearAcceleration((Accel * filter * filter) + friction); //square the filter (matches read out better)
		#endif
	}
	virtual double GetEncoderVelocity()
	{
		#if 0
		return m_Physics.GetVelocity() * m_EncoderScalar;
		#else
		//if (!m_GetEncoderFirstCall) return m_Latency();

		double Voltage = m_Physics.GetVelocity() / m_EncoderProps.GetMaxSpeed();
		double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		double ret = Voltage * m_EncoderProps.GetMaxSpeed() * m_EncoderScalar;
		//ret=m_Latency(ret,m_Time_s);
		m_GetEncoderFirstCall = false; //weed out the repeat calls
		return ret;
		#endif
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		m_GetEncoderFirstCall = true;
		__super::TimeChange(m_Time_s);
	}
	void SetReverseDirection(bool reverseDirection)
	{
		//emulates functionality of the encoder (needed because kids put them in differently)
		m_EncoderScalar = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScalar(double value) {m_EncoderScalar=value;}  //This helps to simulate differences between sides
	void SetFriction(double StaticFriction,double KineticFriction) {}
};

//TorqueAccelerationDampener - formerly known as ForceAppliedOnWheelRadius has become the solution to account for empirical testing of torque acceleration 
//measured in the motor, unfortunately I haven't yet found a way to compute for this, but this is quite effective as its factoring happens in the best
//place (same place as it was before)
struct EncoderSimulation_Props
{
	double Wheel_Mass;  //This is a total mass of all the wheels and gears for one side
	double COF_Efficiency;
	double GearReduction;  //In reciprocal form of spread sheet   driving gear / driven gear
	double TorqueAccelerationDampener; //ratio 1.0 no change
	double DriveWheelRadius; //in meters
	double NoMotors;  //Used to get total torque
	double PayloadMass;  //The robot weight in kg
	double SpeedLossConstant;
	double DriveTrainEfficiency;

	struct Motor_Specs
	{
		double FreeSpeed_RPM;
		double Stall_Torque_NM;
		double Stall_Current_Amp;
		double Free_Current_Amp;
	} motor;
};

//This is used in calibration testing to simulate encoder readings
class COMMON_API EncoderSimulation_Properties
{
	public:
		EncoderSimulation_Properties()
		{
			EncoderSimulation_Props props;
			memset(&props, 0, sizeof(EncoderSimulation_Props));

			props.Wheel_Mass = 1.5;
			props.COF_Efficiency = 1.0;
			props.GearReduction = 12.4158;
			props.TorqueAccelerationDampener = 0.0508;
			props.DriveWheelRadius = 0.0762;
			props.NoMotors = 1.0;
			props.PayloadMass = 200.0 * 0.453592;  //in kilograms
			props.SpeedLossConstant = 0.81;
			props.DriveTrainEfficiency = 0.9;


			props.motor.FreeSpeed_RPM = 5310;
			props.motor.Stall_Torque_NM = 343.4 * c_OunceInchToNewton;
			props.motor.Stall_Current_Amp = 133.0;
			props.motor.Free_Current_Amp = 2.7;

			m_EncoderSimulation_Props = props;
		}
		//virtual void LoadFromScript(GG_Framework::Logic::Scripting::Script& script);
		const EncoderSimulation_Props &GetEncoderSimulationProps() const {return m_EncoderSimulation_Props;}
		//Get and Set the properties
		EncoderSimulation_Props &EncoderSimulationProps() {return m_EncoderSimulation_Props;}

	protected:
		EncoderSimulation_Props m_EncoderSimulation_Props;
};

class COMMON_API Drive_Train_Characteristics
{
	public:
		Drive_Train_Characteristics()
		{
			EncoderSimulation_Properties default_props;
			m_Props = default_props.GetEncoderSimulationProps();
		}
		void UpdateProps(const EncoderSimulation_Props &props) {m_Props=props;}

		__inline double GetAmp_To_Torque_nm(double Amps) const
		{
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double c_Amp_To_Torque_nm = (1.0 / (_.Stall_Current_Amp - _.Free_Current_Amp)) * _.Stall_Torque_NM;
			return std::max((Amps - _.Free_Current_Amp) * c_Amp_To_Torque_nm, 0.0);
		}
		__inline double INV_GetVel_To_Torque_nm(double Vel_rps) const
		{
			//depreciated
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double c_Vel_To_Torque_nm = (1.0 / (_.FreeSpeed_RPM / 60.0)) * _.Stall_Torque_NM;
			return (Vel_rps * c_Vel_To_Torque_nm);
		}
		__inline double GetVel_To_Torque_nm(double motor_Vel_rps) const
		{
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double FreeSpeed_RPS = (_.FreeSpeed_RPM / 60.0);
			//ensure we don't exceed the max velocity
			const double x = std::min(fabs(motor_Vel_rps) / FreeSpeed_RPS, 1.0);  //working with normalized positive number for inversion
			const double Torque_nm((1.0 - x) * _.Stall_Torque_NM);
			return (motor_Vel_rps >= 0) ? Torque_nm : -Torque_nm;
		}
		__inline double GetTorque_To_Vel_nm_V1(double motor_Vel_rps) const
		{  //depreciated
			return 0.0;
		}
		__inline double GetWheelTorque(double Torque) const
		{
			return Torque / m_Props.GearReduction * m_Props.DriveTrainEfficiency;
		}
		__inline double INV_GetWheelTorque(double Torque) const
		{
			//depreciated
			return Torque * m_Props.GearReduction * m_Props.COF_Efficiency;
		}
		__inline double GetWheelStallTorque() const 
		{
			return m_Props.motor.Stall_Torque_NM / m_Props.GearReduction * m_Props.DriveTrainEfficiency;
		}
		__inline double GetTorqueAtWheel(double Torque) const
		{
			return (GetWheelTorque(Torque) / m_Props.DriveWheelRadius);
		}
		__inline double GetWheelRPS(double LinearVelocity) const
		{
			return LinearVelocity / (M_PI * 2.0 * m_Props.DriveWheelRadius);
		}
		__inline double GetLinearVelocity(double wheel_RPS) const
		{
			return wheel_RPS * (M_PI * 2.0 * m_Props.DriveWheelRadius);
		}
		__inline double GetLinearVelocity_WheelAngular(double wheel_AngularVelocity) const
		{
			return wheel_AngularVelocity * m_Props.DriveWheelRadius;
		}
		__inline double GetMotorRPS(double LinearVelocity) const
		{
			return GetWheelRPS(LinearVelocity) / m_Props.GearReduction;
		}
		__inline double GetWheelRPS_Angular(double wheel_AngularVelocity) const
		{
			return wheel_AngularVelocity / (M_PI * 2.0);
		}
		__inline double GetWheelAngular_RPS(double wheel_RPS) const
		{
			return wheel_RPS * (M_PI * 2.0);
		}
		__inline double GetWheelAngular_LinearVelocity(double LinearVelocity) const
		{
			//accurate as long as there is no skid
			return LinearVelocity / m_Props.DriveWheelRadius;
		}
		__inline double GetMotorRPS_Angular(double wheel_AngularVelocity) const
		{
			return GetWheelRPS_Angular(wheel_AngularVelocity) / m_Props.GearReduction;
		}
		__inline double INV_GetMotorRPS_Angular(double wheel_AngularVelocity) const
		{
			//depreciated
			return GetWheelRPS_Angular(wheel_AngularVelocity) * m_Props.GearReduction;
		}
		__inline double GetTorqueFromLinearVelocity(double LinearVelocity) const
		{
			const double MotorTorque = GetVel_To_Torque_nm(GetMotorRPS(LinearVelocity));
			return GetTorqueAtWheel(MotorTorque);
		}
		__inline double GetWheelTorqueFromVoltage(double Voltage) const
		{
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = GetTorqueAtWheel(MotorTorque * 2.0);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double GetTorqueFromVoltage_V1(double Voltage) const
		{
			//depreciated
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = INV_GetWheelTorque(MotorTorque * m_Props.NoMotors);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double GetTorqueFromVoltage(double Voltage) const
		{
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = GetWheelTorque(MotorTorque * m_Props.NoMotors);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double INV_GetTorqueFromVelocity(double wheel_AngularVelocity) const
		{
			//depreciated
			const double MotorTorque_nm = INV_GetVel_To_Torque_nm(INV_GetMotorRPS_Angular(wheel_AngularVelocity));
			return INV_GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
		}
		__inline double GetTorqueFromVelocity(double wheel_AngularVelocity) const
		{
			const double MotorTorque_nm = GetVel_To_Torque_nm(GetMotorRPS_Angular(wheel_AngularVelocity));
			return GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
		}
		__inline double GetMaxTraction() const {return m_Props.PayloadMass*m_Props.COF_Efficiency;}
		__inline double GetMaxDriveForce() const {return GetWheelStallTorque()/m_Props.DriveWheelRadius*2.0;}
		__inline double GetCurrentDriveForce(double WheelTorque) const {return WheelTorque/m_Props.DriveWheelRadius*2.0;}
		__inline double GetMaxPushingForce() const {	return std::min(GetMaxTraction()*9.80665,GetMaxDriveForce());}

		const EncoderSimulation_Props &GetDriveTrainProps() const {return m_Props;}
		void SetGearReduction(double NewGearing) {m_Props.GearReduction=NewGearing;}
	private:
		EncoderSimulation_Props m_Props;
};
class COMMON_API Encoder_Simulator2
{
protected:
	//The wheel physics records the velocity of the wheel in radians
	Framework::Base::PhysicsEntity_1D m_Physics;
	Drive_Train_Characteristics m_DriveTrain;
	double m_Time_s;

	double m_Position;  //also keep track of position to simulate distance use case (i.e. used as a potentiometer)
	double m_EncoderScalar; //used for position updates
	double m_ReverseMultiply; //used to implement set reverse direction
public:
	Encoder_Simulator2(const char* EntityName = "EncSimulator") : m_Time_s(0.0), m_EncoderScalar(1.0), m_ReverseMultiply(1.0), m_Position(0.0)
	{
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		const Rotary_Properties* rotary_props = dynamic_cast<const Rotary_Properties*>(props);
		if (rotary_props)
		{
			//m_DriveTrain.UpdateProps(rotary_props->GetEncoderSimulationProps());
			m_EncoderScalar = rotary_props->GetRotaryProps().EncoderToRS_Ratio;
		}

		#if 0
		//m_Physics.SetMass(68);  //(about 150 pounds)
		m_Physics.SetMass(50);  //adjust this to match latency we observe
		m_Physics.SetFriction(0.8, 0.08);
		#else
		m_Physics.SetMass(m_DriveTrain.GetDriveTrainProps().Wheel_Mass);
		m_Physics.SetFriction(0.8, 0.2);
		m_Physics.SetRadiusOfConcentratedMass(m_DriveTrain.GetDriveTrainProps().DriveWheelRadius);
		#endif
	}
	virtual void UpdateEncoderVoltage(double Voltage)
	{
		double Direction=Voltage<0 ? -1.0 : 1.0;
		Voltage=fabs(Voltage); //make positive
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double *c=Polynomial;
			double x2=Voltage*Voltage;
			double x3=Voltage*x2;
			double x4=x2*x2;
			Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
			Voltage *= Direction;
		}
		//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
		//computed from stall torque ratings of motor with various gear reductions and so forth
		//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
		//where stall torque is ST / GearReduction * DriveTrain efficiency
		//This is all computed in the drive train
		#if 0
		double ForceToApply=m_DriveTrain.GetWheelTorqueFromVoltage(Voltage);
		const double ForceAbsorbed=m_DriveTrain.GetWheelTorqueFromVelocity(m_Physics.GetVelocity());
		ForceToApply-=ForceAbsorbed;
		m_Physics.ApplyFractionalForce(ForceToApply,m_Time_s);
		#else
		double TorqueToApply=m_DriveTrain.GetTorqueFromVoltage_V1(Voltage);
		const double TorqueAbsorbed=m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
		TorqueToApply-=TorqueAbsorbed;
		m_Physics.ApplyFractionalTorque(TorqueToApply,m_Time_s,m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		#endif
	}
	virtual double GetEncoderVelocity() const
	{
		#if 0
		static size_t i = 0;
		double Velocity = m_Physics.GetVelocity();
		if (((i++ % 50) == 0) && (Velocity != 0.0))
			printf("Velocty=%f\n", Velocity * m_DriveTrain.GetDriveTrainProps().DriveWheelRadius);
		return 0.0;
		#else
		//return m_Physics.GetVelocity();
		return m_Physics.GetVelocity() * m_DriveTrain.GetDriveTrainProps().DriveWheelRadius * m_ReverseMultiply;
		#endif
	}
	double GetDistance() const
	{
		return m_Position;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	virtual void TimeChange()
	{
		//m_GetEncoderFirstCall=true;
		const double Ground = 0.0;  //ground in radians
		double FrictionForce = m_Physics.GetFrictionalForce(m_Time_s, Ground);
		m_Physics.ApplyFractionalForce(FrictionForce, m_Time_s);  //apply the friction
		double PositionDisplacement;
		m_Physics.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		m_Position += PositionDisplacement * m_EncoderScalar * m_ReverseMultiply;
	}
	void SetReverseDirection(bool reverseDirection)
	{
		//emulates functionality of the encoder (needed because kids put them in differently)
		m_ReverseMultiply = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScalar(double value) {m_EncoderScalar=value;}  //This helps to simulate differences between sides
	void SetFriction(double StaticFriction,double KineticFriction) {m_Physics.SetFriction(StaticFriction,KineticFriction);}
	virtual void ResetPos()
	{
		m_Physics.ResetVectors();
		m_Position = 0;
	}
	void SetGearReduction(double NewGearing) {m_DriveTrain.SetGearReduction(NewGearing);}
};

class COMMON_API Encoder_Simulator3 : public Encoder_Simulator2
{
private:
	static void TimeChange_UpdatePhysics(double Voltage, Drive_Train_Characteristics dtc,Framework::Base::PhysicsEntity_1D &PayloadPhysics, 
	Framework::Base::PhysicsEntity_1D &WheelPhysics,double Time_s,bool UpdatePayload)
{
	//local function to avoid redundant code
	const double PayloadVelocity=PayloadPhysics.GetVelocity();
	const double WheelVelocity=WheelPhysics.GetVelocity();

	double PositionDisplacement;
	//TODO add speed loss force
	if (UpdatePayload)
	{
		//apply a constant speed loss using new velocity - old velocity
		if (Voltage==0.0)
		{
			const double MaxStop=fabs(PayloadVelocity/Time_s);
			const double SpeedLoss=std::min(5.0,MaxStop);
			const double acceleration=PayloadVelocity>0.0?-SpeedLoss:SpeedLoss;
			//now to factor in the mass
			const double SpeedLossForce = PayloadPhysics.GetMass() * acceleration;
			PayloadPhysics.ApplyFractionalTorque(SpeedLossForce,Time_s);
		}
		PayloadPhysics.TimeChangeUpdate(Time_s,PositionDisplacement);
	}

	#ifdef __USE_PAYLOAD_MASS__
	//Now to add force normal against the wheel this is the difference between the payload and the wheel velocity
	//When the mass is lagging behind it add adverse force against the motor... and if the mass is ahead it will
	//relieve the motor and make it coast in the same direction
	//const double acceleration = dtc.GetWheelAngular_RPS(dtc.GetWheelRPS(PayloadVelocity))-WheelVelocity;
	const double acceleration = dtc.GetWheelAngular_LinearVelocity(PayloadVelocity)-WheelVelocity;
	if (PayloadVelocity!=0.0)
	{
		#if 0
		if (UpdatePayload)
			SmartDashboard::PutNumber("Test",dtc.GetWheelAngular_LinearVelocity(PayloadVelocity));
		if (fabs(acceleration)>50)
			int x=8;
		#endif
		//make sure wheel and payload are going in the same direction... 
		if (PayloadVelocity*WheelVelocity >= 0.0)
		{
			//now to factor in the mass
			const double PayloadForce = WheelPhysics.GetMomentofInertia(dtc.GetDriveTrainProps().TorqueAccelerationDampener) * acceleration;
			WheelPhysics.ApplyFractionalTorque(PayloadForce,Time_s);
		}
		//else
		//	printf("skid %.2f\n",acceleration);
		//if not... we have skidding (unless its some error of applying the force normal) for now let it continue its discourse by simply not applying the payload force
		//this will make it easy to observe cases when skids can happen, but eventually we should compute the kinetic friction to apply
	}
	else
		WheelPhysics.SetVelocity(0.0);
	#endif
}
protected:
	double m_Voltage; //cache voltage for speed loss assistance
	//We are pulling a heavy mass this will present more load on the wheel, we can simulate a bench test vs. an actual run by factoring this in
	static Framework::Base::PhysicsEntity_1D s_PayloadPhysics_Left;
	static Framework::Base::PhysicsEntity_1D s_PayloadPhysics_Right;
	//Cache last state to determine if we are accelerating or decelerating
	static double s_PreviousPayloadVelocity_Left;
	static double s_PreviousPayloadVelocity_Right;
	double m_PreviousWheelVelocity;
	size_t m_EncoderKind;
public:
	//The payload mass is shared among multiple instances per motor, so each instance will need to identify the kind it is
	//multiple instance can read, but only one write per side.  Use ignore and the payload computations will be bypassed
	enum EncoderKind
	{
		eIgnorePayload,  //an easy way to make it on the bench
		eReadOnlyLeft,
		eReadOnlyRight,
		eRW_Left,
		eRW_Right
	};
	Encoder_Simulator3(const char *EntityName="EncSimulator") :
		Encoder_Simulator2(EntityName), m_EncoderKind(eIgnorePayload), m_PreviousWheelVelocity(0.0)
	{
	}
	//has to be late binding for arrays
	void SetEncoderKind(EncoderKind kind)
	{
		//Comment this out to do bench testing
		m_EncoderKind = kind;
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		__super::Initialize(props);
		m_Physics.SetAngularInertiaCoefficient(0.5);  //Going for solid cylinder
		//now to setup the payload physics   Note: each side pulls half the weight
		if (m_EncoderKind == eRW_Left)
			s_PayloadPhysics_Left.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass / 2.0);
		else if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass / 2.0);
		//TODO see if this is needed
		//m_PayloadPhysics.SetFriction(0.8,0.2);
	}
	virtual void UpdateEncoderVoltage(double Voltage)
	{
		m_Voltage = Voltage;
		//depreciated
		#if 0
		const double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		#endif
		//This is line is somewhat subtle... basically if the voltage is in the same direction as the velocity we use the linear distribution of the curve, when they are in
		//opposite directions zero gives the stall torque where we need max current to switch directions (same amount as if there is no motion)
		const double VelocityToUse = m_Physics.GetVelocity() * Voltage > 0 ? m_Physics.GetVelocity() : 0.0;
		double TorqueToApply = m_DriveTrain.GetTorqueFromVelocity(VelocityToUse);
		//Avoid ridiculous division and zero it out... besides a motor can't really turn the wheel in the dead zone range, but I'm not going to factor that in yet
		if ((TorqueToApply != 0.0) && (fabs(TorqueToApply) < m_Physics.GetMass() * 2))
			TorqueToApply = 0.0;
		//Note: Even though TorqueToApply has direction, if it gets saturated to 0 it loses it... ultimately the voltage parameter is sacred to the correct direction
		//in all cases so we'll convert TorqueToApply to magnitude
		m_Physics.ApplyFractionalTorque(fabs(TorqueToApply) * Voltage, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);

		//Apply the appropriate change in linear velocity to the payload.  Note:  This is not necessarily torque because of the TorqueAccelerationDampener... the torque
		//acts as voltage acts to a circuit... it gives the potential but the current doesn't reflect this right away... its the current, or in our case the velocity
		//change of he wheel itself that determines how much force to apply.
		const double Wheel_Acceleration = (m_Physics.GetVelocity() - m_PreviousWheelVelocity) / m_Time_s;
		//Now to reconstruct the torque Ia, where I = cm R^2 /torquedampner
		const double Wheel_Torque = Wheel_Acceleration * m_Physics.GetMomentofInertia(m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		//Grrr I have to blend this... mostly because of the feedback from the payload... I'll see if there is some way to avoid needing to do this
		const double smoothing_value = 0.75;
		const double BlendTorqueToApply = (Wheel_Torque * smoothing_value) + (TorqueToApply * (1.0 - smoothing_value));
		//TODO check for case when current drive force is greater than the traction
		//Compute the pushing force of the mass and apply it just the same
		if (m_EncoderKind == eRW_Left)
		{
			//Testing
			#if 0
			SmartDashboard::PutNumber("Wheel_Torque_Left", Wheel_Torque);
			SmartDashboard::PutNumber("Motor_Torque_Left", TorqueToApply);
			#endif
			s_PayloadPhysics_Left.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply)) * Voltage, m_Time_s);
		}
		else if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply)) * Voltage, m_Time_s);

	}
	virtual double GetEncoderVelocity() const
	{
		//we return linear velocity (which is native to the ship)
		return  m_DriveTrain.GetLinearVelocity(m_DriveTrain.GetWheelRPS_Angular(m_Physics.GetVelocity())) * m_ReverseMultiply;
	}
	virtual void TimeChange()
	{
		//For best results if we locked to the payload we needn't apply a speed loss constant here
		#ifndef __USE_PAYLOAD_MASS__
		//first apply a constant speed loss using new velocity - old velocity
		//if (fabs(m_Voltage)<0.65)
		{
			if ((fabs(m_Physics.GetVelocity()) > 0.01))
			{
				const double acceleration = m_DriveTrain.GetDriveTrainProps().SpeedLossConstant * m_Physics.GetVelocity() - m_Physics.GetVelocity();
				//now to factor in the mass
				const double SpeedLossForce = m_Physics.GetMass() * acceleration;
				const double VoltageMagnitue = fabs(m_Voltage);
				m_Physics.ApplyFractionalTorque(SpeedLossForce, m_Time_s, VoltageMagnitue > 0.5 ? (1.0 - VoltageMagnitue) * 0.5 : 0.25);
			}
			else
				m_Physics.SetVelocity(0.0);
		}
		#endif

		double PositionDisplacement;
		m_Physics.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		m_Position += PositionDisplacement * m_EncoderScalar * m_ReverseMultiply;

		//cache velocities
		m_PreviousWheelVelocity = m_Physics.GetVelocity();
		if ((m_EncoderKind == eRW_Left) || (m_EncoderKind == eReadOnlyLeft))
		{
			TimeChange_UpdatePhysics(m_Voltage, m_DriveTrain, s_PayloadPhysics_Left, m_Physics, m_Time_s, m_EncoderKind == eRW_Left);
			if (m_EncoderKind == eRW_Left)
			{
				s_PreviousPayloadVelocity_Left = s_PayloadPhysics_Left.GetVelocity();
				//SmartDashboard::PutNumber("PayloadLeft", Meters2Feet(s_PreviousPayloadVelocity_Left));
				//SmartDashboard::PutNumber("WheelLeft", Meters2Feet(GetEncoderVelocity()));
			}
		}
		else if ((m_EncoderKind == eRW_Right) || (m_EncoderKind == eReadOnlyRight))
		{
			TimeChange_UpdatePhysics(m_Voltage, m_DriveTrain, s_PayloadPhysics_Right, m_Physics, m_Time_s, m_EncoderKind == eRW_Right);
			if (m_EncoderKind == eReadOnlyRight)
			{
				s_PreviousPayloadVelocity_Right = s_PayloadPhysics_Right.GetVelocity();
				//SmartDashboard::PutNumber("PayloadRight", Meters2Feet(s_PreviousPayloadVelocity_Right));
				//SmartDashboard::PutNumber("WheelRight", Meters2Feet(GetEncoderVelocity()));
			}
		}
	}
	virtual void ResetPos()
	{
		__super::ResetPos();
		if (m_EncoderKind == eRW_Left)
			s_PayloadPhysics_Left.ResetVectors();
		if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.ResetVectors();
	}
};
#pragma region _Encoder simulator 3 global variables_
Framework::Base::PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Left;
Framework::Base::PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Right;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Left = 0.0;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Right = 0.0;
#pragma endregion

class COMMON_API Potentiometer_Tester3 : public Encoder_Simulator2
{
private:
	std::queue<double> m_Slack;  //when going down this will grow to x frames, and going up with shrink... when not moving it will shrink to nothing
	double m_SlackedValue=0.0; // ensure the pop of the slack only happens in the time change
protected:
	double m_InvEncoderToRS_Ratio;
public:
	Potentiometer_Tester3(const char* EntityName = "PotSimulator3") : Encoder_Simulator2(EntityName), m_InvEncoderToRS_Ratio(1.0)
	{
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		__super::Initialize(props);
		const Rotary_Properties* rotary = dynamic_cast<const Rotary_Properties*>(props);
		if (rotary)
			m_InvEncoderToRS_Ratio = 1.0 / rotary->GetRotaryProps().EncoderToRS_Ratio;
		m_SlackedValue = GetDistance();
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
		//computed from stall torque ratings of motor with various gear reductions and so forth
		//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
		//where stall torque is ST / GearReduction * DriveTrain efficiency
		//This is all computed in the drive train
		double TorqueToApply = m_DriveTrain.GetTorqueFromVoltage(Voltage);
		const double TorqueAbsorbed = m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
		TorqueToApply -= TorqueAbsorbed;
		m_Physics.ApplyFractionalTorque(TorqueToApply, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		//If we have enough voltage and enough velocity the locking pin is not engaged... gravity can apply extra torque
		//if ((fabs(Voltage)>0.04)&&(fabs(m_Physics.GetVelocity())>0.05))
		if (fabs(Voltage) > 0.04)
		{
			// t=Ia 
			//I=sum(m*r^2) or sum(AngularCoef*m*r^2)
			const double Pounds2Kilograms = 0.453592;
			const double mass = 16.27 * Pounds2Kilograms;
			const double r2 = 1.82 * 1.82;
			//point mass at radius is 1.0
			const double I = mass * r2;
			double Torque = I * 9.80665 * -0.55;  //the last scalar gets the direction correct with negative... and tones it down from surgical tubing
			double TorqueWithAngle = cos(GetDistance() * m_InvEncoderToRS_Ratio) * Torque * m_Time_s;  //gravity has most effect when the angle is zero
			//add surgical tubing simulation... this works in both directions  //1.6 seemed closest but on weaker battery, and can't hit 9 feet well
			TorqueWithAngle += sin(GetDistance() * m_InvEncoderToRS_Ratio) * -1.5;
			//The pin can protect it against going the wrong direction... if they are opposite... saturate the max opposing direction
			if (((Torque * TorqueToApply) < 0.0) && (fabs(TorqueWithAngle) > fabs(TorqueToApply)))
				TorqueWithAngle = -TorqueToApply;
			m_Physics.ApplyFractionalTorque(TorqueWithAngle, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		}
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		#if 1
		return m_SlackedValue;
		#else
		return GetDistance();
		#endif
	}
	virtual void TimeChange()
	{
		__super::TimeChange();
		double CurrentVelociy = m_Physics.GetVelocity();
		m_Slack.push(GetDistance());
		const size_t MaxLatencyCount = 40;

		if (CurrentVelociy >= 0.0)  //going up or still shrink
		{
			if (m_Slack.size() > (MaxLatencyCount >> 1))
				m_Slack.pop();
			if (!m_Slack.empty())
			{
				m_SlackedValue = m_Slack.front();
				m_Slack.pop();
			}
			else
				m_SlackedValue = GetDistance();
		}
		else  //going down expand
		{
			if (m_Slack.size() >= MaxLatencyCount)
				m_Slack.pop();
			m_SlackedValue = m_Slack.front();
		}
		//SmartDashboard::PutNumber("TestSlack",(double)m_Slack.size());
	}
	virtual void ResetPos()
	{
		__super::ResetPos();
		while (!m_Slack.empty())
			m_Slack.pop();
		m_SlackedValue = GetDistance();
	}
};

class COMMON_API Encoder_Tester
{
private:
	#if 0
	//This is still good for a lesser stress (keeping the latency disabled)
	Encoder_Simulator m_LeftEncoder;
	Encoder_Simulator m_RightEncoder;
	#else
	Encoder_Simulator2 m_LeftEncoder;
	Encoder_Simulator2 m_RightEncoder;
	#endif
public:
	Encoder_Tester() : m_LeftEncoder("LeftEncoder"), m_RightEncoder("RightEncoder")
	{
		m_LeftEncoder.Initialize(NULL);
		m_RightEncoder.Initialize(NULL);
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		m_LeftEncoder.Initialize(props);
		m_RightEncoder.Initialize(props);
		//Cool way to add friction uneven to simulate 
		//m_RightEncoder.SetFriction(0.8,0.12);
	}
	virtual void GetLeftRightVelocity(double& LeftVelocity, double& RightVelocity)
	{
		LeftVelocity = m_LeftEncoder.GetEncoderVelocity();
		RightVelocity = m_RightEncoder.GetEncoderVelocity();
	}
	virtual void UpdateLeftRightVoltage(double LeftVoltage, double RightVoltage)
	{
		//LeftVoltage=GetTweakedVoltage(LeftVoltage);
		//RightVoltage=GetTweakedVoltage(RightVoltage);
		m_LeftEncoder.UpdateEncoderVoltage(LeftVoltage);
		m_RightEncoder.UpdateEncoderVoltage(RightVoltage);
	}
	void SetTimeDelta(double dTime_s)
	{
		m_LeftEncoder.SetTimeDelta(dTime_s);
		m_RightEncoder.SetTimeDelta(dTime_s);
	}
	void TimeChange()
	{
		m_LeftEncoder.TimeChange();
		m_RightEncoder.TimeChange();
	}
	void SetLeftRightReverseDirectionEncoder(bool Left_reverseDirection,bool Right_reverseDirection)
	{
		m_LeftEncoder.SetReverseDirection(Left_reverseDirection),m_RightEncoder.SetReverseDirection(Right_reverseDirection);
	}
	void SetLeftRightScalar(double LeftScalar,double RightScalar)
	{
		m_LeftEncoder.SetEncoderScalar(LeftScalar),m_RightEncoder.SetEncoderScalar(RightScalar);
	}
};
#pragma endregion

#pragma region _Rotary_System_

class Rotary_Control_Interface
{
public:
	virtual void Reset_Rotary(size_t index = 0) = 0;

	/// This is really called get rotary current position or velocity (depending on if we are linear or angular)
	/// current position:  (linear)
	///This is a implemented by reading the potentiometer and converting its value to correspond to the current angle in radians
	/// current velocity (angular)
	/// This is implemented by reading an encoder and converting its value to angular velocity also in radians
	/// \note: Any gearing must be settled within the robot control where the ratio satisfies the rotary system's ratio.  Therefore all conversion
	/// happens at the same place
	virtual double GetRotaryCurrentPorV(size_t index = 0) = 0;
	virtual void UpdateRotaryVoltage(size_t index, double Voltage) = 0;
};

class COMMON_API Rotary_System : public Ship_1D
{
private:
	using Rotary_Props = rotary_properties::Rotary_Props;
	using PolynomialEquation_forth = Framework::Base::PolynomialEquation_forth;
	bool m_UsingRange_props;
protected:
	static void InitNetworkProperties(const Rotary_Props &props, bool AddArmAssist = false)
	{
		//This will PutVariables of all properties needed to tweak PID and gain assists
	}
	static void NetworkEditProperties(Rotary_Props &props, bool AddArmAssist = false)
	{
		//This will GetVariables of all properties needed to tweak PID and gain assists
	}
	PolynomialEquation_forth m_VoltagePoly;
public:
	Rotary_System(const char EntityName[]) : Ship_1D(EntityName), m_UsingRange_props(false) {}
	virtual void Initialize(const Entity1D_Properties *props = NULL)
	{
		//Cache the m_UsingRange props so that we can know what to set back to
		__super::Initialize(props);  //must call predecessor first!
		m_UsingRange_props = m_Ship_1D_Props.UsingRange;
	}
	bool GetUsingRange_Props() const 
	{ 
		//This is basically like m_UsingRange from Ship_1D except that it is dynamic, and disabled when potentiometer is disabled (as we cannot detect limits)
		return m_UsingRange_props; 
	}
};
inline void ComputeDeadZone(double &Voltage, double PositiveDeadZone, double NegativeDeadZone, bool ZeroOut = false)
{
	if ((Voltage > 0.0) && (Voltage < PositiveDeadZone))
		Voltage = ZeroOut ? 0.0 : PositiveDeadZone;
	else if ((Voltage < 0.0) && (Voltage > NegativeDeadZone))
		Voltage = ZeroOut ? 0.0 : NegativeDeadZone;
}

class COMMON_API Rotary_Position_Control : public Rotary_System
{
	///This is the next layer of the linear Ship_1D that converts velocity into voltage, on a system that has sensor feedback
	///It currently has a single PID (Dual PID may either be integrated or a new class)... to manage voltage error.  This is used for fixed point
	/// position setting... like a turret or arm

public:
	enum PotUsage
	{
		eNoPot, //Will never read them (ideal for systems that do not have any encoders)
		ePassive,  //Will read them but never alter velocities
		eActive, //Will attempt to match predicted velocity to actual velocity
	};
private:
	#pragma region _members_
	using Rotary_Props = rotary_properties::Rotary_Props;
	using PIDController2 = Framework::Base::PIDController2;
	//Copy these lines to the subclass that binds the events
	//events are a bit picky on what to subscribe so we'll just wrap from here

	/// \param DisableFeedback this allows ability to bypass feedback
	Rotary_Control_Interface * const m_RobotControl;
	const size_t m_InstanceIndex;
	PIDController2 m_PIDControllerUp;
	PIDController2 m_PIDControllerDown;
	Rotary_Props m_Rotary_Props;

	double m_LastPosition=0.0;  //used for calibration
	double m_MatchVelocity=0.0;
	double m_ErrorOffset=0.0;
	double m_LastTime=0.0; //used for calibration
	double m_MaxSpeedReference=0.0; //used for calibration
	double m_PreviousVelocity=0.0; //used to compute acceleration
	//We use the negative sign bit to indicate it was turned off... or zero
	double m_BurstIntensity=0.0;  //This keeps track of the current level of burst to apply... it usually is full 1.0 or 0.0 but will blend on unaligned frame boundaries
	double m_CurrentBurstTime=0.0; //This keeps track of the time between bursts and the burst itself depending on the current state
	size_t m_PulseBurstCounter=0;  //keeps count of how many pulse bursts have happened
	size_t m_StallCounter=0;   //Keeps count for each cycle the motor stalls
	PotUsage m_PotentiometerState=eNoPot; //dynamically able to turn off (e.g. panic button)
	//A counter to count how many times the predicted position and intended position are withing tolerance consecutively
	size_t m_ToleranceCounter=0;
	#pragma endregion
protected:
	virtual void SetPotentiometerSafety(bool DisableFeedback)
	{
		//printf("\r%f       ",Value);
		if (DisableFeedback)
		{
			if (m_PotentiometerState != eNoPot)
			{
				//first disable it
				m_PotentiometerState = eNoPot;
				//Now to reset stuff
				printf("Disabling potentiometer for %s\n", GetName().c_str());
				//m_PIDController.Reset();
				ResetPos();
				//This is no longer necessary
				m_LastPosition = 0.0;
				m_ErrorOffset = 0.0;
				m_LastTime = 0.0;
				m_Ship_1D_Props.UsingRange = false;
			}
		}
		else
		{
			if (m_PotentiometerState == eNoPot)
			{
				switch (m_Rotary_Props.LoopState)
				{
				case Rotary_Props::eNone:
					m_PotentiometerState = eNoPot;
					//This should not happen but added for completeness
					printf("Rotary_Velocity_Control::SetEncoderSafety %s set to no potentiometer\n", GetName().c_str());
					break;
				case Rotary_Props::eOpen:
					m_PotentiometerState = ePassive;
					break;
				case Rotary_Props::eClosed:
				case Rotary_Props::eClosed_ManualAssist:
					m_PotentiometerState = eActive;
					break;
				}

				//setup the initial value with the potentiometers value
				printf("Enabling potentiometer for %s\n", GetName().c_str());
				ResetPos();
				m_Ship_1D_Props.UsingRange = GetUsingRange_Props();
				m_ErrorOffset = 0.0;
			}
		}
	}
	PotUsage GetPotUsage() const
	{
		return m_PotentiometerState;
	}
	virtual double GetMatchVelocity() const
	{
		return m_MatchVelocity;
	}
	virtual bool DidHitMinLimit() const
	{
		//TODO have callbacks for these
		//Override these methods if the rotary system has some limit switches included in its setup
		return false;
	}
	virtual bool DidHitMaxLimit() const
	{
		return false;
	}
public:
	Rotary_Position_Control(const char EntityName[], Rotary_Control_Interface *robot_control, size_t InstanceIndex = 0) :
		Rotary_System(EntityName), m_RobotControl(robot_control), m_InstanceIndex(InstanceIndex),
		m_PIDControllerUp(0.0, 0.0, 0.0), m_PIDControllerDown(0.0, 0.0, 0.0)
	{
		//Note members will be overridden in properties
	}
	virtual void Initialize(const Entity1D_Properties *props = NULL)
	{
		//The parent needs to call initialize
		if (m_PotentiometerState != eNoPot)
			m_LastPosition = m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
		__super::Initialize(props);
		const Rotary_Properties *Props = dynamic_cast<const Rotary_Properties *>(props);
		assert(Props);
		m_VoltagePoly.Initialize(&Props->GetRotaryProps().Voltage_Terms);
		//This will copy all the props
		m_Rotary_Props = Props->GetRotaryProps();
		m_PIDControllerUp.SetPID(m_Rotary_Props.ArmGainAssist.PID_Up[0], m_Rotary_Props.ArmGainAssist.PID_Up[1], m_Rotary_Props.ArmGainAssist.PID_Up[2]);
		m_PIDControllerDown.SetPID(m_Rotary_Props.ArmGainAssist.PID_Down[0], m_Rotary_Props.ArmGainAssist.PID_Down[1], m_Rotary_Props.ArmGainAssist.PID_Down[2]);

		const double MaxSpeedReference = Props->GetMaxSpeed();
		m_PIDControllerUp.SetInputRange(-MaxSpeedReference, MaxSpeedReference);
		m_PIDControllerDown.SetInputRange(-MaxSpeedReference, MaxSpeedReference);
		double tolerance = 0.99; //we must be less than one (on the positive range) to avoid lockup
		m_PIDControllerUp.SetOutputRange(-MaxSpeedReference * tolerance, MaxSpeedReference*tolerance);
		m_PIDControllerDown.SetOutputRange(-MaxSpeedReference * tolerance, MaxSpeedReference*tolerance);
		//The idea here is that the arm may rest at a stop point that needs consistent voltage to keep steady
		if (m_Rotary_Props.ArmGainAssist.SlowVelocityVoltage != 0.0)
		{
			m_PIDControllerUp.SetAutoResetI(false);
			m_PIDControllerDown.SetAutoResetI(false);
		}
		m_PIDControllerUp.Enable();
		m_PIDControllerDown.Enable();
		m_ErrorOffset = 0.0;
		switch (m_Rotary_Props.LoopState)
		{
		case Rotary_Props::eNone:
			SetPotentiometerSafety(true);
			break;
		case Rotary_Props::eOpen:
			m_PotentiometerState = ePassive;
			break;
		case Rotary_Props::eClosed:
		case Rotary_Props::eClosed_ManualAssist:
			m_PotentiometerState = eActive;
			break;
		}
		//It is assumed that this property is constant throughout the whole session
		if (m_Rotary_Props.PID_Console_Dump)
		{
			Ship_1D::InitNetworkProperties(Props->GetShip_1D_Props());
			InitNetworkProperties(m_Rotary_Props, true);
		}
	}
	virtual void ResetPosition(double Position)
	{
		__super::ResetPosition(Position);  //Let the super do it stuff first

		//TODO find case where I had this duplicate call to reset rotary I may omit this... but want to find if it needs to be here
		//  [2/18/2014 James]
		//We may need this if we use Kalman filters
		//m_RobotControl->Reset_Rotary(m_InstanceIndex);

		if ((m_PotentiometerState != eNoPot) && (!GetBypassPos_Update()))
		{
			m_PIDControllerUp.Reset();
			m_PIDControllerDown.Reset();
			//Only reset the encoder if we are reseting the position to the starting position
			if (Position == GetStartingPosition())
				m_RobotControl->Reset_Rotary(m_InstanceIndex);
			double NewPosition = m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
			Stop();
			SetPos_m(NewPosition);
			m_LastPosition = NewPosition;
		}
		m_ToleranceCounter = 0;
	}
	const Rotary_Props &GetRotary_Properties() const 
	{ 
		return m_Rotary_Props; 
	}
	void SetMatchVelocity(double MatchVel) 
	{ 
		//This is optionally used to lock to another ship (e.g. drive using rotary system)
		m_MatchVelocity = MatchVel;
	}
	double GetActualPos() const
	{
		//Give client code access to the actual position, as the position of the entity cannot be altered for its projected position
		const double NewPosition = (m_PotentiometerState != eNoPot) ? m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex) : GetPos_m();
		return NewPosition;
	}
	virtual void TimeChange(double dTime_s)
	{
		//Intercept the time change to obtain current height as well as sending out the desired velocity
		//TODO we'll probably want velocity PID for turret no load type... we'll need to test to see
		const bool TuneVelocity = false;
		const double CurrentVelocity = m_Physics.GetVelocity();
		const Rotary_Props::Rotary_Arm_GainAssist_Props &arm = m_Rotary_Props.ArmGainAssist;
		const Rotary_Props::Voltage_Stall_Safety &arm_stall_safety = m_Rotary_Props.VoltageStallSafety;
		const bool NeedGainAssistForUp = ((arm.SlowVelocityVoltage != 0.0) && (CurrentVelocity > 0.0));

		//Note: the order has to be in this order where it grabs the potentiometer position first and then performs the time change and finally updates the
		//new arm velocity.  Doing it this way avoids oscillating if the potentiometer and gear have been calibrated
		if (!m_LastTime)
		{
			m_LastTime = dTime_s;
			#if 1
			assert(dTime_s != 0.0);
			#endif
		}

		const double NewPosition = GetActualPos();
		const double Displacement = NewPosition - m_LastPosition;
		const double PotentiometerVelocity = Displacement / m_LastTime;

		const bool UsingMoterStallSafety = (arm_stall_safety.ErrorThreshold > 0.0) || (arm_stall_safety.StallCounterThreshold > 0);
		double BurstIntensity = 0.0;
		bool IsPulsing = false;

		//Update the position to where the potentiometer says where it actually is
		if (m_PotentiometerState == eActive)
		{
			if ((!GetLockShipToPosition()) || (m_Rotary_Props.LoopState == Rotary_Props::eClosed_ManualAssist))
			{
				if (TuneVelocity)
				{
					if (m_PIDControllerUp.GetI() == 0.0)
					{
						m_ErrorOffset = m_PIDControllerUp(CurrentVelocity, PotentiometerVelocity, dTime_s);
						const double Acceleration = (CurrentVelocity - m_PreviousVelocity) / dTime_s;
						const bool Decel = (Acceleration * CurrentVelocity < 0);
						//normalize errors... these will not be reflected for I so it is safe to normalize here to avoid introducing oscillation from P
						//Note: that it is important to bias towards deceleration this can help satisfy both requirements of avoiding oscillation as well
						//As well as avoiding a potential overshoot when trying stop at a precise distance
						m_ErrorOffset = Decel || fabs(m_ErrorOffset) > m_Rotary_Props.PrecisionTolerance ? m_ErrorOffset : 0.0;
					}
				}
				else
				{
					const double PredictedPositionUp = NewPosition + (PotentiometerVelocity * arm.VelocityPredictUp);
					//PID will correct for position... this may need to use I to compensate for latency
					if ((arm.UsePID_Up_Only) || (GetPos_m() > PredictedPositionUp))
					{
						m_PIDControllerDown.ResetI();
						m_ErrorOffset = m_PIDControllerUp(GetPos_m(), PredictedPositionUp, dTime_s);
					}
					else
					{
						const double PredictedPosition = NewPosition + (PotentiometerVelocity * arm.VelocityPredictDown);
						m_PIDControllerUp.ResetI();
						m_ErrorOffset = m_PIDControllerDown(GetPos_m(), PredictedPosition, dTime_s);
					}

					if (UsingMoterStallSafety && (IsZero(Displacement, 1e-2)) && (m_Physics.GetVelocity() > 0.0))
					{
						//to measure motor stall... we evaluate the voltage
						const double MaxSpeed = m_Ship_1D_Props.MAX_SPEED;
						const double Voltage = (m_Physics.GetVelocity() + m_ErrorOffset) / MaxSpeed;
						//we may want a property for this value, but typically 0.1 is below dead zone and safe
						if (Voltage > 0.1)
						{
							//printf("[%d] %d %.3f \n",m_InstanceIndex,m_StallCounter,Displacement);
							m_StallCounter++;
						}
					}
					else
						m_StallCounter = 0;

					IsPulsing = ((arm_stall_safety.StallCounterThreshold > 0) && (m_StallCounter > arm_stall_safety.StallCounterThreshold)) ||
						(arm_stall_safety.ErrorThreshold > 0.0 && fabs(m_ErrorOffset) > arm_stall_safety.ErrorThreshold);

					//unlike for velocity all error offset values are taken... two PIDs and I should help stabilize oscillation
					if (((arm.PulseBurstTimeMs > 0.0) && (fabs(m_ErrorOffset) < arm.PulseBurstRange) && (fabs(m_ErrorOffset) > m_Rotary_Props.PrecisionTolerance)) || (IsPulsing))
					{
						//we are in the pulse burst zone... now to manage the burst intensity
						m_CurrentBurstTime += dTime_s;  //increment the pulse burst time with this new time slice
						//The off-time should really match the latency time where we can sample the change interval and get a reading
						//This could be its own property if necessary
						const double Off_Time = (GetPos_m() > PredictedPositionUp) ? arm.VelocityPredictUp : arm.VelocityPredictDown;
						//We use the negative sign bit to indicate it was turned off... or zero
						if (m_BurstIntensity <= 0.0)
						{	//Burst has been off... is it time to turn it back on
							if (m_CurrentBurstTime >= Off_Time)
							{
								m_PulseBurstCounter++;
								if (UsingMoterStallSafety)
								{
									BurstIntensity = arm_stall_safety.OnBurstLevel;
								}
								else
								{
									//Turn on the pulse... the intensity here is computed by the overlap
									const double overlap = m_CurrentBurstTime - Off_Time;
									if (overlap < dTime_s)
										BurstIntensity = overlap / dTime_s;
									else
									{
										BurstIntensity = 1.0;
										//This shouldn't happen often... probably shouldn't matter much... but keep this for diagnostic testing
										printf("test burst begin[%d]... overlap=%.2f vs delta slice=%.2f\n", m_InstanceIndex, overlap, dTime_s);
									}
								}
								m_CurrentBurstTime = 0.0;  //reset timer
							}
						}
						else
						{  //Burst has been on... is it time to turn it back off
							if (m_CurrentBurstTime >= arm.PulseBurstTimeMs)
							{
								//Turn off the pulse... the intensity here is computed by the overlap
								const double overlap = m_CurrentBurstTime - arm.PulseBurstTimeMs;
								//We use the negative sign bit to indicate it was turned off... or zero
								if ((overlap < dTime_s) && (!UsingMoterStallSafety))
									BurstIntensity = -(overlap / dTime_s);
								else
								{
									BurstIntensity = 0.0;
									//This shouldn't happen often... probably shouldn't matter much... but keep this for diagnostic testing
									if (!UsingMoterStallSafety)
										printf("test burst end[%d]... overlap=%.2f vs delta slice=%.2f\n", m_InstanceIndex, overlap, dTime_s);
								}
								m_CurrentBurstTime = 0.0;  //reset timer
							}
							else
							{
								BurstIntensity = (!UsingMoterStallSafety) ? 1.0 : arm_stall_safety.OnBurstLevel;  //still on under time... so keep it on full
							}
						}
					}
					//When in eClosed_ManualAssist check the locked position to end weak voltage when it is within tolerance
					//setpoint will turn locked position on once the tolerance is achieved with its count
					if (GetLockShipToPosition())
						m_ErrorOffset = fabs(m_ErrorOffset) > m_Rotary_Props.PrecisionTolerance ? m_ErrorOffset : 0.0;

				}
			}
			else
			{
				//If we are manually controlling, we should still update displacement to properly work with limits and maintain where the position really
				//is to seamlessly transfer between manual and auto
				m_ErrorOffset = 0.0;
				m_PIDControllerUp.ResetI();
				m_PIDControllerDown.ResetI();
			}


			if (!IsPulsing)
				m_PulseBurstCounter = 0;
			//have a way to handle to end pulsing when timeout is exceeded
			if (m_PulseBurstCounter > arm_stall_safety.PulseBurstTimeOut)
				BurstIntensity = 0.0;
			//We do not want to alter position if we are using position control PID
			if ((TuneVelocity) || (IsZero(NewPosition - m_LastPosition) && (!IsPulsing)) || NeedGainAssistForUp)
				SetPos_m(NewPosition);
		}

		else if (m_PotentiometerState == ePassive)
		{
			//ensure the positions are calibrated when we are not moving
			if (IsZero(NewPosition - m_LastPosition) || NeedGainAssistForUp)
				SetPos_m(NewPosition);  //this will help min and max limits work properly even though we do not have PID
		}

		#if 0
		if (m_Rotary_Props.PID_Console_Dump)
		{
			const bool ManualPositionTesting = SmartDashboard::GetBoolean("ManualPositionTesting");
			const bool InDegrees = SmartDashboard::GetBoolean("ManualInDegrees");
			if (InDegrees)
			{
				if (ManualPositionTesting)
					SetIntendedPosition(DEG_2_RAD(SmartDashboard::GetNumber("IntendedPosition")));
				else
					SmartDashboard::PutNumber("IntendedPosition", RAD_2_DEG(m_IntendedPosition));
			}
			else
			{
				if (ManualPositionTesting)
					SetIntendedPosition(SmartDashboard::GetNumber("IntendedPosition"));
				else
					SmartDashboard::PutNumber("IntendedPosition", m_IntendedPosition);
			}
		}
		#endif

		//if we are heading for an intended position and we graze on it... turn off the corrections
		if (!GetLockShipToPosition())
		{

			if (fabs(NewPosition - m_IntendedPosition) < m_Rotary_Props.PrecisionTolerance)
				m_ToleranceCounter++;
			else
				m_ToleranceCounter = 0;

			if (m_ToleranceCounter >= arm.ToleranceConsecutiveCount)
			{
				SetRequestedVelocity(0.0);  //ensure the requested velocity is zero once it gets locked to ship position
				SetCurrentLinearAcceleration(0.0);  //lock ship to position
				m_ErrorOffset = 0.0;  //no error correction to factor in (avoids noisy resting point)
			}
		}

		m_LastPosition = NewPosition;
		m_LastTime = dTime_s;

		__super::TimeChange(dTime_s);

		//Note: CurrentVelocity variable is retained before the time change (for proper debugging of PID) we use the new velocity (called Velocity) here for voltage
		const double Velocity = m_Physics.GetVelocity();
		const double Acceleration = (Velocity - m_PreviousVelocity) / dTime_s;

		const double MaxSpeed = m_Ship_1D_Props.MAX_SPEED;
		double Voltage = (Velocity + m_ErrorOffset) / MaxSpeed;

		bool IsAccel = (Acceleration * Velocity > 0);
		if (Velocity > 0)
			Voltage += Acceleration * (IsAccel ? arm.InverseMaxAccel_Up : arm.InverseMaxDecel_Up);
		else
			Voltage += Acceleration * (IsAccel ? arm.InverseMaxAccel_Down : arm.InverseMaxDecel_Down);

		//See if we are using the arm gain assist (only when going up)
		if ((NeedGainAssistForUp) || ((arm.SlowVelocityVoltage != 0.0) && m_ErrorOffset > 0.0))
		{
			//first start out by allowing the max amount to correspond to the angle of the arm... this assumes the arm zero degrees is parallel to the ground
			//90 is straight up... should work for angles below zero (e.g. hiking viking)... angles greater than 90 will be negative which is also correct
			const double MaxVoltage = cos(NewPosition * arm.GainAssistAngleScalar) * arm.SlowVelocityVoltage;
			double BlendStrength = 0.0;
			const double SlowVelocity = arm.SlowVelocity;
			//Now to compute blend strength... a simple linear distribution of how much slower it is from the slow velocity
			if (MaxVoltage > 0.0)
			{
				if (PotentiometerVelocity < SlowVelocity)
					BlendStrength = (SlowVelocity - PotentiometerVelocity) / SlowVelocity;
			}
			else
			{
				if (PotentiometerVelocity > -SlowVelocity)
					BlendStrength = (SlowVelocity - fabs(PotentiometerVelocity)) / SlowVelocity;
			}
			Voltage += MaxVoltage * BlendStrength;
		}

		//apply additional pulse burst as needed
		m_BurstIntensity = BurstIntensity;
		if (!UsingMoterStallSafety)
		{
			//The intensity we use is the same as full speed with the gain assist... we may want to have own properties for these if they do not work well
			const double PulseBurst = fabs(m_BurstIntensity) * ((m_ErrorOffset > 0.0) ? MaxSpeed * 1.0 * arm.InverseMaxAccel_Up : (-MaxSpeed) * 1.0 * arm.InverseMaxAccel_Down);
			Voltage += PulseBurst;
		}
		else
		{
			if (IsPulsing)
				Voltage = m_BurstIntensity;
		}

		//if (PulseBurst!=0.0)
		//	printf("pb=%.2f\n",PulseBurst);
		#if 0
		if ((arm.PulseBurstTimeMs > 0.0) && (fabs(m_ErrorOffset) < arm.PulseBurstRange) && (fabs(m_ErrorOffset) > m_Rotary_Props.PrecisionTolerance))
		{
			if (BurstIntensity == 0)
				printf("\rOff %.2f       ", m_CurrentBurstTime);
			else
				printf("\n+On=%.2f  pb=%.2f from bi=%.2f\n", m_CurrentBurstTime, PulseBurst, m_BurstIntensity);
		}
		#endif

		//Keep track of previous velocity to compute acceleration
		m_PreviousVelocity = Velocity;
		Voltage = m_VoltagePoly(Voltage);

		if (IsZero(PotentiometerVelocity) && (CurrentVelocity == 0.0))
		{
			//avoid dead zone... if we are accelerating set the dead zone to the minim value... all else zero out
			if (IsAccel)
				ComputeDeadZone(Voltage, m_Rotary_Props.Positive_DeadZone, m_Rotary_Props.Negative_DeadZone);
			else
				ComputeDeadZone(Voltage, m_Rotary_Props.Positive_DeadZone, m_Rotary_Props.Negative_DeadZone, true);
		}

		{
			//Clamp range, PID (i.e. integral) controls may saturate the amount needed
			if (Voltage > 0.0)
			{
				if (Voltage > 1.0)
					Voltage = 1.0;
			}
			else if (Voltage < 0.0)
			{
				if (Voltage < -1.0)
					Voltage = -1.0;
			}
			else
				Voltage = 0.0;  //is nan case
		}

		if (m_Rotary_Props.PID_Console_Dump)
		{
			NetworkEditProperties(m_Rotary_Props, true);
			Ship_1D::NetworkEditProperties(m_Ship_1D_Props);
			m_PIDControllerUp.SetPID(m_Rotary_Props.ArmGainAssist.PID_Up[0], m_Rotary_Props.ArmGainAssist.PID_Up[1], m_Rotary_Props.ArmGainAssist.PID_Up[2]);
			m_PIDControllerDown.SetPID(m_Rotary_Props.ArmGainAssist.PID_Down[0], m_Rotary_Props.ArmGainAssist.PID_Down[1], m_Rotary_Props.ArmGainAssist.PID_Down[2]);
			switch (m_Rotary_Props.LoopState)
			{
			case Rotary_Props::eNone:
				SetPotentiometerSafety(true);
				break;
			case Rotary_Props::eOpen:
				m_PotentiometerState = ePassive;
				break;
			case Rotary_Props::eClosed:
			case Rotary_Props::eClosed_ManualAssist:
				m_PotentiometerState = eActive;
				break;
			}

			#ifdef __DebugLUA__
			const double PosY = m_LastPosition * arm.GainAssistAngleScalar; //The scalar makes position more readable
			const double PredictedPosY = GetPos_m()  * arm.GainAssistAngleScalar;
			if ((fabs(PotentiometerVelocity) > 0.03) || (CurrentVelocity != 0.0) || (Voltage != 0.0))
			{
				//double PosY=RAD_2_DEG(m_LastPosition * arm.GainAssistAngleScalar);
				printf("v=%.2f y=%.2f py=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, PosY, PredictedPosY, CurrentVelocity, PotentiometerVelocity, m_ErrorOffset);
			}
			//We may want a way to pick these separately 
			#if 1
			SmartDashboard::PutNumber("voltage", Voltage);
			SmartDashboard::PutNumber("actual y", PosY);
			SmartDashboard::PutNumber("desired y", PredictedPosY);
			SmartDashboard::PutNumber("desired velocity", CurrentVelocity);
			SmartDashboard::PutNumber("actual velocity", PotentiometerVelocity);
			SmartDashboard::PutNumber("pid error offset", m_ErrorOffset);
			#endif
			#endif
		}

		//Finally check to see if limit switches have been activated
		if ((Voltage > 0.0) && DidHitMaxLimit())
		{
			Voltage = 0.0;  // disallow voltage in this direction
			ResetPosition(m_Rotary_Props.MaxLimitRange);
		}
		else if ((Voltage < 0.0) && DidHitMinLimit())
		{
			Voltage = 0.0;
			ResetPosition(m_Rotary_Props.MinLimitRange);
		}
		m_RobotControl->UpdateRotaryVoltage(m_InstanceIndex, Voltage);

	}
};

class COMMON_API Rotary_Velocity_Control : public Rotary_System
{
	///This is the next layer of the linear Ship_1D that converts velocity into voltage, on a system that has sensor feedback
	///This models itself much like the drive train and encoders where it allows an optional encoder sensor read back to calibrate.
	///This is a kind of speed control system that manages the velocity and does not need to keep track of position (like the drive or a shooter)
public:
	enum EncoderUsage
	{
		eNoEncoder, //Will never read them (ideal for systems that do not have any encoders)
		ePassive,  //Will read them but never alter velocities
		eActive, //Will attempt to match predicted velocity to actual velocity
	};
private:
	#pragma region _members_
	using PIDController2 = Framework::Base::PIDController2;
	using LatencyPredictionFilter = Framework::Base::LatencyPredictionFilter;
	using Rotary_Props = rotary_properties::Rotary_Props;
	//Copy these lines to the subclass that binds the events
	//events are a bit picky on what to subscribe so we'll just wrap from here
	//void SetRequestedVelocity_FromNormalized(double Velocity) {__super::SetRequestedVelocity_FromNormalized(Velocity);}

	/// \param DisableFeedback this allows ability to bypass feedback
	Rotary_Control_Interface * const m_RobotControl;
	const size_t m_InstanceIndex;
	PIDController2 m_PIDController;
	Rotary_Props m_Rotary_Props;
	#ifdef __Rotary_UseInducedLatency__
	LatencyFilter m_PID_Input_Latency;
	#else
	LatencyPredictionFilter m_PID_Input_Latency;
	#endif

	//We have both ways to implement PID calibration depending on if we have aggressive stop property enabled
	double m_MatchVelocity=0.0;
	double m_CalibratedScaler=1.0; //used for calibration
	double m_ErrorOffset=0.0; //used for calibration

	double m_MaxSpeedReference=0.0; //used for calibration
	double m_EncoderVelocity=0.0;  //cache for later use
	double m_RequestedVelocity_Difference=0.0;
	EncoderUsage m_EncoderState=eNoEncoder; //dynamically able to change state
	double m_PreviousVelocity=0.0; //used to compute acceleration
	#pragma endregion
protected:
	virtual void RequestedVelocityCallback(double VelocityToUse, double DeltaTime_s)
	{
		if ((m_EncoderState == eActive) || (m_EncoderState == ePassive))
			m_RequestedVelocity_Difference = VelocityToUse - m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
	}
	virtual bool InjectDisplacement(double DeltaTime_s, double &PositionDisplacement)
	{
		bool ret = false;
		const bool UpdateDisplacement = ((m_EncoderState == eActive) || (m_EncoderState == ePassive));
		if (UpdateDisplacement)
		{
			double computedVelocity = m_Physics.GetVelocity();
			m_Physics.SetVelocity(m_EncoderVelocity);
			m_Physics.TimeChangeUpdate(DeltaTime_s, PositionDisplacement);
			//We must set this back so that the PID can compute the entire error
			m_Physics.SetVelocity(computedVelocity);
			ret = true;
		}
		if (!ret)
			ret = __super::InjectDisplacement(DeltaTime_s, PositionDisplacement);
		return ret;
	}
	virtual double GetMatchVelocity() const { return m_MatchVelocity; }
public:
	Rotary_Velocity_Control(const char EntityName[], Rotary_Control_Interface *robot_control, size_t InstanceIndex = 0, EncoderUsage EncoderState = eNoEncoder) :
		Rotary_System(EntityName), m_RobotControl(robot_control), m_InstanceIndex(InstanceIndex),
		m_PIDController(0.0, 0.0, 0.0), //This will be overridden in properties
		m_EncoderState(EncoderState)
	{}
	virtual void Initialize(const Entity1D_Properties *props = NULL)
	{
		//The parent needs to call initialize
		if ((m_EncoderState == eActive) || (m_EncoderState == ePassive))
			m_EncoderVelocity = m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
		__super::Initialize(props);
		const Rotary_Properties *Props = dynamic_cast<const Rotary_Properties *>(props);
		assert(Props);
		m_VoltagePoly.Initialize(&Props->GetRotaryProps().Voltage_Terms);
		//This will copy all the props
		m_Rotary_Props = Props->GetRotaryProps();
		m_PIDController.SetPID(m_Rotary_Props.PID[0], m_Rotary_Props.PID[1], m_Rotary_Props.PID[2]);

		//Note: for the drive we create a large enough number that can divide out the voltage and small enough to recover quickly,
		//but this turned out to be problematic when using other angular rotary systems... therefore I am going to use the same computation
		//I do for linear, where it allows as slow to max speed as possible.
		//  [3/20/2012 Terminator]
		const double MaxSpeed_Forward = m_Ship_1D_Props.MaxSpeed_Forward;
		const double MaxSpeed_Reverse = m_Ship_1D_Props.MaxSpeed_Reverse;
		m_PIDController.SetInputRange(MaxSpeed_Reverse, MaxSpeed_Forward);
		double tolerance = 0.99; //we must be less than one (on the positive range) to avoid lockup
		m_PIDController.SetOutputRange(MaxSpeed_Reverse*tolerance, MaxSpeed_Forward*tolerance);
		m_PIDController.Enable();
		m_CalibratedScaler = m_Ship_1D_Props.MAX_SPEED;
		m_ErrorOffset = 0.0;

		switch (m_Rotary_Props.LoopState)
		{
		case Rotary_Props::eNone:
			SetEncoderSafety(true);
			break;
		case Rotary_Props::eOpen:
			m_EncoderState = ePassive;
			break;
		case Rotary_Props::eClosed:
			m_EncoderState = eActive;
			break;
		default:
			assert(false);
		}
		//It is assumed that this property is constant throughout the whole session
		if (m_Rotary_Props.PID_Console_Dump)
		{
			Ship_1D::InitNetworkProperties(Props->GetShip_1D_Props());
			InitNetworkProperties(m_Rotary_Props);
		}
	}
	virtual void ResetPos()
	{
		__super::ResetPos();  //Let the super do it stuff first

		m_PIDController.Reset();
		//We may need this if we use Kalman filters
		m_RobotControl->Reset_Rotary(m_InstanceIndex);
		if ((m_EncoderState == eActive) || (m_EncoderState == ePassive))
			m_EncoderVelocity = m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
		else
			m_EncoderVelocity = 0.0;

		//ensure teleop has these set properly
		m_CalibratedScaler = m_Ship_1D_Props.MAX_SPEED;
		m_ErrorOffset = 0.0;
		m_RequestedVelocity_Difference = 0.0;
	}
	double GetRequestedVelocity_Difference() const { return m_RequestedVelocity_Difference; }
	const Rotary_Props &GetRotary_Properties() const { return m_Rotary_Props; }
	void SetMatchVelocity(double MatchVel) 
	{ 
		//This is optionally used to lock to another ship (e.g. drive using rotary system)
		m_MatchVelocity = MatchVel;
	}
	void UpdateRotaryProps(const Rotary_Props &RotaryProps)
	{
		//Give ability to change properties
		m_Rotary_Props = RotaryProps;
		m_CalibratedScaler = m_Ship_1D_Props.MAX_SPEED;
		m_PIDController.SetPID(m_Rotary_Props.PID[0], m_Rotary_Props.PID[1], m_Rotary_Props.PID[2]);
		switch (m_Rotary_Props.LoopState)
		{
		case Rotary_Props::eNone:
			SetEncoderSafety(true);
			break;
		case Rotary_Props::eOpen:
			m_EncoderState = ePassive;
			break;
		case Rotary_Props::eClosed:
			m_EncoderState = eActive;
			break;
		default:
			assert(false);
		}
	}
	virtual void SetEncoderSafety(bool DisableFeedback)
	{
		//printf("\r%f       ",Value);
		if (DisableFeedback)
		{
			if (m_EncoderState != eNoEncoder)
			{
				//first disable it
				m_EncoderState = eNoEncoder;
				//Now to reset stuff
				printf("Disabling encoder for %s\n", GetName().c_str());
				//m_PIDController.Reset();
				ResetPos();
				//This is no longer necessary
				//m_MaxSpeed=m_MaxSpeedReference;
				m_EncoderVelocity = 0.0;
				m_CalibratedScaler = m_Ship_1D_Props.MAX_SPEED;
				m_ErrorOffset = 0;
				m_Ship_1D_Props.UsingRange = false;
			}
		}
		else
		{
			if (m_EncoderState == eNoEncoder)
			{
				switch (m_Rotary_Props.LoopState)
				{
				case Rotary_Props::eNone:
					m_EncoderState = eNoEncoder;
					//This should not happen but added for completeness
					printf("Rotary_Velocity_Control::SetEncoderSafety %s set to no encoder\n", GetName().c_str());
					break;
				case Rotary_Props::eOpen:
					m_EncoderState = ePassive;
					break;
				case Rotary_Props::eClosed:
					m_EncoderState = eActive;
					break;
				default:
					assert(false);
				}
				//setup the initial value with the potentiometers value
				printf("Enabling encoder for %s\n", GetName().c_str());
				ResetPos();
				m_Ship_1D_Props.UsingRange = GetUsingRange_Props();
				m_CalibratedScaler = m_Ship_1D_Props.MAX_SPEED;
				m_ErrorOffset = 0;
			}
		}
	}
	EncoderUsage GetEncoderUsage() const { return m_EncoderState; }
	void SetAggresiveStop(bool UseAggresiveStop) 
	{ 
		//Give client code ability to change rotary system to break or coast
		m_Rotary_Props.UseAggressiveStop = UseAggresiveStop;
	}
	virtual void TimeChange(double dTime_s)
	{
		//Intercept the time change to obtain current height as well as sending out the desired velocity
		const double CurrentVelocity = m_Physics.GetVelocity();
		const double MaxSpeed = m_Ship_1D_Props.MAX_SPEED;

		double Encoder_Velocity = 0.0;
		if ((m_EncoderState == eActive) || (m_EncoderState == ePassive))
		{
			Encoder_Velocity = m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);

			//Unlike linear there is no displacement measurement therefore no need to check GetLockShipToPosition()
			if (m_EncoderState == eActive)
			{
				if (!m_Rotary_Props.UseAggressiveStop)
				{
					double control = 0.0;
					//only adjust calibration when both velocities are in the same direction, or in the case where the encoder is stopped which will
					//allow the scaler to normalize if it need to start up again.
					if (((CurrentVelocity * Encoder_Velocity) > 0.0) || IsZero(Encoder_Velocity))
					{
						control = -m_PIDController(fabs(CurrentVelocity), fabs(Encoder_Velocity), dTime_s);
						m_CalibratedScaler = MaxSpeed + control;
					}
				}
				else
				{
					m_ErrorOffset = m_PIDController(CurrentVelocity, Encoder_Velocity, dTime_s);
					const double Acceleration = (CurrentVelocity - m_PreviousVelocity) / dTime_s;
					const bool Decel = (Acceleration * CurrentVelocity < 0);
					//normalize errors... these will not be reflected for I so it is safe to normalize here to avoid introducing oscillation from P
					//Note: that it is important to bias towards deceleration this can help satisfy both requirements of avoiding oscillation as well
					//As well as avoiding a potential overshoot when trying stop at a precise distance
					m_ErrorOffset = Decel || fabs(m_ErrorOffset) > m_Rotary_Props.PrecisionTolerance ? m_ErrorOffset : 0.0;
				}
			}
			else
				m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);  //For ease of debugging the controls (no harm to read)


			m_EncoderVelocity = Encoder_Velocity;
		}
		__super::TimeChange(dTime_s);
		const double Velocity = m_Physics.GetVelocity();
		double Acceleration = (Velocity - m_PreviousVelocity) / dTime_s;
		//CurrentVelocity is retained before the time change (for proper debugging of PID) we use the new velocity here for voltage
		//Either error offset or calibrated scaler will be used depending on the aggressive stop property, we need not branch this as
		//they both can be represented in the same equation
		double Voltage = (Velocity + m_ErrorOffset) / m_CalibratedScaler;

		bool IsAccel = (Acceleration * Velocity > 0);
		//if we are coasting we must not apply reverse voltage when decelerating
		if (!IsAccel && m_Rotary_Props.UseAggressiveStop == false)
			Acceleration = 0.0;
		Voltage += Acceleration * (IsAccel ? m_Rotary_Props.InverseMaxAccel : m_Rotary_Props.InverseMaxDecel);

		//Keep track of previous velocity to compute acceleration
		m_PreviousVelocity = Velocity;

		//Apply the polynomial equation to the voltage to linearize the curve
		Voltage = m_VoltagePoly(Voltage);

		if ((IsZero(m_EncoderVelocity)) && IsAccel)
			ComputeDeadZone(Voltage, m_Rotary_Props.Positive_DeadZone, m_Rotary_Props.Negative_DeadZone);

		//Keep voltage override disabled for simulation to test precision stability
		//if (!m_VoltageOverride)
		if (true)
		{
			//Clamp range, PID (i.e. integral) controls may saturate the amount needed
			if (Voltage > 0.0)
			{
				if (Voltage > 1.0)
					Voltage = 1.0;
			}
			else if (Voltage < 0.0)
			{
				if (Voltage < -1.0)
					Voltage = -1.0;
			}
			else
				Voltage = 0.0;  //is nan case
		}
		else
		{
			Voltage = 0.0;
			m_PIDController.ResetI(m_MaxSpeedReference * -0.99);  //clear error for I for better transition back
		}

		#if 0
		Voltage *= Voltage;  //square them for more give
		//restore the sign
		if (CurrentVelocity < 0)
			Voltage = -Voltage;
		#endif

		if (m_Rotary_Props.PID_Console_Dump)
		{
			NetworkEditProperties(m_Rotary_Props);
			Ship_1D::NetworkEditProperties(m_Ship_1D_Props);
			m_PIDController.SetPID(m_Rotary_Props.PID[0], m_Rotary_Props.PID[1], m_Rotary_Props.PID[2]);
			switch (m_Rotary_Props.LoopState)
			{
			case Rotary_Props::eNone:
				SetEncoderSafety(true);
				break;
			case Rotary_Props::eOpen:
				m_EncoderState = ePassive;
				break;
			case Rotary_Props::eClosed:
				m_EncoderState = eActive;
				break;
			default:
				assert(false);
			}

			#ifdef __DebugLUA__
			if (Encoder_Velocity != 0.0)
			{
				if (m_Rotary_Props.UseAggressiveStop)
					printf("v=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset);
				else
				{
					if (m_PIDController.GetI() == 0.0)
						printf("v=%.2f p=%.2f e=%.2f eo=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset, m_CalibratedScaler / MaxSpeed);
					else
						printf("v=%.2f p=%.2f e=%.2f i=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_PIDController.GetTotalError(), m_CalibratedScaler / MaxSpeed);
				}
			}
			#endif
		}

		m_RobotControl->UpdateRotaryVoltage(m_InstanceIndex, Voltage);
	}
};

#pragma endregion

}
#pragma endregion

#pragma region _RotaryPosition_Internal_
class RotaryPosition_Internal : public Legacy::Rotary_Control_Interface
{
private:
	using Rotary_Position_Control = Legacy::Rotary_Position_Control;
	#pragma region _bypass members_
	double m_Position = 0.0;
	double m_maxspeed = 1.0;
	Framework::Base::PhysicsEntity_1D m_bypass_physics;
	#pragma endregion
	std::unique_ptr<Rotary_Position_Control> m_rotary_legacy;
	#pragma region _members_
	std::function<void(double new_voltage)> m_VoltageCallback = nullptr;
	std::function<double()> m_OdometryCallack = nullptr;
	double m_OpenLoop_position = 0.0;  //keep track internally if we are open loop
	#pragma endregion
	void time_slice_bypass(double d_time_s)
	{
		//This example is the simplest logic needed to handle setpoint.  It first computes the delta of where it is vs. where
		//it needs to be and from this computes the velocity using the s=1/2at^2 equation (solved for time, more details 
		//of this later). Finally the velocity get's normalized for voltage. The actual rotary system code uses PID and other
		//vices to deal with real-world stresses

		//First grab our current position
		const double pot_pos = m_OdometryCallack ? m_OdometryCallack() : m_OpenLoop_position;
		//now our delta always in direction of destination - previous
		const double distance = m_Position - pot_pos;
		//Now to compute the velocity, our units are radians
		//For this to work, we have to be mindful of our max acceleration, the way this method GetVelocityFromDistance_Angular
		//works is that we compute the max torque the motor can supply (Restraint parameter) against the mass that it needs to
		//turn.  We can compute the torque with pretty good accuracy, and the mass can be compensated to account for other loads
		//such as friction and mechanical transfer efficiency etc. This can be tuned empirically and in the real code have PID to
		//work out any error.

		const double stall_torque = 0.38; //using a RS-550 which is plenty of power
		const double torque_restraint = stall_torque * 100.0; //the total torque simple factors any gear reduction (counting the shaft to final gear reduction)
		const double velocity = m_bypass_physics.GetVelocityFromDistance_Angular(distance, torque_restraint, d_time_s, 0.0);
		//almost there now to normalize the voltage we need the max speed, this is provided by the free speed of the motor, and since it is the speed of the
		//swivel itself we factor in the gear reduction.  This should go very quick because the load is light
		const double free_speed_RPM = 19000.0 / 30.0;  //factor in gear reduction
		const double max_speed_rad = free_speed_RPM * Pi2;
		//Note: At some point I should determine why the numbers do not quite align, they are empirically tweaked to have a good acceleration, and
		//should be fine for bypass demo
		double voltage = velocity / 8;
		m_OpenLoop_position += velocity * d_time_s; //update our internal position
		//avoid oscillation by cutting out minor increments
		if (fabs(voltage) < 0.01)
			voltage = 0.0;
		m_VoltageCallback(voltage);
	}
	#pragma region _Rotary_Control_Interface_
protected: //from Rotary_Control_Interface
	virtual void Reset_Rotary(size_t index = 0)
	{
		//TODO may need to provide reset to physical odometry
		//potentiometers would not need this
	}
	virtual double GetRotaryCurrentPorV(size_t index = 0)
	{
		const double pot_pos = m_OdometryCallack ? m_OdometryCallack() : m_OpenLoop_position;
		return pot_pos;
	}
	virtual void UpdateRotaryVoltage(size_t index, double Voltage)
	{
		m_VoltageCallback(Voltage);
	}
	#pragma endregion
public:
	void Init(size_t InstanceIndex, rotary_properties *props=nullptr)
	{
		using namespace Legacy;
		//this is our wheel module weight and really does not count weight of robot or friction
		m_bypass_physics.SetMass(2.0 * 0.453592);
		if (m_rotary_legacy == nullptr)
		{
			m_rotary_legacy = std::make_unique<Rotary_Position_Control>("rotary_position", this, InstanceIndex);
			//create a new rotary properties and init with it
			Rotary_Properties legacy_props;
			//if client has provided properties, update them
			if (props)
			{
				//Hack: If you are reading this.. the memory copy is a result of tightly coupled classes from the legacy
				//code.  Ideally I'd like to get a proper fix to this, but for now this will do.
				//of each section
				//legacy_props.AsEntityProps() = props->entity_props;
				memcpy((void*)&legacy_props.AsEntityProps(), &props->entity_props, sizeof(Entity1D_Props));
				//legacy_props.GetShip_1D_Props_rw() = props->ship_props;
				memcpy((void*)&legacy_props.GetShip_1D_Props(), &props->ship_props, sizeof(Ship_1D_Props));
				legacy_props.RotaryProps() = props->rotary_props;
			}
			m_rotary_legacy->Initialize(&legacy_props);
		}
	}
	void ShutDown()
	{}
	void SetPosition(double position)
	{
		m_Position = position;
		m_rotary_legacy->SetIntendedPosition(position);
	}
	void TimeSlice(double d_time_s)
	{
		//Use our setpoint input m_Position compute needed velocity and apply normalized voltage to voltage callback
		#ifdef __UseBypass__
		time_slice_bypass(d_time_s);
		#else
		m_rotary_legacy->TimeChange(d_time_s);
		#endif
	}
	void Reset(double position = 0.0)
	{
		m_Position = 0.0;
		m_rotary_legacy->ResetPosition(position);
	}
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
	{
		m_VoltageCallback = callback;
	}
	void SetOdometryCallback(std::function<double()> callback)
	{
		m_OdometryCallack = callback;
	}
};
#pragma endregion
#pragma region _RotaryVelocity_Internal_
class RotaryVelocity_Internal :public Legacy::Rotary_Control_Interface
{
private:
	using Rotary_Velocity_Control = Legacy::Rotary_Velocity_Control;
	#pragma region _members_
	double m_Velocity = 0.0;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	std::unique_ptr<Rotary_Velocity_Control> m_rotary_legacy;
	std::function<void(double new_voltage)> m_VoltageCallback;
	std::function<double()> m_OdometryCallack = nullptr;
	#pragma endregion
	#pragma region _Rotary_Control_Interface_
protected: //from Rotary_Control_Interface
	virtual void Reset_Rotary(size_t index = 0)
	{
		//TODO may need to provide reset to physical odometry
		//encoders have a reset in WPI but it shouldn't be needed for velocity
	}
	virtual double GetRotaryCurrentPorV(size_t index = 0)
	{
		const double velocity = m_OdometryCallack ? m_OdometryCallack() : m_Velocity;
		return velocity;
	}
	virtual void UpdateRotaryVoltage(size_t index, double Voltage)
	{
		m_VoltageCallback(Voltage);
	}
	#pragma endregion
public:
	void Init(size_t InstanceIndex, rotary_properties *props=nullptr)
	{
		using namespace Legacy;
		if (m_rotary_legacy == nullptr)
		{
			m_rotary_legacy = std::make_unique<Rotary_Velocity_Control>("rotary_velocity", this, InstanceIndex);
			//create a new rotary properties and init with it
			Rotary_Properties legacy_props;
			//if client has provided properties, update them
			if (props)
			{
				//Hack: If you are reading this.. the memory copy is a result of tightly coupled classes from the legacy
				//code.  Ideally I'd like to get a proper fix to this, but for now this will do.
				//of each section
				//legacy_props.AsEntityProps() = props->entity_props;
				memcpy((void*)&legacy_props.AsEntityProps(), &props->entity_props, sizeof(Entity1D_Props));
				//legacy_props.GetShip_1D_Props_rw() = props->ship_props;
				memcpy((void *)&legacy_props.GetShip_1D_Props(), &props->ship_props, sizeof(Ship_1D_Props));
				legacy_props.RotaryProps() = props->rotary_props;
			}
			m_rotary_legacy->Initialize(&legacy_props);
		}
	}
	void ShutDown()
	{}
	void SetVelocity(double rate)
	{
		m_Velocity = rate;
		m_rotary_legacy->SetRequestedVelocity(rate);
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		//voltage in the bypass case is a normalized velocity
		m_VoltageCallback(m_Velocity/m_maxspeed);
		#else
		m_rotary_legacy->TimeChange(d_time_s);
		#endif
	}
	void Reset(double position = 0.0)
	{
		m_Velocity = 0.0;
		m_rotary_legacy->ResetPos();
	}
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
	{
		m_VoltageCallback = callback;
	}
	void SetOdometryCallback(std::function<double()> callback)
	{
		m_OdometryCallack = callback;
	}
};
#pragma endregion
#pragma region _wrapper methods_
void rotary_properties::Init()
{
	//create a new rotary properties and init with it
	Legacy::Rotary_Properties legacy_props;  //this will take care defaults
	//now to update each section
	//Hack: If you are reading this.. the memory copy is a result of tightly coupled classes from the legacy
	//code.  Ideally I'd like to get a proper fix to this, but for now this will do.
	//entity_props = legacy_props.AsEntityProps();
	memcpy((void*)&entity_props, &legacy_props.AsEntityProps(),sizeof(Entity1D_Props));
	//ship_props = legacy_props.GetShip_1D_Props();
	memcpy((void*)&ship_props, &legacy_props.GetShip_1D_Props(),sizeof(Ship_1D_Props));
	rotary_props = legacy_props.GetRotaryProps();
}
#pragma region _Rotary Position_
RotarySystem_Position::RotarySystem_Position()
{
	m_rotary_system = std::make_shared<RotaryPosition_Internal>();
}
void RotarySystem_Position::Init(size_t InstanceIndex, rotary_properties *props)
{
	m_rotary_system->Init(InstanceIndex,props);
}
void RotarySystem_Position::ShutDown()
{
	m_rotary_system->ShutDown();
}
void RotarySystem_Position::SetPosition(double position)
{
	m_rotary_system->SetPosition(position);
}
void RotarySystem_Position::TimeSlice(double d_time_s)
{
	m_rotary_system->TimeSlice(d_time_s);
}
void RotarySystem_Position::Reset(double position)
{
	m_rotary_system->Reset(position);
}
void RotarySystem_Position::Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
{
	m_rotary_system->Set_UpdateCurrentVoltage(callback);
}
void RotarySystem_Position::SetOdometryCallback(std::function<double()> callback)
{
	m_rotary_system->SetOdometryCallback(callback);
}
#pragma endregion
#pragma region _Rotary Velocity_
RotarySystem_Velocity::RotarySystem_Velocity()
{
	m_rotary_system = std::make_shared<RotaryVelocity_Internal>();
}
void RotarySystem_Velocity::Init(size_t InstanceIndex, rotary_properties *props)
{
	m_rotary_system->Init(InstanceIndex,props);
}
void RotarySystem_Velocity::ShutDown()
{
	m_rotary_system->ShutDown();
}
void RotarySystem_Velocity::SetVelocity(double rate)
{
	m_rotary_system->SetVelocity(rate);
}
void RotarySystem_Velocity::TimeSlice(double d_time_s)
{
	m_rotary_system->TimeSlice(d_time_s);
}
void RotarySystem_Velocity::Reset()
{
	m_rotary_system->Reset();
}
void RotarySystem_Velocity::Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
{
	m_rotary_system->Set_UpdateCurrentVoltage(callback);
}
void RotarySystem_Velocity::SetOdometryCallback(std::function<double()> callback)
{
	m_rotary_system->SetOdometryCallback(callback);
}

#pragma endregion
#pragma endregion

	}
}