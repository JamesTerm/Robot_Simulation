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
#include "../../../../Base/PIDController.h"

#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../../../Properties/RegistryV1.h"
#include "SimulatedOdometry.h"
#include "RotaryProperties_Legacy.h"

//Note __UseBypass__ is replaced by a properties driven asset

//TODO
//Get simulator 3 working
//Tweak properties for ideal ride in the example script
//Ensure pot simulator is unlimited

//Keep enabled until we have a better simulator
#define __UseLegacySimulation__

#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _CalibrationTesting_
//I do not want to use any legacy code for the new simulation
#ifdef __UseLegacySimulation__
namespace Legacy {
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
	double m_EncoderScaler; //used to implement reverse
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
		m_EncoderScaler(1.0), m_GetEncoderFirstCall(false)
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
		return m_Physics.GetVelocity() * m_EncoderScaler;
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
		double ret = Voltage * m_EncoderProps.GetMaxSpeed() * m_EncoderScaler;
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
		m_EncoderScaler = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScalar(double value) {m_EncoderScaler=value;}  //This helps to simulate differences between sides
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
		void UpdateProps(const Framework::Base::asset_manager* props = NULL)
		{ 
			if (!props)
				return;
			using namespace ::properties::registry_v1;
			#define GET_NUMBER(x,y) \
			y = props->get_number(csz_##x, y);

			GET_NUMBER(EncoderSimulation_Wheel_Mass,m_Props.Wheel_Mass);
			GET_NUMBER(EncoderSimulation_COF_Efficiency, m_Props.COF_Efficiency);
			GET_NUMBER(EncoderSimulation_GearReduction, m_Props.GearReduction);
			GET_NUMBER(EncoderSimulation_TorqueAccelerationDampener, m_Props.TorqueAccelerationDampener);
			GET_NUMBER(EncoderSimulation_DriveWheelRadius, m_Props.DriveWheelRadius);
			GET_NUMBER(EncoderSimulation_NoMotors, m_Props.NoMotors);
			GET_NUMBER(EncoderSimulation_PayloadMass, m_Props.PayloadMass);
			GET_NUMBER(EncoderSimulation_SpeedLossConstant, m_Props.SpeedLossConstant);
			GET_NUMBER(EncoderSimulation_DriveTrainEfficiency, m_Props.DriveTrainEfficiency);
			//	struct Motor_Specs
			EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			GET_NUMBER(EncoderSimulation_FreeSpeed_RPM, motor.FreeSpeed_RPM);
			GET_NUMBER(EncoderSimulation_Stall_Torque_NM, motor.Stall_Torque_NM);
			GET_NUMBER(EncoderSimulation_Stall_Current_Amp, motor.Stall_Current_Amp);
			GET_NUMBER(EncoderSimulation_Free_Current_Amp, motor.Free_Current_Amp);
			#undef GET_NUMBER
		}

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
	double m_EncoderScaler; //used for position updates
	double m_ReverseMultiply; //used to implement set reverse direction
public:
	Encoder_Simulator2(const char* EntityName = "EncSimulator") : m_Time_s(0.0), m_EncoderScaler(1.0), m_ReverseMultiply(1.0), m_Position(0.0)
	{
	}
	virtual void Initialize(const Framework::Base::asset_manager *props = NULL)
	{
		//const Rotary_Properties* rotary_props = dynamic_cast<const Rotary_Properties*>(props);
		//if (rotary_props)
		//{
			//m_DriveTrain.UpdateProps(rotary_props->GetEncoderSimulationProps());
		//	m_EncoderScaler = rotary_props->GetRotaryProps().EncoderToRS_Ratio;
		//}
		m_DriveTrain.UpdateProps(props);
		if (props)
			m_EncoderScaler = props->get_number(properties::registry_v1::csz_EncoderSimulation_EncoderScaler, 1.0);

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
		m_Position += PositionDisplacement * m_EncoderScaler * m_ReverseMultiply;
	}
	void SetReverseDirection(bool reverseDirection)
	{
		//emulates functionality of the encoder (needed because kids put them in differently)
		m_ReverseMultiply = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScalar(double value) {m_EncoderScaler=value;}  //This helps to simulate differences between sides
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
	virtual void Initialize(const Framework::Base::asset_manager *props = NULL)
	{

		const double Pounds2Kilograms = 0.453592;
		const double wheel_mass = 1.5;
		const double cof_efficiency = 0.9;
		const double gear_reduction = 1.0;
		const double torque_on_wheel_radius = Inches2Meters(1.8);
		const double drive_wheel_radius = Inches2Meters(4.0);
		const double number_of_motors = 1;
		const double payload_mass = 200 * Pounds2Kilograms;
		const double speed_loss_constant = 0.81;
		const double drive_train_effciency = 0.9;

		const double free_speed_rpm = 263.88;
		const double stall_torque = 34;
		const double stall_current_amp = 84;
		const double free_current_amp = 0.4;
		EncoderSimulation_Props defaults_for_sim3 =
		{
			wheel_mass,   //This is a total mass of all the wheels and gears for one side
			cof_efficiency,  //double COF_Efficiency;
			gear_reduction,  //In reciprocal form of spread sheet   driving gear / driven gear
			torque_on_wheel_radius, //double TorqueAccelerationDampener; //ratio.. where 1.0 is no change
			drive_wheel_radius, //double DriveWheelRadius; //in meters
			number_of_motors, //double NoMotors;  //Used to get total torque
			payload_mass, //double PayloadMass;  //The robot weight in kg
			speed_loss_constant, //double SpeedLossConstant;
			drive_train_effciency, //double DriveTrainEfficiency;
			//struct Motor_Specs
			{
				free_speed_rpm, //double FreeSpeed_RPM;
				stall_torque, //double Stall_Torque_NM;
				stall_current_amp, //double Stall_Current_Amp;
				free_current_amp //double Free_Current_Amp;
			} 
		};
		m_DriveTrain.UpdateProps(defaults_for_sim3);
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
		m_Position += PositionDisplacement * m_EncoderScaler * m_ReverseMultiply;

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
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
	{
		__super::Initialize(props);
		//const Rotary_Properties* rotary = dynamic_cast<const Rotary_Properties*>(props);
		//if (rotary)
		//	m_InvEncoderToRS_Ratio = 1.0 / rotary->GetRotaryProps().EncoderToRS_Ratio;
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
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
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

}
#endif
#pragma endregion

#pragma region _Simulated Odometry Internal_
class SimulatedOdometry_Internal
{
private:
	SwerveVelocities m_CurrentVelocities;
	std::function<SwerveVelocities()> m_VoltageCallback;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	double m_current_position[4] = {};  //keep track of the pot's position of each angle
	//const Framework::Base::asset_manager *m_properties=nullptr;  <----reserved
	struct properties
	{
		double swivel_max_speed[4];
	} m_bypass_properties;
	#ifdef __UseLegacySimulation__
	Legacy::Potentiometer_Tester2 m_Potentiometers[4]; //simulate a real potentiometer for calibration testing
	std::shared_ptr<Legacy::Encoder_Simulator2> m_Encoders[4];
	#endif
	bool m_UseBypass = true;

	inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}

public:
	void Init(const Framework::Base::asset_manager *props)
	{
		using namespace ::properties::registry_v1;
		//m_properties = props;  <---- reserved... most likely should restrict properties here
		m_UseBypass = props ? props->get_bool(csz_Build_bypass_simulation, m_UseBypass) : true;
		//Since we define these props as we need them the client code will not need default ones
		m_bypass_properties.swivel_max_speed[0] =
			m_bypass_properties.swivel_max_speed[1] =
			m_bypass_properties.swivel_max_speed[2] =
			m_bypass_properties.swivel_max_speed[3] = 8.0;

		#ifdef __UseLegacySimulation__
		
		for (size_t i = 0; i < 4; i++)
		{
			//TODO hook up to properties... the defaults do not work with sim 3, as they were made for sim 2
			//I may want to make the defaults for 3 override as well
			const bool Simulator3 = props ? !props->get_bool(csz_EncoderSimulation_UseEncoder2,true) : false;
			if (Simulator3)
			{
				using Encoder_Simulator3 = Legacy::Encoder_Simulator3;
				m_Encoders[i] = std::make_shared<Encoder_Simulator3>();
				Encoder_Simulator3* ptr = dynamic_cast<Encoder_Simulator3 *>(m_Encoders[i].get());

				switch (i)
				{

				case 0:
					ptr->SetEncoderKind(Encoder_Simulator3::eRW_Left);
					break;
				case 1:
					ptr->SetEncoderKind(Encoder_Simulator3::eRW_Right);
					break;
				case 2:
					ptr->SetEncoderKind(Encoder_Simulator3::eReadOnlyLeft);
					break;
				case 3:
					ptr->SetEncoderKind(Encoder_Simulator3::eReadOnlyRight);
					break;
				}
			}
			else
				m_Encoders[i] = std::make_shared<Legacy::Encoder_Simulator2>();
			m_Encoders[i]->Initialize(props);
			m_Potentiometers[i].Initialize();
		}
		#endif
		ResetPos();
	}
	void ResetPos()
	{
		#ifdef __UseLegacySimulation__

		for (size_t i = 0; i < 4; i++)
		{
			m_Encoders[i]->ResetPos();
			m_Potentiometers[i].ResetPos();
		}
		#else
		#endif
	}
	void ShutDown()
	{
		//reserved
	}
	~SimulatedOdometry_Internal()
	{
		ShutDown();
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		//Input get it from client
		m_VoltageCallback = callback;
	}
	//Run the simulation time-slice
	void TimeSlice(double d_time_s)
	{
		if (m_UseBypass)
		{
			//If only life were this simple, but alas robots are not god-ships
			m_CurrentVelocities = m_VoltageCallback();
			//convert voltages back to velocities
			//for drive this is easy
			for (size_t i = 0; i < 4; i++)
			{
				m_CurrentVelocities.Velocity.AsArray[i] *= m_maxspeed;
			}
			//for position we have to track this ourselves
			for (size_t i = 0; i < 4; i++)
			{
				//go ahead and apply the voltage to the position... this is over-simplified but effective for a bypass
				//from the voltage determine the velocity delta
				const double velocity_delta = m_CurrentVelocities.Velocity.AsArray[i + 4] * m_bypass_properties.swivel_max_speed[i];
				m_current_position[i] = NormalizeRotation2(m_current_position[i] + velocity_delta * d_time_s);
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];
			}
		}
		else
		{
			#ifdef __UseLegacySimulation__
			SwerveVelocities CurrentVoltage;
			CurrentVoltage = m_VoltageCallback();

			for (size_t i = 0; i < 4; i++)
			{
				//We'll put the pot update first in case the simulation factors in the direction (probably shouldn't matter though)
				m_Potentiometers[i].SetTimeDelta(d_time_s);
				m_Potentiometers[i].UpdatePotentiometerVoltage(CurrentVoltage.Velocity.AsArray[i + 4]);
				m_Potentiometers[i].TimeChange();
				m_current_position[i] = NormalizeRotation2(m_Potentiometers[i].GetPotentiometerCurrentPosition());
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];

				m_Encoders[i]->SetTimeDelta(d_time_s);
				m_Encoders[i]->UpdateEncoderVoltage(CurrentVoltage.Velocity.AsArray[i]);
				m_Encoders[i]->TimeChange();
				m_CurrentVelocities.Velocity.AsArray[i] = m_Encoders[i]->GetEncoderVelocity();
			}
			#else
			//TODO reserved
			//m_CurrentVelocities = m_VoltageCallback();
			#endif
		}
	}
	const SwerveVelocities &GetCurrentVelocities() const
	{
		//Output: contains the current speeds and positions of any given moment of time
		return m_CurrentVelocities;
	}
};
#pragma endregion
#pragma region _wrapper methods_
SimulatedOdometry::SimulatedOdometry()
{
	m_simulator = std::make_shared<SimulatedOdometry_Internal>();
}
void SimulatedOdometry::Init(const Framework::Base::asset_manager* props)
{
	m_simulator->Init(props);
}
void SimulatedOdometry::ResetPos()
{
	m_simulator->ResetPos();
}
void SimulatedOdometry::Shutdown()
{
	m_simulator->ShutDown();
}
void SimulatedOdometry::SetVoltageCallback(std::function<SwerveVelocities()> callback)
{
	m_simulator->SetVoltageCallback(callback);
}
void SimulatedOdometry::TimeSlice(double d_time_s)
{
	m_simulator->TimeSlice(d_time_s);
}
const SwerveVelocities &SimulatedOdometry::GetCurrentVelocities() const
{
	return m_simulator->GetCurrentVelocities();
}

#pragma endregion
}}