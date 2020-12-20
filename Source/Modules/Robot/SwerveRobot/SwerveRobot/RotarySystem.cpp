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
#include "RotaryProperties_Legacy.h"

//Useful for diagnostics
//#define __UseBypass__

//We may want to disable spamming if we are looking for other printf statements
//Note: this only is used if the properties turns it on
//#define __DisableConsoleOutPID_Dump__

#ifndef __DisableConsoleOutPID_Dump__
#define pid_cout(x,...) printf(x,__VA_ARGS__);
#else
#define pid_cout(x,...)
#endif

#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Rotary System Legacy_
namespace Legacy {
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
	std::function<RotarySystem_Position::PID_Monitor_proto> m_PID_Monitor_callback = nullptr;
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
		//m_Rotary_Props = Props->GetRotaryProps();
		memcpy(&m_Rotary_Props,&Props->GetRotaryProps(),sizeof(Rotary_Props)); //Hack, legacy ties :(
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

			const double PosY = m_LastPosition * arm.GainAssistAngleScalar; //The scalar makes position more readable
			const double PredictedPosY = GetPos_m()  * arm.GainAssistAngleScalar;
			if ((fabs(PotentiometerVelocity) > 0.03) || (CurrentVelocity != 0.0) || (Voltage != 0.0))
			{
				if (m_PID_Monitor_callback)
					m_PID_Monitor_callback(Voltage, PosY, PredictedPosY, CurrentVelocity, PotentiometerVelocity, m_ErrorOffset);
				else
				{
					//double PosY=RAD_2_DEG(m_LastPosition * arm.GainAssistAngleScalar);
					pid_cout("v=%.2f y=%.2f py=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, PosY, PredictedPosY, CurrentVelocity, PotentiometerVelocity, m_ErrorOffset);
				}
			}
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
	void SetExternal_PID_Monitor_Callback(std::function<RotarySystem_Position::PID_Monitor_proto> callback)
	{
		m_PID_Monitor_callback = callback;
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
	std::function<RotarySystem_Velocity::PID_Monitor_proto> m_PID_Monitor_callback=nullptr;
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
		//m_Rotary_Props = Props->GetRotaryProps();
		memcpy(&m_Rotary_Props, &Props->GetRotaryProps(), sizeof(Rotary_Props)); //Hack, legacy ties :(
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

			if (Encoder_Velocity != 0.0)
			{
				//Note: if we want a console dump and we have an external plugin... it can be implemented and managed there
				if (m_Rotary_Props.UseAggressiveStop)
				{
					if (m_PID_Monitor_callback)
						m_PID_Monitor_callback(Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset, 0.0);
					else
						pid_cout("v=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset);
				}
				else
				{
					if (m_PIDController.GetI() == 0.0)
					{
						pid_cout("v=%.2f p=%.2f e=%.2f eo=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset, m_CalibratedScaler / MaxSpeed);
						if (m_PID_Monitor_callback)
							m_PID_Monitor_callback(Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset, m_CalibratedScaler / MaxSpeed);
						else
							pid_cout("v=%.2f p=%.2f e=%.2f eo=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_ErrorOffset);
					}
					else
					{
						if (m_PID_Monitor_callback)
							m_PID_Monitor_callback(Voltage, CurrentVelocity, Encoder_Velocity, m_PIDController.GetTotalError(), m_CalibratedScaler / MaxSpeed);
						else
							pid_cout("v=%.2f p=%.2f e=%.2f i=%.2f cs=%.2f\n", Voltage, CurrentVelocity, Encoder_Velocity, m_PIDController.GetTotalError(), m_CalibratedScaler / MaxSpeed);
					}
				}
			}
		}

		m_RobotControl->UpdateRotaryVoltage(m_InstanceIndex, Voltage);
	}
	void SetExternal_PID_Monitor_Callback(std::function<RotarySystem_Velocity::PID_Monitor_proto> callback)
	{
		m_PID_Monitor_callback = callback;
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
	std::function<RotarySystem_Position::PID_Monitor_proto> m_PID_callback=nullptr; //cache to delay hook until init
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
			//now we can set hooks
			m_rotary_legacy->SetExternal_PID_Monitor_Callback(m_PID_callback);

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
				//legacy_props.RotaryProps() = props->rotary_props;
				memcpy(&legacy_props.RotaryProps(), &props->rotary_props, sizeof(Rotary_Props));
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
	void SetExternal_PID_Monitor_Callback(std::function<RotarySystem_Position::PID_Monitor_proto> callback)
	{
		m_PID_callback = callback;
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
	std::function<RotarySystem_Velocity::PID_Monitor_proto> m_PID_callback=nullptr; //cache to delay hook until init
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
			//now we can set hooks
			m_rotary_legacy->SetExternal_PID_Monitor_Callback(m_PID_callback);
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
				//legacy_props.RotaryProps() = props->rotary_props;
				memcpy(&legacy_props.RotaryProps(), &props->rotary_props, sizeof(Rotary_Props));
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
	void SetExternal_PID_Monitor_Callback(std::function<RotarySystem_Velocity::PID_Monitor_proto> callback)
	{
		m_PID_callback = callback;
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
	//rotary_props = legacy_props.GetRotaryProps();
	memcpy(&rotary_props, &legacy_props.GetRotaryProps(), sizeof(Rotary_Props));
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
void RotarySystem_Position::SetExternal_PID_Monitor_Callback(std::function<PID_Monitor_proto> callback)
{
	m_rotary_system->SetExternal_PID_Monitor_Callback(callback);
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
void RotarySystem_Velocity::SetExternal_PID_Monitor_Callback(std::function<PID_Monitor_proto> callback)
{
	m_rotary_system->SetExternal_PID_Monitor_Callback(callback);
}
#pragma endregion
#pragma endregion

	}
}