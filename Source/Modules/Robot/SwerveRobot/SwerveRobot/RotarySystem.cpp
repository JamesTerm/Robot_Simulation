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
//Useful for diagnostics
#define __UseBypass__
#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Rotary System Legacy_
#pragma region _Rotary System Properties_
#pragma region _Entity1D_Properties_

//TODO: may wish to aggregate the props to be consistent later
class COMMON_API Entity1D_Properties : public rotary_properties::Entity1D_Props
{
private:
	std::string m_EntityName;  //derived classes can let base class know what type to read
public:
	Entity1D_Properties()
	{
		m_EntityName = "Entity1D";
		m_Mass = 10000.0;
		m_Dimension = 12.0;
		m_IsAngular = false;
		m_StartingPosition = 0.0;
	}
	Entity1D_Properties(const char EntityName[], double Mass, double Dimension, bool IsAngular = false)
	{
		m_EntityName = EntityName;
		m_Mass = Mass;
		m_Dimension = Dimension;
		m_IsAngular = IsAngular;
		m_StartingPosition = 0.0;
	}
	virtual ~Entity1D_Properties() {}
	double GetMass() const { return m_Mass; }
	Entity1D_Props &AsEntityProps() { return *this; }
};
#pragma endregion
#pragma region _Ship_1D_Properties_

class COMMON_API Ship_1D_Properties : public Entity1D_Properties
{
private:
	using Ship_1D_Props = rotary_properties::Ship_1D_Props;
	Ship_1D_Props m_Ship_1D_Props;
public:
	//typedef Ship_1D_Props::Ship_Type Ship_Type;

	Ship_1D_Properties()
	{
		double Scale = 0.2;  //we must scale everything down to see on the view
		m_Ship_1D_Props.MAX_SPEED = m_Ship_1D_Props.MaxSpeed_Forward =
			m_Ship_1D_Props.MaxSpeed_Reverse = 400.0 * Scale;
		m_Ship_1D_Props.MaxAccelReverse = -m_Ship_1D_Props.MAX_SPEED;
		m_Ship_1D_Props.ACCEL = 60.0 * Scale;
		m_Ship_1D_Props.BRAKE = 50.0 * Scale;

		m_Ship_1D_Props.MaxAccelForward = 87.0 * Scale;
		m_Ship_1D_Props.MaxAccelReverse = 70.0 * Scale;
		m_Ship_1D_Props.MinRange = m_Ship_1D_Props.MaxRange = 0.0;
		m_Ship_1D_Props.UsingRange = false;
		m_Ship_1D_Props.DistanceDegradeScalar = 1.0;  //only can be changed in script!
	}
	//Allow to construct props in constructor instead of using script
	Ship_1D_Properties(const char EntityName[], double Mass, double Dimension,
		double MAX_SPEED, double ACCEL, double BRAKE, double MaxAccelForward, double MaxAccelReverse,
		//Ship_Type ShipType = Ship_1D_Props::eDefault, 
		bool UsingRange = false, double MinRange = 0.0, double MaxRange = 0.0,
		bool IsAngular = false)
	{
		m_Ship_1D_Props.MAX_SPEED = m_Ship_1D_Props.MaxSpeed_Forward = MAX_SPEED;
		m_Ship_1D_Props.MaxSpeed_Reverse = -MAX_SPEED;
		m_Ship_1D_Props.ACCEL = ACCEL;
		m_Ship_1D_Props.BRAKE = BRAKE;
		m_Ship_1D_Props.MaxAccelForward = MaxAccelForward;
		m_Ship_1D_Props.MaxAccelReverse = MaxAccelReverse;
		//m_Ship_1D_Props.ShipType = ShipType;
		m_Ship_1D_Props.MinRange = MinRange;
		m_Ship_1D_Props.MaxRange = MaxRange;
		m_Ship_1D_Props.UsingRange = UsingRange;
		m_Ship_1D_Props.DistanceDegradeScalar = 1.0;  //only can be changed in script!
	}

	//virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults = false);
	void UpdateShip1DProperties(const Ship_1D_Props &props)
	{
		//explicitly allow updating of ship props here
		m_Ship_1D_Props = props;
	}
	//Ship_Type GetShipType() const { return m_Ship_1D_Props.ShipType; }
	double GetMaxSpeed() const { return m_Ship_1D_Props.MAX_SPEED; }
	const Ship_1D_Props &GetShip_1D_Props() const { return m_Ship_1D_Props; }
	Ship_1D_Props &GetShip_1D_Props_rw() { return m_Ship_1D_Props; }
public:
	//These are for testing purposes only (do not use)
	void SetMinRange(double MinRange) { m_Ship_1D_Props.MinRange = MinRange; }
	void SetMaxRange(double MaxRange) { m_Ship_1D_Props.MaxRange = MaxRange; }
	void SetUsingRange(bool UsingRange) { m_Ship_1D_Props.UsingRange = UsingRange; }
	//copy constructor that can interpret the other type
	//void SetFromShip_Properties(const Ship_Props & NewValue) { m_Ship_1D_Props.SetFromShip_Properties(NewValue); }
};
#pragma endregion
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

#pragma region _Entity1D_
class COMMON_API Entity1D
{
private:
	#pragma region _members_
	using PhysicsEntity_1D = Framework::Base::PhysicsEntity_1D;
	friend class Entity1D_Properties;
	//Base::EventMap* m_eventMap;
	double m_StartingPosition=0.0;  //the position used when reset position is called
	double m_Dimension=1.0;
	double m_Position=0.0;
	std::string m_Name;
	bool m_BypassPos_Update=false;  //used to preserve pos during a ResetPos() call
	#pragma endregion
	inline void NormalizeRotation(double &Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
	}
	#pragma region _protected_
protected:
	PhysicsEntity_1D m_Physics;
	bool m_IsAngular;
	virtual bool InjectDisplacement(double DeltaTime_s, double &PositionDisplacement)
	{
		///This gives derived class the ability to manipulate the displacement
		/// \ret true if this is to be used and manipulated, false uses the default displacement
		return false;
	}
	#pragma endregion
public:
	Entity1D(const char EntityName[]) : m_Name(EntityName)
	{
		ResetPos();
	}
	//This allows the game client to setup the ship's characteristics
	virtual void Initialize(const Entity1D_Properties *props = NULL)
	{
		if (props)
		{
			m_Dimension = props->m_Dimension;
			GetPhysics().SetMass(props->m_Mass);
			m_IsAngular = props->m_IsAngular;
			m_StartingPosition = props->m_StartingPosition;
		}
	}
	virtual ~Entity1D()
	{
		//Game Client will be nuking this pointer
	}
	const std::string &GetName() const { return m_Name; }
	virtual void TimeChange(double dTime_s)
	{
		double PositionDisplacement = 0.0;
		//Either we apply displacement computations here or the derived class will handle it
		if (!InjectDisplacement(dTime_s, PositionDisplacement))
			m_Physics.TimeChangeUpdate(dTime_s, PositionDisplacement);

		m_Position += PositionDisplacement;
		if (m_IsAngular)
			NormalizeRotation(m_Position);
	}
	PhysicsEntity_1D &GetPhysics() { return m_Physics; }
	const PhysicsEntity_1D &GetPhysics() const { return m_Physics; }
	virtual double GetDimension() const { return m_Dimension; }
	virtual double GetStartingPosition() const { return m_StartingPosition; }
	virtual void ResetPosition(double Position)
	{
		//CancelAllControls();
		m_Physics.ResetVectors();
		if (!m_BypassPos_Update)
			m_Position = Position;
	}
	virtual void ResetPos()
	{
		ResetPosition(m_StartingPosition);
	}
	virtual const double &GetIntendedPosition() const 
	{ 
		// This is where both the entity and camera need to align to, by default we use the actual position
		return m_Position;
	}
	//Base::EventMap* GetEventMap() { return m_eventMap; }
	virtual double GetPos_m() const { return m_Position; }
	void SetPos_m(double value) 
	{ 
		//This is used when a sensor need to correct for the actual position
		m_Position = value; 
	}
	void SetBypassPos_Update(bool bypass) 
	{ 
		//Be sure to always set this back to false!
		m_BypassPos_Update = bypass; 
	}
	bool GetBypassPos_Update() const { return m_BypassPos_Update; }
};
#pragma endregion
#pragma region _Ship_1D_

///This is really stripped down from the Ship_2D.  Not only is their one dimension (given), but there is also no controller.  This class is intended
///To be controlled directly from a parent class which has controller support.
class COMMON_API Ship_1D : public Entity1D
{
private:
	#pragma region _members_
	using PhysicsEntity_1D = Framework::Base::PhysicsEntity_1D;
	//Only used with SetRequestedVelocity_FromNormalized()
	//this is managed direct from being set to avoid need for precision tolerance
	double m_LastNormalizedVelocity;
	bool m_LockShipToPosition; ///< Locks the ship to intended position (Joystick and Keyboard controls use this)
	#pragma endregion
protected:
	#pragma region _protected members_
	using Ship_1D_Props = rotary_properties::Ship_1D_Props;
	friend class Ship_1D_Properties;
	Ship_1D_Props m_Ship_1D_Props;

	//Stuff needed for physics
	double m_Mass;

	//Use this technique when m_AlterTrajectory is true
	double m_RequestedVelocity;
	//All input for turn pitch and roll apply to this, both the camera and ship need to align to it
	double m_IntendedPosition;
	//We need the m_IntendedPosition to work with its own physics
	PhysicsEntity_1D m_IntendedPositionPhysics;

	//For slide mode all strafe is applied here
	double m_currAccel;  //This is the immediate request for thruster levels
	double m_Last_RequestedVelocity;  ///< This monitors the last caught requested velocity  from a speed delta change
	bool m_SimFlightMode;  ///< If true auto strafing will occur to keep ship in line with its position
	#pragma endregion
	static void InitNetworkProperties(const Ship_1D_Props &props)
	{
		//This will PutVariables of all properties needed
		//TODO if we want this.. make callbacks, no smart dashboard in here!
		//SmartDashboard::PutNumber("max_accel_forward", props.MaxAccelForward);
		//SmartDashboard::PutNumber("max_accel_reverse", props.MaxAccelReverse);
	}
	static void NetworkEditProperties(Ship_1D_Props &props)
	{
		//This will GetVariables of all properties needed
		//props.MaxAccelForward = SmartDashboard::GetNumber("max_accel_forward");
		//props.MaxAccelReverse = SmartDashboard::GetNumber("max_accel_reverse");
	}
	virtual double GetMatchVelocity() const 
	{
		///override if the intended position has a known velocity to match (this is great for locking)
		return 0.0; 
	}
	void UpdateIntendedPosition(double dTime_s)
	{
		///This will apply turn pitch and roll to the intended orientation
		if (m_LockShipToPosition)
		{
			//Keep the intended position locked to the current position, since we are not managing it like we do in 2D/3D.
			//once the mouse kicks in it will be in the correct starting place to switch modes
			m_IntendedPosition = GetPos_m();
		}
		else
			m_IntendedPosition += m_currAccel;
	}
	virtual void PosDisplacementCallback(double posDisplacement_m) 
	{
		///Client code can use this to evaluate the distance to intervene as necessary
		///For example this could be used to allow more tolerance for position by killing voltage in the tolerance zone
	}
	virtual void TimeChange(double dTime_s)
	{
		const Ship_1D_Props &props=m_Ship_1D_Props;
		// Find the current velocity and use to determine the flight characteristics we will WANT to us
		double LocalVelocity=m_Physics.GetVelocity();
		double currFwdVel = LocalVelocity;
		bool manualMode = !((m_SimFlightMode)&&(m_currAccel==0));

		double ForceToApply;
		double posDisplacement_m=0;

		{
			UpdateIntendedPosition(dTime_s);
			//Determine the angular distance from the intended orientation
			posDisplacement_m=m_IntendedPosition-GetPos_m();
			//TODO determine why this fails in the goals, and if we really want it here
			//apply shortest angle equation to handle end cases properly
			//posDisplacement_m -= Pi2*floor(posDisplacement_m/Pi2+0.5);
			PosDisplacementCallback(posDisplacement_m);  //call the callback with this value
		}

		//Apply the restraints now... I need this to compute my roll offset
		const double AccRestraintPositive=props.MaxAccelForward;
		const double AccRestraintNegative=props.MaxAccelReverse;

		const double DistanceRestraintPositive=props.MaxAccelForward*props.DistanceDegradeScalar;
		const double DistanceRestraintNegative=props.MaxAccelReverse*props.DistanceDegradeScalar;

		//Unlike in 2D the intended position and velocity control now resides in the same vector to apply force.  To implement, we'll branch depending on
		//which last LockShipToPosition was used.  Typically speaking the mouse, AI, or SetIntendedPosition() will branch to the non locked mode, while the
		//joystick and keyboard will conduct the locked mode
		if (m_LockShipToPosition)
		{
			if (!manualMode)
			{
				//This first system combined the speed request and the accel delta's as one but this runs into undesired effects with the accel deltas
				//The most undesired effect is that when no delta is applied neither should any extra force be applied.  There is indeed a distinction
				//between cruise control (e.g. slider) and using a Key button entry in this regard.  The else case here keeps these more separated where
				//you are either using one mode or the other
				double VelocityDelta=m_currAccel*dTime_s;

				bool UsingRequestedVelocity=false;

				//Note: m_RequestedVelocity is not altered with the velocity delta, but it will keep up to date
				if (VelocityDelta!=0) //if user is changing his adjustments then reset the velocity to current velocity
					m_RequestedVelocity=m_Last_RequestedVelocity=currFwdVel+VelocityDelta;
				else
					UsingRequestedVelocity=(m_RequestedVelocity!=m_Last_RequestedVelocity);

				//Just transfer the acceleration directly into our velocity to use variable
				double VelocityToUse=(UsingRequestedVelocity)? m_RequestedVelocity:currFwdVel;


				#ifndef __DisableSpeedControl__
				{
					// Watch for braking too far backwards, we do not want to go beyond -ENGAGED_MAX_SPEED
					if ((VelocityToUse) < props.MaxSpeed_Reverse)
					{
						m_RequestedVelocity = VelocityToUse = props.MaxSpeed_Reverse;
						m_currAccel=0.0;
					}
					else if ((VelocityToUse) > props.MaxSpeed_Forward)
					{
						m_RequestedVelocity = VelocityToUse=props.MaxSpeed_Forward;
						m_currAccel=0.0;
					}
				}
				#endif

				if (props.UsingRange)
				{
					double Position=GetPos_m();
					//check to see if we are going reach limit
					if (VelocityToUse>0.0)
					{
						double Vel=m_Physics.GetVelocityFromDistance_Linear(props.MaxRange-Position,DistanceRestraintPositive*m_Mass,DistanceRestraintNegative*m_Mass,dTime_s,0.0);
						if (Vel<VelocityToUse)
							VelocityToUse=Vel;
					}
					else
					{
						double Vel=m_Physics.GetVelocityFromDistance_Linear(props.MinRange-Position,DistanceRestraintPositive*m_Mass,DistanceRestraintNegative*m_Mass,dTime_s,0.0);
						if (fabs(Vel)<fabs(VelocityToUse))
							VelocityToUse=Vel;
					}
				}
				ForceToApply=m_Physics.GetForceFromVelocity(VelocityToUse,dTime_s);
				if (!UsingRequestedVelocity)
					ForceToApply+=m_currAccel * m_Mass;
				//Allow subclass to evaluate the requested velocity in use;
				RequestedVelocityCallback(VelocityToUse,dTime_s);
			}
			else   //Manual mode
			{
				#ifndef __DisableSpeedControl__
				{
					{
						double VelocityDelta=m_currAccel*dTime_s;
						if ((LocalVelocity+VelocityDelta>props.MaxSpeed_Forward)&&(m_currAccel>0))
							m_currAccel= (props.MaxSpeed_Forward-LocalVelocity) / dTime_s;  //saturate the delta
						else if ((LocalVelocity+VelocityDelta<props.MaxSpeed_Forward)&&(m_currAccel<0))
							m_currAccel=(props.MaxSpeed_Reverse-LocalVelocity) / dTime_s;  //saturate the delta
					}
				}
				#endif
				ForceToApply=m_currAccel*m_Mass;

				//Note: in this case lock to position should not have set point operations when it is angular... this logic should be sound, as it has no effect with position
				//This will be managed in the speed control section
				if ((props.UsingRange)&&(!m_IsAngular))
				{
					double Position=GetPos_m();
					double Vel;
					//check to see if we are going reach limit
					if (ForceToApply>0.0)
						Vel=m_Physics.GetVelocityFromDistance_Linear(props.MaxRange-Position,DistanceRestraintPositive*m_Mass,DistanceRestraintNegative*m_Mass,dTime_s,0.0);
					else
						Vel=m_Physics.GetVelocityFromDistance_Linear(props.MinRange-Position,DistanceRestraintPositive*m_Mass,DistanceRestraintNegative*m_Mass,dTime_s,0.0);
					double TestForce=m_Physics.GetForceFromVelocity(Vel,dTime_s);
					if (fabs(ForceToApply)>fabs(TestForce)) 
						ForceToApply=TestForce;
				}
			}
			ForceToApply=m_Physics.ComputeRestrainedForce(ForceToApply,AccRestraintPositive*m_Mass,AccRestraintNegative*m_Mass,dTime_s);
		}
		else
		{
			double Vel;

			{
				double DistanceToUse=posDisplacement_m;
				double MatchVelocity=GetMatchVelocity();
				//Most likely these should never get triggered unless there is some kind of control like the mouse that can go beyond the limit
				if (props.UsingRange)
				{
					if (m_IntendedPosition>props.MaxRange)
						DistanceToUse=props.MaxRange-GetPos_m();
					else if(m_IntendedPosition<props.MinRange)
						DistanceToUse=props.MinRange-GetPos_m();
				}
				if (!m_IsAngular)
				{
					//The match velocity needs to be in the same direction as the distance (It will not be if the ship is banking)
					Vel=m_Physics.GetVelocityFromDistance_Linear(DistanceToUse,DistanceRestraintPositive*m_Mass,DistanceRestraintNegative*m_Mass,dTime_s,MatchVelocity);
				}
				else
					Vel=m_Physics.GetVelocityFromDistance_Angular(DistanceToUse,DistanceRestraintPositive*m_Mass,dTime_s,MatchVelocity);
			}

			#ifndef __DisableSpeedControl__
			{
				if ((Vel) < props.MaxSpeed_Reverse)
				{
					Vel = props.MaxSpeed_Reverse;
					m_currAccel=0.0;
				}
				else if ((Vel) > props.MaxSpeed_Forward) 
				{
					Vel=props.MaxSpeed_Forward;
					m_RequestedVelocity=props.MaxSpeed_Forward;
					m_currAccel=0.0;
				}
			}
			#endif

			ForceToApply=m_Physics.ComputeRestrainedForce(m_Physics.GetForceFromVelocity(Vel,dTime_s),AccRestraintPositive*m_Mass,AccRestraintNegative*m_Mass,dTime_s);
		}


		//To be safe we reset this to zero (I'd put a critical section around this line of code if there are thread issues
		posDisplacement_m=0.0;

		//ApplyThrusters(m_Physics,ForceToApply,TorqueToApply,Ships_TorqueRestraint,dTime_s);
		m_Physics.ApplyFractionalForce(ForceToApply,dTime_s);


		// Now to run the time updates (displacement plus application of it)
		Entity1D::TimeChange(dTime_s);

		m_currAccel=0.0;
	}
	virtual bool IsPlayerControllable() 
	{ 
		// Watch for being made the controlled ship
		return true; 
	}
public:
	Ship_1D(const char EntityName[]) : Entity1D(EntityName)
	{
		SetSimFlightMode(true);  //this sets up the initial speed as well
		m_LockShipToPosition = false;  //usually this is false (especially for AI and Remote controllers)
		ResetPos();
	}
	void UpdateShip1DProperties(const Ship_1D_Props &props)
	{
		m_Ship_1D_Props = props;
	}
	virtual void Initialize(const Entity1D_Properties *props = NULL)
	{
		__super::Initialize(props);
		const Ship_1D_Properties *ship_props = dynamic_cast<const Ship_1D_Properties *>(props);
		if (ship_props)
		{
			//UpdateShip1DProperties(ship_props->GetShip_1D_Props());  depreciated
			m_Ship_1D_Props = ship_props->GetShip_1D_Props();  //if we support it
		}
		else
		{
			Ship_1D_Props &_ = m_Ship_1D_Props;
			_.MAX_SPEED = 1.0;
			_.ACCEL = 1.0;
			_.BRAKE = 1.0;

			_.MaxAccelForward = 1.0;
			_.MaxAccelReverse = 1.0;
			_.UsingRange = false;
			_.MinRange = _.MaxRange = 0;
			m_IsAngular = false;
		}
		m_Mass = m_Physics.GetMass();

		m_IntendedPosition = 0.0;
		m_IntendedPositionPhysics.SetMass(m_Mass);
	}
	virtual ~Ship_1D()
	{
	}
	void Stop() 
	{
		///This implicitly will place back in auto mode with a speed of zero
		SetRequestedVelocity(0.0); 
	}
	virtual void RequestedVelocityCallback(double VelocityToUse, double DeltaTime_s) 
	{
		///This allows subclass to evaluate the requested velocity when it is in use
	}
	virtual void ResetPosition(double Position)
	{
		// Places the ship back at its initial position and resets all vectors
		__super::ResetPosition(Position);
		m_RequestedVelocity = m_currAccel = 0.0;
		//See case 397... for the 1D this is not really an issue as once we get it started it works itself out... it just cannot be zero to get it started
		m_Last_RequestedVelocity = -1.0;
		m_IntendedPosition = Position;
		m_IntendedPositionPhysics.ResetVectors();
		m_LastNormalizedVelocity = 0.0;
		//m_Physics.ResetVectors(); called from entity 1D's reset
		SetSimFlightMode(true);  //This one is a tough call... probably should do it on reset
	}
	virtual void BindAdditionalEventControls(bool Bind) 
	{
		//The UI controller will call this when attaching or detaching control.  The Bind parameter will either bind or unbind.  Since these are 
		//specific controls to a specific ship there is currently no method to transfer these specifics from one ship to the next.  Ideally there
		//should be no member variables needed to implement the bindings
	}
	#pragma region _mutators_
	void SetIntendedPosition(double Position)
	{
		/// This is like setting a way point since there is one dimension there is only one setting to use here
		m_LockShipToPosition = false, m_IntendedPosition = Position;
	}
	void SetRequestedVelocity(double Velocity)
	{
		SetSimFlightMode(true);
		m_LockShipToPosition = true;  //unlike in 2D/3D setting this has an impact on the locking management
		if (Velocity > 0.0)
			m_RequestedVelocity = MIN(Velocity, m_Ship_1D_Props.MaxSpeed_Forward);
		else
			m_RequestedVelocity = MAX(Velocity, m_Ship_1D_Props.MaxSpeed_Reverse);
	}
	void SetMaxSpeedForward(double Velocity)
	{
		//Allow read write access to the speed controls, these are native as velocity so typically the reverse will have a negative direction
		//Note: client should ensure these get restored back to their default property value (hence the read access)
		m_Ship_1D_Props.MaxSpeed_Forward = Velocity;
	}
	void SetMaxSpeedReverse(double Velocity) 
	{ 
		m_Ship_1D_Props.MaxSpeed_Reverse = Velocity; 
	}
	void SetRequestedVelocity_FromNormalized(double Velocity)
	{
		//This will scale the velocity by max speed and also handle flood control
		//we must have flood control so that other controls may work (the joystick will call this on every time slice!)
		if (Velocity != m_LastNormalizedVelocity)
		{
			//scale the velocity to the max speed's magnitude
			double VelocityScaled = Velocity * GetMaxSpeed();
			SetRequestedVelocity(VelocityScaled);
			m_LastNormalizedVelocity = Velocity;
		}
	}
	void SetCurrentLinearAcceleration(double Acceleration, bool LockShipToPosition = true)
	{
		/// \param LockShipHeadingToPosition for this given time slice if this is true the intended orientation is restrained
		/// to the ships restraints and the ship is locked to the orientation (Joy/Key mode).  If false (Mouse/AI) the intended orientation
		/// is not restrained and the ship applies its restraints to catch up to the orientation. \note this defaults to true since this is 
		/// most likely never going to be used with a mouse or AI

		m_LockShipToPosition = LockShipToPosition, m_currAccel = Acceleration;
	}
	void SetSimFlightMode(bool SimFlightMode)
	{
		//It seems that some people want/need to call this function repeatedly so I have included a valve branch here to prevent the debug flooding
		//And to not do extra work on the m_RequestedVelocity.
		if (m_SimFlightMode != SimFlightMode)
		{
			m_RequestedVelocity = m_Physics.GetVelocity();
			m_SimFlightMode = SimFlightMode;
			//DebugOutput("SimFlightMode=%d\n", SimFlightMode);
		}
	}
	#pragma endregion
	#pragma region _accessors_
	double GetMaxSpeedForward() const 
	{ 
		return m_Ship_1D_Props.MaxSpeed_Forward; 
	}
	double GetMaxSpeedReverse() const 
	{ 
		return m_Ship_1D_Props.MaxSpeed_Reverse; 
	}
	double GetRequestedVelocity() const 
	{ 
		return m_RequestedVelocity; 
	}
	virtual const double &GetIntendedPosition() const
	{
		// This is where both the vehicle entity and camera need to align to
		return m_IntendedPosition;
	}
	bool GetAlterTrajectory() const
	{ 
		return m_SimFlightMode; 
	}
	double GetMaxSpeed() const
	{
		//Note:  This returns the maximum speed possible and not the desired max speed
		return m_Ship_1D_Props.MAX_SPEED;
	}
	double GetACCEL() const 
	{ 
		return m_Ship_1D_Props.ACCEL; 
	}
	double GetBRAKE() const 
	{ 
		return m_Ship_1D_Props.BRAKE; 
	}
	bool GetLockShipToPosition() const
	{
		return m_LockShipToPosition;
	}
	double GetMinRange() const 
	{ 
		return m_Ship_1D_Props.MinRange; 
	}
	double GetMaxRange() const 
	{ 
		return m_Ship_1D_Props.MaxRange; 
	}
	bool GetUsingRange() const 
	{ 
		return m_Ship_1D_Props.UsingRange; 
	}
	#pragma endregion

	Entity1D &AsEntity1D() { return *this; }
	Ship_1D &AsShip1D() { return *this; }
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
#pragma endregion

#pragma region _RotaryPosition_Internal_
class RotaryPosition_Internal : public Rotary_Control_Interface
{
private:
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
				//of each section
				legacy_props.AsEntityProps() = props->entity_props;
				legacy_props.GetShip_1D_Props_rw() = props->ship_props;
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
class RotaryVelocity_Internal :public Rotary_Control_Interface
{
private:
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
		if (m_rotary_legacy == nullptr)
		{
			m_rotary_legacy = std::make_unique<Rotary_Velocity_Control>("rotary_velocity", this, InstanceIndex);
			//create a new rotary properties and init with it
			Rotary_Properties legacy_props;
			//if client has provided properties, update them
			if (props)
			{
				//of each section
				legacy_props.AsEntityProps() = props->entity_props;
				legacy_props.GetShip_1D_Props_rw() = props->ship_props;
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
	Rotary_Properties legacy_props;  //this will take care defaults
	//now to update each section
	entity_props = legacy_props.AsEntityProps();
	ship_props = legacy_props.GetShip_1D_Props();
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