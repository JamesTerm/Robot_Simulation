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
	//virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL);
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

struct COMMON_API Ship_1D_Props
{
	//void SetFromShip_Properties(const Ship_Props & NewValue);

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
	double DistanceDegradeScalar;

	//TODO these are somewhat specific, we may want to move subclass them or have more generic meaning
	enum Ship_Type
	{
		eDefault,
		eRobotArm,
		eSimpleMotor,
		eSwivel,
	};
	Ship_Type ShipType;
	bool UsingRange;
};

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
	//virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL);
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
#pragma region _Rotory_System_
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
	//virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL)
	virtual void Initialize()
	{
		//Cache the m_UsingRange props so that we can know what to set back to
		//__super::Initialize(em, props);  //must call predecessor first!
		m_UsingRange_props = m_Ship_1D_Props.UsingRange;
	}
	bool GetUsingRange_Props() const 
	{ 
		//This is basically like m_UsingRange from Ship_1D except that it is dynamic, and disabled when potentiometer is disabled (as we cannot detect limits)
		return m_UsingRange_props; 
	}
};
///This is the next layer of the linear Ship_1D that converts velocity into voltage, on a system that has sensor feedback
///It currently has a single PID (Dual PID may either be integrated or a new class)... to manage voltage error.  This is used for fixed point
/// position setting... like a turret or arm
class COMMON_API Rotary_Position_Control : public Rotary_System
{
public:
	enum PotUsage
	{
		eNoPot, //Will never read them (ideal for systems that do not have any encoders)
		ePassive,  //Will read them but never alter velocities
		eActive, //Will attempt to match predicted velocity to actual velocity
	};
	//Give client code access to the actual position, as the position of the entity cannot be altered for its projected position
	double GetActualPos() const;
private:
	using PIDController2 = Framework::Base::PIDController2;
	//Copy these lines to the subclass that binds the events
	//events are a bit picky on what to subscribe so we'll just wrap from here

	/// \param DisableFeedback this allows ability to bypass feedback
	Rotary_Control_Interface * const m_RobotControl;
	const size_t m_InstanceIndex;
	PIDController2 m_PIDControllerUp;
	PIDController2 m_PIDControllerDown;
	Rotary_Props m_Rotary_Props;

	double m_LastPosition;  //used for calibration
	double m_MatchVelocity;
	double m_ErrorOffset;
	double m_LastTime; //used for calibration
	double m_MaxSpeedReference; //used for calibration
	double m_PreviousVelocity; //used to compute acceleration
	//We use the negative sign bit to indicate it was turned off... or zero
	double m_BurstIntensity;  //This keeps track of the current level of burst to apply... it usually is full 1.0 or 0.0 but will blend on unaligned frame boundaries
	double m_CurrentBurstTime; //This keeps track of the time between bursts and the burst itself depending on the current state
	size_t m_PulseBurstCounter;  //keeps count of how many pulse bursts have happened
	size_t m_StallCounter;   //Keeps count for each cycle the motor stalls
	PotUsage m_PotentiometerState; //dynamically able to turn off (e.g. panic button)
	//A counter to count how many times the predicted position and intended position are withing tolerance consecutively
	size_t m_ToleranceCounter;
public:
	Rotary_Position_Control(const char EntityName[], Rotary_Control_Interface *robot_control, size_t InstanceIndex = 0);
	//IEvent::HandlerList ehl;
	//The parent needs to call initialize
	//virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL);
	virtual void ResetPosition(double Position);
	const Rotary_Props &GetRotary_Properties() const { return m_Rotary_Props; }
	//This is optionally used to lock to another ship (e.g. drive using rotary system)
	void SetMatchVelocity(double MatchVel) { m_MatchVelocity = MatchVel; }
protected:
	//Intercept the time change to obtain current height as well as sending out the desired velocity
	virtual void TimeChange(double dTime_s);
	virtual void SetPotentiometerSafety(bool DisableFeedback);
	PotUsage GetPotUsage() const { return m_PotentiometerState; }
	virtual double GetMatchVelocity() const { return m_MatchVelocity; }
	//Override these methods if the rotary system has some limit switches included in its setup
	virtual bool DidHitMinLimit() const { return false; }
	virtual bool DidHitMaxLimit() const { return false; }
};

///This is the next layer of the linear Ship_1D that converts velocity into voltage, on a system that has sensor feedback
///This models itself much like the drive train and encoders where it allows an optional encoder sensor read back to calibrate.
///This is a kind of speed control system that manages the velocity and does not need to keep track of position (like the drive or a shooter)
class COMMON_API Rotary_Velocity_Control : public Rotary_System
{
public:
	enum EncoderUsage
	{
		eNoEncoder, //Will never read them (ideal for systems that do not have any encoders)
		ePassive,  //Will read them but never alter velocities
		eActive, //Will attempt to match predicted velocity to actual velocity
	};
private:
	using PIDController2 = Framework::Base::PIDController2;
	using LatencyPredictionFilter = Framework::Base::LatencyPredictionFilter;
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
	double m_MatchVelocity;
	double m_CalibratedScaler; //used for calibration
	double m_ErrorOffset; //used for calibration

	double m_MaxSpeedReference; //used for calibration
	double m_EncoderVelocity;  //cache for later use
	double m_RequestedVelocity_Difference;
	EncoderUsage m_EncoderState; //dynamically able to change state
	double m_PreviousVelocity; //used to compute acceleration
public:
	Rotary_Velocity_Control(const char EntityName[], Rotary_Control_Interface *robot_control, size_t InstanceIndex = 0, EncoderUsage EncoderState = eNoEncoder);
	//IEvent::HandlerList ehl;
	//The parent needs to call initialize
	//virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props = NULL);
	virtual void ResetPos();
	double GetRequestedVelocity_Difference() const { return m_RequestedVelocity_Difference; }
	const Rotary_Props &GetRotary_Properties() const { return m_Rotary_Props; }
	//This is optionally used to lock to another ship (e.g. drive using rotary system)
	void SetMatchVelocity(double MatchVel) { m_MatchVelocity = MatchVel; }
	//Give ability to change properties
	void UpdateRotaryProps(const Rotary_Props &RotaryProps);
	virtual void SetEncoderSafety(bool DisableFeedback);
	EncoderUsage GetEncoderUsage() const { return m_EncoderState; }
	//Give client code ability to change rotary system to break or coast
	void SetAggresiveStop(bool UseAggresiveStop) { m_Rotary_Props.UseAggressiveStop = UseAggresiveStop; }
protected:
	//Intercept the time change to obtain current height as well as sending out the desired velocity
	virtual void TimeChange(double dTime_s);
	virtual void RequestedVelocityCallback(double VelocityToUse, double DeltaTime_s);

	virtual bool InjectDisplacement(double DeltaTime_s, double &PositionDisplacement);
	virtual double GetMatchVelocity() const { return m_MatchVelocity; }
};

#pragma endregion
#pragma endregion

#pragma region _RotaryPosition_Internal_
class RotaryPosition_Internal
{
private:
	double m_Position = 0.0;
	double m_maxspeed = 1.0;
	std::function<void(double new_voltage)> m_VoltageCallback=nullptr;
	std::function<double()> m_OdometryCallack = nullptr;
	double m_OpenLoop_position=0.0;  //keep track internally if we are open loop
	static double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel, double EntityMass)
	{
		//pulled from physics 1D, needed for computing the velocity (bypass only) slightly altered to be member free
		double ret;

		//This is how many radians the ship is capable to turn for this given time frame
		double Acceleration = (Restraint / EntityMass); //obtain acceleration

		{
			//first compute which direction to go
			double DistanceDirection = Distance;
			DistanceDirection -= matchVel * DeltaTime_s;
			if (IsZero(DistanceDirection))
			{
				ret = matchVel;
				return ret;
			}

			//Unlike in the 3D physics, we'll need while loops to ensure all of the accumulated turns are normalized, in the 3D physics the
			//Quat is auto normalized to only require one if check here
			while (DistanceDirection > M_PI)
				DistanceDirection -= Pi2;
			while (DistanceDirection < -M_PI)
				DistanceDirection += Pi2;
			double DistanceLength = fabs(DistanceDirection);

			//Ideal speed needs to also be normalized
			double IDS = Distance;
			if (IDS > M_PI)
				IDS -= Pi2;
			else if (IDS < -M_PI)
				IDS += Pi2;

			double IdealSpeed = fabs(IDS / DeltaTime_s);

			if (Restraint != -1)
			{
				//Given the distance compute the time needed
				//Place the division first keeps the multiply small
				double Time = sqrt(2.0*(DistanceLength / Acceleration));
				//With torque and its fixed point nature... it is important to have the jump ahead of the slope so that it doesn't overshoot
				//this can be accomplished by subtracting this delta time and working with that value... this should work very well but it could
				//be possible for a slight overshoot when the delta times slices are irregular. 
				if (Time > DeltaTime_s)
				{
					Time -= DeltaTime_s;
					if (IsZero(Time))
						Time = 0.0;
				}

				//Now compute maximum speed for this time
				double MaxSpeed = Acceleration * Time;
				ret = std::min(IdealSpeed, MaxSpeed);

				if (DistanceDirection < 0)
					ret = -ret;
				ret += matchVel;
			}
			else
			{
				ret = IdealSpeed;  //i.e. god speed
				if (IDS < 0)
					ret = -ret;
			}
		}
		return ret;
	}

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

		//this is our wheel module weight and really does not count weight of robot or friction
		const double EntityMass = 2.0 * 0.453592; //essentially our load of what we are applying the torque to
		const double stall_torque = 0.38; //using a RS-550 which is plenty of power
		const double torque_restraint = stall_torque * 100.0; //the total torque simple factors any gear reduction (counting the shaft to final gear reduction)
		const double velocity = GetVelocityFromDistance_Angular(distance, torque_restraint, d_time_s, 0.0, EntityMass);
		//almost there now to normalize the voltage we need the max speed, this is provided by the free speed of the motor, and since it is the speed of the
		//swivel itself we factor in the gear reduction.  This should go very quick because the load is light
		const double free_speed_RPM = 19000.0 / 30.0;  //factor in gear reduction
		const double max_speed_rad = free_speed_RPM * Pi2;
		//Note: At some point I should determine why the numbers do not quite align, they are empirically tweaked to have a good acceleration, and
		//should be fine for bypass demo
		double voltage = velocity / 8;
		//avoid oscillation by cutting out minor increments
		if (fabs(voltage) < 0.01)
			voltage = 0.0;
		m_VoltageCallback(voltage);
	}
public:
	void Init()
	{
		//TODO
	}
	void ShutDown()
	{}
	void SetPosition(double position)
	{
		m_Position = position;
	}
	void TimeSlice(double d_time_s)
	{
		//Use our setpoint input m_Position compute needed velocity and apply normalized voltage to voltage callback
		#ifdef __UseBypass__
		time_slice_bypass(d_time_s);
		#else
		#endif
	}
	void Reset(double position = 0.0)
	{}
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
class RotaryVelocity_Internal
{
private:
	double m_Velocity = 0.0;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	std::function<void(double new_voltage)> m_VoltageCallback;
public:
	void Init()
	{
		//TODO bind max speed to properties used in main assembly via external properties
		//we'll have other properties, and we'll need to work out the common ones
	}
	void ShutDown()
	{}
	void SetVelocity(double rate)
	{
		m_Velocity = rate;
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseBypass__
		//voltage in the bypass case is a normalized velocity
		m_VoltageCallback(m_Velocity/m_maxspeed);
		#else
		#endif
	}
	void Reset(double position = 0.0)
	{
		m_Velocity = 0.0;
	}
	void Set_UpdateCurrentVoltage(std::function<void(double new_voltage)> callback)
	{
		m_VoltageCallback = callback;
	}
	void SetOdometryCallback(std::function<double()> callback)
	{}
};
#pragma endregion
#pragma region _wrapper methods_
#pragma region _Rotary Position_
RotarySystem_Position::RotarySystem_Position()
{
	m_rotary_system = std::make_shared<RotaryPosition_Internal>();
}
void RotarySystem_Position::Init()
{
	m_rotary_system->Init();
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
void RotarySystem_Velocity::Init()
{
	m_rotary_system->Init();
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