#pragma once

namespace Module {
	namespace Robot {
		namespace Legacy {
#pragma region _Ship_1D Properties_
#pragma region _Entity1D_Properties_


struct Entity1D_Props
{
	//Stuff needed for physics
	double m_StartingPosition;  //the position used when reset position is called
	double m_Mass;
	//This can be used for the wheel diameter to work with RPS to linear conversions
	double m_Dimension; //Dimension- Length for linear and diameter for angular
	bool m_IsAngular;
};

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
	double DistanceDegradeScalar;
	bool UsingRange;
};



//TODO: may wish to aggregate the props to be consistent later
class COMMON_API Entity1D_Properties : public Entity1D_Props
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
}}}