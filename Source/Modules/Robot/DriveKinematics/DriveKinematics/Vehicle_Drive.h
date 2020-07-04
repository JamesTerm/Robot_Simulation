#pragma once

namespace Module {
	namespace Robot {

class  Tank_Drive
{
public:
	struct properties
	{
		double TurningDiameter;  //distance from diagonal for 4WD or one set of 4 wheels for 6WD
		double WheelBase;  //Length between wheels
		double TrackWidth; //Width between wheels
	};
	void SetProperties(const properties &props) { m_props = props; }
	// Places the ship back at its initial position and resets all vectors
	void ResetPos();
	/// \param FWD forward desired velocity (negative is reverse) recommended units meters per second
	/// \param RCW rotational velocity in radians per second (e.g. full rotation is 2 pi)
	void UpdateVelocities(double FWD, double RCW);

	double GetLeftVelocity() const { return m_LeftLinearVelocity; }
	double GetRightVelocity() const { return m_RightLinearVelocity; }
private:
	double m_LeftLinearVelocity = 0.0, m_RightLinearVelocity = 0.0;
	properties m_props;
};

struct SwerveVelocities
{
	enum SectionOrder
	{
		eFrontLeft,
		eFrontRight,
		eRearLeft,
		eRearRight
	};
	SwerveVelocities()
	{
		memset(this,0,sizeof(SwerveVelocities));
	}
	union uVelocity
	{
		struct Explicit
		{
			double sFL,sFR,sRL,sRR; //wheel tangential speeds in MPS
			double aFL,aFR,aRL,aRR; //wheel angles in radians clockwise from straight ahead
		} Named;
		double AsArray[8];
	} Velocity;
};

class Swerve_Drive
{
	public:
		struct properties
		{
			double TurningDiameter;  //distance from diagonal for 4WD or one set of 4 wheels for 6WD
			double WheelBase;  //Length between wheels
			double TrackWidth; //Width between wheels
		};
		void SetProperties(const properties &props) { m_props = props; }
		// Places the ship back at its initial position and resets all vectors
		void ResetPos();

		double GetIntendedVelocitiesFromIndex(size_t index) const { return m_Velocities.Velocity.AsArray[index]; }
		double GetSwerveVelocitiesFromIndex(size_t index) const { return m_Velocities.Velocity.AsArray[index+4]; }
		//Overload this for optimal time between the update and position to avoid oscillation
		void UpdateVelocities(double FWD, double STR, double RCW);

		const SwerveVelocities &GetIntendedVelocities() const { return m_Velocities; }
private:
		SwerveVelocities m_Velocities;
		properties m_props;
};

//TODO later
#if 0
class COMMON_API Butterfly_Drive : public Swerve_Drive
{
	public:
		Butterfly_Drive(Swerve_Drive_Interface *Parent);

		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);
		virtual void InterpolateVelocities(const SwerveVelocities &Velocities,Vec2D &LocalVelocity,double &AngularVelocity,double dTime_s);
	protected:
		virtual double GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const;
		Vec2D m_GlobalStrafeVelocity; //Cache the strafe velocity in its current direction
	    Vec2D m_LocalVelocity;  //keep track of the current velocity (for better interpreted displacement of strafing)
		Vec2D m_PreviousGlobalVelocity;
		void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
};

class COMMON_API Nona_Drive : public Butterfly_Drive
{
	private:
		#ifndef Robot_TesterCode
		typedef Butterfly_Drive __super;
		#endif
		double m_KickerWheel;
	public:
		Nona_Drive(Swerve_Drive_Interface *Parent);

		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);
		double GetKickerWheelIntendedVelocity() const {return m_KickerWheel;}
		//This is used as a cheat to avoid needing to resolve velocity using a callback technique... this needs to be set back to what it was when
		//altered
		void SetKickerWheelVelocity(double velocity) {m_KickerWheel=velocity;}
	protected:
		virtual double GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const;
		void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
};

#endif

}}
