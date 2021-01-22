#pragma once

#pragma region _includes macros_
#include "Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "Vec2D.h"
#include "Misc.h"

// Standard C++ support
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <bitset>
#include <map>
#include <algorithm>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>
#pragma endregion

namespace Framework
{
	namespace Base
	{

#pragma region _Physics 1D_

//The actual force between two objects are f=(G m1 m2)/ r^2
//For example g = ( G * (Me->5.98E+24)) / (Re->6.38E+6)^2 = 9.8 m/s^2
//G is ideal to compute forces on ships from various planets
const double G = 6.673E-11;

class COMMON_API PhysicsEntity_1D
{
private:
	#pragma region _members_
	double m_EntityMass;
	double m_StaticFriction, m_KineticFriction;

	double m_AngularInertiaCoefficient;
	double m_RadiusOfConcentratedMass; //This is used to compute the moment of inertia for torque (default 1,1,1)

	double m_Velocity;

	///This variable is factored in but is managed externally 
	double m_SummedExternalForces;
	double m_lastTime_s;
	#pragma endregion
	inline bool PosBNE(double val, double t)
	{
		return !(fabs(val - t) < 1E-3);
	}
public:
	PhysicsEntity_1D()
	{
		//Plug in some good defaults
		m_EntityMass = 5.0;
		m_StaticFriction = 0.8;
		m_KineticFriction = 0.2;
		m_AngularInertiaCoefficient = 1.0;
		m_RadiusOfConcentratedMass = 1.0;
		m_SummedExternalForces = 0.0;
		m_lastTime_s = 0.0;

		ResetVectors();
	}
	void SetMass(double mass)
	{
		///You must set the mass otherwise you will get the default
		m_EntityMass = mass;
	}
	double GetMass() const
	{
		return m_EntityMass;
	}
	virtual void ResetVectors()
	{
		///This will zero out all vectors
		m_Velocity = 0.0;
	}
	virtual void CopyFrom(const PhysicsEntity_1D& rhs)
	{
		// An overloaded operator for matching the current physics and position
		m_Velocity = rhs.m_Velocity;
	}
	virtual void TimeChangeUpdate(double DeltaTime_s, double &PositionDisplacement)
	{
		///This will compute all the displacement. Call this each time slice to obtain the rotation and position displacements.  
		///You must call this to run the physics engine!

		//Transfer the velocity to displacement
		PositionDisplacement = m_Velocity * DeltaTime_s;
	}
	inline double GetVelocityFromCollision(double ThisVelocityToUse, double otherEntityMass, double otherEntityVelocity)
	{
		//almost not quite
		//return (m_Velocity*(m_EntityMass-otherEntityMass)) / (m_EntityMass+otherEntityMass);

		/// en.wikipedia.org/wiki/Elastic_collision
		// Here is the equation
		// ((vel1 ( mass1 - mass2 )) + (2 * mass2 * vel2))) / (m1+m2)
		double ret = (ThisVelocityToUse *(m_EntityMass - otherEntityMass));
		ret += (otherEntityVelocity*(2 * otherEntityMass));
		ret /= (m_EntityMass + otherEntityMass);
		return ret;
	}
	virtual double GetVelocityFromDistance_Linear(double Distance, double ForceRestraintPositive, double ForceRestraintNegative,double DeltaTime_s, double matchVel)
	{
		///These distance methods simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		/// For 1D, client code needs to determine if the dimension is linear or angular which is identified in Entity 1D via boolean value

		///This can either work with local or global orientation that all depends on the orientation of the restraints typically this works in local
		double ret;
		double DistToUse = Distance; //save the original distance

		DistToUse -= (matchVel*DeltaTime_s);
		double Distance_Length = fabs(DistToUse);
		if (IsZero(Distance_Length))
			return matchVel;

		//This is how many meters per second the ship is capable to stop for this given time frame
		//Compute the restraint based off of its current direction
		double Restraint = DistToUse > 0 ? ForceRestraintPositive : ForceRestraintNegative;
		double Acceleration = (Restraint / m_EntityMass); //obtain acceleration

		double IdealSpeed = Distance_Length / DeltaTime_s;
		double AccelerationMagnitude = fabs(Acceleration);
		double Time = sqrt(2.0*(Distance_Length / AccelerationMagnitude));

		if (Time > DeltaTime_s)
		{
			Time -= DeltaTime_s;
			if (IsZero(Time))
				Time = 0.0;
		}

		double MaxSpeed = AccelerationMagnitude * Time;
		double SpeedToUse = std::min(IdealSpeed, MaxSpeed);

		//DebugOutput("d=%f i=%f m=%f\n",Distance[1],IdealSpeed,MaxSpeed);
		//Now to give this magnitude a direction based of the velocity
		double scale = SpeedToUse / Distance_Length;
		ret = DistToUse * scale;
		ret += matchVel;
		return ret;
	}
	virtual double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel)
	{
		double ret;

		//This is how many radians the ship is capable to turn for this given time frame
		double Acceleration = (Restraint / m_EntityMass); //obtain acceleration

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
	void SetFriction(double StaticFriction,double KineticFriction )
	{
		///These are coefficients to use when dealing with a force of friction typically 0.8 and 0.2 respectively
		/// \param StaticFriction The amount of friction to be applied when object is not moving
		/// \param KineticFriction The amount of friction to be applied when object is moving
		m_StaticFriction = StaticFriction;
		m_KineticFriction = KineticFriction;
	}
	void SetAngularInertiaCoefficient(double AngularInertiaCoefficient)
	{
		///This is the direct way to handle torque for various mass distributions
		///Here are some examples:
		/// - Disk rotating around its center				0.5
		/// - Hollow cylinder rotating around its center	1.0
		/// - Hollow sphere									0.66 (2/3)
		/// - Hoop rotating around its center				1.0
		/// - Point mass rotating at radius r				1.0
		/// - Solid cylinder								0.5
		/// - Solid sphere									0.4
		/// \todo Provide helper methods to compute this value
		m_AngularInertiaCoefficient = AngularInertiaCoefficient;
	}
	void SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass)
	{
		///This sets the radius for yaw axis, pitch axis, and roll axis.  Default = 1,1,1
		///This may represent the object's bounding box when used in conjunction with the angular inertia coefficient
		m_RadiusOfConcentratedMass = RadiusOfConcentratedMass;
	}
	double GetRadiusOfConcentratedMass() const
	{
		return m_RadiusOfConcentratedMass;
	}
	void SetVelocity(double Velocity)
	{
		m_Velocity = Velocity;
	}
	double GetVelocity() const
	{
		return m_Velocity;
	}
	void ApplyFractionalForce(double force, double FrameDuration)
	{
		///These work like the above except that the force applied only happens for a fraction of a second.  For accurate results FrameDuration
		///should be <= 1/framerate.  Ideally these should be used for high precision movements like moving a ship, where the FrameDuration is
		///typically the TimeDelta value

		//I'm writing this out so I can easily debug
		double AccelerationDelta = force / m_EntityMass;
		double VelocityDelta = AccelerationDelta * FrameDuration;
		m_Velocity += VelocityDelta;

		//if (AccelerationDelta[1]!=0)
		//	DebugOutput("Acc%f Vel%f\n",AccelerationDelta[1],m_Velocity[1]);
	}
	__inline double GetMomentofInertia(double RadialArmDistance)
	{
		//Doing it this way keeps the value of torque down to a reasonable level
		double RadiusRatio(m_RadiusOfConcentratedMass * m_RadiusOfConcentratedMass / RadialArmDistance);
		assert(RadiusRatio != 0);  //no-one should be using a zero sized radius!
		double ret = (m_AngularInertiaCoefficient * m_EntityMass * RadiusRatio);
		return ret;
	}

	inline double GetAngularAccelerationDelta(double torque, double RadialArmDistance = 1.0)
	{
		//This will give the acceleration delta given the torque which is: torque / AngularInertiaCoefficient * Mass

		// We want a cross product here, and divide by the mass and angular inertia
		//return (RadialArmDistance*torque) / (m_EntityMass*m_AngularInertiaCoefficient);

		//We are solving for angular acceleration so a=t / I

		// t=Ia 
		//I=sum(m*r^2) or sum(AngularCoef*m*r^2)

		double AngularAcceleration = 0;
		//Avoid division by zero... no radial arm distance no acceleration!
		if (RadialArmDistance != 0)
		{
			//Doing it this way keeps the value of torque down to a reasonable level
			//double RadiusRatio(m_RadiusOfConcentratedMass*m_RadiusOfConcentratedMass / RadialArmDistance);
			//assert(RadiusRatio != 0);  //no-one should be using a zero sized radius!
			//AngularAcceleration = (torque / (m_AngularInertiaCoefficient*m_EntityMass*RadiusRatio));

			AngularAcceleration = (torque / GetMomentofInertia(RadialArmDistance));
			//This is another way to view it
			//AngularAcceleration=((RadialArmDistance*torque)/(m_AngularInertiaCoefficient*m_EntityMass*m_RadiusOfConcentratedMass*m_RadiusOfConcentratedMass));
		}
		return AngularAcceleration;
	}
	void ApplyFractionalTorque(double torque, double FrameDuration, double RadialArmDistance = 1.0)
	{
		double AccelerationDelta = GetAngularAccelerationDelta(torque, RadialArmDistance);
		double VelocityDelta = AccelerationDelta * FrameDuration;
		m_Velocity += VelocityDelta;
	}
	double GetForceFromVelocity(double vDesiredVelocity,double DeltaTime_s)
	{
		///You may prefer to set a desired speed instead of computing the forces.  These values returned are intended to be used with 
		///ApplyFractionalForce, and ApplyFractionalTorque respectively.
		/// For the force, this works with the current linear velocity; therefore the desired velocity and return must work in global orientation
		/// \param vDesiredVelocity How fast do you want to go (in a specific direction)
		/// \param DeltaTime_s How quickly do you want to get there

		double DeltaVelocity = (vDesiredVelocity - GetVelocity());
		//A=Delta V / Delta T
		double Acceleration = DeltaVelocity / DeltaTime_s;  //This may be pretty quick (apply Force restrictions later)


		//if (Acceleration!=osg::Vec2d(0,0,0))
		//	printf(" x=%f,y=%f,z=%f\n",Acceleration[0],Acceleration[1],Acceleration[2]);
		//Now that we know what the acceleration needs to be figure out how much force to get it there
		double Force = Acceleration * m_EntityMass;
		//if (PosBNE(Force[0],0)||(PosBNE(Force[1],0))||(PosBNE(Force[2],0)))
		//	printf("tx=%f,ty=%f,tz=%f\n",Force[0],Force[1],Force[2]);

		//if (PosBNE(Heading[2],0.0))
		//	DebugOutput(" s=%f,a=%f,w=%f,h=%f,z=%f,t=%f\n",Distance[2],m_AngularAcceleration[2],m_AngularVelocity[2],Heading[2],CurrentOrientation[2],Force[2]);

		return Force;
	}
	double ComputeRestrainedForce(double LocalForce, double ForceRestraintPositive, double ForceRestraintNegative, double dTime_s)
	{
		///This is a clean way to compute how much torque that can be applied given the maximum amount available (e.g. thruster capacity)
		///It should be noted that this treats roll as a separate factor, which is best suited for avionic type of context
		/// \Note all restraint parameters are positive (i.e. ForceRestraintNegative)
		//Note: This could be simplified; however, (at least for now) I am keeping it intact as it is in 2D to see how the function behaves
		double ForceToApply = LocalForce;
		if (ForceRestraintPositive != -1)
		{
			double SmallestRatio = 1.0;
			//Apply Force restraints; This method computes the smallest ratio needed to scale down the vector.  It should give the maximum amount
			//of magnitude available without sacrificing the intended direction
			{
				double Temp;
				//separate the positive and negative coordinates
				if (LocalForce > 0)
				{
					if (LocalForce > ForceRestraintPositive)
					{
						Temp = ForceRestraintPositive / LocalForce;
						SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
					}
				}
				else
				{
					double AbsComponent = fabs(LocalForce);
					if (AbsComponent > ForceRestraintNegative)
					{
						Temp = ForceRestraintNegative / AbsComponent;
						SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
					}
				}
			}
			ForceToApply *= SmallestRatio;
			//Test my restraints
			//printf("\r lr %f fr %f ud %f                ",LocalForce[0],LocalForce[1],LocalForce[2]);
		}
		return ForceToApply;
	}
	__inline double GetForceNormal(double gravity = 9.80665) const
	{
		//for now there is little tolerance to become kinetic, but we may want more
		double fc = IsZero(m_Velocity) ? m_StaticFriction : m_KineticFriction;
		return (fc * m_EntityMass * gravity);
	}
	double GetFrictionalForce(double DeltaTime_s, double Ground = 0.0, double gravity = 9.80665, double BrakeResistence = 0.0) const
	{
		/// \param Brake is a brake coast parameter where if gravity pulls it down it can apply a scaler to slow down the reversed rate
		/// where 0 is full stop and 1 is full coast (range is 0 - 1)

		if (!DeltaTime_s) return 0.0;  //since we divide by time avoid division by zero
		double NormalForce = GetForceNormal(gravity) * cos(Ground);
		const double StoppingForce = (fabs(m_Velocity) * m_EntityMass) / DeltaTime_s;
		NormalForce = std::min(StoppingForce, NormalForce); //friction can never be greater than the stopping force
		double GravityForce = (m_EntityMass * gravity * sin(Ground));
		double FrictionForce = NormalForce;
		//If the friction force overflows beyond stopping force, apply a scale to the overflow of force 
		if (FrictionForce > StoppingForce) FrictionForce = (BrakeResistence * (FrictionForce - StoppingForce));

		//Give the sense of fractional force in the *opposite* (i.e. greater = negative) direction of the current velocity
		if (m_Velocity > 0.0)
			FrictionForce = -FrictionForce;
		//Add the Gravity force... it already has its direction
		FrictionForce += GravityForce;
		return FrictionForce;

	}
};
#pragma endregion

#pragma region _Physics 2D_

class PhysicsEntity_2D
{
protected:
	#pragma region _members_
	double m_EntityMass;
	double m_StaticFriction, m_KineticFriction;

	double m_AngularInertiaCoefficient;
	double m_RadiusOfConcentratedMass; //This is used to compute the moment of inertia for torque (default 1,1,1)

	Vec2D m_LinearVelocity;		///< This must represent global orientation for external forces to work properly
	double m_AngularVelocity;		///< All angle variables are in radians!

	///This variable is factored in but is managed externally 
	Vec2D m_SummedExternalForces;
	double m_lastTime_s;
	#pragma endregion
public:
	PhysicsEntity_2D()
	{
		//Plug in some good defaults
		m_EntityMass = 500; //about 5000 pounds
		//m_EntityMass=200; //about 2000 pounds
		m_StaticFriction = 0.8;
		m_KineticFriction = 0.2;
		m_AngularInertiaCoefficient = 1.0;
		m_RadiusOfConcentratedMass = 1.0;
		m_SummedExternalForces = Vec2D(0, 0);
		m_lastTime_s = 0.0;

		ResetVectors();
	}
	virtual ~PhysicsEntity_2D() {}
		
	void SetMass(double mass)
	{
		///You must set the mass otherwise you will get the default
		m_EntityMass = mass;
	}
	double GetMass() const
	{
		return m_EntityMass;
	}
	virtual void ResetVectors()
	{
		///This will zero out all vectors
		m_LinearVelocity = Vec2D(0, 0);
		m_AngularVelocity = 0.0;
	}
	virtual void CopyFrom(const PhysicsEntity_2D& rhs)
	{
		// An overloaded operator for matching the current physics and position
		m_LinearVelocity = rhs.m_LinearVelocity;
		m_AngularVelocity = rhs.m_AngularVelocity;
	}
	virtual void TimeChangeUpdate(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement)
	{
		///This will compute all the displacement. Call this each time slice to obtain the rotation and position displacements.  
		///You must call this to run the physics engine!

		//Transfer the velocity to displacement
		RotationDisplacement = m_AngularVelocity * DeltaTime_s;
		PositionDisplacement = m_LinearVelocity * DeltaTime_s;
	}
	double GetVelocityFromDistance_Angular(double Distance, double Restraint, double DeltaTime_s, double matchVel, bool normalize = true)
	{
		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This is ideal to be used with GetTorqueFromVelocity
		/// \param normalize will normalize the distance between -pi - pi

		double ret;

		//This is how many radians the ship is capable to turn for this given time frame
		double Acceleration = (Restraint / m_EntityMass); //obtain acceleration

		{
			//first compute which direction to go
			double DistanceDirection = Distance;
			DistanceDirection -= matchVel * DeltaTime_s;
			if (IsZero(DistanceDirection))
			{
				ret = matchVel;
				return ret;
			}

			if (normalize)
			{
				//Unlike in the 3D physics, we'll need while loops to ensure all of the accumulated turns are normalized, in the 3D physics the
				//Quat is auto normalized to only require one if check here
				while (DistanceDirection > M_PI)
					DistanceDirection -= Pi2;
				while (DistanceDirection < -M_PI)
					DistanceDirection += Pi2;
			}

			double DistanceLength = fabs(DistanceDirection);

			//Ideal speed needs to also be normalized
			double IDS = Distance;
			if (normalize)
			{
				if (IDS > M_PI)
					IDS -= Pi2;
				else if (IDS < -M_PI)
					IDS += Pi2;
			}

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
		#if 0
		if (fabs(m_AngularVelocity) > 0.0)
			printf("y=%.2f p=%.2f e=%.2f cs=0\n", Distance, ret, m_AngularVelocity);
		#endif

		return ret;

	}
	inline Vec2D GetVelocityFromCollision(Vec2D ThisVelocityToUse, double otherEntityMass, Vec2D otherEntityVelocity)
	{
		//almost not quite
		//return (m_LinearVelocity*(m_EntityMass-otherEntityMass)) / (m_EntityMass+otherEntityMass);

		/// en.wikipedia.org/wiki/Elastic_collision
		// Here is the equation
		// ((vel1 ( mass1 - mass2 )) + (2 * mass2 * vel2))) / (m1+m2)
		Vec2D ret = (ThisVelocityToUse *(m_EntityMass - otherEntityMass));
		ret += (otherEntityVelocity*(2 * otherEntityMass));
		ret /= (m_EntityMass + otherEntityMass);
		return ret;
	}
	virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel)
	{
		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This can either work with local or global orientation that all depends on the orientation of the restraints typically this works in local

		//just like GetVelocityFromDistance_Angular except we do not normalize the DistanceDirection
		Vec2D ret;
		//These are initialized as we go
		double Acceleration;
		double Restraint;

		for (size_t i = 0; i < 2; i++)
		{
			double DistanceDirection = Distance[i];
			DistanceDirection -= matchVel[i] * DeltaTime_s;
			if (IsZero(DistanceDirection))
			{
				ret[i] = matchVel[i];
				continue;
			}
			double DistanceLength = fabs(DistanceDirection);

			//Compose our restraint and acceleration based on the component direction
			Restraint = (DistanceDirection > 0) ? ForceRestraintPositive[i] : ForceRestraintNegative[i];
			Acceleration = (Restraint / m_EntityMass);

			double IdealSpeed = fabs(Distance[i] / DeltaTime_s);

			if (Restraint != -1)
			{
				//Given the distance compute the time needed
				//Place the division first keeps the multiply small
				double Time = sqrt(2.0*(DistanceLength / Acceleration));

				if (Time > DeltaTime_s)
				{
					Time -= DeltaTime_s;
					if (IsZero(Time))
						Time = 0.0;
				}

				//Now compute maximum speed for this time
				double MaxSpeed = Acceleration * Time;
				ret[i] = std::min(IdealSpeed, MaxSpeed);

				if (DistanceDirection < 0.0)
					ret[i] = -ret[i];
				ret[i] += matchVel[i];
			}
			else
			{
				ret[i] = IdealSpeed;  //i.e. god speed
				if (Distance[i] < 0.0)
					ret[i] = -ret[i];
			}
		}
		return ret;
	}
	virtual Vec2D GetVelocityFromDistance_Linear_v1(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel)
	{
		Vec2D ret;
		Vec2D DistToUse = Distance; //save the original distance as it is const anyhow

		DistToUse -= (matchVel*DeltaTime_s);
		double dDistance = DistToUse.length();
		if (IsZero(dDistance))
			return matchVel;

		//This is how many meters per second the ship is capable to stop for this given time frame
		Vec2D Restraint;
		//Compute the restraint based off of its current direction
		for (size_t i = 0; i < 2; i++)
			Restraint[i] = DistToUse[i] > 0 ? ForceRestraintPositive[i] : ForceRestraintNegative[i];

		Vec2D Acceleration = (Restraint / m_EntityMass); //obtain acceleration

		double IdealSpeed = Distance.length() / DeltaTime_s;
		double AccelerationMagnitude = Acceleration.length();
		double Time = sqrt(2.0*(dDistance / AccelerationMagnitude));

		double MaxSpeed = AccelerationMagnitude * Time;
		double SpeedToUse = std::min(IdealSpeed, MaxSpeed);

		//DebugOutput("d=%f i=%f m=%f\n",Distance[1],IdealSpeed,MaxSpeed);
		//Now to give this magnitude a direction based of the velocity
		double scale = SpeedToUse / dDistance;
		ret = DistToUse * scale;
		ret += matchVel;
		return ret;
	}
	void SetFriction(double StaticFriction,	double KineticFriction )
	{
		// These are coefficients to use when dealing with a force of friction typically 0.8 and 0.2 respectively
		// StaticFriction- The amount of friction to be applied when object is not moving
		// KineticFriction- The amount of friction to be applied when object is moving
		m_StaticFriction = StaticFriction;
		m_KineticFriction = KineticFriction;
	}
	void SetAngularInertiaCoefficient(double AngularInertiaCoefficient)
	{
		///This is the direct way to handle torque for various mass distributions
		///Here are some examples:
		/// - Disk rotating around its center				0.5
		/// - Hollow cylinder rotating around its center	1.0
		/// - Hollow sphere									0.66 (2/3)
		/// - Hoop rotating around its center				1.0
		/// - Point mass rotating at radius r				1.0
		/// - Solid cylinder								0.5
		/// - Solid sphere									0.4
		/// \todo Provide helper methods to compute this value
		m_AngularInertiaCoefficient = AngularInertiaCoefficient;
	}
	void SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass)
	{
		///This sets the radius for yaw axis, pitch axis, and roll axis.  Default = 1,1,1
		m_RadiusOfConcentratedMass = RadiusOfConcentratedMass;
	}
	double GetRadiusOfConcentratedMass() const
	{
		return m_RadiusOfConcentratedMass;
	}
	void SetLinearVelocity(const Vec2D &LinearVelocity)
	{
		m_LinearVelocity = LinearVelocity;
	}
	Vec2D GetLinearVelocity() const
	{
		return m_LinearVelocity;
	}
	void SetAngularVelocity(double AngularVelocity)
	{
		m_AngularVelocity = AngularVelocity;
	}
	double GetAngularVelocity() const
	{
		return m_AngularVelocity;
	}
	inline double GetAngularAccelerationDelta(double torque, double RadialArmDistance = 1.0)
	{
		//This will give the acceleration delta given the torque which is: torque / AngularInertiaCoefficient * Mass

		/* We want a cross product here, and divide by the mass and angular inertia
		return (RadialArmDistance^torque) / (m_EntityMass*m_AngularInertiaCoefficient);

		// [Rick Notes], Why divide by the arm distance?  Shouldn't we be multiplying?  Why square that, and along just the component?
		// We divide by I to solve for a... see formula below
		*/

		// t=Ia 
		//I=sum(m*r^2) or sum(AngularCoef*m*r^2)

		double ret;
		{
			//Avoid division by zero... no radial arm distance no acceleration!
			if (RadialArmDistance == 0)
			{
				ret = 0;
				return ret;
			}
			//Doing it this way keeps the value of torque down to a reasonable level
			// [Rick Notes]  What does a "Reasonable Level" mean?  Perhaps we should see the equation somewhere
			// I forgot what the equation was and I get a bit lost.
			double RadiusRatio(m_RadiusOfConcentratedMass*m_RadiusOfConcentratedMass / RadialArmDistance);
			assert(RadiusRatio != 0);  //no-one should be using a zero sized radius!
			ret = (torque / (m_AngularInertiaCoefficient*m_EntityMass*RadiusRatio));
		}
		return ret;

	}
	void ApplyFractionalForce(const Vec2D &force, double FrameDuration)
	{
		///These apply for a fraction of a second.  For accurate results FrameDuration
		///should be <= 1/framerate.  Ideally these should be used for high precision movements like moving a ship, where the FrameDuration is
		///typically the TimeDelta value

		//I'm writing this out so I can easily debug
		Vec2D AccelerationDelta = force / m_EntityMass;
		Vec2D VelocityDelta = AccelerationDelta * FrameDuration;
		m_LinearVelocity += VelocityDelta;

		//if (AccelerationDelta[1]!=0)
		//	DebugOutput("Acc%f Vel%f\n",AccelerationDelta[1],m_LinearVelocity[1]);
	}
	void ApplyFractionalTorque(double torque, double FrameDuration, double RadialArmDistance = 1.0)
	{
		double AccelerationDelta = GetAngularAccelerationDelta(torque, RadialArmDistance);
		double VelocityDelta = AccelerationDelta * FrameDuration;
		m_AngularVelocity += VelocityDelta;
	}
	void ApplyFractionalForce(const Vec2D &force, const Vec2D &point, double FrameDuration)
	{
		///This one is ideal to use for collision detection.  It will basically evaluate the point and determine the amount of force and torque to
		///apply.  It will implicitly call ApplyFractionalForce and ApplyFractionalTorque.
			//Use this as a "get by" if the code doesn't work properly
		#if 0
		ApplyFractionalForce(force, FrameDuration);
		return;
		#endif
		//Here is a rough draft to solve in 2 dimensions
		//A=atan2(py,px)   point
		//M=pi/2 - A
		//L=atan2(fy,fx)  force
		//N=L+M
		//Y=sin(N)*f.length = contribution for force
		//X=cos(N)*f.length = contribution for torque

		double TorqueToApply;
		Vec2D ForceToApply;
		double RadialArmDistance;

		{
			double A = atan2(point[1], point[0]);
			double M = (M_PI / 2) - A;
			double L = atan2(-force[1], -force[0]);
			double N = L + M;

			double ForceLength = sqrt((force[1] * force[1]) + (force[0] * force[0]));
			RadialArmDistance = sqrt((point[1] * point[1]) + (point[0] * point[0]));
			//I've reserved a special case for ships which haven't specified  their radius size, in which case we simply factor out the radial arm too
			if ((m_RadiusOfConcentratedMass == 1.0) && (RadialArmDistance > 1.0)) RadialArmDistance = 1.0;

			//Fr = t   ... We should multiply force by the radial arm distance to get the torque
			//but instead,  we pass it off to physics where the multiply gets applied directly against the Radius of Concentrated Mass
			//We could multiply here but doing it the other way keeps the torque value low, which also makes it easier to debug
			TorqueToApply = (cos(N)*ForceLength);
		}

		Vec2D vecToCenter = -point;
		//Note we should be able to support a point set at 0,0,0 in which case we use the force itself as the direction... otherwise a zero'd point
		//results in a zero'd vector which would omit applying the force
		if (vecToCenter.length2() == 0.0)
			vecToCenter = -force;

		vecToCenter.normalize();

		ForceToApply = vecToCenter * (force * vecToCenter);

		ApplyFractionalForce(ForceToApply, FrameDuration);
		ApplyFractionalTorque(TorqueToApply, FrameDuration, RadialArmDistance);

	}
	virtual Vec2D GetForceFromVelocity(	const Vec2D &vDesiredVelocity,	double DeltaTime_s	) const
	{
		//vDesiredVelocity - How fast do you want to go (in a specific direction)
		//DeltaTime_s  How quickly do you want to get there
		///You may prefer to set a desired speed instead of computing the forces.  These values returned are intended to be used with 
		///ApplyFractionalForce, and ApplyFractionalTorque respectively.
		/// For the force, this works with the current linear velocity; therefore the desired velocity and return must work in global orientation

		Vec2D DeltaVelocity = (vDesiredVelocity - GetLinearVelocity());
		//A=Delta V / Delta T
		Vec2D Acceleration = DeltaVelocity / DeltaTime_s;  //This may be pretty quick (apply Force restrictions later)


		//if (Acceleration!=Vec2D(0,0,0))
		//	printf(" x=%f,y=%f,z=%f\n",Acceleration[0],Acceleration[1],Acceleration[2]);
		//Now that we know what the acceleration needs to be figure out how much force to get it there
		Vec2D Force = Acceleration * m_EntityMass;
		//if (PosBNE(Force[0],0)||(PosBNE(Force[1],0))||(PosBNE(Force[2],0)))
		//	printf("tx=%f,ty=%f,tz=%f\n",Force[0],Force[1],Force[2]);

		//if (PosBNE(Heading[2],0.0))
		//	DebugOutput(" s=%f,a=%f,w=%f,h=%f,z=%f,t=%f\n",Distance[2],m_AngularAcceleration[2],m_AngularVelocity[2],Heading[2],CurrentOrientation[2],Force[2]);

		return Force;
	}
	virtual double GetTorqueFromVelocity(double vDesiredVelocity,	double DeltaTime_s	) const
	{
		// vDesiredVelocity - How fast do you want to go (in a specific direction)
		//DeltaTime_s - How quickly do you want to get there (usually in time slices)

		//TODO input torque restraints from script (this would be due to the capabilities of the engines)
		//Note: desired speed is a separated variable controlled from the ship's speed script, which we fine tune given the torque restraints
		//And also by minimizing the amount of G's pulled at the outer most edge of the ship... so for large ships that rotate this could be
		//significant, and you wouldn't want people slamming into the walls.
		//Note: if the speed is too high and the torque restraint is too low the ship will "wobble" because it trying to to go a faster speed that it
		//can "brake" for... ideally a little wobble is reasonable and this is controlled by a good balance between desired speed and torque restraints

		double DeltaVelocity = (vDesiredVelocity - GetAngularVelocity());
		//A=Delta V / Delta T
		double Acceleration = DeltaVelocity / DeltaTime_s;  //This may be pretty quick (apply torque restrictions later)

		//if (Acceleration!=Vec2D(0,0,0))
		//	printf(" x=%f,y=%f,z=%f\n",Acceleration[0],Acceleration[1],Acceleration[2]);
		//Now that we know what the acceleration needs to be figure out how much force to get it there
		double Torque = Acceleration * m_EntityMass;
		//if (PosBNE(Torque[0],0)||(PosBNE(Torque[1],0))||(PosBNE(Torque[2],0)))
		//	printf("tx=%f,ty=%f,tz=%f\n",Torque[0],Torque[1],Torque[2]);

		//if (PosBNE(Heading[2],0.0))
		//	DebugOutput(" s=%f,a=%f,w=%f,h=%f,z=%f,t=%f\n",Distance[2],m_AngularAcceleration[2],m_AngularVelocity[2],Heading[2],CurrentOrientation[2],Torque[2]);

		return Torque;
	}
	double ComputeRestrainedTorque(double Torque, double TorqueRestraint, double dTime_s)
	{
		///This is a clean way to compute how much torque that can be applied given the maximum amount available (e.g. thruster capacity)
		///It should be noted that this treats roll as a separate factor, which is best suited for avionic type of context
		/// \Note all restraint parameters are positive (i.e. ForceRestraintNegative)
		double TorqueToApply = Torque;

		if (TorqueRestraint != -1)
		{
			double SmallestRatio = 1.0;
			{
				double AbsComponent = fabs(TorqueToApply);
				if ((AbsComponent > TorqueRestraint) && (TorqueRestraint > 0.0))
				{
					double Temp = TorqueRestraint / AbsComponent;
					SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
				}
			}
			TorqueToApply *= SmallestRatio;
		}

		return TorqueToApply;
	}
	Vec2D ComputeRestrainedForce(const Vec2D &LocalForce, const Vec2D &ForceRestraintPositive, const Vec2D &ForceRestraintNegative, double dTime_s)
	{
		Vec2D ForceToApply = LocalForce;
		if (ForceRestraintPositive[0] != -1)
		{
			double SmallestRatio = 1.0;
			//Apply Force restraints; This method computes the smallest ratio needed to scale down the vector.  It should give the maximum amount
			//of magnitude available without sacrificing the intended direction
			for (size_t i = 0; i < 2; i++)
			{
				double Temp;
				//separate the positive and negative coordinates
				if (LocalForce[i] > 0)
				{
					if (LocalForce[i] > ForceRestraintPositive[i])
					{
						Temp = ForceRestraintPositive[i] / LocalForce[i];
						SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
					}
				}
				else
				{
					double AbsComponent = fabs(LocalForce[i]);
					if (AbsComponent > ForceRestraintNegative[i])
					{
						Temp = ForceRestraintNegative[i] / AbsComponent;
						SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
					}
				}
			}
			ForceToApply *= SmallestRatio;
			//Test my restraints
			//printf("\r lr %f fr %f ud %f                ",LocalForce[0],LocalForce[1],LocalForce[2]);
		}
		return ForceToApply;
	}
	static double GetCentripetalAcceleration(double LinearVelocity, double AngularVelocity, double DeltaTime_s)
	{
		//This returns in the form of magnitude using the proper equations

		//centripetal_a = v^2 / r
		//first we'll need to find r given the current angular velocity
		//r = s / theta  (where theta is rotational displacement or angular velocity * time)
		const double theta = LinearVelocity > 0.0 ? -AngularVelocity : AngularVelocity;
		if (IsZero(theta)) return 0.0;
		const double v = LinearVelocity;
		const double s = v / DeltaTime_s;
		const double r = s / theta;
		if (IsZero(r)) return 0.0;
		const double centripetal_acceleration = v * v / r;
		return LinearVelocity > 0.0 ? centripetal_acceleration : -centripetal_acceleration;
	}
	double GetCentripetalAcceleration(double DeltaTime_s) const
	{
		//Note the length of linear velocity works because it does not matter which direction the velocity is facing
		return GetCentripetalAcceleration(m_LinearVelocity.length(), m_AngularVelocity, DeltaTime_s);
	}
	Vec2D GetCentripetalAcceleration_2D(double DeltaTime_s) const
	{
		///This returns the global acceleration needed to maintain linear velocity
		return GetCentripetalForce(DeltaTime_s) / m_EntityMass;
	}
	Vec2D GetCentripetalForce(double DeltaTime_s) const
	{
		///This returns the global force needed to maintain the current linear velocity

		//F_centripetal = m v^2 / r
		//return GetCentripetalAcceleration(DeltaTime_s) * m_EntityMass;

		//This way is more efficient
		Vec2D AlteredVelocity = GlobalToLocal(m_AngularVelocity*DeltaTime_s, m_LinearVelocity);
		return GetForceFromVelocity(AlteredVelocity, DeltaTime_s) * DeltaTime_s;
	}
};

///This class is a expands on some common tasks that deal more specifically with flight.  This attempts to pull common tasks needed from physics in a way
///Where it is easy to use for ships and other objects that deal with orientation and position
class COMMON_API FlightDynamics_2D : public PhysicsEntity_2D
{
public:
	struct LinearAccelerationRates
	{
		Vec2D AccDeltaPos;        //when increasing from a positive position
		Vec2D AccDeltaNeg;        //when -increasing from a negative position
		Vec2D DecDeltaPos;        //when decreasing from a positive position
		Vec2D DecDeltaNeg;        //when -decreasing from a negative position
	};
private:
	#pragma region _members_
	double StructuralDmgGLimit;
	double G_Dampener;
	double m_DefaultHeading;
	// I'll try to keep this read only, so that client who own their own Heading can use this code, without worrying about the Heading being changed
	std::function <double()> m_HeadingToUse=nullptr;
	const double* m_HeadingToUse_legacy;
	//This keeps track of the current rate of acceleration.  These are in local orientation.
	Vec2D m_CurrentAcceleration, m_TargetAcceleration;
	LinearAccelerationRates m_LinearAccelerationRates;
	//Some objects may not need to use this (by default they will not)
	bool m_UsingAccelerationRate;
	bool m_UseDefaultHeading = true;
	#pragma endregion
public:
	void init()
	{
		//provide common area to initialize members
		//populate LinearAccelerationRates with some good defaults
		const Vec2D AccDelta(30.0, 30.0);
		const Vec2D DecDelta(60.0, 60.0);
		LinearAccelerationRates &_ = m_LinearAccelerationRates;
		_.AccDeltaNeg = _.AccDeltaPos = AccDelta;
		_.DecDeltaNeg = _.DecDeltaPos = DecDelta;
		m_UsingAccelerationRate = false;
		StructuralDmgGLimit = 10.0;
		G_Dampener = 1.0;
	}
	FlightDynamics_2D()
	{
		init();
		m_HeadingToUse = [&]()
		{
			return m_DefaultHeading;
		};
	}
	FlightDynamics_2D(const double *HeadingToUse) : m_HeadingToUse_legacy(HeadingToUse)
	{
		init();
		m_HeadingToUse = [&]()
		{
			return *m_HeadingToUse_legacy;
		};
		m_UseDefaultHeading = false;
	}
	FlightDynamics_2D(const double &HeadingToUse) : m_HeadingToUse_legacy(&HeadingToUse)
	{
		init();
		m_HeadingToUse = [&]()
		{
			return *m_HeadingToUse_legacy;
		};
		m_UseDefaultHeading = false;
	}
	virtual ~FlightDynamics_2D() {}
	//Allow late binding of heading to use (client code should set this once as soon as possible)
	void SetHeadingToUse(std::function <double()> callback) 
	{
		m_HeadingToUse= callback;
		m_UseDefaultHeading = false;
	}
	virtual void ResetVectors()
	{
		PhysicsEntity_2D::ResetVectors();
		if (m_UseDefaultHeading)
			m_DefaultHeading = 0.0;
		m_CurrentAcceleration = m_TargetAcceleration = Vec2D(0.0, 0.0);
	}
	Vec2D ComputeAngularDistance_asLookDir(const Vec2D &lookDir)
	{
		///This will measure the distance between this quat and the look dir quat.  With the current algorithm the most desired results occur when the
		///delta's are less than 90 degrees for yaw and pitch.  Roll is computed separately.
		/// \param lookDir This is another orientation that you are comparing against
		/// \param UpDir is usually the same quat's orientation * 0,0,1

		double distance = ComputeAngularDistance(lookDir);
		return Vec2D(sin(distance), cos(distance));
	}
	double ComputeAngularDistance(const Vec2D &lookDir)
	{
		double lookDir_radians = atan2(lookDir[0], lookDir[1]);
		double distance = m_HeadingToUse() - lookDir_radians;
		return shortest_angle(distance);
	}
	double ComputeAngularDistance(double Orientation)
	{
		/// \param Orientation this will break down the quat into its lookDir and UpDir for you
		double DistanceDirection = m_HeadingToUse() - Orientation;
		return shortest_angle(DistanceDirection);
	}
	double GetHeading() 
	{
		return m_HeadingToUse();
	}
	virtual void TimeChangeUpdate(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement)
	{
		PhysicsEntity_2D::TimeChangeUpdate(DeltaTime_s, PositionDisplacement, RotationDisplacement);
		if (m_UseDefaultHeading)
			m_DefaultHeading += RotationDisplacement;
	}

	//Acceleration rate methods:
	void SetUsingAccelerationRate(bool UseIt) 
	{
		///If these methods are being used, this must be set to true
		m_UsingAccelerationRate=UseIt;
	} 
	LinearAccelerationRates &GetLinearAccelerationRates()
	{
		///Get and set the linear acceleration rates here.
		///For now this is the parameters for the simple model, which may or may not be applied for other engine models
		///Typically this should be a one time setup, but can be dynamic (e.g. afterburner)
		return m_LinearAccelerationRates;
	}
	void SetTargetAcceleration(const Vec2D &TargetAcceleration)
	{
		//Set to the desired acceleration level.  This should be called first per time interval, so the other methods can work properly
		//This must use local orientation
		m_TargetAcceleration = TargetAcceleration;
	}
	Vec2D GetAcceleration_Delta(double dTime_s)
	{
		///This can be defined to just about anything, but to keep things simple it is a linear constant depending on direction
		///We could later have different models and have client choose which to use
		return GetAcceleration_Delta(dTime_s, m_TargetAcceleration);
	}
	Vec2D GetAcceleration_Delta(double dTime_s, const Vec2D &TargetAcceleration, bool Clipping = true)
	{
		///With this version you may use to predict the capabilities of max acceleration or deceleration
		/// \param Clipping this chooses whether to extract the full power when the acceleration approaches target acceleration, or
		/// whether to compute the actual force necessary to hit target.  Typically this will be on when manipulating the force
		/// The no clip is useful for things which need know full capability to get to threshold such as in GetForceFromVelocity

		//implicitly initialized in case we need to do no work
		Vec2D ret(0.0, 0.0);
		//Note: this is somewhat simplified when we cross-over the zero thresh-hold, where instead of blending one derivative against the other
		//it will follow through with first tagged one.  Since the deltas are encapsulated it should work, because it will still
		//accurately predict the force that will be applied.  The amount of error for this case should be minimal enough to not be noticeable.
		//We can provide different engine models in the future
		//Compute the acceleration given the derivatives
		LinearAccelerationRates &_ = m_LinearAccelerationRates;
		for (size_t i = 0; i < 2; i++)
		{
			if ((TargetAcceleration[i] == m_CurrentAcceleration[i]) && (Clipping))
				continue;
			//determine if we are going in a positive or negative direction
			if (TargetAcceleration[i] > m_CurrentAcceleration[i]) //We are going in a positive direction
			{
				double Delta = m_CurrentAcceleration[i] > 0 ? _.AccDeltaPos[i] : _.DecDeltaNeg[i];
				Delta *= dTime_s;
				if (Clipping)
					ret[i] = std::min(Delta, TargetAcceleration[i] - m_CurrentAcceleration[i]);
				else
					ret[i] = Delta;
			}
			else //We are going in a negative direction
			{
				double Delta = m_CurrentAcceleration[i] > 0 ? _.DecDeltaPos[i] : _.AccDeltaNeg[i];
				Delta *= dTime_s;
				if (Clipping)
					ret[i] = std::max(-Delta, TargetAcceleration[i] - m_CurrentAcceleration[i]);
				else
					ret[i] = Delta;
			}
		}
		return ret;
	}
	void Acceleration_TimeChangeUpdate(double dTime_s)
	{
		///This applies the current acceleration delta (i.e. calls GetAcceleration_Delta) to acceleration.
		/// \note Special care has been taken to ensure that any reads to GetAcceleration_Delta will be the same value applied here.
		///This must be consistent per time slice so that other clients can properly predict the acceleration.

		const Vec2D Acceleration_Delta = GetAcceleration_Delta(dTime_s);
		m_CurrentAcceleration += Acceleration_Delta;
	}
	const Vec2D &GetCurrentAcceleration() 
	{
		///Read-only access of the current acceleration
		return m_CurrentAcceleration;
	}
	virtual Vec2D GetForceFromVelocity(const Vec2D &vDesiredVelocity, double DeltaTime_s)
	{
		/// These are overloaded to optionally factor in the acceleration period
		Vec2D Force;
		if (!m_UsingAccelerationRate)
			Force = PhysicsEntity_2D::GetForceFromVelocity(vDesiredVelocity, DeltaTime_s);
		else
		{
			Vec2D Zerod = Vec2D(0.0, 0.0);
			Vec2D Acceleration = Zerod;
			Vec2D HeadingToUse_NV = Vec2D(sin(m_HeadingToUse()), cos(m_HeadingToUse()));
			const Vec2D DeltaVelocity = (vDesiredVelocity - GetLinearVelocity());
			//compute the maximum deceleration speed, since we need to reach 0 by the time we reach our desired velocity
			//Note: these are in local orientation so they need to be converted to global
			const Vec2D MaxDeceleration = Vec2Multiply(HeadingToUse_NV, GetAcceleration_Delta(DeltaTime_s, Vec2D(0.0, 0.0), false));
			Vec2D Global_CurrentAcceleration(Vec2Multiply(HeadingToUse_NV, m_CurrentAcceleration));

			//A=Delta V / Delta T
			const Vec2D NewAcceleration_target = DeltaVelocity / DeltaTime_s;  //This may be pretty quick (apply Force restrictions later)

			{
				double MaxDeceleration_Length = MaxDeceleration.length();
				double CurrentAcceleration_Length = Global_CurrentAcceleration.length();
				double SpeedUpTime = CurrentAcceleration_Length ? DeltaVelocity.length() / (CurrentAcceleration_Length*DeltaTime_s) : 0.0;
				double SpeedDownTime = MaxDeceleration_Length ? CurrentAcceleration_Length / MaxDeceleration_Length : 0.0;
				//Choose whether to speed up or slow down based on which would take longer to do
				if ((SpeedUpTime > SpeedDownTime) || (MaxDeceleration_Length >= CurrentAcceleration_Length))
					Acceleration = NewAcceleration_target;
				else
					Acceleration = DeltaVelocity / SpeedDownTime;

			}
			Force = Acceleration * m_EntityMass;
		}
		return Force;
	}
	virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance, const Vec2D &ForceRestraintPositive, 
		const Vec2D &ForceRestraintNegative, double DeltaTime_s, const Vec2D& matchVel)
	{
		Vec2D ret;

		if (!m_UsingAccelerationRate)
		{
			ret = PhysicsEntity_2D::GetVelocityFromDistance_Linear(Distance, ForceRestraintPositive, ForceRestraintNegative, DeltaTime_s, matchVel);
			#if 0
			assert(m_HeadingToUse!=nullptr);
			{
				Vec2D LocalVelocity = GlobalToLocal(m_HeadingToUse(), m_LinearVelocity);
				if (fabs(m_LinearVelocity[1]) > 0.0)
					printf("y=%.2f p=%.2f e=%.2f cs=0\n", Distance[1], ret[1], m_LinearVelocity[1]);
			}
			#endif
		}
		else
		{
			const Vec2D MaxDeceleration = GetAcceleration_Delta(1.0, Vec2D(0.0, 0.0), false);
			#if 0
			//I thought there may be a problem with solving each element separately so I wrote this version
			double dDistance = Distance.length();
			const double RampError = 12.0;

			//double AccelerationMagnitude=Acceleration.length();
			double AccelerationMagnitude = ForceRestraintNegative[1] / m_EntityMass; //for now this is the amount of reverse thrust, but I need to work this out at some point

			double Time = sqrt(2.0*dDistance / AccelerationMagnitude);
			ret = (dDistance > RampError) ? Distance / max(Time, DeltaTime_s) : Vec2D(0, 0, 0);

			//DebugOutput("d=%f %f ds=%f s=%f\n",Distance[1],dDistance,ret.length(),m_LinearVelocity.length());

			#else
			//Solving each element gives the advantage of using strafe techniques, we are assuming all calls work with local orientation
			for (size_t i = 0; i < 2; i++)
			{
				double DistanceDirection = Distance[i];
				DistanceDirection -= matchVel[i] * DeltaTime_s;
				if (IsZero(DistanceDirection))
				{
					ret[i] = matchVel[i];
					continue;
				}

				double DistanceLength = fabs(DistanceDirection);

				//Compose our restraint and acceleration based on the component direction
				double Restraint = (DistanceDirection > 0) ? ForceRestraintPositive[i] : ForceRestraintNegative[i];
				double Acceleration = (Restraint / m_EntityMass);

				double MaxDeceleration_Length = fabs(MaxDeceleration[i]);

				//This number is a work in progress
				const double RampError = 2.0;

				{
					//Solve the Acceleration... given a distance what would it have to be going its fastest ramp rate
					double Time = sqrt(2.0*(DistanceLength / MaxDeceleration[i]));
					Acceleration = std::min(Acceleration, MaxDeceleration_Length*Time);
				}
				double Time = sqrt(2.0*(DistanceLength / Acceleration));

				//TODO at some point this should PID to solve from the super class force feed method... I've put the correct equation below, but it does not work
				//properly with this attempt to solve... these ramp functions are only used in game code so I'll table it for now
				//  [8/26/2012 Terminator]
				#if 1
				//using distance/DTime in the max helps taper off the last frame to not over compensate
				ret[i] = DistanceLength / std::max(Time, DeltaTime_s);
				#else
				const double MaxSpeed = Acceleration * Time;
				const double IdealSpeed = DistanceLength / DeltaTime_s;
				ret[i] = min(MaxSpeed, IdealSpeed);
				#endif

				//if (i==1)
					//DebugOutput("Distance=%f,Time=%f,Speed=%f,vel=%f\n",DistanceDirection,Time,ret[i],m_LinearVelocity[i]);
					//DebugOutput("Distance=%f,Time=%f,Speed=%f,acc=%f\n",DistanceDirection,Time,ret[i],Acceleration);

				if (DistanceLength > RampError)
				{
					if (DistanceDirection < 0)
						ret[i] = -ret[i];
					ret[i] += matchVel[i];
				}
				else
					ret[i] = matchVel[i];
			}
			#endif
		}
		return ret;

	}
};

#pragma endregion
}}
