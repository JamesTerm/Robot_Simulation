#pragma region _includes macros_
#include "../../../../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../../../../Base/Vec2D.h"
#include "../../../../Base/Misc.h"

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

#include "MotionControl2D.h"

//#define __UseSimpleFallback__
#pragma endregion
namespace Module {
	namespace Robot	{
		namespace Physics {

#pragma region _helper functions_
//The actual force between two objects are f=(G m1 m2)/ r^2
//For example g = ( G * (Me->5.98E+24)) / (Re->6.38E+6)^2 = 9.8 m/s^2
//G is ideal to compute forces on ships from various planets
const double G = 6.673E-11;

inline Vec2D GlobalToLocal(double Heading, const Vec2D &GlobalVector)
{
	return Vec2D(sin(-Heading)*GlobalVector[1] + cos(Heading)*GlobalVector[0],
		cos(-Heading)*GlobalVector[1] + sin(Heading)*GlobalVector[0]);
}

inline Vec2D LocalToGlobal(double Heading, const Vec2D &LocalVector)
{
	return Vec2D(sin(Heading)*LocalVector[1] + cos(-Heading)*LocalVector[0],
		cos(Heading)*LocalVector[1] + sin(-Heading)*LocalVector[0]);
}

inline bool PosBNE(double val, double t)
{
	return !(fabs(val - t) < 1E-3);
}


inline const Vec2D Vec2Multiply(const Vec2D &A, const Vec2D &rhs)
{
	return Vec2D(A[0] * rhs._v[0], A[1] * rhs._v[1]);
}

inline const Vec2D Vec2_abs(const Vec2D &A)
{
	return Vec2D(fabs(A[0]), fabs(A[1]));
}
inline const Vec2D Vec2_min(const Vec2D &A, const Vec2D &B)
{
	return Vec2D(std::min(A[0], B[0]), std::min(A[1], B[1]));
}


//Normalize it in the bound of Pi2
inline void NormalizeToPi2(double &A)
{
	if (A < 0)
		A += Pi2;
	else if (A > Pi2)
		A -= Pi2;
}

inline void NormalizeToPi2wDirection(double &A)
{
	if (A < -Pi2)
		A += Pi2;
	else if (A > Pi2)
		A -= Pi2;
}

__inline double shortest_angle(double Distance)
{
	return Distance - Pi2 * floor(Distance / Pi2 + 0.5);
}

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
		for (size_t i = 0; i < 3; i++)
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
	const double *m_HeadingToUse;
	//This keeps track of the current rate of acceleration.  These are in local orientation.
	Vec2D m_CurrentAcceleration, m_TargetAcceleration;
	LinearAccelerationRates m_LinearAccelerationRates;
	//Some objects may not need to use this (by default they will not)
	bool m_UsingAccelerationRate;
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
	FlightDynamics_2D() : m_HeadingToUse(&m_DefaultHeading)
	{
		init();
	}
	FlightDynamics_2D(const double *HeadingToUse) : m_HeadingToUse(HeadingToUse)
	{
		init();
	}
	FlightDynamics_2D(const double &HeadingToUse) : m_HeadingToUse(&HeadingToUse)
	{
		init();
	}
	virtual ~FlightDynamics_2D() {}
	//Allow late binding of heading to use (client code should set this once as soon as possible)
	void SetHeadingToUse(const double *HeadingToUse) {m_HeadingToUse=HeadingToUse;}
	virtual void ResetVectors()
	{
		__super::ResetVectors();
		if (m_HeadingToUse == &m_DefaultHeading)
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
		double distance = *m_HeadingToUse - lookDir_radians;
		return shortest_angle(distance);
	}
	double ComputeAngularDistance(double Orientation)
	{
		/// \param Orientation this will break down the quat into its lookDir and UpDir for you
		double DistanceDirection = *m_HeadingToUse - Orientation;
		return shortest_angle(DistanceDirection);
	}
	const double &GetHeading() 
	{
		return *m_HeadingToUse;
	}
	virtual void TimeChangeUpdate(double DeltaTime_s, Vec2D &PositionDisplacement, double &RotationDisplacement)
	{
		__super::TimeChangeUpdate(DeltaTime_s, PositionDisplacement, RotationDisplacement);
		if (m_HeadingToUse == &m_DefaultHeading)
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
			Force = __super::GetForceFromVelocity(vDesiredVelocity, DeltaTime_s);
		else
		{
			Vec2D Zerod = Vec2D(0.0, 0.0);
			Vec2D Acceleration = Zerod;
			Vec2D HeadingToUse_NV = Vec2D(sin(*m_HeadingToUse), cos(*m_HeadingToUse));
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
			ret = __super::GetVelocityFromDistance_Linear(Distance, ForceRestraintPositive, ForceRestraintNegative, DeltaTime_s, matchVel);
			#if 0
			if (m_HeadingToUse)
			{
				Vec2D LocalVelocity = GlobalToLocal(*m_HeadingToUse, m_LinearVelocity);
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

#pragma region _Goal_
//TODO move this to a hpp file
class Goal
{
public:
	enum Goal_Status
	{
		eInactive,  //The goal is waiting to be activated
		eActive,    //The goal has been activated and will be processed each update step
		eCompleted, //The goal has completed and will be removed on the next update
		eFailed     //The goal has failed and will either re-plan or be removed on the next update
	};
protected:
	Goal_Status m_Status;
	//TODO see if Owner and Type are necessary
public:
	virtual ~Goal() {}
	///This contains initialization logic and represents the planning phase of the goal.  A Goal is able to call its activate method
	///any number of times to re-plan if the situation demands.
	virtual void Activate() = 0;

	//TODO we may want to pass in the delta time slice here
	/// This is executed during the update step
	virtual Goal_Status Process(double dTime_s) = 0;
	/// This undertakes any necessary tidying up before a goal is exited and is called just before a goal is destroyed.
	virtual void Terminate() = 0;
	//bool HandleMessage()  //TODO get event equivalent
	//TODO see if AddSubgoal really needs to be at this level 
	Goal_Status GetStatus() const { return m_Status; }
	//Here is a very common call to do in the first line of a process update
	inline void ActivateIfInactive() { if (m_Status == eInactive) Activate(); }
	inline void ReActivateIfFailed() { if (m_Status == eFailed) Activate(); }

	// This ensures that Composite Goals can safely allocate atomic goals and let the base implementation delete them
	static void* operator new (const size_t size)
	{
		return malloc(size);
	}
	static void  operator delete (void* ptr)
	{
		free(ptr);
	}
	static void* operator new[](const size_t size)
	{
		return malloc(size);
	}
	static void  operator delete[](void* ptr) 
	{	
		free(ptr);
	}
};
#pragma endregion

#pragma region _helper functions_
//This is really Local to Global for just the Y Component
inline Vec2D GetDirection(double Heading, double Intensity)
{
	return Vec2D(sin(Heading)*Intensity, cos(Heading)*Intensity);
}

inline void NormalizeRotation(double &Rotation)
{
	const double Pi2 = M_PI * 2.0;
	//Normalize the rotation
	if (Rotation > M_PI)
		Rotation -= Pi2;
	else if (Rotation < -M_PI)
		Rotation += Pi2;
}

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

inline double NormalizeRotation_HalfPi(double Orientation)
{
	if (Orientation > PI_2)
		Orientation -= M_PI;
	else if (Orientation < -PI_2)
		Orientation += M_PI;
	return Orientation;
}

inline double SaturateRotation(double Rotation)
{
	//Normalize the rotation
	if (Rotation > M_PI)
		Rotation = M_PI;
	else if (Rotation < -M_PI)
		Rotation = -M_PI;
	return Rotation;
}


#pragma endregion
#pragma region _Ship_
class Ship_2D
{
public:
	#pragma region _Ship Props_
	struct Ship_Props
	{
		// This is the rate used by the keyboard
		double dHeading;

		//May need these later to simulate pilot error in the AI
		//! G-Force limits
		//double StructuralDmgGLimit, PilotGLimit, PilotTimeToPassOut, PilotTimeToRecover, PilotMaxTimeToRecover;

		//! We can break this up even more if needed
		double EngineRampForward, EngineRampReverse, EngineRampAfterBurner;
		double EngineDeceleration, EngineRampStrafe;

		//! Engaged max speed is basically the fastest speed prior to using after-burner.  For AI and auto pilot it is the trigger speed to
		//! enable the afterburner
		double MAX_SPEED, ENGAGED_MAX_SPEED;
		double ACCEL, BRAKE, STRAFE, AFTERBURNER_ACCEL, AFTERBURNER_BRAKE;

		double MaxAccelLeft, MaxAccelRight, MaxAccelForward, MaxAccelReverse;
		double MaxAccelForward_High, MaxAccelReverse_High;
		//Note the SetPoint property is tuned to have quick best result for a set point operation
		double MaxTorqueYaw, MaxTorqueYaw_High;
		double MaxTorqueYaw_SetPoint, MaxTorqueYaw_SetPoint_High;
		//depreciated-
		//These are used to avoid overshoot when trying to rotate to a heading
		//double RotateTo_TorqueDegradeScalar,RotateTo_TorqueDegradeScalar_High;
		double Rotation_Tolerance;
		//If zero this has no effect, otherwise when rotating to intended position if it consecutively reaches the count it will flip the
		//lock heading status to lock... to stop trying to rotate to intended position
		double Rotation_ToleranceConsecutiveCount;
		//This supersedes RotateTo_TorqueDegradeScalar to avoid overshoot without slowing down rate
		//This applies linear blended scale against the current distance based on current velocity
		//default using 1.0 will produce no change
		double Rotation_TargetDistanceScalar;
	};
	#pragma endregion
	enum eThrustState { TS_AfterBurner_Brake = 0, TS_Brake, TS_Coast, TS_Thrust, TS_AfterBurner, TS_NotVisible };
	eThrustState GetThrustState() { return m_thrustState; }
	eThrustState SetThrustState(eThrustState ts)
	{
		// Handles the ON/OFF events, only for controlled entities
			//Apply a threshold averager here to avoid thrashing of animation sequences
		ts = m_thrustState_Average.GetValue(ts);
		// Watch for no changes
		//if (ts == m_thrustState) return m_thrustState;
		return m_thrustState;
	}
private:
	#pragma region _members
	#pragma region _Ship Properties_
	class Ship_Properties
	{
	private:
		Ship_Props m_ShipProps;
		//class ControlEvents : public LUA_Controls_Properties_Interface
		//{
		//protected: //from LUA_Controls_Properties_Interface
		//	virtual const char *LUA_Controls_GetEvents(size_t index) const;
		//};
		//static ControlEvents s_ControlsEvents;
		//LUA_ShipControls_Properties m_ShipControls;
	public:
		Ship_Properties()
		{
			Ship_Props props;
			memset(&props, 0, sizeof(Ship_Props));

			props.dHeading = DEG_2_RAD(270.0);

			double Scale = 0.2;  //we must scale everything down to see on the view
			props.MAX_SPEED = 2000.0 * Scale;
			props.ENGAGED_MAX_SPEED = 400.0 * Scale;
			props.ACCEL = 60.0 * Scale;
			props.BRAKE = 50.0 * Scale;
			props.STRAFE = props.BRAKE; //could not find this one
			props.AFTERBURNER_ACCEL = 107.0 * Scale;
			props.AFTERBURNER_BRAKE = props.BRAKE;

			props.MaxAccelLeft = 40.0 * Scale;
			props.MaxAccelRight = 40.0 * Scale;
			props.MaxAccelForward = props.MaxAccelForward_High = 87.0 * Scale;
			props.MaxAccelReverse = props.MaxAccelReverse_High = 70.0 * Scale;
			props.MaxTorqueYaw_SetPoint = 2.5;

			double RAMP_UP_DUR = 1.0;
			double RAMP_DOWN_DUR = 1.0;
			props.EngineRampAfterBurner = props.AFTERBURNER_ACCEL / RAMP_UP_DUR;
			props.EngineRampForward = props.ACCEL / RAMP_UP_DUR;
			props.EngineRampReverse = props.BRAKE / RAMP_UP_DUR;
			props.EngineRampStrafe = props.STRAFE / RAMP_UP_DUR;
			props.EngineDeceleration = props.ACCEL / RAMP_DOWN_DUR;
			//depreciated
			//props.RotateTo_TorqueDegradeScalar=props.RotateTo_TorqueDegradeScalar_High=1.0;
			props.Rotation_Tolerance = 0.0;
			props.Rotation_ToleranceConsecutiveCount = 0.0;  //zero is disabled
			props.Rotation_TargetDistanceScalar = 1.0;
			m_ShipProps = props;
		}
		virtual ~Ship_Properties() {}
		//const char *SetUpGlobalTable(Scripting::Script& script);
		//virtual void LoadFromScript(Scripting::Script& script);
		//This is depreciated (may need to review game use-case)
		//void Initialize(Ship_2D *NewShip) const;
		//This is depreciated... use GetShipProps_rw
		void UpdateShipProperties(const Ship_Props &props)
		{
			//explicitly allow updating of ship props here
			m_ShipProps = props;
		}
		//Ship_Props::Ship_Type GetShipType() const { return m_ShipProps.ShipType; }
		double GetEngagedMaxSpeed() const
		{
			return m_ShipProps.ENGAGED_MAX_SPEED;
		}
		//These methods are really more for the simulation... so using the high yields a better reading for testing
		double GetMaxAccelForward() const
		{
			return m_ShipProps.MaxAccelForward_High;
		}
		double GetMaxAccelReverse() const
		{
			return m_ShipProps.MaxAccelReverse_High;
		}
		double GetMaxAccelForward(double Velocity) const
		{
			const Ship_Props &props = m_ShipProps;
			const double ratio = fabs(Velocity) / props.MAX_SPEED;
			const double  &Low = props.MaxAccelForward;
			const double &High = props.MaxAccelForward_High;
			return (ratio * High) + ((1.0 - ratio) * Low);
		}
		double GetMaxAccelReverse(double Velocity) const
		{
			const Ship_Props &props = m_ShipProps;
			const double ratio = fabs(Velocity) / props.MAX_SPEED;
			const double  &Low = props.MaxAccelReverse;
			const double &High = props.MaxAccelReverse_High;
			return (ratio * High) + ((1.0 - ratio) * Low);
		}
		double GetMaxTorqueYaw(double Velocity) const
		{
			const Ship_Props &props = m_ShipProps;
			const double ratio = fabs(Velocity) / props.dHeading;
			const double  &Low = props.MaxTorqueYaw;
			const double &High = props.MaxTorqueYaw_High;
			return (ratio * High) + ((1.0 - ratio) * Low);
		}
		double GetMaxTorqueYaw_SetPoint(double Velocity) const
		{
			const Ship_Props &props = m_ShipProps;
			const double ratio = fabs(Velocity) / props.dHeading;
			const double  &Low = props.MaxTorqueYaw_SetPoint;
			const double &High = props.MaxTorqueYaw_SetPoint_High;
			return (ratio * High) + ((1.0 - ratio) * Low);
		}
		const Ship_Props &GetShipProps() const
		{
			return m_ShipProps;
		}
		Ship_Props &GetShipProps_rw()
		{
			return m_ShipProps;
		}
		//const LUA_ShipControls_Properties &Get_ShipControls() const { return m_ShipControls; }
	};
	#pragma endregion
	#pragma region _AI Base Controller_
	class AI_Base_Controller
	{
	protected:
		friend class Ship_Tester;
		Goal *m_Goal; //Dynamically set a goal for this controller
	private:
		//TODO determine way to properly introduce UI_Controls here	
		Ship_2D &m_ship;

		//friend class UI_Controller;
		//UI_Controller *m_UI_Controller;
	public:
		AI_Base_Controller(Ship_2D &ship) : m_ship(ship)
		{
		}
		virtual ~AI_Base_Controller() {}

		///This is the single update point to all controlling of the ship.  The base class contains no goal arbitration, but does implement
		///Whatever goal is passed into it if the UI controller is off line
		virtual void UpdateController(double dTime_s)
		{

		}

		//This is mostly for pass-thru since the timing of this is in alignment during a ship att-pos update
		void UpdateUI(double dTime_s)
		{

		}

		/// I put the word try, as there may be some extra logic to determine if it has permission
		/// This is a bit different than viewing an AI with no controls, where it simply does not
		/// Allow a connection
		/// \return true if it was allowed to bind
		//virtual bool Try_SetUIController(UI_Controller *controller);
		virtual void ResetPos() {}

		bool HasAutoPilotRoute() { return true; }
		bool GetCanUserPilot() { return true; }

		void SetShipVelocity(double velocity_mps) { m_ship.SetRequestedVelocity(velocity_mps); }

		/// \param TrajectoryPoint- This is the point that your nose of your ship will orient to from its current position (usually the same as PositionPoint)
		/// \param PositionPoint- This is the point where your ship will be to position to (Usually the same as TrajectoryPoint)
		/// \power- The scaled value multiplied to the ships max speed.  If > 1 it can be interpreted as explicit meters per second speed
		/// \matchVel- You can make it so the velocity of the ship when it reaches the point.  
		/// Use NULL when flying through way-points
		/// Use (0,0) if you want to come to a stop, like at the end of a way-point series
		/// Otherwise, use the velocity of the ship you are targeting or following to keep up with it
		void DriveToLocation(Vec2D TrajectoryPoint, Vec2D PositionPoint, double power, double dTime_s, Vec2D* matchVel, bool LockOrientation = false)
		{
		}
		void SetIntendedOrientation(double IntendedOrientation) { m_ship.SetIntendedOrientation(IntendedOrientation); }

		Ship_2D &GetShip() { return m_ship; }
		//const UI_Controller *GetUIController() const { return m_UI_Controller; }
		//I want it to be clear when we intend to write
		//UI_Controller *GetUIController_RW() { return m_UI_Controller; }
	};
	#pragma endregion

	AI_Base_Controller* m_controller=nullptr;
	Ship_Properties m_ShipProperties;

	double m_RadialArmDefault; //cache the radius of concentrated mass square, which will allow us to apply torque in a r = 1 case

	//Stuff needed for physics
	double Camera_Restraint;
	double G_Dampener;

	//Use this technique when m_AlterTrajectory is true
	Vec2D m_RequestedVelocity;
	double m_AutoLevelDelay; ///< The potential Gimbal lock, and user rolling will trigger a delay for the auto-level (when enabled)
	double m_HeadingSpeedScale; //used by auto pilot control to have slower turn speeds for way points
	double m_rotAccel_rad_s;

	//All input for turn pitch and roll apply to this, both the camera and ship need to align to it
	double m_IntendedOrientation;
	//We need the m_IntendedOrientation quaternion to work with its own physics
	FlightDynamics_2D m_IntendedOrientationPhysics;

	//For slide mode all strafe is applied here
	Vec2D m_currAccel;  //This is the immediate request for thruster levels

	///Typically this contains the distance of the intended direction from the actual direction.  This is the only variable responsible for
	///changing the ship's orientation
	double m_rotDisplacement_rad;

	bool m_SimFlightMode;  ///< If true auto strafing will occur to keep ship in line with its position
	bool m_StabilizeRotation=true;  ///< If true (should always be true) this will fire reverse thrusters to stabilize rotation when idle
	bool m_CoordinateTurns=true;   ///< Most of the time this is true, but in some cases (e.g. Joystick w/rudder pedals) it may be false

	Threshold_Averager<eThrustState, 5> m_thrustState_Average;
	eThrustState m_thrustState;
	//double m_Last_AccDel;  ///< This monitors a previous AccDec session to determine when to reset the speed
	Vec2D m_Last_RequestedVelocity;  ///< This monitors the last caught requested velocity from a speed delta change
	FlightDynamics_2D m_Physics;
protected:
	//A counter to count how many times the predicted position and intended position are withing tolerance consecutively
	size_t m_RotationToleranceCounter;
	bool m_LockShipHeadingToOrientation; ///< Locks the ship and intended orientation (Joystick and Keyboard controls use this)

	Vec2D m_current_position;
	double m_current_heading = 0.0;

	std::function<void(const Vec2D &new_velocity)> m_ExternSetVelocity = nullptr;
	std::function<void(double new_velocity)> m_ExternSetHeadingVelocity = nullptr;
	std::function <Vec2D()> m_ExternGetCurrentPosition = nullptr;
	std::function <double()> m_ExternGetCurrentHeading = nullptr;

	#pragma endregion
protected:
	#pragma region _protected members_
	const PhysicsEntity_2D &GetPhysics() const 
	{ 
		return m_Physics; 
	}
	void SetStabilizeRotation(bool StabilizeRotation) 
	{ 
		m_StabilizeRotation = StabilizeRotation; 
	}
	void UpdateIntendedOrientaton(double dTime_s)
	{
		///This will apply turn pitch and roll to the intended orientation
		double rotVel = 0.0, rotVelControlled = 0.0;

		//distribute the rotation velocity to the correct case
		if (m_LockShipHeadingToOrientation)
			rotVelControlled = m_rotAccel_rad_s;
		else
			rotVel = m_rotAccel_rad_s;

		//Make sure the look ahead is within a reasonable distance from the ship; otherwise the quat delta's may give error as to yaw and pitch corrections
		//To make this look smooth we will compute it as resistance
		double YawResistance = 1.0;
		#ifdef __EnableOrientationResistance__
		if (!m_UseHeadingSpeed)
		{
			Vec3d Offset(m_Physics.ComputeAngularDistance(m_IntendedOrientation));
			const double MaxLookAt = M_PI * 0.4; //the max amount to look ahead
			YawResistance = fabs(Offset[0]) < MaxLookAt ? 1.0 - (fabs(Offset[0]) / MaxLookAt) : 0.0;
		}
		#endif

		// From Rick: James, Why are we not multiplying by time here?  I think the m_rotAccel_rad_s might be artificially high
		// From James: rotVel represents the delta to use at that given moment and should be artificially high as this gives you the
		// "snappiness" feeling when looking around this the mouse
		m_IntendedOrientation += rotVel * YawResistance;

		double TorqueToApply = m_IntendedOrientationPhysics.GetTorqueFromVelocity(rotVelControlled, dTime_s);
		ApplyTorqueThrusters(m_IntendedOrientationPhysics, TorqueToApply,m_ShipProperties.GetShipProps().MaxTorqueYaw_SetPoint, dTime_s);
		{
			//Run physics update for displacement
			Vec2D PositionDisplacement;
			double RotationDisplacement;
			m_IntendedOrientationPhysics.TimeChangeUpdate(dTime_s, PositionDisplacement, RotationDisplacement);
			m_IntendedOrientation += RotationDisplacement * YawResistance;
		}
	}
	void ApplyTorqueThrusters(PhysicsEntity_2D &PhysicsToUse, double Torque, double TorqueRestraint, double dTime_s)
	{
		//assert(IsLocallyControlled());
		//ApplyTorqueThrusters
		//Note: desired speed is a separated variable isControlled from the ship's speed script, which we fine tune given the torque restraints
		//And also by minimizing the amount of G's pulled at the outer most edge of the ship... so for large ships that rotate this could be
		//significant, and you wouldn't want people slamming into the walls.
		//Note: if the speed is too high and the torque restraint is too low the ship will "wobble" because it trying to to go a faster speed that it
		//can "brake" for... ideally a little wobble is reasonable and this is isControlled by a good balance between desired speed and torque restraints

		double TorqueToApply = PhysicsToUse.ComputeRestrainedTorque(Torque, TorqueRestraint, dTime_s);

		#if 0  //This case is only for test purposes (I will eventually remove)
		PhysicsToUse.ApplyTorque(TorqueToApply);
		#else
		PhysicsToUse.ApplyFractionalTorque(TorqueToApply, dTime_s, m_RadialArmDefault);
		#endif
	}
	void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse, const Vec2D &LocalForce, double LocalTorque, double TorqueRestraint, double dTime_s)
	{
		///Putting force and torque together will make it possible to translate this into actual force with position
		//assert(IsLocallyControlled());
		 //Apply force
		Vec2D ForceToApply = LocalToGlobal(GetAtt_r(), LocalForce);
		PhysicsToUse.ApplyFractionalForce(ForceToApply, dTime_s);
		// Apply Torque
		ApplyTorqueThrusters(PhysicsToUse, LocalTorque, TorqueRestraint, dTime_s);
	}
	void TestPosAtt_Delta(const Vec2D pos_m, double att, double dTime_s)
	{
		#if 0
		if (m_controller->IsUIControlled())
			DOUT1("%f %f %f %f", dTime_s, pos_m[0], pos_m[1], pos_m[2]);
		#endif
	}
	bool IsPlayerControllable() 
	{ 
		// Watch for being made the controlled ship
		return true;
	}
	AI_Base_Controller *Create_Controller()
	{
		//TODO this is no longer necessary (but still works)
		//Override with the controller to be used with ship.  Specific ships have specific type of controllers.
		return new AI_Base_Controller(*this);
	}
	void RequestedVelocityCallback(double VelocityToUse, double DeltaTime_s) 
	{
		///This allows subclass to evaluate the requested velocity when it is in use
		//TODO no implementation, should remove
	}
	Vec2D Get_DriveTo_ForceDegradeScalar() const 
	{ 
		//override to manipulate a distance force degrade, which is used to compensate for deceleration inertia
		return Vec2D(1.0, 1.0); 
	}
	static void InitNetworkProperties(const Ship_Props &ship_props)  
	{
		//TODO We shouldn't be using smart dashboard in here, plan to remove
		#if 0
		//This will GetVariables of all properties needed to tweak PID and gain assists
		SmartDashboard::PutNumber("Rotation Tolerance", RAD_2_DEG(props.Rotation_Tolerance));
		SmartDashboard::PutNumber("Rotation Tolerance Count", props.Rotation_ToleranceConsecutiveCount);
		SmartDashboard::PutNumber("rotation_distance_scalar", props.Rotation_TargetDistanceScalar);
		#endif
	}
	static void NetworkEditProperties(Ship_Props &ship_props)
	{
		//TODO We shouldn't be using smart dashboard in here, plan to remove
		#if 0
		//This will GetVariables of all properties needed to tweak PID and gain assists
		ship_props.Rotation_Tolerance = DEG_2_RAD(SmartDashboard::GetNumber("Rotation Tolerance"));
		ship_props.Rotation_ToleranceConsecutiveCount = SmartDashboard::GetNumber("Rotation Tolerance Count");
		ship_props.Rotation_TargetDistanceScalar = SmartDashboard::GetNumber("rotation_distance_scalar");
		#endif
	}
	#pragma region _Moved from public_
	void UpdateController(double &AuxVelocity, Vec2D &LinearAcceleration, double &AngularAcceleration, bool &LockShipHeadingToOrientation, double dTime_s)
	{
		//TODO empty... omit
		//callback from UI_Controller for custom controls override if ship has specific controls... all outputs to be written are optional
		//so derived classes can only write to things of interest
	}
	void BindAdditionalEventControls(bool Bind)
	{
		//TODO this was empty can omit
		//The UI controller will call this when attaching or detaching control.  The Bind parameter will either bind or unbind.  Since these are 
		//specific controls to a specific ship there is currently no method to transfer these specifics from one ship to the next.  Ideally there
		//should be no member variables needed to implement the bindings
	}
	void BindAdditionalUIControls(bool Bind, void *joy, void *key)
	{
		//TODO omit.. no controls 
		//Its possible that each ship may have its own specific controls
		//m_ShipProperties.Get_ShipControls().BindAdditionalUIControls(Bind, joy, key);
	}
	void CancelAllControls()
	{
		//TODO this was already all disabled we can omit
		// Turn off all thruster controls
		//__super::CancelAllControls();
		//if (m_controller )
		//	m_controller->CancelAllControls();
	}

	AI_Base_Controller *GetController() const
	{
		return m_controller;
	}
	bool GetStabilizeRotation() const
	{
		return m_StabilizeRotation;
	}
	bool GetAlterTrajectory() const
	{
		return m_SimFlightMode;
	}
	bool GetCoordinateTurns() const
	{
		return m_CoordinateTurns;
	}
	void SetHeadingSpeedScale(double scale)
	{
		m_HeadingSpeedScale = scale;
	}
	bool GetIsAfterBurnerOn() const
	{
		/// We use a toggling mechanism to use afterburners since there is some internal functionality that behave differently
		return (m_thrustState == TS_AfterBurner);
	}
	bool GetIsAfterBurnerBrakeOn() const
	{
		return (m_thrustState == TS_AfterBurner_Brake);
	}
	bool CanStrafe() const
	{
		//Override if the ship is not able to strafe... used for DriveToLocation()
		return true;
	}
	double GetMaxSpeed() const
	{
		return m_ShipProperties.GetShipProps().MAX_SPEED;
	}
	double GetEngaged_Max_Speed() const
	{
		return m_ShipProperties.GetShipProps().ENGAGED_MAX_SPEED;
	}
	double GetStrafeSpeed() const
	{
		return m_ShipProperties.GetShipProps().STRAFE;
	}
	double GetAccelSpeed() const
	{
		return m_ShipProperties.GetShipProps().ACCEL;
	}
	double GetBrakeSpeed() const
	{
		return m_ShipProperties.GetShipProps().BRAKE;
	}
	double GetCameraRestraintScaler() const
	{
		return Camera_Restraint;
	}
	double GetHeadingSpeed() const
	{
		return m_ShipProperties.GetShipProps().dHeading;
	}
	#pragma endregion
	#pragma endregion
public:
	#pragma region _public members_
	Ship_2D() : m_IntendedOrientationPhysics(m_IntendedOrientation)
	{
		m_HeadingSpeedScale = 1.0;
		m_LockShipHeadingToOrientation = false;  //usually this is false (especially for AI and Remote controllers)
		m_thrustState = TS_NotVisible;
		ResetPos();
		m_RotationToleranceCounter = 0;
		//TODO take out initialize
		Initialize();
	}
	void UpdateShipProperties(const Ship_Props &props)
	{
		//Give ability to change ship properties 
		m_ShipProperties.UpdateShipProperties(props);
	}
	void Initialize(const Ship_Properties *ship_props=nullptr)
	{
		//TODO see if we really need to initialize
		m_Physics.SetMass(120.0 / 2.205); //the typical weight of the robot
		//virtual void Initialize(Entity2D_Kind::EventMap& em, const Entity_Properties *props = NULL);
		if (!m_controller)
			m_controller = Create_Controller();
		//These default values are pulled from 2014 robot (perhaps the best tuned robot for driver)
		const double g_wheel_diameter_in = 4;
		const double Inches2Meters = 0.0254;
		const double Meters2Inches = 39.3700787;
		const double HighGearSpeed = (873.53 / 60.0) * Pi * g_wheel_diameter_in * Inches2Meters  * 0.85;
		const double WheelBase_Width_In = 26.5;	  //The wheel base will determine the turn rate, must be as accurate as possible!
		const double WheelBase_Length_In = 10.0;  //was 9.625
		const double WheelTurningDiameter_In = sqrt((WheelBase_Width_In * WheelBase_Width_In) + (WheelBase_Length_In * WheelBase_Length_In));
		const double skid = cos(atan2(WheelBase_Length_In, WheelBase_Width_In));
		const double Drive_MaxAccel = 5.0;
		const double gMaxTorqueYaw = (2 * Drive_MaxAccel * Meters2Inches / WheelTurningDiameter_In) * skid;
		Ship_Props props =
		{
		//double dHeading;
			(2 * HighGearSpeed * Meters2Inches / WheelTurningDiameter_In) * skid,
		//double EngineRampForward, EngineRampReverse, EngineRampAfterBurner;
		10.0,10.0,10.0,
		//double EngineDeceleration, EngineRampStrafe;
		10.0,10.0,
		//double MAX_SPEED, ENGAGED_MAX_SPEED;
		HighGearSpeed,HighGearSpeed,
		//double ACCEL, BRAKE, STRAFE, AFTERBURNER_ACCEL, AFTERBURNER_BRAKE;
		10.0,10.0,10.0,10.0,10.0,
		//double MaxAccelLeft, MaxAccelRight, MaxAccelForward, MaxAccelReverse;
		20.0,20.0,Drive_MaxAccel,Drive_MaxAccel,
		//double MaxAccelForward_High, MaxAccelReverse_High;
		Drive_MaxAccel*2,Drive_MaxAccel*2,
		//double MaxTorqueYaw, MaxTorqueYaw_High;
		gMaxTorqueYaw * 0.78,gMaxTorqueYaw * 5.0,
		//double MaxTorqueYaw_SetPoint, MaxTorqueYaw_SetPoint_High;
		gMaxTorqueYaw * 2,gMaxTorqueYaw * 10,
		//double Rotation_Tolerance;
		DEG_2_RAD(2),
		//double Rotation_ToleranceConsecutiveCount;
		0.0,  //disabled by default using  0.0
		//double Rotation_TargetDistanceScalar;
		1.0
		};
		if (ship_props)
		{
			m_ShipProperties = *ship_props;
			props = ship_props->GetShipProps();
		}
		UpdateShipProperties(props);

		Camera_Restraint = G_Dampener = 1.0;

		//For now I don't really care about these numbers yet, so I'm pulling from the q33
		//m_Physics.StructuralDmgGLimit = 10.0;
		double RadiusOfConcentratedMass = m_Physics.GetRadiusOfConcentratedMass();
		m_IntendedOrientationPhysics.SetRadiusOfConcentratedMass(RadiusOfConcentratedMass);
		m_RadialArmDefault = RadiusOfConcentratedMass * RadiusOfConcentratedMass;

		//Pass these acceleration derivatives on to the Physics/Flight-Dynamics
		{
			FlightDynamics_2D::LinearAccelerationRates &_ = m_Physics.GetLinearAccelerationRates();
			_.AccDeltaPos = Vec2D(props.EngineRampStrafe, props.EngineRampForward);
			_.AccDeltaNeg = Vec2D(props.EngineRampStrafe, props.EngineRampReverse);
			Vec2D Deceleration(props.EngineDeceleration, props.EngineDeceleration);
			_.DecDeltaPos = _.DecDeltaNeg = Deceleration;
		}

		m_IntendedOrientation = GetAtt_r();
		m_IntendedOrientationPhysics.SetMass(25);
	}
	void TimeChange(double dTime_s)
	{
		const double Mass = m_Physics.GetMass();
		// Update my controller
		m_controller->UpdateController(dTime_s);

		// Find the current velocity and use to determine the flight characteristics we will WANT to us
		//Vec3d LocalVelocity(GetAtt_quat().conj() * m_Physics.GetLinearVelocity());
		Vec2D LocalVelocity = GlobalToLocal(GetAtt_r(), m_Physics.GetLinearVelocity());
		double currVelocity = LocalVelocity[1];
		bool manualMode = !m_SimFlightMode;
		bool afterBurnerOn = (m_RequestedVelocity[1] > GetEngaged_Max_Speed());
		bool afterBurnerBrakeOn = (fabs(currVelocity) > GetEngaged_Max_Speed());
		//const FlightCharacteristics& currFC((afterBurnerOn||afterBurnerBrakeOn) ? Afterburner_Characteristics : GetFlightCharacteristics());
		const Ship_Props &props = m_ShipProperties.GetShipProps();

		Vec2D ForceToApply;
		//Enable to monitor current speed
		#if 0
		{
			Vec3d Velocity = m_Physics.GetLinearVelocity();
			if (m_controller->IsUIControlled())
				printf("\r%s %f mps               ", GetID().c_str(), m_Physics.GetSpeed(Velocity));
			//printf("\r%f mph               ",m_Physics.GetSpeed(Velocity)*2.237);  //here's a cool quick conversion to get mph http://www.chrismanual.com/Intro/convfact.htm
		}
		#endif

		if (m_StabilizeRotation)
		{

			//Note: I use to have the intended orientation lead and apply physics on it to result in the desired lock on effect, but this proved to be problematic
			//if the ship was unable to keep up with the intended rate.  That is actually a good error test to keep around here, but true locking to ship will
			//slave the intended orientation to the ship
			//  [1/12/2012 Terminator]

			#ifdef _TestIndendedDirction_properties__
			UpdateIntendedOrientaton(dTime_s);

			//Determine the angular distance from the intended orientation
			m_rotDisplacement_rad = -m_Physics.ComputeAngularDistance(m_IntendedOrientation);
			#else
			if (m_LockShipHeadingToOrientation)
			{
				m_rotDisplacement_rad = m_rotAccel_rad_s;
				m_IntendedOrientation = GetAtt_r();
			}
			else
			{
				UpdateIntendedOrientaton(dTime_s);
				m_rotDisplacement_rad = -m_Physics.ComputeAngularDistance(m_IntendedOrientation);
				#ifdef __DisplaySetPoint_LinePlot__
				SmartDashboard::PutNumber("desired y", RAD_2_DEG(m_IntendedOrientation));
				SmartDashboard::PutNumber("actual y", RAD_2_DEG(NormalizeRotation2(m_Physics.GetHeading())));
				#endif
				const double TargetDistanceScalar = props.Rotation_TargetDistanceScalar;
				if (TargetDistanceScalar != 1.0)
				{
					//a simple linear blend on the scalar should be fine (could upgrade to poly if needed)
					const double ratio = fabs(m_Physics.GetAngularVelocity()) / props.dHeading;
					//Note the slower it goes the more blend of the TargetDistanceScalar needs to become to slow it down more towards
					//the last moments
					const double scale = (ratio * 1.0) + ((1.0 - ratio) * TargetDistanceScalar);
					#if 0
					if (ratio != 0.0)
						printf("%.2f %.2f %.2f\n", m_Physics.GetAngularVelocity(), ratio, scale);
					#endif
					m_rotDisplacement_rad *= scale;
				}
				if (fabs(m_rotDisplacement_rad) < props.Rotation_Tolerance)
				{
					//If we are not using a tolerance counter we'll zero out anytime it hits the tolerance threshold
					//If we are using it... then we keep feeding the displacement give PID and tolerance count a change to center into
					//the tolerance threshold
					if (props.Rotation_ToleranceConsecutiveCount == 0.0)
						m_rotDisplacement_rad = 0.0;
					m_RotationToleranceCounter++;
				}
				else
					m_RotationToleranceCounter = 0;

				//Normalize this
				if (IsZero(m_rotDisplacement_rad, 1e-2))
					m_rotDisplacement_rad = 0.0;

				//SmartDashboard::PutBoolean("Test m_rotDisplacement_rad",m_rotDisplacement_rad==0.0);

				if ((m_RotationToleranceCounter > 0.0) && (m_RotationToleranceCounter >= props.Rotation_ToleranceConsecutiveCount))
					m_LockShipHeadingToOrientation = true;
			}
			//SmartDashboard::PutBoolean("m_LockShipHeadingToOrientation",m_LockShipHeadingToOrientation);
			#endif
		}
		else
		{
			m_IntendedOrientation = GetAtt_r(); //If we can't stabilize the rotation then the intended orientation is slaved to the ship!
		}

		// apply restraints based off if we are driving or if it is being auto piloted... for auto pilot it should not blend the max force high
		//bool AutoPilot = m_controller->GetUIController() ? m_controller->GetUIController()->GetAutoPilot() : true;
		const bool AutoPilot = false;
		//Apply the restraints now... for now the lock ship member is a good way to know if it is being driven or autonomous, but we wouldn't want
		//to do this in the game
		Vec2D AccRestraintPositive(props.MaxAccelRight, AutoPilot ? props.MaxAccelForward : m_ShipProperties.GetMaxAccelForward(currVelocity));
		Vec2D AccRestraintNegative(props.MaxAccelLeft, AutoPilot ? props.MaxAccelReverse : m_ShipProperties.GetMaxAccelReverse(currVelocity));

		if (!manualMode)
		{
			//This first system combined the velocity request and the accel delta's as one but this runs into undesired effects with the accel deltas
			//The most undesired effect is that when no delta is applied neither should any extra force be applied.  There is indeed a distinction
			//between cruise control (e.g. slider) and using a Key button entry in this regard.  The else case here keeps these more separated where
			//you are either using one mode or the other
			Vec2D VelocityDelta = m_currAccel * dTime_s;

			bool UsingRequestedVelocity = false;
			bool YawPitchActive = (fabs(m_rotDisplacement_rad) > 0.001);

			//Note: m_RequestedVelocity is not altered with the velocity delta, but it will keep up to date
			if (VelocityDelta[1] != 0) //if user is changing his adjustments then reset the velocity to current velocity
			{
				if (!YawPitchActive)
					m_RequestedVelocity = m_Last_RequestedVelocity = LocalVelocity + VelocityDelta;
				else
				{
					//If speeding/braking during hard turns do not use currVelocity as the centripetal forces will lower it
					m_RequestedVelocity += VelocityDelta;
					m_Last_RequestedVelocity = m_RequestedVelocity;
					UsingRequestedVelocity = true;
				}
			}
			else
			{
				//If there is any turning while no deltas are on... kick on the requested velocity
				if (YawPitchActive)
				{
					//active the requested velocity mode by setting this to 0 (this will keep it on until a new velocity delta is used)
					//TODO work out a new system for resetting this I cannot use zero, but -1 is not correct either since we can use
					//negative direction
					m_Last_RequestedVelocity[0] = m_Last_RequestedVelocity[1] = -1.0;
					UsingRequestedVelocity = true;
				}
				else
					UsingRequestedVelocity = (m_RequestedVelocity != m_Last_RequestedVelocity);
			}

			//Just transfer the acceleration directly into our velocity to use variable
			Vec2D VelocityToUse = (UsingRequestedVelocity) ? m_RequestedVelocity : LocalVelocity + VelocityDelta;

			#if 0
			if (stricmp(GetName().c_str(), "Q33_2") == 0)
			{
				//DOUT2("%f %f %f",m_RequestedVelocity,m_Last_RequestedVelocity,m_RequestedVelocity-m_Last_RequestedVelocity);
				//DOUT2("%f %f %f",m_RequestedVelocity,currVelocity,m_RequestedVelocity-currVelocity);
				//DOUT3("%f",VelocityDelta);
			}
			#endif

			#ifndef __DisableSpeedControl__
			{
				//Note: this was checking m_currAccel[1] for a long time, but that fails for backwards case when using keyboard as it would be zero while
				//VelocityToUse[1] would be negative, in which case it should use the negative bounds check
				//  [2/14/2013 James]
				if (VelocityToUse[1] < 0) // Watch for braking too far backwards, we do not want to go beyond -ENGAGED_MAX_SPEED
				{
					if ((VelocityToUse[1]) < -props.ENGAGED_MAX_SPEED)
					{
						m_RequestedVelocity[1] = VelocityToUse[1] = -props.ENGAGED_MAX_SPEED;
						m_currAccel[1] = 0.0;
					}
				}
				else
				{
					double MaxSpeed = afterBurnerOn ? props.MAX_SPEED : props.ENGAGED_MAX_SPEED;
					if ((VelocityToUse[1]) > MaxSpeed)
					{
						m_RequestedVelocity[1] = VelocityToUse[1] = MaxSpeed;
						m_currAccel[1] = 0.0;
					}
				}
			}
			#endif

			Vec2D GlobalForce;
			if (UsingRequestedVelocity)
			{
				GlobalForce = m_Physics.GetForceFromVelocity(LocalToGlobal(GetAtt_r(), VelocityToUse), dTime_s);
				//Allow subclass to evaluate the requested velocity in use;
				RequestedVelocityCallback(VelocityToUse[1], dTime_s);
			}
			else
			{
				//We basically are zeroing the strafe here, and adding the forward/reverse element next
				GlobalForce = m_Physics.GetForceFromVelocity(GetDirection(GetAtt_r(), currVelocity), dTime_s);
				//Allow subclass to evaluate the requested velocity in use;
				RequestedVelocityCallback(currVelocity, dTime_s);
			}

			//so we'll need to convert to local
			//ForceToApply=(GetAtt_quat().conj() * GlobalForce);
			ForceToApply = GlobalToLocal(GetAtt_r(), GlobalForce);

			#if 0
			if (fabs(LocalVelocity[1]) > 0.0)
				printf("v=%.2f ", (ForceToApply[1] / Mass) / AccRestraintPositive[1]);
			#endif

			if (!UsingRequestedVelocity)
				ForceToApply[1] += m_currAccel[1] * Mass;
			//This shows no force being applied when key is released
			#if 0
			if (stricmp(GetName().c_str(), "Q33_2") == 0)
			{
				Vec3d acc = ForceToApply / Mass;
				DOUT2("%f %f", acc[0], acc[1]);
				DOUT3("%f %f", VelocityToUse, Mass);
			}
			#endif
		}
		else   //Manual mode
		{
			#ifndef __DisableSpeedControl__
			{
				//TODO we may want to compute the fractional forces to fill in the final room left, but before that the engine ramp up would need to
				//be taken into consideration as it currently causes it to slightly go beyond the boundary
				double MaxForwardSpeed = afterBurnerOn ? props.MAX_SPEED : props.ENGAGED_MAX_SPEED;
				for (size_t i = 0; i < 2; i++)
				{
					double MaxSpeedThisAxis = i == 1 ? MaxForwardSpeed : props.ENGAGED_MAX_SPEED;
					double VelocityDelta = (m_currAccel[1] * dTime_s);
					if ((LocalVelocity[i] + VelocityDelta > MaxSpeedThisAxis) && (m_currAccel[i] > 0))
						m_currAccel[i] = 0.0;
					else if ((LocalVelocity[i] + VelocityDelta < -props.ENGAGED_MAX_SPEED) && (m_currAccel[i] < 0))
						m_currAccel[i] = 0.0;
				}
			}
			#endif
			//Hand off m_curAccel to a local... we want to preserve the members state
			Vec2D currAccel(m_currAccel);

			#ifdef __TestFullForce__
			ForceToApply = currAccel * Mass*dTime_s;
			#else
			ForceToApply = currAccel * Mass;
			#endif
		}


		//for afterburner up the forward restraint
		if (afterBurnerOn || afterBurnerBrakeOn)
		{
			// Set the maximum restraint values based on Burning or Braking afterburners
			//TODO this property should be the MaxAcceleration type of property where AFTERBURNER_ACCEL is used for keyboard acceleration
			//AccRestraintPositive[1]= afterBurnerOn ? AFTERBURNER_ACCEL : AFTERBURNER_BRAKE;

			//This is not perfect in that all the accelerated and deceleration vector elements need to have this ramp value for non-sliding mode
			//We may alternately consider putting it in slide mode when using afterburner
			m_Physics.GetLinearAccelerationRates().AccDeltaPos = Vec2D(props.EngineRampAfterBurner, props.EngineRampAfterBurner);
			m_Physics.GetLinearAccelerationRates().DecDeltaPos = Vec2D(props.EngineRampAfterBurner, props.EngineRampAfterBurner);
		}
		else
		{
			m_Physics.GetLinearAccelerationRates().AccDeltaPos = Vec2D(props.EngineRampStrafe, props.EngineRampForward);
			m_Physics.GetLinearAccelerationRates().DecDeltaPos = Vec2D(props.EngineDeceleration, props.EngineDeceleration);
		}

		ForceToApply = m_Physics.ComputeRestrainedForce(ForceToApply, AccRestraintPositive*Mass, AccRestraintNegative*Mass, dTime_s);

		double TorqueToApply;

		//If we are using set point we don't want the blend of torque high as this will throw off the computations
		//Note: GetMaxTorqueYaw get it as angular acceleration (we should rename it)... so it needs to be multiplied by mass to become torque
		const double Ships_TorqueRestraint = m_LockShipHeadingToOrientation ?
			m_ShipProperties.GetMaxTorqueYaw(std::min(m_Physics.GetAngularVelocity(), props.dHeading)) * Mass :
			m_ShipProperties.GetMaxTorqueYaw_SetPoint(std::min(m_Physics.GetAngularVelocity(), props.dHeading)) * Mass;

		if (m_StabilizeRotation)
		{
			//Here we have the original way to turn the ship
			//Vec3d rotVel=m_Physics.GetVelocityFromDistance_Angular(m_rotDisplacement_rad,Ships_TorqueRestraint,dTime_s);

			//And now the new way using the match velocity, Notice how I divide by time to get the right numbers
			//When this is locked to orientation (e.g. joystick keyboard) this will be ignored since the restraint is -1
			double rotVel;

			if (!m_LockShipHeadingToOrientation)
			{
				double DistanceToUse = m_rotDisplacement_rad;
				//The match velocity needs to be in the same direction as the distance (It will not be if the ship is banking)
				double MatchVel = 0.0;
				//Note using  * m_ShipProperties.GetRotateToScaler(DistanceToUse) is now depreciated, as the set point restraint really needs to be constant
				//we can have a higher torque restraint to deal with overshoot... but keep the rate steady and lower
				rotVel = m_Physics.GetVelocityFromDistance_Angular(DistanceToUse, props.MaxTorqueYaw_SetPoint, dTime_s, MatchVel, !m_LockShipHeadingToOrientation);
			}
			else
				rotVel = m_rotDisplacement_rad;
			//testing stuff  (eventually nuke this)
			//Vec3d rotVel=m_Physics.GetVelocityFromDistance_Angular_v2(m_rotDisplacement_rad,Ships_TorqueRestraint,dTime_s,Vec3d(0,0,0));
			#if 0
			{
				//Vec3d test2=m_Physics.GetVelocityFromDistance_Angular(m_rotDisplacement_rad,Ships_TorqueRestraint,dTime_s);
				Vec3d test = m_Physics.GetVelocityFromDistance_Angular_v2(m_rotDisplacement_rad, Ships_TorqueRestraint, dTime_s, Vec3d(0, 0, 0));
				//DOUT2("%f %f %f",m_rotAccel_rad_s[0],m_rotAccel_rad_s[1],m_rotAccel_rad_s[2]);
				DOUT2("%f %f %f", rotVel[0], test[0], rotVel[0] - test[0]);
				//DOUT2("%f %f %f",test2[0],test[0],test2[0]-test[0]);
				if (fabs(test[0]) >= 0.0001)
					DebugOutput("%f %f %f %f", rotVel[0], test[0], rotVel[0] - test[0], m_rotAccel_rad_s[0]);
			}
			#endif

			//enforce the turning speeds
			#if 1
			//While joystick and keyboard never exceed the speeds, the mouse can... so cap it off here
			//Note: The m_HeadingSpeedScale applies to both yaw and pitch since we perform coordinated turns
			{
				double SpeedRestraint = props.dHeading * m_HeadingSpeedScale;
				double SmallestRatio = 1.0;
				//This works similar to LocalTorque restraints; 
				//This method computes the smallest ratio needed to scale down the vectors.  It should give the maximum amount
				//of magnitude available without sacrificing the intended direction
				{
					double AbsComponent = fabs(rotVel);
					if (AbsComponent > SpeedRestraint)
					{
						double Temp = SpeedRestraint / AbsComponent;
						SmallestRatio = Temp < SmallestRatio ? Temp : SmallestRatio;
					}
				}
				rotVel *= SmallestRatio;
			}
			#endif
			//printf("\r%f %f            ",m_rotDisplacement_rad,rotVel);
			TorqueToApply = m_Physics.GetTorqueFromVelocity(rotVel, dTime_s);
			#if 0
			if (fabs(rotVel) > 0.0)
				printf("v=%.2f ", TorqueToApply / Ships_TorqueRestraint);
			#endif
		}
		else
			TorqueToApply = m_rotAccel_rad_s * Mass;


		//To be safe we reset this to zero (I'd put a critical section around this line of code if there are thread issues
		m_rotDisplacement_rad = 0.0;

		ApplyThrusters(m_Physics, ForceToApply, TorqueToApply, Ships_TorqueRestraint, dTime_s);
		#if 0
		{
			double Torque = m_Physics.ComputeRestrainedTorque(TorqueToApply, Ships_TorqueRestraint, dTime_s);
			DOUT4("%.3f %.3f %.3f", ForceToApply[0] / Mass, ForceToApply[1] / Mass, Torque / Mass);
			//DOUT4("%.3f %.3f %.3f",m_currAccel[0],m_currAccel[1],TorqueToApply/Mass);
		}
		#endif

		// Now to run the time updates (displacement plus application of it)
		//GetPhysics().G_Dampener = G_Dampener;
		//__super::TimeChange(dTime_s);

		m_controller->UpdateUI(dTime_s);

		#ifdef _TestNoIndendedDirction_properties__
		if (m_LockShipHeadingToOrientation)
			m_IntendedOrientation = GetAtt_r();
		#endif

		//Reset my controller vars
		m_rotAccel_rad_s = 0.0;
		m_currAccel = Vec2D(0, 0);

		//send this velocity to entity if it exists
		if (m_ExternSetVelocity)
		{
			Vec2D LocalVelocity = GlobalToLocal(-GetAtt_r(), m_Physics.GetLinearVelocity());
			//send local velocity
			m_ExternSetVelocity(LocalVelocity);
		}
		if (m_ExternSetHeadingVelocity)
			m_ExternSetHeadingVelocity(m_Physics.GetAngularVelocity());
	}
	~Ship_2D()
	{
		delete m_controller;
		m_controller = NULL;
	}
	void Stop() 
	{ 
		///This implicitly will place back in auto mode with a speed of zero
		SetRequestedVelocity(0.0); 
	}
	void FireAfterburner() 
	{ 
		SetRequestedVelocity(GetMaxSpeed()); 
	}
	void ResetPos()
	{
		// Places the ship back at its initial position and resets all vectors
		//Setting this to true allows starting from an arbitrary position and it not auto adjust to the intended orientation
		//for AI and Remote... they will implicitly clear this once the send the first intended orientation
		m_LockShipHeadingToOrientation = true;
		m_RequestedVelocity[0] = m_RequestedVelocity[1] = 0.0;
		//m_Last_AccDel = 0.0;
		m_Last_RequestedVelocity[0] = m_Last_RequestedVelocity[1] = -1.0;
		m_rotAccel_rad_s = m_rotDisplacement_rad = 0.0;
		m_currAccel = Vec2D(0, 0);
		m_IntendedOrientation = GetAtt_r();
		m_IntendedOrientationPhysics.ResetVectors();
		SetStabilizeRotation(true); //This should always be true unless there is some ship failure
		//SetStabilizeRotation(false); //This is for testing
		SetSimFlightMode(true);  //This one is a tough call... probably should do it on reset
		m_RotationToleranceCounter = 0;
	}
	#pragma region _mutators_
	void SetRequestedVelocity(double Velocity)
	{
		//assert(IsLocallyControlled());
		SetSimFlightMode(true);
		if (Velocity > 0.0)
			m_RequestedVelocity[1] = MIN(Velocity, GetMaxSpeed());
		else
			m_RequestedVelocity[1] = MAX(Velocity, -GetMaxSpeed());
		m_RequestedVelocity[0] = 0;
	}
	void SetRequestedVelocity(Vec2D Velocity)
	{
		//Note: with 2D submissions we do not need to manually set sim mode 
		//assert(IsLocallyControlled());
		m_RequestedVelocity = Velocity;  //I'm not doing speed checking for this one (this will be checked later anyhow)
		//DOUT5("%f %f",m_RequestedVelocity[0],m_RequestedVelocity[1]);
	}
	void SetCurrentLinearAcceleration(const Vec2D &Acceleration)
	{
		m_currAccel = Acceleration;
	}
	void SetCurrentAngularAcceleration(double Acceleration, bool LockShipHeadingToOrientation)
	{
		/// \param LockShipHeadingToOrientation for this given time slice if this is true the intended orientation is restrained
		/// to the ships restraints and the ship is locked to the orientation (Joy/Key mode).  If false (Mouse/AI) the intended orientation
		/// is not restrained and the ship applies its restraints to catch up to the orientation
		m_LockShipHeadingToOrientation = LockShipHeadingToOrientation;
		m_rotAccel_rad_s = Acceleration;
	}
	void SetIntendedOrientation(double IntendedOrientation, bool Absolute = true)
	{
		///This is used by AI controller (this will have LockShipHeadingToOrientation set to false)
		///This allows setting the desired heading directly either relative to the current heading or absolute
		m_LockShipHeadingToOrientation = false; //this must be false for this to work (if not already)
		m_IntendedOrientation = Absolute ? IntendedOrientation : NormalizeRotation2(m_IntendedOrientation + IntendedOrientation);
	}
	void SetSimFlightMode(bool SimFlightMode)
	{
		//It seems that some people want/need to call this function repeatedly so I have included a valve branch here to prevent the debug flooding
		//And to not do extra work on the m_RequestedVelocity.
		if (m_SimFlightMode != SimFlightMode)
		{
			//Vec2D LocalVelocity=GlobalToLocal(GetAtt_r(),m_Physics.GetLinearVelocity());
			//m_RequestedVelocity=LocalVelocity;
			//unfortunately a slide turn maneuver requires this, but fortunately it is for UI.  This is not perfect if the user intended to go backwards
			//but that would not be something desirable
			m_RequestedVelocity = GlobalToLocal(GetAtt_r(), m_Physics.GetLinearVelocity());
			m_SimFlightMode = SimFlightMode;
			//DebugOutput("SimFlightMode=%d\n",SimFlightMode);
		}
	}
	#pragma endregion
	#pragma region _accessors_
	const Vec2D GetPos_m() const
	{
		if (m_ExternGetCurrentPosition)
			return m_ExternGetCurrentPosition();
		else
			return m_current_position;
	}
	double GetAtt_r() const
	{
		{
			if (m_ExternGetCurrentHeading)
				return m_ExternGetCurrentHeading();
			else
				return m_current_heading;
		}
	}
	double GetRequestedSpeed() const
	{
		//Note:This has some direction so technically it is also velocity
		return m_RequestedVelocity[1];
	}
	Vec2D GetRequestedVelocity() const
	{
		return m_RequestedVelocity;
	}
	const double &GetIntendedOrientation() const
	{
		/// This is where both the vehicle entity and camera need to align to
		return m_IntendedOrientation;
	}
	Vec2D GetLinearVelocity_ToDisplay() const
	{ 
		//Override to get sensor/encoder's real velocity
		return GlobalToLocal(GetAtt_r(), GetPhysics().GetLinearVelocity()); 
	}
	double GetAngularVelocity_ToDisplay() const
	{ 
		return GetPhysics().GetAngularVelocity(); 
	}
	const Ship_Properties &GetShipProperties() const
	{ 
		return m_ShipProperties;
	}
	#pragma endregion
	#pragma endregion
};

#pragma endregion

#pragma region _helper functions_
__inline MotionControl2D::Vector2D as_vector_2d(Vec2D input)
{
	MotionControl2D::Vector2D ret = { input[0],input[1] };
	return ret;
}

__inline  Vec2D as_Vec2D(MotionControl2D::Vector2D input)
{
	Vec2D ret(input.x, input.y);
	return ret;
}
#pragma endregion
#pragma region _Simple fallback case_
#ifdef __UseSimpleFallback__
#pragma region _Motion Control 2D Internal_


class MotionControl2D_internal
{
private:
	#pragma region _member varibles_
	using properties = MotionControl2D::properties;
	//provide some meaningful defaults
	properties m_properties=
	{
		Feet2Meters(12.0),  //meters per second of fastest speed available
		Pi, //radians fastest turn rate
		5,  //max acceleration meters per second square
		5,  //these may be different since we may need to stop quicker
		Pi2  //for both acceleration and deceleration
	};

	Vec2D m_requested_local;
	Vec2D m_requested_global;
	Vec2D m_current_velocity;
	Vec2D m_requested_position;
	Vec2D m_current_position;
	double m_requested_heading=0.0;  //always absolute relative method solves on the spot
	double m_requested_angular_velocity=0.0;
	double m_current_angular_velocity = 0.0;
	double m_current_heading=0.0;
	double m_driven_max_speed=0.0;
	//keep note of last modes
	bool m_is_linear_driven = false;
	bool m_is_angular_driven = false;
	bool m_is_velocity_global = false;
	bool m_stop_at_waypoint = true;
	bool m_can_strafe = true;

	std::function<void(const MotionControl2D::Vector2D &new_velocity)> m_ExternSetVelocity=nullptr;
	std::function<void(double new_velocity)> m_ExternSetHeadingVelocity=nullptr;
	std::function <MotionControl2D::Vector2D()> m_ExternGetCurrentPosition=nullptr;
	std::function <double()> m_ExternGetCurrentHeading=nullptr;

	#pragma endregion
	void process_slice(double d_time_s)
	{
		//apply linear velocity first, then angular

		//For linear determine if we are driving or driven
		if (m_is_linear_driven == false)
		{
			//First break apart current velocity into magnitude and direction
			Vec2D normlized_current = m_current_velocity;
			const double current_magnitude = normlized_current.normalize();  //observe for diagnostics
			const Vec2D DesiredVelocity =(m_is_velocity_global)? m_requested_global : LocalToGlobal(GetCurrentHeading(),m_requested_local);
			//break this apart
			Vec2D normalized_request = DesiredVelocity;
			double request_magnitude = normalized_request.normalize();  //always use this direction
			//This shouldn't be needed but just in case
			request_magnitude = std::max(std::min(request_magnitude, m_properties.max_speed_linear), 0.0);
			//use current direction if the requested magnitude is zero (because there is no direction then)
			if (request_magnitude == 0.0)
				normalized_request = normlized_current;
			double adjusted_magnitude = current_magnitude;
			//adjust the velocity to match request
			const double acc_increment = (m_properties.max_acceleration_linear * d_time_s);
			const double dec_increment = -(m_properties.max_deceleration_linear * d_time_s);
			//TODO while a 1D I can get perfect handling of switching direction stresses, I can't do that here
			//for now I'll let this go as it will do the correct math most of the time, and will address this
			//again once I start working with feedback
			//Check if we can go a full increment in either direction
			if (current_magnitude + acc_increment < request_magnitude)
				 adjusted_magnitude += acc_increment; //accelerate and clip as needed
			else if (current_magnitude + dec_increment > request_magnitude)
				 adjusted_magnitude += dec_increment; //decelerate and clip as needed
			else if (fabs(current_magnitude - request_magnitude) < acc_increment)
			{
				//we are close enough to the requested to just become it
				adjusted_magnitude = request_magnitude;
			}

			//now update the current velocity
			m_current_velocity = normalized_request * adjusted_magnitude;
			//send this velocity to entity if it exists
			if (m_ExternSetVelocity)
				m_ExternSetVelocity(as_vector_2d(m_current_velocity));
			//If we don't have the callbacks for both set and get... we'll update our own default implementation
			if ((!m_ExternGetCurrentPosition) || (!m_ExternSetVelocity))
			{
				//from current velocity we can update the position
				m_current_position += m_current_velocity * d_time_s;
			}
		}
		//TODO way point here

		//For angular determine if we are driving or driven
		const bool is_waypoint_managing_heading = m_is_linear_driven && m_can_strafe == false;
		if (!is_waypoint_managing_heading)
		{
			if (m_is_angular_driven == false)
			{
				//Very similar trapezoidal profile as with the velocity except we are 1D, so the direction is easier to manage
				//First break apart current heading into magnitude and direction
				const double current_magnitude = fabs(m_current_angular_velocity);
				const double current_direction = current_magnitude>0.0?current_magnitude / m_current_angular_velocity:0.0;  //order does not matter
				double request_magnitude = fabs(m_requested_angular_velocity);
				double direction_to_use = request_magnitude>0.0? request_magnitude / m_requested_angular_velocity : 0.0;
				//This shouldn't be needed but just in case (must be after direction_to_use)
				request_magnitude = std::max(std::min(request_magnitude, m_properties.max_speed_linear), 0.0);
				//use current direction if the requested magnitude is zero (because there is no direction then)
				if (request_magnitude == 0.0)
					direction_to_use = current_direction;
				double adjusted_magnitude = current_magnitude;
				//adjust the velocity to match request
				const double acc_increment = (m_properties.max_acceleration_angular * d_time_s);
				const double dec_increment = -(m_properties.max_acceleration_angular * d_time_s);
				//Check the signs
				if (current_direction * direction_to_use >= 0.0)
				{
					//Check if we can go a full increment in either direction
					if (current_magnitude + acc_increment < request_magnitude)
						adjusted_magnitude += acc_increment; //accelerate and clip as needed
					else if (current_magnitude + dec_increment > request_magnitude)
						adjusted_magnitude += dec_increment; //decelerate and clip as needed
					else if (fabs(current_magnitude - request_magnitude) < acc_increment)
					{
						//we are close enough to the requested to just become it
						adjusted_magnitude = request_magnitude;
					}
				}
				else
				{
					//signs are different, so the checking is as well
					//To keep this easy to debug we'll preserve current_magnitude, and create a combined one
					const double combined_magnitude = current_magnitude + request_magnitude;
					if (combined_magnitude < acc_increment)
						adjusted_magnitude = request_magnitude;  //close enough; otherwise...
					else
					{
						adjusted_magnitude += dec_increment;  //always decelerate when going towards opposite sign
						direction_to_use = current_direction;  //always keep the sign
					}
				}

				//now update the current velocity,  Note: this can go beyond the boundary of 2 pi
				m_current_angular_velocity = direction_to_use * adjusted_magnitude;
				//send this velocity to entity if it exists
				if (m_ExternSetHeadingVelocity)
					m_ExternSetHeadingVelocity(m_current_angular_velocity);
				//If we don't have the callbacks for both set and get... we'll update our own default implementation
				if ((!m_ExternGetCurrentHeading)||(!m_ExternSetHeadingVelocity))
				{
					//from current velocity we can update the position
					m_current_heading += m_current_angular_velocity * d_time_s;
					//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
					m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging

				}
			}
			else
			{
			}
		}
	}
public:
	void TimeSlice(double d_time_s)
	{
		process_slice(d_time_s);  //keep public small and compact
	}

	void SetProperties(const properties &props)
	{
		m_properties = props;
	}
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_requested_local = m_requested_global = m_current_velocity = Vec2D(0.0, 0.0);
		m_current_position[0] = X;
		m_current_position[1] = Y;
		m_requested_heading = m_requested_angular_velocity = m_current_angular_velocity=0.0;
		m_current_heading = heading;
		//optional update last modes for consistency
		m_is_linear_driven = m_is_angular_driven = m_is_velocity_global = false;
	}

	//Tele-op methods:-----------------------------------------------------------------------
	void SetLinearVelocity_local(double forward, double right)
	{
		//for ease of debugging, keep this in local here (translate in time slice)
		m_requested_local[0] = right;
		m_requested_local[1] = forward;
	}
	void SetLinearVelocity_global(double north, double east)
	{
		m_requested_global[0] = east;
		m_requested_global[1] = north;
	}
	void SetAngularVelocity(double clockwise)
	{
		m_requested_angular_velocity = clockwise;
		m_is_angular_driven = false;
	}
	//AI methods: ---------------------------------------------------------------------------
	void SetIntendedOrientation(double intended_orientation, bool absolute = true)
	{
		m_requested_heading = absolute?intended_orientation:GetCurrentHeading()+intended_orientation;
		m_is_angular_driven = true;
	}
	void DriveToLocation(double north, double east, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true)
	{
		m_requested_position[0] = east, m_requested_position[1] = north;
		m_stop_at_waypoint = stop_at_destination;
		m_driven_max_speed = max_speed;
		m_can_strafe = can_strafe;
	}
	#pragma region _Accessors_
	//Accessors:---------------------------------------------
	bool GetIsDrivenLinear() const
	{
		return m_is_linear_driven;
	}
	bool GetIsVelocityGlobal() const
	{
		return m_is_velocity_global;
	}
	bool GetIsDrivenAngular() const
	{
		return m_is_angular_driven;
	}
	Vec2D Get_SetVelocity_local() const
	{
		return m_requested_local;
	}
	Vec2D Get_SetVelocity_global() const
	{
		return m_requested_global;
	}
	double Get_IntendedOrientation() const
	{
		return m_requested_heading;
	}
	Vec2D GetCurrentVelocity() const
	{
		return m_current_velocity;
	}
	Vec2D GetCurrentPosition() const
	{
		if (m_ExternGetCurrentPosition)
			return as_Vec2D(m_ExternGetCurrentPosition());
		else
			return m_current_position;
	}
	double GetCurrentHeading() const
	{
		if (m_ExternGetCurrentHeading)
			return m_ExternGetCurrentHeading();
		else
			return m_current_heading;
	}
	double GetCurrentAngularVelocity() const
	{
		return m_current_angular_velocity;
	}
	#pragma endregion
	#pragma region _Callbacks_
	//Optional Linker callbacks:
	void Set_UpdateGlobalVelocity(std::function<void(const MotionControl2D::Vector2D &new_velocity)> callback)
	{
		m_ExternSetVelocity = callback;
	}
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
	{
		m_ExternSetHeadingVelocity = callback;
	}
	void Set_GetCurrentPosition(std::function <MotionControl2D::Vector2D()> callback)
	{
		m_ExternGetCurrentPosition = callback;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_ExternGetCurrentHeading = callback;
	}
	#pragma endregion
};
#pragma endregion
#pragma region _wrapper methods_
MotionControl2D::MotionControl2D()
{
	m_MotionControl2D = std::make_shared<MotionControl2D_internal>();
}
void MotionControl2D::SetProperties(const properties &props)
{
	m_MotionControl2D->SetProperties(props);
}
void MotionControl2D::TimeSlice(double d_time_s)
{
	m_MotionControl2D->TimeSlice(d_time_s);
}
void MotionControl2D::Reset(double X, double Y, double heading)
{
	m_MotionControl2D->Reset(X, Y, heading);
}

//Tele-op methods:-----------------------------------------------------------------------
void MotionControl2D::SetLinearVelocity_local(double forward, double right)
{
	m_MotionControl2D->SetLinearVelocity_local(forward, right);
}
void MotionControl2D::SetLinearVelocity_global(double north, double east)
{
	m_MotionControl2D->SetLinearVelocity_global(north, east);
}
void MotionControl2D::SetAngularVelocity(double clockwise)
{
	m_MotionControl2D->SetAngularVelocity(clockwise);
}
//AI methods: ---------------------------------------------------------------------------
void MotionControl2D::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_MotionControl2D->SetIntendedOrientation(intended_orientation, absolute);
}
void MotionControl2D::DriveToLocation(double north, double east, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_MotionControl2D->DriveToLocation(north, east, stop_at_destination, max_speed, can_strafe);
}

//Accessors:---------------------------------------------
bool MotionControl2D::GetIsDrivenLinear() const
{
	return m_MotionControl2D->GetIsDrivenLinear();
}
bool MotionControl2D::GetIsVelocityGlobal() const
{
	return m_MotionControl2D->GetIsVelocityGlobal();
}
bool MotionControl2D::GetIsDrivenAngular() const
{
	return m_MotionControl2D->GetIsDrivenAngular();
}

MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_local() const
{
	return as_vector_2d(m_MotionControl2D->Get_SetVelocity_local());
}
MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_global() const
{
	return as_vector_2d(m_MotionControl2D->Get_SetVelocity_global());
}
double MotionControl2D::Get_IntendedOrientation() const
{
	return m_MotionControl2D->Get_IntendedOrientation();
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentVelocity() const
{
	return as_vector_2d(m_MotionControl2D->GetCurrentVelocity());
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentPosition() const
{
	return as_vector_2d(m_MotionControl2D->GetCurrentPosition());
}
double MotionControl2D::GetCurrentHeading() const
{
	return m_MotionControl2D->GetCurrentHeading();
}
double MotionControl2D::GetCurrentAngularVelocity() const
{
	return m_MotionControl2D->GetCurrentAngularVelocity();
}
//Optional Linker callbacks:
void MotionControl2D::Set_UpdateGlobalVelocity(std::function<void(const Vector2D &new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateGlobalVelocity(callback);
}
void MotionControl2D::Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateHeadingVelocity(callback);
}
void MotionControl2D::Set_GetCurrentPosition(std::function <Vector2D()> callback)
{
	m_MotionControl2D->Set_GetCurrentPosition(callback);
}
void MotionControl2D::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_MotionControl2D->Set_GetCurrentHeading(callback);
}

#pragma endregion
#endif
#pragma endregion
#pragma region _Using Ship case_
#ifndef __UseSimpleFallback__

//Using inheritance to avoid renaming the class, also can adapt methods to fit here
class MotionControl2D_internal : public Ship_2D
{
private:
	std::function<void(const MotionControl2D::Vector2D &new_velocity)> m_ExternSetVelocity_AsVector2D = nullptr;
	std::function <MotionControl2D::Vector2D()> m_ExternGetCurrentPosition_AsVector2D = nullptr;

public:
	using properties = MotionControl2D::properties;
	void Reset(double X, double Y, double heading)
	{
		//TODO support parms
		ResetPos();
	}
	void SetLinearVelocity_local(double forward, double right)
	{
		SetRequestedVelocity(GlobalToLocal(GetAtt_r(), Vec2D(right, forward)));
	}
	void SetProperties(const properties &props)
	{
		//TODO translate here
	}
	void DriveToLocation(double north, double east, bool stop_at_destination, double max_speed, bool can_strafe)
	{
		//TODO route to controller
	}
	bool GetIsDrivenLinear() const
	{
		//Note: auto pilot was managed in the UI controller, so we'll have to work out how to manage here
		return false;
	}
	bool GetIsDrivenAngular() const
	{
		return !m_LockShipHeadingToOrientation;
	}
	Vec2D Get_SetVelocity_local() const
	{
		return GlobalToLocal(GetAtt_r(), GetRequestedVelocity());
	}
	#pragma region _Callbacks_
	//Optional Linker callbacks:
	void Set_UpdateGlobalVelocity(std::function<void(const MotionControl2D::Vector2D &new_velocity)> callback)
	{
		m_ExternSetVelocity_AsVector2D = callback;
		//do the conversion
		if (callback)
		{
			m_ExternSetVelocity =
				[&](const Vec2D &new_velocity)
			{
				m_ExternSetVelocity_AsVector2D(as_vector_2d(new_velocity));
			};
		}
		else
			m_ExternSetVelocity = nullptr;
	}
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
	{
		m_ExternSetHeadingVelocity = callback;
	}
	void Set_GetCurrentPosition(std::function <MotionControl2D::Vector2D()> callback)
	{
		m_ExternGetCurrentPosition_AsVector2D = callback;
		//do the conversion
		if (callback)
		{
			m_ExternGetCurrentPosition =
				[&]()
			{
				return as_Vec2D(m_ExternGetCurrentPosition_AsVector2D());
			};
		}
		else
			m_ExternGetCurrentPosition = nullptr;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_ExternGetCurrentHeading = callback;
	}
	#pragma endregion

};

#pragma region _wrapper methods_
MotionControl2D::MotionControl2D()
{
	m_MotionControl2D = std::make_shared<MotionControl2D_internal>();
}
void MotionControl2D::SetProperties(const properties &props)
{
	m_MotionControl2D->SetProperties(props);
}
void MotionControl2D::TimeSlice(double d_time_s)
{
	m_MotionControl2D->TimeChange(d_time_s);
}
void MotionControl2D::Reset(double X, double Y, double heading)
{
	m_MotionControl2D->Reset(X, Y, heading);
}

//Tele-op methods:-----------------------------------------------------------------------
void MotionControl2D::SetLinearVelocity_local(double forward, double right)
{
	m_MotionControl2D->SetLinearVelocity_local(forward, right);
}
void MotionControl2D::SetLinearVelocity_global(double north, double east)
{
	m_MotionControl2D->SetRequestedVelocity(Vec2D(north, east));
}
void MotionControl2D::SetAngularVelocity(double clockwise)
{
	m_MotionControl2D->SetCurrentAngularAcceleration(clockwise,true);
}
//AI methods: ---------------------------------------------------------------------------
void MotionControl2D::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_MotionControl2D->SetIntendedOrientation(intended_orientation, absolute);
}
void MotionControl2D::DriveToLocation(double north, double east, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_MotionControl2D->DriveToLocation(north, east, stop_at_destination, max_speed, can_strafe);
}

//Accessors:---------------------------------------------
bool MotionControl2D::GetIsDrivenLinear() const
{
	return m_MotionControl2D->GetIsDrivenLinear();
}
bool MotionControl2D::GetIsVelocityGlobal() const
{
	return true;
}
bool MotionControl2D::GetIsDrivenAngular() const
{
	return m_MotionControl2D->GetIsDrivenAngular();
}

MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_local() const
{
	return as_vector_2d(m_MotionControl2D->Get_SetVelocity_local());
}
MotionControl2D::Vector2D MotionControl2D::Get_SetVelocity_global() const
{
	return as_vector_2d(m_MotionControl2D->GetRequestedVelocity());
}
double MotionControl2D::Get_IntendedOrientation() const
{
	return m_MotionControl2D->GetIntendedOrientation();
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentVelocity() const
{
	return as_vector_2d(m_MotionControl2D->GetLinearVelocity_ToDisplay());
}
MotionControl2D::Vector2D MotionControl2D::GetCurrentPosition() const
{
	return as_vector_2d(m_MotionControl2D->GetPos_m());
}
double MotionControl2D::GetCurrentHeading() const
{
	return m_MotionControl2D->GetAtt_r();
}
double MotionControl2D::GetCurrentAngularVelocity() const
{
	return m_MotionControl2D->GetAngularVelocity_ToDisplay();
}
//Optional Linker callbacks:
void MotionControl2D::Set_UpdateGlobalVelocity(std::function<void(const Vector2D &new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateGlobalVelocity(callback);
}
void MotionControl2D::Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
{
	m_MotionControl2D->Set_UpdateHeadingVelocity(callback);
}
void MotionControl2D::Set_GetCurrentPosition(std::function <Vector2D()> callback)
{
	m_MotionControl2D->Set_GetCurrentPosition(callback);
}
void MotionControl2D::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_MotionControl2D->Set_GetCurrentHeading(callback);
}

#pragma endregion
#endif
#pragma endregion
}}}