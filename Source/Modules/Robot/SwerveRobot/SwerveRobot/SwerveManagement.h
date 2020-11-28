#pragma once
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace Module {
	namespace Robot {
class SwerveManagement
{
private:
	SwerveVelocities m_Velocity;
	std::function< SwerveVelocities ()> m_OdometryCallback=nullptr;

	inline double NormalizeRotation_HalfPi(double Orientation)
	{
		if (Orientation > PI_2)
			Orientation -= M_PI;
		else if (Orientation < -PI_2)
			Orientation += M_PI;
		return Orientation;
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
	//TODO: we may want to bind these to properties
	//each module may have it's own limited range and unique to other modules, because of how the potentiometer
	//can be assembled different in each
	bool GetUsingRange(size_t index) const
	{
		return false;
	}
	double GetMaxRange(size_t index) const
	{
		return 0.0;
	}
	double GetMinRange(size_t index) const
	{
		return 0.0;
	}
public:
	//Input
	void InterpolateVelocities(const SwerveVelocities &Velocities)
	{
		//This adds final management on the velocities and saves the result to m_Velocity
		//There are 2 important problems this method addresses:
		//  1. Provide shortest direction path from where the swivel currently is to where it needs to go
		//  2. Slow down the intended drive velocity or halt to the min of any module who's swivel is not where it is
		//     suppose to be
		//Typically they all take about the same amount of time to get where they are going, but more important the direction
		//of travel should reflect the intended direction at any given moment, so if it cannot go in the direction because
		//any of the wheels swivel is greater than a skid threshold (currently set to 25 degrees) then it shouldn't be moving
		//We allow some skid for better response by allowing a matched velocity of the same component that of the vector
		//that is broken down.  Keeping in mind the goal is to match the intended velocities is top priority, so less movement
		//Is preferred to have better control (and safer) for the drive
		//[9/11/2020 Terminator]
		#if 0
		m_Velocity = Velocities;
		#else
		//Grab odometry if possible; otherwise just use what we have
		const SwerveVelocities encoders= m_OdometryCallback? m_OdometryCallback():Velocities;
		double aSwivelDirection[4];
		double VelocityToUse[4];
		double aDistanceToIntendedSwivel[4];
		bool RestrictMotion=false;  //flagged if any of the wheels do not reach a threshold of their position
		//Now the new UpdateVelocities was just called... work with these intended velocities
		for (size_t i=0;i<4;i++)
		{
			const double IntendedDirection = Velocities.Velocity.AsArray[i+4];
			double SwivelDirection=IntendedDirection;  //this is either the intended direction or the reverse of it
			//const Ship_1D &Swivel=m_DrivingModule[i]->GetSwivel();

			//This is normalized implicitly
			//const double LastSwivelDirection=Swivel.GetPos_m();
			const double LastSwivelDirection=encoders.Velocity.AsArray[i+4];

			double DirectionMultiplier=1.0; //changes to -1 when reversed


			//Anything above 90 might need to be flipped favorably to the least traveled angle
			const double test_forward = fabs(NormalizeRotation2(SwivelDirection - LastSwivelDirection));
			const double test_reverse = fabs(NormalizeRotation2(SwivelDirection + Pi - LastSwivelDirection));

			//This test identifies when the directions are north and south (0 and 180)
			//Where we have a significant amount of difference between the directions (less north and south)
			if (fabs(test_reverse - test_forward) > DEG_2_RAD(5.0))
			{
				//We will not have gimbal lock issues with this logic, works great for east west
				if (test_forward > PI_2)
				{
					if (test_reverse < test_forward)
					{
						const double OtherDirection = NormalizeRotation2(SwivelDirection + Pi);
						SwivelDirection = OtherDirection;
						DirectionMultiplier = -1;
					}
				}
			}
			else
			{
				//Using the legacy logic best suited for North South
				if (fabs(SwivelDirection) > PI_2)
				{
					const double TestOtherDirection = NormalizeRotation_HalfPi(SwivelDirection);
					if (fabs(TestOtherDirection) < fabs(SwivelDirection))
					{
						SwivelDirection = TestOtherDirection;
						DirectionMultiplier = -1;
					}
				}
			}

			//if we are using range... clip to the max range available
			if (GetUsingRange(i))
			{
				if (SwivelDirection>GetMaxRange(i))
					SwivelDirection=GetMaxRange(i);
				else if (SwivelDirection<GetMinRange(i))
					SwivelDirection=GetMinRange(i);
			}

			aSwivelDirection[i]=SwivelDirection;

			//recompute as SwivelDirection may be reduced
			const double DistanceToIntendedSwivel=fabs(NormalizeRotation2(LastSwivelDirection-SwivelDirection));
			aDistanceToIntendedSwivel[i]=DistanceToIntendedSwivel;  //export to other loop

			//Note the velocity is checked once before the time change here, and once after for the current
			//Only apply swivel adjustments if we have significant movement (this matters in targeting tests)
			//if ((fabs(LocalForce[0])>1.5)||(fabs(LocalForce[1])>1.5)||(fabs(m_DrivingModule[i]->GetDrive().GetPhysics().GetVelocity()) > 0.05))
			//This logic fails when driving a direct strafe from a stop

			const double IntendedSpeed= Velocities.Velocity.AsArray[i]*DirectionMultiplier;

			//To minimize error only apply the Y component amount to the velocity
			//The less the difference between the current and actual swivel direction the greater the full amount can be applied
			//On slower swerve drive mechanisms a check for a threshold of distance is included to minimize skid
			VelocityToUse[i]=sin(DistanceToIntendedSwivel)<DEG_2_RAD(25.0)?cos(DistanceToIntendedSwivel)*IntendedSpeed:0.0;
			if (sin(DistanceToIntendedSwivel)>=DEG_2_RAD(25.0))
				RestrictMotion=true;
			#if 0
			string sm_name="a2_";
			const char * const sm_name_suffix[]={"FL","FR","RL","RR"};
			sm_name+=sm_name_suffix[i];
			SmartDashboard::PutNumber(sm_name.c_str(),RAD_2_DEG(SwivelDirection));
			if (i==0)
			{
				SmartDashboard::PutNumber("TestPredicted",RAD_2_DEG(LastSwivelDirection));
				SmartDashboard::PutNumber("TestActual",RAD_2_DEG(encoders.Velocity.AsArray[i+4]));
				SmartDashboard::PutNumber("TestDistance",RAD_2_DEG(sin(DistanceToIntendedSwivel)));
				SmartDashboard::PutNumber("TestVTU",Meters2Feet(VelocityToUse));
			}
			#endif
		}

		//Note: This is more conservative and is favorable for autonomous navigation, but may not be favorable for teleop as it restricts movement
		if (RestrictMotion)
			for (size_t i=0;i<4;i++)
				VelocityToUse[i]=0.0;

		for (size_t i=0;i<4;i++)
		{
			const double IntendedDirection= Velocities.Velocity.AsArray[i+4];
			m_Velocity.Velocity.AsArray[i + 4] = aSwivelDirection[i];
			#ifdef __DebugLUA__
			if (m_SwerveProperties.GetRotaryProps(i).GetRotaryProps().PID_Console_Dump && (m_RobotControl->GetRotaryCurrentPorV(i)!=0.0))
			{
				double PosY=GetPos_m()[1];
				printf("y=%.2f ",PosY);
			}
			#endif

			m_Velocity.Velocity.AsArray[i] = VelocityToUse[i];
			#if 1
			//This is kind of a hack, but since there a threshold on the angular distance tolerance... for open loop we can evaluate when the intended direction is straight ahead and
			//if the current position distance is substantially small then lock it to zero.  This is no impact on closed loop which can solve this using 'I' in PID, and has no impact
			//on Nona drive
			//  [9/9/2012 Terminator]
			if ((m_OdometryCallback==nullptr) && (IntendedDirection==0.0) && (aDistanceToIntendedSwivel[i]<0.005))
				m_Velocity.Velocity.AsArray[i+4]=0.0;
			#endif
		}
		#endif
	}
	//Output
	const SwerveVelocities &GetIntendedVelocities() const
	{
		return m_Velocity;
	}
	//Thru_put
	const SwerveVelocities &GetOdometryVelocities() const
	{
		if (m_OdometryCallback)
			return m_OdometryCallback();
		else
		{
			return m_Velocity;
		}
	}

	//Odometry callback of each wheel module
	void SetOdometryCallback(std::function< SwerveVelocities ()> callback)
	{
		m_OdometryCallback = callback;
	}
};
	}
}