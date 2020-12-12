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

#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
#include "../../MotionControl2D_simple/MotionControl2D/MotionControl2D.h"
#include "../../MotionControl2D_physics/MotionControl2D_physics/MotionControl2D.h"
#include "SwerveManagement.h"
#include "RotarySystem.h"
#include "SimulatedOdometry.h"
#include "OdometryManager.h"
#include "SwerveRobot.h"

//#define __UseSimpleMotionControl__
#pragma endregion

namespace Module {
	namespace Robot {

class SwerveRobot_Internal
{
private:
	#pragma region _member variables_
	//going down the list of each object to the next
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	#ifdef __UseSimpleMotionControl__
	Simple::MotionControl2D m_MotionControl2D;
	#else
	Physics::MotionControl2D m_MotionControl2D;
	#endif
	Swerve_Drive m_robot;  //keep track of our intended velocities... kinematics
	SwerveManagement m_swerve_mgmt;
	RotarySystem_Position m_Swivel[4];
	RotarySystem_Velocity m_Drive[4];
	SimulatedOdometry m_Simulation;
	OdometryManager m_Odometry;
	Inv_Swerve_Drive m_Entity_Input;

	SwerveVelocities m_Voltage;
	//For entity info if not external
	Vec2D m_current_position;
	Vec2D m_current_velocity;
	double m_current_angular_velocity=0.0;
	double m_current_heading = 0.0;

	std::function<void(const Vec2D &new_velocity)> m_ExternSetVelocity = nullptr;
	std::function<void(double new_velocity)> m_ExternSetHeadingVelocity = nullptr;
	std::function <Vec2D()> m_ExternGetCurrentPosition = nullptr;
	std::function <double()> m_ExternGetCurrentHeading = nullptr;

	#pragma endregion
	void SetHooks(bool enable)
	{
		#ifdef __UseSimpleMotionControl__
		using namespace Simple;
		#else
		using namespace Physics;
		#endif

		//Oh what fun 22 hooks!
		if (enable)
		{
			//Link the swerve management to the odometry
			m_swerve_mgmt.SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities();
			});
			m_MotionControl2D.Set_GetCurrentHeading(
				[&]()
			{
				return GetCurrentHeading();
			});
			//used for way points and motion control
			m_MotionControl2D.Set_GetCurrentPosition(
				[&]()
			{
				Vec2D postion = GetCurrentPosition();
				MotionControl2D::Vector2D ret = { postion[0],postion[1] };
				return ret;
			});
			m_Odometry.SetOdometryHeadingCallback(
				[&]()
			{
				return GetCurrentHeading();
			});
			m_Odometry.SetOdometryCallback(
				[&]()
			{
				return m_Simulation.GetCurrentVelocities();
			});
			m_Simulation.SetVoltageCallback(
				[&]()
			{
				return m_Voltage;
			});
			#pragma region _Drive hooks_
			//These have to be unrolled unfortunately
			m_Drive[0].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[0] = new_voltage;
			});
			m_Drive[1].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[1] = new_voltage;
			});
			m_Drive[2].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[2] = new_voltage;
			});
			m_Drive[3].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[3] = new_voltage;
			});
			m_Drive[0].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[0];
			});
			m_Drive[1].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[1];
			});
			m_Drive[2].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[2];
			});
			m_Drive[3].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[3];
			});
			#pragma endregion
			#pragma region _Swivel hooks_
			m_Swivel[0].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[4 + 0] = new_voltage;
			});
			m_Swivel[1].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[4 + 1] = new_voltage;
			});
			m_Swivel[2].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[4 + 2] = new_voltage;
			});
			m_Swivel[3].Set_UpdateCurrentVoltage(
				[&](double new_voltage)
			{
				m_Voltage.Velocity.AsArray[4 + 3] = new_voltage;
			});
			m_Swivel[0].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 0];
			});
			m_Swivel[1].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 1];
			});
			m_Swivel[2].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 2];
			});
			m_Swivel[3].SetOdometryCallback(
				[&]()
			{
				return m_Odometry.GetCurrentVelocities().Velocity.AsArray[4 + 3];
			});
			#pragma endregion
		}
		else
		{
			m_swerve_mgmt.SetOdometryCallback(nullptr);
			m_MotionControl2D.Set_GetCurrentHeading(nullptr);
			m_MotionControl2D.Set_GetCurrentPosition(nullptr);
			m_Odometry.SetOdometryHeadingCallback(nullptr);
			m_Odometry.SetOdometryCallback(nullptr);
			m_Simulation.SetVoltageCallback(nullptr);
			for (size_t i = 0; i < 4; i++)
			{
				m_Drive[i].Set_UpdateCurrentVoltage(nullptr);
				m_Drive[i].SetOdometryCallback(nullptr);
				m_Swivel[i].Set_UpdateCurrentVoltage(nullptr);
				m_Swivel[i].SetOdometryCallback(nullptr);
			}
		}
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

public:
	SwerveRobot_Internal()
	{
		SetHooks(true);
	}
	void Shutdown()
	{
		SetHooks(false);
	}
	~SwerveRobot_Internal()
	{
		Shutdown();
	}
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0)
	{
		m_MotionControl2D.Reset(X, Y, heading);
		m_robot.ResetPos();
		m_Entity_Input.ResetPos();
		for (size_t i = 0; i < 4; i++)
		{
			m_Drive[i].Reset();
			m_Swivel[i].Reset();
		}
		m_Voltage = {};
		m_current_position = m_current_velocity = Vec2D(0.0,0.0);
		m_current_heading = 0.0;
	}
	void Init(const Framework::Base::asset_manager *asset_properties)
	{
		//Go ahead and grab all the default properties first, then if we have asset properties fill in the ones we have and pass
		//the asset properties down to children
		//Provide rotary properties
		rotary_properties props_rotary_drive, props_rotary_swivel;
		//grab defaults
		props_rotary_drive.Init();
		props_rotary_swivel.Init();

		//At some point we may want to callback here for external method to set properties
		//if (!external_properties) defaults below
		#pragma region _default properties_
		//setup some robot properties
		using properties = Swerve_Drive::properties;
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);

		//redundant but reserved to be different
		Inv_Swerve_Drive::properties inv_props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		m_Entity_Input.SetProperties(inv_props);
		//Default rotary properties... use what we can from other systems to match these
		//Since the drive rotary system is using linear velocity, we can pull the same characteristics
		{
			//Entity props
			rotary_properties::Entity1D_Props &rw_drv_entity = props_rotary_drive.entity_props;
			rotary_properties::Entity1D_Props &rw_swl_entity = props_rotary_swivel.entity_props;
			rw_drv_entity.m_Mass = rw_swl_entity.m_Mass = 3.0 * 0.453592; //pounds to kilograms
			rw_drv_entity.m_Dimension = rw_swl_entity.m_Dimension = Inches2Meters(6); //wheel diameter... used for RPS to linear conversions
			//Note: for legacy rotary system position control, GetVelocityFromDistance_Angular() is called if m_IsAngular is true
			rw_swl_entity.m_IsAngular = true; //must have for rotary system to properly implement swerve management direction
		}
		{
			rotary_properties::Ship_1D_Props &rw_drv_ship = props_rotary_drive.ship_props;
			rotary_properties::Ship_1D_Props &rw_swl_ship = props_rotary_swivel.ship_props;
			//make use of modern c++ way to populate them...
			rw_swl_ship =
			{
				//double MAX_SPEED;
				//2,  //These match Curivator
				//double MaxSpeed_Forward, MaxSpeed_Reverse;
				//2,-2,
				8,8,-8,  //used in bypass
				//double ACCEL, BRAKE;
				10.0,10.0,
				//double MaxAccelForward, MaxAccelReverse;
				//7.0,7.0, //These match Curivator
				38.0,38.0, //used in bypass
				//double MinRange, MaxRange;
				0.0,0.0,
				//This is used to avoid overshoot when trying to rotate to a heading
				//double DistanceDegradeScalar;
				1.0,
				//bool UsingRange;
				false
			};
			rw_drv_ship =
			{
				//double MAX_SPEED;
				Feet2Meters(12),  //These match motion control
				//double MaxSpeed_Forward, MaxSpeed_Reverse;
				Feet2Meters(12),-Feet2Meters(12),
				//double ACCEL, BRAKE;
				10.0,10.0,
				//double MaxAccelForward, MaxAccelReverse;
				5.0,5.0 //These match motion control
			};
		}
		{
			rotary_properties::Rotary_Props &rw_drv_rotary = props_rotary_drive.rotary_props;
			rotary_properties::Rotary_Props &rw_swl_rotary = props_rotary_swivel.rotary_props;
			rw_drv_rotary.UseAggressiveStop = rw_swl_rotary.UseAggressiveStop = true;
			//Go ahead an close the loop with a default 100 0 25
			//Even for best case simulation this is needed when using the reset button
			//(While wheel angles could be reset as well, it is better they remain compatible to physical case)
			rw_swl_rotary.LoopState = rotary_properties::Rotary_Props::eClosed;
			rw_swl_rotary.PID[0] = 100.0;
			rw_swl_rotary.PID[2] = 25.0;
		}

		#pragma endregion
		//all cases come here to init rotary systems with correct properties
		for (size_t i = 0; i < 4; i++)
		{
			m_Drive[i].Init(i,&props_rotary_drive);
			m_Swivel[i].Init(i+4,&props_rotary_swivel);
		}
		//only disable when rotary is in bypass
		#if 1
		SimulatedOdometry::properties sim_props;
		sim_props.swivel_max_speed[0] = 
			sim_props.swivel_max_speed[1] =
			sim_props.swivel_max_speed[2] = 
			sim_props.swivel_max_speed[3] = 8.0;
		m_Simulation.Init(&sim_props);
		#else
		m_Simulation.Init(nullptr);
		#endif
		Reset();
	}
	void TimeSlice(double d_time_s)
	{
		#ifdef __UseSimpleMotionControl__
		using namespace Simple;
		#else
		using namespace Physics;
		#endif
		using namespace Framework::Base;
		m_MotionControl2D.TimeSlice(d_time_s);
		MotionControl2D::Vector2D intended_global_velocity = m_MotionControl2D.GetCurrentVelocity();
		Vec2D intended_local_velocity = GlobalToLocal(GetCurrentHeading(),
			Vec2D(intended_global_velocity.x,intended_global_velocity.y));
		//break down the motion control to its kinematic components
		m_robot.UpdateVelocities(intended_local_velocity.y(), intended_local_velocity.x(), m_MotionControl2D.GetCurrentAngularVelocity());
		//pass this to the final swerve management
		m_swerve_mgmt.InterpolateVelocities(m_robot.GetIntendedVelocities());
		//our final intended velocities get sent to the input as velocity to the rotary systems
		for (size_t i = 0; i < 4; i++)
		{
			m_Drive[i].SetVelocity(m_swerve_mgmt.GetIntendedVelocities().Velocity.AsArray[i]);
			m_Swivel[i].SetPosition(m_swerve_mgmt.GetIntendedVelocities().Velocity.AsArray[4 + i]);
			//rotary system in place... invoke its time change
			m_Drive[i].TimeSlice(d_time_s);
			m_Swivel[i].TimeSlice(d_time_s);
			//These will hook their updates to here in m_Voltage
		}
		//The simulation already is hooked to m_Voltage its ready to simulate
		//This step is skipped in real robot code as it physically happens instead
		m_Simulation.TimeSlice(d_time_s);
		//Now to update the odometry
		m_Odometry.TimeSlice(d_time_s);
		//We'll go ahead and maintain an internal state of the position and heading even if this gets managed
		//We can bind to our entity here if client supports it
		//externally since it doesn't cost any overhead
		m_Entity_Input.InterpolateVelocities(m_Odometry.GetCurrentVelocities());
		//send this velocity to entity if it exists
		m_current_velocity = LocalToGlobal(GetCurrentHeading(), Vec2d(m_Entity_Input.GetLocalVelocityX(), m_Entity_Input.GetLocalVelocityY()));
		if (m_ExternSetVelocity)
			m_ExternSetVelocity(m_current_velocity);
		m_current_position += m_current_velocity * d_time_s;
		//now for our turning
		m_current_angular_velocity = m_Entity_Input.GetAngularVelocity();
		//send this velocity to entity if it exists
		if (m_ExternSetHeadingVelocity)
			m_ExternSetHeadingVelocity(m_current_angular_velocity);
		//If we don't have the callbacks for both set and get... we'll update our own default implementation
		if ((!m_ExternGetCurrentHeading) || (!m_ExternSetHeadingVelocity))
		{
			//from current velocity we can update the position
			m_current_heading += m_current_angular_velocity * d_time_s;
			//Almost there... the heading needs to be adjusted to fit in the range from -pi2 to pi2
			m_current_heading = NormalizeRotation2(m_current_heading);  //written out for ease of debugging
		}
	}
	#pragma region _mutators_
	void SetLinearVelocity_local(double forward, double right)
	{
		m_MotionControl2D.SetLinearVelocity_local(forward, right);
	}
	void SetAngularVelocity(double clockwise)
	{
		m_MotionControl2D.SetAngularVelocity(clockwise);
	}

	void SetIntendedOrientation(double intended_orientation, bool absolute = true)
	{
		m_MotionControl2D.SetIntendedOrientation(intended_orientation, absolute);
	}
	void DriveToLocation(double north, double east, bool absolute = true, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true)
	{
		m_MotionControl2D.DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
	}
	#pragma endregion
	#pragma region _accessors_
	//accessors
	Vec2D GetCurrentPosition() const
	{
		if (m_ExternGetCurrentPosition)
			return m_ExternGetCurrentPosition();
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
	const SwerveVelocities &GetCurrentVelocities() const
	{
		return m_Odometry.GetCurrentVelocities();
	}
	const SwerveVelocities &GetCurrentVoltages() const
	{
		return m_Voltage;
	}
	const SwerveVelocities &GetIntendedVelocities() const
	{
		return m_robot.GetIntendedVelocities();
	}
	bool GetIsDrivenLinear() const
	{
		return m_MotionControl2D.GetIsDrivenLinear();
	}
	bool GetIsDrivenAngular() const
	{
		return m_MotionControl2D.GetIsDrivenAngular();
	}
	double Get_IntendedOrientation() const
	{
		return m_MotionControl2D.Get_IntendedOrientation();
	}

	#pragma endregion
	#pragma region _callbacks_
	void Set_UpdateGlobalVelocity(std::function<void(const Vec2D &new_velocity)> callback)
	{
		m_ExternSetVelocity = callback;
	}
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
	{
		m_ExternSetHeadingVelocity = callback;
	}
	void Set_GetCurrentPosition(std::function <Vec2D()> callback)
	{
		m_ExternGetCurrentPosition = callback;
	}
	void Set_GetCurrentHeading(std::function <double()> callback)
	{
		m_ExternGetCurrentHeading = callback;
	}
	#pragma endregion
};
#pragma region _wrapper methods_
SwerveRobot::SwerveRobot()
{
	m_SwerveRobot = std::make_shared<SwerveRobot_Internal>();
}
void SwerveRobot::Init(const Framework::Base::asset_manager* asset_properties)
{
	m_SwerveRobot->Init(asset_properties);
}
void SwerveRobot::Shutdown()
{
	m_SwerveRobot->Shutdown();
}
void SwerveRobot::SetLinearVelocity_local(double forward, double right)
{
	m_SwerveRobot->SetLinearVelocity_local(forward, right);
}
void SwerveRobot::SetAngularVelocity(double clockwise)
{
	m_SwerveRobot->SetAngularVelocity(clockwise);
}
void SwerveRobot::SetIntendedOrientation(double intended_orientation, bool absolute)
{
	m_SwerveRobot->SetIntendedOrientation(intended_orientation, absolute);
}
void SwerveRobot::DriveToLocation(double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
{
	m_SwerveRobot->DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
}
void SwerveRobot::TimeSlice(double d_time_s)
{
	m_SwerveRobot->TimeSlice(d_time_s);
}
void SwerveRobot::Reset(double X, double Y, double heading)
{
	m_SwerveRobot->Reset(X, Y, heading);
}
Vec2D SwerveRobot::GetCurrentPosition() const
{
	return m_SwerveRobot->GetCurrentPosition();
}
double SwerveRobot::GetCurrentHeading() const
{
	return m_SwerveRobot->GetCurrentHeading();
}
void SwerveRobot::Set_UpdateGlobalVelocity(std::function<void(const Vec2D &new_velocity)> callback)
{
	m_SwerveRobot->Set_UpdateGlobalVelocity(callback);
}
void SwerveRobot::Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback)
{
	m_SwerveRobot->Set_UpdateHeadingVelocity(callback);
}
void SwerveRobot::Set_GetCurrentPosition(std::function <Vec2D()> callback)
{
	m_SwerveRobot->Set_GetCurrentPosition(callback);
}
void SwerveRobot::Set_GetCurrentHeading(std::function <double()> callback)
{
	m_SwerveRobot->Set_GetCurrentHeading(callback);
}
const SwerveVelocities &SwerveRobot::GetCurrentVelocities() const
{
	return m_SwerveRobot->GetCurrentVelocities();
}
const SwerveVelocities &SwerveRobot::GetCurrentVoltages() const
{
	return m_SwerveRobot->GetCurrentVoltages();
}
const SwerveVelocities &SwerveRobot::GetIntendedVelocities() const
{
	return m_SwerveRobot->GetIntendedVelocities();
}
bool SwerveRobot::GetIsDrivenLinear() const
{
	return m_SwerveRobot->GetIsDrivenLinear();
}
bool SwerveRobot::GetIsDrivenAngular() const
{
	return m_SwerveRobot->GetIsDrivenAngular();
}
double SwerveRobot::Get_IntendedOrientation() const
{
	return m_SwerveRobot->Get_IntendedOrientation();
}

#pragma endregion

}}