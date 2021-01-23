#pragma once
#include <memory>
#include <functional>
//need Vec2D
#include "../../../../Base/Vec2d.h"
//need SwerveVelocities
#include "../../DriveKinematics/DriveKinematics/Vehicle_Drive.h"
//need asset manager for properties
#include "../../../../Base/AssetManager.h"

namespace Module {
	namespace Robot {

class SwerveRobot_Internal;


class SwerveRobot
{
public:
	SwerveRobot();
	//Handle property initialization
	void Init(const Framework::Base::asset_manager * asset_properties=nullptr);
	void Shutdown(); //give ability to shut down early
	//These are local to current orientation
	void SetLinearVelocity_local(double forward, double right);
	void SetLinearVelocity_global(double north, double east);  //For field centric use
	//Note: 0 north, pi/2 east, pi south, pi+pi/2 west, so positive gives clockwise direction
	void SetAngularVelocity(double clockwise);

	//AI methods: ---------------------------------------------------------------------------
	//This allows setting the desired heading directly either relative to the current heading or absolute
	//If relative positive is a clockwise offset from where it is; otherwise the direction is
	//like written before where 0 is north, pi/2 east, etc.
	void SetIntendedOrientation(double intended_orientation, bool absolute = true);
	//Drives to a location at given by coordinates at max speed given
	//It can either stop at location or continue to drive past it once it hits (allows for path driving)
	//max speed is optional where 0.0 means the max speed in properties
	//if can strafe is true caller manages its own orientation as it deems fit; otherwise if it can't strafe
	//it must manage the orientation to always drive forward in the direction toward the way point
	void DriveToLocation(double north, double east, bool absolute = true, bool stop_at_destination = true, double max_speed = 0.0, bool can_strafe = true);


	//Give entity a time slice to update its position
	void TimeSlice(double d_time_s);
	//If simulator is running this will get a slice
	void SimulatorTimeSlice(double dTime_s);
	//allows entity to be stopped and reset to a position and heading
	void Reset(double X = 0.0, double Y = 0.0, double heading = 0.0);

	//accessors ------
	Vec2D GetCurrentPosition() const;
	double GetCurrentHeading() const;
	const SwerveVelocities &GetCurrentVelocities() const;  //access to odometry readings
	const SwerveVelocities &GetSimulatedVelocities() const; //explicit access to simulations velocities
	const SwerveVelocities &GetCurrentVoltages() const;    //access to voltage writings
	const SwerveVelocities &GetIntendedVelocities() const;  //access to kinematics

	bool GetIsDrivenLinear() const; //returns if its driving or driven (false by default)
	bool GetIsDrivenAngular() const; //returns if its driving or driven (false by default)
	double Get_IntendedOrientation() const; //Get set point heading where 0 is north in radians

	//callbacks ------
	//Linker callbacks:  This will link a type of entity object to be updated with the output of this module to the input
	//of the adjoining module.  This is optional (important) because each module should be completely independent of other
	//classes or modules.  These are called on the time slice
	void Set_UpdateGlobalVelocity(std::function<void(const Vec2D& new_velocity)> callback);
	void Set_UpdateHeadingVelocity(std::function<void(double new_velocity)> callback);
	//These are needed for driven functions, optional as well because we have internal method if its not outsourced
	//The dependency on the position and heading is needed for set-point computations
	void Set_GetCurrentPosition(std::function <Vec2D()> callback);  //returns x y coordinates of current position
	void Set_GetCurrentHeading(std::function <double()> callback);  //returns heading where 0 is north in radians
	using PID_Velocity_proto = void(double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler);
	void SetExternal_Velocity_PID_Monitor_Callback(std::function<PID_Velocity_proto> callback);
	using PID_Position_proto = void(double Voltage, double Position, double PredictedPosition, double CurrentVelocity, double Encoder_Velocity, double ErrorOffset);
	void SetExternal_Position_PID_Monitor_Callback(std::function<PID_Position_proto> callback);
	//Hook up the physical odometry if we have it
	void SetPhysicalOdometry(std::function<Robot::SwerveVelocities ()> callback);
private:
	std::shared_ptr<SwerveRobot_Internal> m_SwerveRobot;
};
	}
}