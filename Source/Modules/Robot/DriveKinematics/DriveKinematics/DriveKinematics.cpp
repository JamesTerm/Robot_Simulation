#include "../../../../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../../../../Base/Vec2d.h"
#include "../../../../Base/Misc.h"
#include "Vehicle_Drive.h"

#include <iostream>

using namespace Module::Robot;

//This test is real light mostly to show how to setup the object without needing anything else, so we'll
//setup some properties, hard-coded inputs make calculations and output the results
void TankTest()
{
	//setup some robot properties
	using properties = Tank_Drive::properties;
	//For 6WD the length is distance between the center wheels to one set of the outer wheels, this is because
	//with the drop center only 4 wheels are touching at any given time (most of the time)
	Vec2D wheel_dimensions(Inches2Meters(12), Inches2Meters(24));
	//Note: In inch units the length of 12 and 24 is roughly 26.83 where it's the
	//hypotonuse of a 12 24 triangle
	properties props=
	{
	wheel_dimensions.length(),	//distance from diagonal for 4WD or one set of 4 wheels for 6WD
	wheel_dimensions[0], //Length between wheels
	wheel_dimensions[1] //Width between wheels
	};
	printf("Tank test, begin----------------\n");
	//Instantiate our tank object
	Tank_Drive tank;
	//Set the properties... this should be a one-time setup operation
	tank.SetProperties(props);
	//Get an input from the controllers to feed in... we'll hard code something interesting
	//Assume our robot's top speed is 12 fps
	//Let's try half speed forward as an easy test
	tank.UpdateVelocities(Feet2Meters(6.0), 0.0);
	//Send to output
	printf("half speed forward left=%.2f right=%.2f\n ",
		Meters2Feet(tank.GetLeftVelocity()), Meters2Feet(tank.GetRightVelocity()));

	//Try a full turn in one second
	tank.UpdateVelocities(0.0, Pi2);

	printf("360 in a second left=%.2f right=%.2f\n ",
		Meters2Feet(tank.GetLeftVelocity()), Meters2Feet(tank.GetRightVelocity()));

	//Try a half speed and half turn
	tank.UpdateVelocities(Feet2Meters(3.0), PI_2);
	printf("90 in a second and quarter speed forward left=%.2f right=%.2f\n ",
		Meters2Feet(tank.GetLeftVelocity()), Meters2Feet(tank.GetRightVelocity()));

	printf("Tank test, end----------------\n");
}

int main()
{
	TankTest();
	//TODO swerve
	return 0;
}
