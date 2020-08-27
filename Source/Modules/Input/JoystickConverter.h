#pragma once
#include <math.h>

namespace Module
{
	namespace Input
	{

struct Analog_EventEntry
{
	double FilterRange;   //The higher this is the more dead zone
	double Multiplier;    //scales the range, sometimes used to avoid higher values
	double CurveIntensity; //the sharper the curve the more sensitive it becomes to slower speeds
	bool IsFlipped;
};

inline double AnalogConversion(double InValue, const Analog_EventEntry &key)
{
	double ValueABS = fabs(InValue); //take out the sign... put it back in the end

	//Now to use the attributes to tweak the value
	//First evaluate dead zone range... if out of range subtract out the offset for no loss in precision
	//The /(1.0-filter range) will restore the full range

	ValueABS = (ValueABS >= key.FilterRange) ? ValueABS - key.FilterRange : 0.0;

	ValueABS = key.Multiplier*(ValueABS / (1.0 - key.FilterRange)); //apply scale first then
	if (key.CurveIntensity <= 1.0)
		ValueABS = key.CurveIntensity*pow(ValueABS, 3) + (1.0 - key.CurveIntensity)*ValueABS; //apply the curve intensity
	else
		ValueABS = pow(ValueABS, key.CurveIntensity); //apply the curve intensity

	//Now to restore the sign
	const double OutValue = (InValue < 0.0) ^ key.IsFlipped ? -ValueABS : ValueABS;
	return OutValue;
}

}}
