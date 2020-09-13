#pragma once
#include <queue>
#include "Misc.h"
namespace Framework
{
	namespace Base
	{

//To assist in better control on the PID in addition to the linearization of the victor, there can also be latency from the time the voltage is
//applied to the time it takes effect.  This class makes it easy for systems to account for the latency to reduce error for PID and possibly
//unnecessary oscillation that would otherwise occur
class LatencyFilter
{
private:
	std::queue<double> m_Queue; //This grows as needed
	double m_Latency_s;  //Latency in seconds
public:
	LatencyFilter(double Latency = 0.0) : m_Latency_s(Latency)
	{
		assert(m_Latency_s >= 0);  //must have a positive value
	}
	double operator()(double input, double dTime_s)
	{
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
		m_Queue.push(input);
		//We'll cheat and work with the current dTime_s to obtain a ballpark idea of how much time each entry is... this should be fine as long
		//as the slices are somewhat even... the idea here is to be simple and quick instead of complex and slow with a trade off of accuracy
		double CurrentLatency = m_Queue.size() * dTime_s;
		double value = m_Queue.front();
		while (CurrentLatency > m_Latency_s)
		{
			value = m_Queue.front();
			m_Queue.pop();
			CurrentLatency = m_Queue.size() * dTime_s;
		};
		return value;
	}
	double operator()()
	{
		//This is a passive operation that simply allows multiple calls to obtain the last known value
		return m_Queue.size() ? m_Queue.front() : 0.0;
	}
	void SetLatency(double Latency)
	{
		m_Latency_s = Latency;
	}
};

//This is the same idea and interface to the latency filter, but instead of inducing latency it will predict what the value will be by first
//derivative prediction
class LatencyPredictionFilter
{
private:
	double m_Predicted=0.0;
	//cache two iterations of values for prediction
	double m_Prev_Input=0.0, m_Prev_Target=0.0;
	double m_Latency_s;  //Latency in seconds
public:
	LatencyPredictionFilter(double Latency = 0.0) : m_Latency_s(Latency)
	{
		assert(m_Latency_s >= 0);  //must have a positive value
	}
	double operator()(double input, double target_point, double dTime_s)
	{
		/// \param input is the actual position where it is
		/// \target_point is the current predicted point from the force feed 
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
			//avoid division by zero
		if (dTime_s == 0.0)
			return m_Predicted;

		const double CurrentRate = (input - m_Prev_Input) / dTime_s;
		const double TargetRate = (target_point - m_Prev_Target) / dTime_s;
		const double PredictedRate = (CurrentRate + TargetRate) * 0.5;
		m_Predicted = input + (PredictedRate * m_Latency_s);
		m_Prev_Input = input;
		m_Prev_Target = target_point;
		return m_Predicted;
	}
	double operator()(double input, double dTime_s)
	{
		//This version does not blend in the target rate
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
			//avoid division by zero
		if (dTime_s == 0.0)
			return m_Predicted;

		const double CurrentRate = (input - m_Prev_Input) / dTime_s;
		m_Predicted = input + (CurrentRate * m_Latency_s);
		m_Prev_Input = input;
		return m_Predicted;
	}
	double operator()()
	{
		//This is a passive operation that simply allows multiple calls to obtain the last known value
		return m_Predicted;  //This is the last value that was submitted
	}
	void SetLatency(double Latency)
	{
		m_Latency_s = Latency;
	}
};

class COMMON_API KalmanFilter
{
private:
	//initial values for the Kalman filter
	double m_x_est_last;
	double m_last;
	//the noise in the system... defaults are good
	const double m_Q=0.022;
	const double m_R=0.617;
	bool m_FirstRun=true; //This avoids a stall when first starting
public:
	KalmanFilter()
	{
	}
	double operator()(double input)
	{
		/// \return the filtered value
		//For first run set the last value to the measured value
		if (m_FirstRun)
		{
			m_x_est_last = input;
			m_FirstRun = false;
		}
		//do a prediction
		double x_temp_est = m_x_est_last;
		double P_temp = m_last + m_Q;
		//calculate the Kalman gain
		double K = P_temp * (1.0 / (P_temp + m_R));
		//the 'noisy' value we measured
		double z_measured = input;
		//correct
		double x_est = x_temp_est + K * (z_measured - x_temp_est);
		double P = (1 - K) * P_temp;

		//update our last's
		m_last = P;
		m_x_est_last = x_est;

		//Test for NAN
		if ((!(m_x_est_last > 0.0)) && (!(m_x_est_last < 0.0)))
			m_x_est_last = 0;

		return x_est;
	}
	void Reset()
	{
		m_FirstRun = true;
		//initial values for the kalman filter
		m_x_est_last = 0.0;
		m_last = 0.0;
	}
};

/// This manages a PID control loop.  This was originally written for First WPI library, but refactored to be non-threaded, where both input and output
/// can be called on within this class.  It manages the integral calculations, and provides the PIDOutput
class COMMON_API PIDController2
{
private:
	#pragma region _members_
	double m_P;			// factor for "proportional" control
	double m_I;			// factor for "integral" control
	double m_D;			// factor for "derivative" control
	double m_maximumOutput;	// |maximum output|
	double m_minimumOutput;	// |minimum output|
	double m_maximumInput;		// maximum input - limit setpoint to this
	double m_minimumInput;		// minimum input - limit setpoint to this
	double m_prevError;	// the prior sensor input (used to compute velocity)
	double m_totalError; //the sum of the errors for use in the integral calculation
	double m_tolerance;	//the percentage error that is considered on target
	double m_error;
	double m_result;
	bool m_continuous;	// do the endpoints wrap around? eg. Absolute encoder
	bool m_enabled;		//is the pid controller enabled
	bool m_AutoResetI;
	#pragma endregion
public:
	PIDController2(	double p, double i, double d, bool AutoResetI=true,	double maximumOutput=1.0,double minimumOutput=-1.0,	
		double maximumInput=1.0,double minimumInput=-1.0,double m_tolerance=.05,bool continuous=false,bool enabled=false) :
		m_P(p), m_I(i), m_D(d), m_maximumOutput(maximumOutput), m_minimumOutput(minimumOutput), m_maximumInput(maximumInput), 
		m_minimumInput(minimumInput),m_continuous(continuous), m_enabled(enabled), m_AutoResetI(AutoResetI)
	{
		/// \param p proportional coefficient
		/// \param i integral coefficient
		/// \param d derivative coefficient
		/// \param AutoResetI If both setpoint and input are zero total error is reset (great for all velocity control cases except for lifting mechanisms)
		/// \param maximumOutput
		/// \param minimumOutput
		/// \param maximumInput - limit setpoint to this
		/// \param minimumInput - limit setpoint to this
		/// \param m_tolerance the percentage error that is considered on target
		/// \param continuous do the endpoints wrap around? eg. Absolute encoder
		/// \param enabled If client knows all the above, set to true; otherwise enable use Enable() for late binding
		m_prevError = 0;
		m_totalError = 0;
		m_tolerance = .05;

		m_result = 0;
	}
	~PIDController2()
	{
	}
	double operator()(double setpoint, double input, double dTime_s)
	{
		///This is the main method which performs the computations, and must be called for each time slice
		/// \return the final output result
		/// \param setpoint is the actual desired position.  \note The range of this must be managed by client code!
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		if (m_enabled)
		{
			//While it is true it forces client to use large values it is consistent and will yield much better results
			m_error = (setpoint - input) * dTime_s;  //Using dTime_s will keep the errors consistent if time is erratic
			if (m_continuous)
			{
				if (fabs(m_error) >
					(m_maximumInput - m_minimumInput) / 2)
				{
					if (m_error > 0)
						m_error = m_error - m_maximumInput + m_minimumInput;
					else
						m_error = m_error +
						m_maximumInput - m_minimumInput;
				}
			}


			//If both the setpoint and input are zero then there should be no total error
			if (m_AutoResetI && IsZero(setpoint + input))
				m_totalError = 0.0;

			//Note: here is the original code, which is correct but incomplete; 
			//this check really needs an else case, where if the error grows too large it would stop accumulating error and become stuck
			//on an undesirable value
			//if (((m_totalError + m_error) * m_I < m_maximumOutput) && ((m_totalError + m_error) * m_I > m_minimumOutput))
			//	m_totalError += m_error;

			double TotalErrorCheck = (m_totalError + m_error) * m_I;
			if (TotalErrorCheck < m_maximumOutput)
			{
				if (TotalErrorCheck > m_minimumOutput)
					m_totalError += m_error;
				else //less than the minimum output
				{
					//accumulate by an error which would equal the minimum output
					double MinError = (m_minimumOutput - (m_I * m_totalError)) / m_I;
					m_totalError += MinError;
				}
			}
			else //greater than the maximum output
			{
				//accumulate by an error which would equal the maximum output
				double MaxError = (m_maximumOutput - (m_I * m_totalError)) / m_I;
				m_totalError += MaxError;
			}

			m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);
			m_prevError = m_error;

			if (m_result > m_maximumOutput)
				m_result = m_maximumOutput;
			else if (m_result < m_minimumOutput)
				m_result = m_minimumOutput;
		}
		return m_result;

	}
	double Get()
	{
		///	This is always centered on zero and constrained the the max and min outs
		///	\return the latest calculated output
		return m_result;
	}
	void SetContinuous(bool continuous = true)
	{
		///Set the PID controller to consider the input to be continuous, rather than using the max and min in as constraints, it considers them to
		///be the same point and automatically calculates the shortest route to the setpoint.
		///	\param continuous Set to true turns on continuous, false turns off continuous
		m_continuous = continuous;
	}
	void SetAutoResetI(bool AutoReset = true)
	{
		m_AutoResetI = AutoReset;
	}
	void SetInputRange(double minimumInput, double maximumInput)
	{
		m_minimumInput = minimumInput;
		m_maximumInput = maximumInput;
	}
	void SetOutputRange(double minimumOutput, double maximumOutput)
	{
		m_minimumOutput = minimumOutput;
		m_maximumOutput = maximumOutput;
	}
	void SetPID(double p, double i, double d)
	{
		m_P = p;
		m_I = i;
		m_D = d;
	}
	double GetP()
	{
		return m_P;
	}
	double GetI()
	{
		return m_I;
	}
	double GetD()
	{
		return m_D;
	}
	double GetError()
	{
		///< Returns the current difference of the input from the setpoint
		return m_error;
	}
	double GetTotalError() 
	{
		//Help track I
		return m_totalError;
	}
	void SetTolerance(double percent)
	{
		///Set the percentage error which is considered tolerable for use with OnTarget.
		///	\param percent percentage of error which is tolerable
		m_tolerance = percent;
	}
	bool OnTarget()
	{
		/// \return true if the error is within the percentage of the total input range,
		/// determined by SetTolerance. 
		/// \note this uses the input range therefore ensure it has been set properly
		return (fabs(m_error) < m_tolerance / 100 * (m_maximumInput - m_minimumInput));
	}
	void Enable()
	{
		m_enabled = true;
	}
	void Disable()
	{
		m_result = 0.0;  //I cannot see a case where we would want to retain the last result during a disabled state
		m_enabled = false;
	}
	void Reset()
	{
		///Resets the previous error, the integral term  (This does not disable as it does in the WPI lib)
		m_prevError = 0;
		m_totalError = 0;
		m_result = 0;
	}
	void ResetI()
	{
		/// A quick call often function that zero's the I only
		m_totalError = 0;
	}
	void ResetI(double totalError)
	{
		//allow client to set to specific value
		m_totalError = totalError;
	}
};
}}