#pragma once
#undef min
#undef max
#include <algorithm>
//This helper class is a self contained example of translating OSG Keyboard callbacks to a state.  Due to the actual keyboard
//codes themselves this remains simple and a part of OSG Viewer, even though it is a part of the module input namespace
//It is possible to re-use the internal working of this to be mapped to keys, and that could then be generic enough to stand
//on its own

namespace Module
{
	namespace Input {
class Keyboard_State
{
private:
	union uKeyState
	{
		struct defined
		{
			double m_X;
			double m_Y;
			double m_Z;
		} bits;
		struct AsRaw
		{
			double element[3];
		} raw;
	} m_state,m_rate;
public:
	Keyboard_State()
	{
		m_state = { 0 };
		m_rate.raw = { 0.0 };
	}
	void UpdateKeys(double dTime_s,int key, bool press)
	{
		const double max_rate = 8.0;  //have some max acceleration
		const bool StickForwardReverse = true;
		if ((key == 'w')||(key == 0xff52))  //ff52 = up
		{
			m_rate.bits.m_Y = press ? -max_rate : 0.0;
			if ((!StickForwardReverse)||(key==0xff52))
			{
				//Stop moving forward if key is released
				if (!press)
					m_state.bits.m_Y = 0.0;
			}
		}
		else if ((key == 's')||(key==0xFF54)) //ff54 = down
		{
			m_rate.bits.m_Y = press ? max_rate : 0.0;
			if ((!StickForwardReverse)||(key == 0xFF54))
			{
				//Stop moving forward if key is released
				if (!press)
					m_state.bits.m_Y = 0.0;
			}
		}
		if (key == 0xFF53) //right arrow
		{
			m_rate.bits.m_X = press ? max_rate : 0.0;
			if (!press)
				m_state.bits.m_X = 0.0;
		}
		else if (key == 0xFF51) //left arrow
		{
			m_rate.bits.m_X = press ? -max_rate : 0.0;
			if (!press)
				m_state.bits.m_X = 0.0;
		}

		if (key == 'd')
		{
			m_rate.bits.m_Z = press ? max_rate : 0.0;
			if (!press)
				m_state.bits.m_Z = 0.0;
		}
		else if (key == 'a')
		{
			m_rate.bits.m_Z = press ? -max_rate : 0.0;
			if (!press)
				m_state.bits.m_Z = 0.0;
		}

		//just do a full state update here
		for (size_t i = 0; i < 3; i++)
		{
			if (key == 'x')
			{
				m_state.raw.element[i] = m_rate.raw.element[i] = 0.0;
			}

			m_state.raw.element[i] += m_rate.raw.element[i] * dTime_s;
			//clip range
			m_state.raw.element[i] = std::min(1.0, std::max(-1.0, m_state.raw.element[i]));
		}
	}
	uKeyState GetState() const { return m_state; }
	void Reset()
	{
		m_state = m_rate = { 0 };
	}
};
	}
}