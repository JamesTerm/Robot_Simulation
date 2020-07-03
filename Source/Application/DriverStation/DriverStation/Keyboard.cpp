#include "stdafx.h"
//all binding magic defined here
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Base/Event.h"
#include "../../../Base/EventMap.h"
#include "../../../Base/Script.h"
#include "Keyboard.h"

using namespace Framework::Base;
const double c_DoubleClickTime = 0.25;


class Keyboard_Internal
{
private:
	// Keep track of the buttons already pressed
	std::set<int> m_pressedKeys;
	int m_flags=0;
	EventMap *m_controlledEventMap=nullptr;
	int m_lastReleasedKey;
	double m_lastReleaseTime;
	double m_eventTime;
	Key* m_dblPress=nullptr;
	std::map<Key, std::vector<std::string>*, std::greater<Key> > m_KeyBindings;
	std::map<std::string, std::vector<Key>*, std::greater<std::string> > m_AssignedKeys;
	std::map<Key, std::vector<std::string>*, std::greater<Key> > m_KeyBindings_OnOff;
	std::map<std::string, std::vector<Key>*, std::greater<std::string> > m_AssignedKeys_OnOff;
	//ConfigurationManager * const m_Config;

	// Have an inner handle that does the work based on where the message is coming from
	//bool innerHandle(const osgGA::GUIEventAdapter& ea, bool fromEA);

	void KeyPressRelease(Key key, bool press)
	{
		// Find all of the bound events
		std::vector<std::string>* DnEvents = press ? GetBindingsForKey(key, false) : NULL;
		std::vector<std::string>* DnUpEvents = GetBindingsForKey(key, true);

		#if 0
		// Single press checks happen on the PRESS
		if (press)
		{
			GlobalEventMap.KBM_Events.KBCB_KeyDn.Fire(key);
			if (DnEvents)
			{
				std::vector<std::string>::iterator pos;
				for (pos = DnEvents->begin(); pos != DnEvents->end(); ++pos)
					GlobalEventMap.Event_Map[*pos].Fire();
			}
		}

		GlobalEventMap.KBM_Events.KBCB_KeyDnUp.Fire(key, press);
		if (DnUpEvents)
		{
			std::vector<std::string>::iterator pos;
			for (pos = DnUpEvents->begin(); pos != DnUpEvents->end(); ++pos)
				GlobalEventMap.EventOnOff_Map[*pos].Fire(press);
		}
		#endif

		//No longer check for null... expecting it in the constructor
		//if (m_controlledEventMap)
		{
			//Just fire the events:
			if (press)
			{
				m_controlledEventMap->KBM_Events.KBCB_KeyDn.Fire(key);
				if (DnEvents)
				{
					std::vector<std::string>::iterator pos;
					for (pos = DnEvents->begin(); pos != DnEvents->end(); ++pos)
						m_controlledEventMap->Event_Map[*pos].Fire();
				}
			}
			m_controlledEventMap->KBM_Events.KBCB_KeyDnUp.Fire(key, press);
			if (DnUpEvents)
			{
				std::vector<std::string>::iterator pos;
				for (pos = DnUpEvents->begin(); pos != DnUpEvents->end(); ++pos)
					m_controlledEventMap->EventOnOff_Map[*pos].Fire(press);
			}
		}

	}

	std::vector<std::string> *GetBindingsForKey(Key key, bool useOnOff)
	{
		return useOnOff ? m_KeyBindings_OnOff[key] : m_KeyBindings[key];
	}
	public:
	bool AddKeyBinding(Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey=false)
	{
		//we do not have multiple keyboard binding environments
		#if 0
		if (!ForceBindThisKey)
		{
			//See if there is a key assignment for this already
			if (m_Config->InterceptDefaultKey(eventName, "keyboard"))
				return false;		//All intercepted keys will be "force bound" so we can exit once that has happened
		}
		#endif

		std::map<Key, std::vector<std::string>*, std::greater<Key> >& keyBindings(useOnOff ? m_KeyBindings_OnOff : m_KeyBindings);
		std::vector<std::string>* eventNames = keyBindings[key];
		if (!eventNames)
		{
			eventNames = new std::vector<std::string>;
			eventNames->push_back(eventName);
			keyBindings[key] = eventNames;
		}
		else
		{
			bool exists = false;
			std::vector<std::string>::iterator pos;
			for (pos = eventNames->begin(); pos != eventNames->end() && !exists; ++pos)
				exists = (eventName == *pos);
			if (!exists)
				eventNames->push_back(eventName);
		}

		std::map<std::string, std::vector<Key>*, std::greater<std::string> >& assignedKeys(useOnOff ? m_AssignedKeys_OnOff : m_AssignedKeys);
		std::vector<Key>* keys = assignedKeys[eventName];
		if (!keys)
		{
			keys = new std::vector<Key>;
			keys->push_back(key);
			assignedKeys[eventName] = keys;
		}
		else
		{
			bool exists = false;
			std::vector<Key>::iterator pos;
			//Check for duplicate entries of the same key (This may be a typical case)
			for (pos = keys->begin(); pos != keys->end() && !exists; ++pos)
				exists = (key == *pos);
			if (!exists)
				keys->push_back(key);
		}
		return true;
	}

	void RemoveKeyBinding(Key key, std::string eventName, bool useOnOff)
	{
		std::map<Key, std::vector<std::string>*, std::greater<Key> >& keyBindings(useOnOff ? m_KeyBindings_OnOff : m_KeyBindings);
		std::vector<std::string>* eventNames = keyBindings[key];
		if (eventNames)
		{
			std::vector<std::string>::iterator pos;
			for (pos = eventNames->begin(); pos != eventNames->end(); ++pos)
			{
				if (eventName == *pos)
				{
					eventNames->erase(pos);
					break;	// There should be only one if any
				}
			}
		}

		std::map<std::string, std::vector<Key>*, std::greater<std::string> >& assignedKeys(useOnOff ? m_AssignedKeys_OnOff : m_AssignedKeys);
		std::vector<Key>* keys = assignedKeys[eventName];
		if (keys)
		{
			std::vector<Key>::iterator pos;
			for (pos = keys->begin(); pos != keys->end(); ++pos)
			{
				if (key == *pos)
				{
					keys->erase(pos);
					break;	// There should be only one if any
				}
			}
		}
	}
	private:
	/// This version of the function is just for easy porting from the old KB technique
	void AddKeyBindingR(bool useOnOff, std::string eventName, Key key)
	{
		AddKeyBinding(key, eventName, useOnOff);
	}

	//These are time tested good defaults to have
	void BindShipDefaults()
	{
		AddKeyBindingR(true, "Thrust", 'w');
		AddKeyBindingR(true, "Brake", 's');
		AddKeyBindingR(false, "Stop", 'x');
		AddKeyBindingR(true, "Turn_R", 'd');
		AddKeyBindingR(true, "Turn_L", 'a');
		AddKeyBindingR(false, "Turn_180", '`');
		AddKeyBindingR(true, "StrafeRight", 'e');
		AddKeyBindingR(true, "StrafeLeft", 'q');
		AddKeyBindingR(false, "UserResetPos", ' ');
		//Using g for goal now ;)
		//kbm.AddKeyBindingR(false, "Slide", 'g');
	}
public:
	//Client obtains the event map so the keyboard implementation can bind to it
	Keyboard_Internal(EventMap *em) : m_controlledEventMap(em)
	{
		//setup set default keys
		BindShipDefaults();
	}
	void SetEventMap(EventMap *em) { m_controlledEventMap = em; }
	//Key presses from dispatch go here
	void KeyPressRelease(int key, bool press)
	{
		if (press == !m_pressedKeys.count(key))
		{
			//TODO fancy keys
			//if ((key == osgGA::GUIEventAdapter::KEY_Alt_L) || (key == osgGA::GUIEventAdapter::KEY_Alt_R))
			//	m_flags += press ? osgGA::GUIEventAdapter::MODKEY_ALT : -osgGA::GUIEventAdapter::MODKEY_ALT;
			//if ((key == osgGA::GUIEventAdapter::KEY_Control_L) || (key == osgGA::GUIEventAdapter::KEY_Control_R))
			//	m_flags += press ? osgGA::GUIEventAdapter::MODKEY_CTRL : -osgGA::GUIEventAdapter::MODKEY_CTRL;
			//if ((key == osgGA::GUIEventAdapter::KEY_Shift_L) || (key == osgGA::GUIEventAdapter::KEY_Shift_R))
			//	m_flags += press ? osgGA::GUIEventAdapter::MODKEY_SHIFT : -osgGA::GUIEventAdapter::MODKEY_SHIFT;

			if (press)
			{
				m_pressedKeys.insert(key);
				if ((key == m_lastReleasedKey) && ((m_eventTime - m_lastReleaseTime) < c_DoubleClickTime))
				{
					if (m_dblPress)
					{	// This should never happen, but in case something weird happens, we want to make sure we got the release
						KeyPressRelease(*m_dblPress, false);
						delete m_dblPress;
					}
					m_dblPress = new Key(key, m_flags + Key::DBL);

					//See if there are any event requests using double click for this key... if not use regular click (allows "nudging" on other keys)
					//  [4/5/2009 JamesK]
					std::vector<std::string>* DnUpEvents = GetBindingsForKey(*m_dblPress, true);
					#if 0
					if (DnUpEvents)
						DOUT3("%d", DnUpEvents->size());
					else
						DOUT3("NULL");
					#endif
					if (!DnUpEvents)
					{
						delete m_dblPress;
						m_dblPress = NULL;
						goto UseRegularClick;
					}

					KeyPressRelease(*m_dblPress, true);
					return;
				}
			}
			else
			{
				m_pressedKeys.erase(key);
				m_lastReleaseTime = m_eventTime;
				if ((key == m_lastReleasedKey) && m_dblPress)
				{
					KeyPressRelease(*m_dblPress, false);
					delete m_dblPress;
					m_dblPress = NULL;
					return;
				}
				else
					m_lastReleasedKey = key;
			}
		UseRegularClick:
			// Not a double click, just a regular click
			KeyPressRelease(Key(key, m_flags), press);
		}

	}
};

void Keyboard::Keyboard_init(EventMap *em)
{
	m_p_Keyboard = std::make_shared<Keyboard_Internal>(em);
}

void Keyboard::KeyPressRelease(int key, bool press)
{
	m_p_Keyboard->KeyPressRelease(key, press);
}

bool Keyboard::AddKeyBinding(Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey)
{
	return m_p_Keyboard->AddKeyBinding(key, eventName, useOnOff, ForceBindThisKey);
}

void Keyboard::RemoveKeyBinding(Key key, std::string eventName, bool useOnOff)
{
	m_p_Keyboard->RemoveKeyBinding(key, eventName, useOnOff);
}

void Keyboard::SetEventMap(EventMap *em)
{ 
	m_p_Keyboard->SetEventMap(em);
}
