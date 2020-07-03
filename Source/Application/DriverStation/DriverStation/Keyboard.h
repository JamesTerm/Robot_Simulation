#pragma once
namespace Framework
{
	namespace Base
	{
		class  EventMap;
	}
}
class Keyboard_Internal; //forward declare... 

class Keyboard
{
public:
	//Client obtains the event map so the keyboard implementation can bind to it
	void Keyboard_init(Framework::Base::EventMap *em);  // allows the declaration to remain here
	void SetEventMap(Framework::Base::EventMap *em); //for late binding
	//Keypresses from dispatch go here
	void KeyPressRelease(int key, bool press);
	bool AddKeyBinding(Framework::Base::Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey = false);
	void RemoveKeyBinding(Framework::Base::Key key, std::string eventName, bool useOnOff);
private:
	std::shared_ptr<Keyboard_Internal> m_p_Keyboard; //a pimpl idiom (using shared_ptr allows declaration to be hidden from destructor)
};
