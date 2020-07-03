#pragma once 

#include <assert.h>
#include <list>
#include <functional>
#include <algorithm>

//It is great to see how c++ has evolved as EventV2_0 has the same functionality as Event0, this version makes use of std::function and lambdas.
class EventV2_0
{
public:
	using function_signature = void();
	using protocol = std::function<function_signature>;  //make this type available for client code
	using packet = std::pair<size_t, protocol>;
private:
	using EventHandlerList = std::list<packet>;
	EventHandlerList m_handlerList;
public:
	__inline void Fire() const
	{
		for (auto &pos : m_handlerList)
			pos.second();
	}

	//Use this simpler version if you do not intend to manually remove
	//Removing is optional, because nothing is allocated cleanup will happen automatically
	//Remove should be used if the event is still active while a particular listening client is unsubscribing
	__inline void Subscribe(protocol callback)
	{
		m_handlerList.emplace_back(std::make_pair(0, callback));
	}

	//Use this method if you plan to remove:
	//https://stackoverflow.com/questions/21328229/c11-compare-lambda-expression
	//Because of the nature of lambda's being unique we need an ID to find the instance
	__inline void Subscribe(size_t ID,protocol callback)
	{
		m_handlerList.emplace_back(std::make_pair(ID, callback));
	}

	__inline void Remove(size_t ID)
	{
		//EventHandlerList_iter pos;
		auto pos = std::find_if(m_handlerList.begin(), m_handlerList.end(),
			[ID](packet &m) -> bool
		{	
			return (m.first == ID && ID!=0);
		});
		if (pos != m_handlerList.end())
			m_handlerList.erase(pos);
		else
			assert(false);  //what callback is this?
		
	}
};



template<class... Args>
class EventV2_parms
{
public:
	using function_signature = void(Args...);
	using protocol = std::function<function_signature>;  //make this type available for client code
	using packet = std::pair<size_t, protocol>;
private:
	using EventHandlerList = std::list<packet>;
	EventHandlerList m_handlerList;
public:
	__inline void Fire(Args... args) const
	{
		for (auto &pos : m_handlerList)
			pos.second(args...);
	}
	__inline void Subscribe(protocol callback)
	{
		m_handlerList.emplace_back(std::make_pair(0, callback));
	}

	__inline void Subscribe(size_t ID, protocol callback)
	{
		m_handlerList.emplace_back(std::make_pair(ID, callback));
	}

	__inline void Remove(size_t ID)
	{
		auto pos = std::find_if(m_handlerList.begin(), m_handlerList.end(),
			[ID](packet &m) -> bool
			{	return (m.first == ID && ID!=0);
			});
		if (pos != m_handlerList.end())
			m_handlerList.erase(pos);
		else
			assert(false);  //what callback is this?
	}
};


class IEvent
{
protected:
	class IEventHandler;
public:
	virtual ~IEvent(){}

	class HandlerList
	{

		friend class IEvent::IEventHandler;

	public:
		~HandlerList()
		{
			std::list<IEvent::IEventHandler*>::iterator pos;
			for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
			{
				IEventHandler* eh = *pos;
				eh->_ehl = NULL;		// Set to NULL to avoid circular problems
				IEvent& event = eh->_event;
				event.RemoveEventHandler(eh);	// Note that this will DELETE the eh
			}
			_handlerList.clear();
		}

	private:
		std::list<IEvent::IEventHandler*>	_handlerList;
		void AddEventHandler(IEvent::IEventHandler* eh){_handlerList.push_back(eh);}
		void RemoveEventHandler(IEvent::IEventHandler* eh){_handlerList.remove(eh);}
	};

protected:

	class IEventHandler
	{
	public:
		IEventHandler(IEvent& event, IEvent::HandlerList& ehl) : 
		  _event(event), _ehl(&ehl) {_ehl->AddEventHandler(this);}
		  virtual ~IEventHandler() {if (_ehl) _ehl->RemoveEventHandler(this);}
	private:
		IEvent& _event;
		IEvent::HandlerList* _ehl;
		friend class HandlerList;
	};

	virtual void RemoveEventHandler(IEventHandler* eh) = 0;

	friend class HandlerList;
};

class Event0 : public IEvent
{
private:
	class IEventHandler0 : public IEventHandler
	{
	public:
		IEventHandler0(Event0& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire() const = 0;
	};

	template<class T>
	class EventHandler0 : public IEventHandler0
	{
	public:
		EventHandler0(Event0& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)()) : 
		  IEventHandler0(event, ehl), _client(client), _delegate(delegate) {}
		  virtual void Fire() const {(_client.*_delegate)();}

		  T& _client;
		  void (T::*_delegate)();
	};

	std::list<IEventHandler0*>	_handlerList;

public:

	virtual ~Event0() {ClearAllHandlers();}

	void ClearAllHandlers()
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler0* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire() const
	{
		std::list<IEventHandler0*>::const_iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler0* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire();
		}
	}

	template<class T>
		void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)())
	{
		_handlerList.push_back(new EventHandler0<T>(*this, ehl, client, delegate));
	}

	template<class T>
		void Remove(T& client, void (T::*delegate)())
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler0<T>* posPtr = dynamic_cast<EventHandler0<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		//ASSERT_MSG(false, "Event0::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		std::list<IEventHandler0*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler0* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}
};




template<class P1>
class Event1 : public IEvent
{
private:
	
	class IEventHandler1 : public IEventHandler
	{
		public:
			IEventHandler1(Event1& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
			virtual void Fire(P1 p1) = 0;
	};

	template<class T>
	class EventHandler1 : public IEventHandler1
	{
		public:
			EventHandler1(Event1& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1)) : 
			  IEventHandler1(event, ehl), _client(client), _delegate(delegate) {}
			  virtual void Fire(P1 p1){(_client.*_delegate)(p1);}
	
			  T& _client;
			  void (T::*_delegate)(P1 p1);
	};

	std::list<IEventHandler1*>	_handlerList;
	
public:

	virtual ~Event1() {ClearAllHandlers();}

	void ClearAllHandlers()
	{
		//https://en.cppreference.com/w/cpp/language/dependent_name
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler1* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1) const
	{
		typename std::list<IEventHandler1*>::const_iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler1* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1);
		}
	}

	template<class T>
		void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1))
	{
		_handlerList.push_back(new EventHandler1<T>(*this, ehl, client, delegate));
	}

	template<class T>
		void Remove(T& client, void (T::*delegate)(P1 p1))
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler1<T>* posPtr = dynamic_cast<EventHandler1<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		//ASSERT_MSG(false, "Event1::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler1*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler1* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

};





template<class P1, class P2>
class Event2 : public IEvent
{
private:
	class IEventHandler2 : public IEventHandler
	{
	public:
		IEventHandler2(Event2& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2) = 0;
	};

	template<class T>
	class EventHandler2 : public IEventHandler2
	{
	public:
		EventHandler2(Event2& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2)) : 
		  IEventHandler2(event, ehl), _client(client), _delegate(delegate) {}
		  virtual void Fire(P1 p1, P2 p2){(_client.*_delegate)(p1, p2);}

		  T& _client;
		  void (T::*_delegate)(P1 p1, P2 p2);
	};

	std::list<IEventHandler2*>	_handlerList;
	
public:

	virtual ~Event2() {ClearAllHandlers();}

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler2* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2) const
	{
		typename std::list<IEventHandler2*>::const_iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler2* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2);
		}
	}

	template<class T>
		void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2))
	{
		_handlerList.push_back(new EventHandler2<T>(*this, ehl, client, delegate));
	}

	template<class T>
		void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2))
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler2<T>* posPtr = dynamic_cast<EventHandler2<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		//ASSERT_MSG(false, "Event2::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler2*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler2* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

};



template<class P1, class P2, class P3>
class Event3 : public IEvent
{
private:
	class IEventHandler3 : public IEventHandler
	{
	public:
		IEventHandler3(Event3& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3) = 0;
	};

	template<class T>
	class EventHandler3 : public IEventHandler3
	{
	public:
		EventHandler3(Event3& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3)) : 
		  IEventHandler3(event, ehl), _client(client), _delegate(delegate) {}
		  virtual void Fire(P1 p1, P2 p2, P3 p3){(_client.*_delegate)(p1, p2, p3);}

		  T& _client;
		  void (T::*_delegate)(P1 p1, P2 p2, P3 p3);
	};

	std::list<IEventHandler3*>	_handlerList;
	
public:

	virtual ~Event3() {ClearAllHandlers();}

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler3* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2, P3 p3) const
	{
		typename std::list<IEventHandler3*>::const_iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler3* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2, p3);
		}
	}

	template<class T>
		void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3))
	{
		_handlerList.push_back(new EventHandler3<T>(*this, ehl, client, delegate));
	}

	template<class T>
		void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3))
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler3<T>* posPtr = dynamic_cast<EventHandler3<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		//ASSERT_MSG(false, "Event3::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler3*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler3* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

};





template<class P1, class P2, class P3, class P4>
class Event4 : public IEvent
{
private:
	class IEventHandler4 : public IEventHandler
	{
	public:
		IEventHandler4(Event4& event, IEvent::HandlerList& ehl) : IEventHandler(event, ehl) {}
		virtual void Fire(P1 p1, P2 p2, P3 p3, P4 p4) = 0;
	};

	template<class T>
	class EventHandler4 : public IEventHandler4
	{
	public:
		EventHandler4(Event4& event, IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4)) : 
		  IEventHandler4(event, ehl), _client(client), _delegate(delegate) {}
		  virtual void Fire(P1 p1, P2 p2, P3 p3, P4 p4){(_client.*_delegate)(p1, p2, p3, p4);}

		  T& _client;
		  void (T::*_delegate)(P1 p1, P2 p2, P3 p3, P4 p4);
	};

	std::list<IEventHandler4*>	_handlerList;
	
public:

	virtual ~Event4() {ClearAllHandlers();}

	void ClearAllHandlers()
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); ++pos)
		{
			IEventHandler4* eh = *pos;
			delete eh;
		}
		_handlerList.clear();
	}

	void Fire(P1 p1, P2 p2, P3 p3, P4 p4) const
	{
		typename std::list<IEventHandler4*>::const_iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end(); )
		{
			IEventHandler4* eh = *pos;
			++pos;	// Placed here in case Firing the event removes this element
			eh->Fire(p1, p2, p3, p4);
		}
	}

	template<class T>
		void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4))
	{
		_handlerList.push_back(new EventHandler4<T>(*this, ehl, client, delegate));
	}

	template<class T>
		void Remove(T& client, void (T::*delegate)(P1 p1, P2 p2, P3 p3, P4 p4))
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			EventHandler4<T>* posPtr = dynamic_cast<EventHandler4<T>*>(*pos);
			if ((posPtr) && (&posPtr->_client == &client) && (posPtr->_delegate == delegate))
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
				return;
			}
			else
				pos++;
		}
		//ASSERT_MSG(false, "Event4::Remove() failed to find the handler");
	}

protected:
	/// Only called from EventHandlerList when it is destroyed
	virtual void RemoveEventHandler(IEvent::IEventHandler* eh)
	{
		typename std::list<IEventHandler4*>::iterator pos;
		for (pos = _handlerList.begin(); pos != _handlerList.end();)
		{
			IEventHandler4* posPtr = *pos;
			if (posPtr == eh)
			{
				pos = _handlerList.erase(pos);
				delete posPtr;
			}
			else
				pos++;
		}
	}

};



//-----------------------------------------------------------------------------------------------------
//backward compatible (bc) composite events:
//These may be desirable to use for transitional code, or code that may want to use the legacy methods for less overhead


class Event0bc
{
private:
	Event0 m_Event0;
	using function_signature = void();
	EventV2_0 m_EventV2_0;
public:
	using protocol = std::function<function_signature>;  //allow client to access
	using packet = std::pair<size_t, protocol>;
	__inline void Fire() const
	{	m_Event0.Fire(), m_EventV2_0.Fire();
	}

	//Pick which subscribe your prefer... older version may have better performance in debug mode, but shouldn't be significant for typical use
	__inline void Subscribe(protocol delegate)
	{	m_EventV2_0.Subscribe(delegate);
	}

	__inline void Subscribe(size_t ID, protocol callback)
	{
		m_EventV2_0.Subscribe(ID, callback);
	}
	__inline void Subscribe(void *instance, protocol callback)
	{
		m_EventV2_0.Subscribe((size_t)instance, callback);
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)())
	{	m_Event0.Subscribe(ehl, client, delegate);
	}

	//Pick which remove, which correpsonds to which was used for subscribe
	__inline void Remove(size_t ID)
	{	m_EventV2_0.Remove(ID);
	}
	__inline void Remove(void *ID)
	{
		m_EventV2_0.Remove((size_t)ID);
	}
	template<class T>
	void Remove(T& client, void (T::*delegate)())
	{	m_Event0.Remove(client, delegate);
	}
};

template<class P1>
class Event1bc
{
private:
	Event1<P1> m_Event1;
	using function_signature = void(P1);
	EventV2_parms<P1> m_EventV2_1;
public:
	using protocol = std::function<function_signature>;  //allow client to access
	using packet = std::pair<size_t, protocol>;
	__inline void Fire(P1 p1) const
	{
		m_Event1.Fire(p1), m_EventV2_1.Fire(p1);
	}

	//Pick which subscribe your prefer... older version may have better performance in debug mode, but shouldn't be significant for typical use
	__inline void Subscribe(protocol delegate)
	{
		m_EventV2_1.Subscribe(delegate);
	}
	__inline void Subscribe(size_t ID, protocol callback)
	{
		m_EventV2_1.Subscribe(ID, callback);
	}
	__inline void Subscribe(void *instance, protocol callback)
	{
		m_EventV2_1.Subscribe((size_t)instance, callback);
	}

	template<class T>
	void Subscribe(IEvent::HandlerList& ehl, T& client, void (T::*delegate)(P1 p1))
	{
		m_Event1.Subscribe(ehl, client, delegate);
	}

	//Pick which remove, which correpsonds to which was used for subscribe
	__inline void Remove(size_t ID)
	{
		m_EventV2_1.Remove(ID);
	}
	__inline void Remove(void *ID)
	{
		m_EventV2_1.Remove((size_t)ID);
	}
	template<class T>
	void Remove(T& client, void (T::*delegate)(P1 p1))
	{
		m_Event1.Remove(client, delegate);
	}
};
