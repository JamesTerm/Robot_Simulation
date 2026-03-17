/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SMART_DASHBOARD_H__
#define __SMART_DASHBOARD_H__

//#include "SensorBase.h"
//#include "../Base.h"
#include <map>
#include <string>
#include <vector>
#include "Sendable.h"
#include "NamedSendable.h"
#include "tables/ITable.h"
#include "networktables2/type/ComplexData.h"
#include "ErrorBase.h"

class SmartDashboardDirectPublishSink
{
public:
	virtual ~SmartDashboardDirectPublishSink() {}
	virtual void PublishBoolean(const std::string& keyName, bool value) = 0;
	virtual void PublishNumber(const std::string& keyName, double value) = 0;
	virtual void PublishString(const std::string& keyName, const std::string& value) = 0;
	virtual void PublishStringArray(const std::string& keyName, const std::vector<std::string>& values) = 0;
};

class SmartDashboardDirectQuerySource
{
public:
	virtual ~SmartDashboardDirectQuerySource() {}
	virtual bool TryGetBoolean(const std::string& keyName, bool& value) = 0;
	virtual bool TryGetNumber(const std::string& keyName, double& value) = 0;
	virtual bool TryGetString(const std::string& keyName, std::string& value) = 0;
};

enum class SmartDashboardConnectionMode
{
	eLegacySmartDashboard = 0,
	eDirectConnect = 1,
	eShuffleboard = 2
};

class SmartDashboard //: public SensorBase
{
public:
	static void init();
	static bool is_initialized();
	static void shutdown();
	static void SetConnectionMode(SmartDashboardConnectionMode mode);
	static SmartDashboardConnectionMode GetConnectionMode();
	static void SetDirectPublishSink(SmartDashboardDirectPublishSink* sink);
	static void ClearDirectPublishSink();
	static void SetDirectQuerySource(SmartDashboardDirectQuerySource* source);
	static void ClearDirectQuerySource();

	static void PutData(std::string key, Sendable *data);
	static void PutData(NamedSendable *value);
	//static Sendable* GetData(std::string keyName);
	
	static void PutBoolean(std::string keyName, bool value);
	static bool GetBoolean(std::string keyName);
	
	static void PutNumber(std::string keyName, double value);
	static double GetNumber(std::string keyName);
	
	static void PutString(std::string keyName, std::string value);
	static void PutStringArray(std::string keyName, const std::vector<std::string>& values);
	static int GetString(std::string keyName, char *value, unsigned int valueLen);
	static std::string GetString(std::string keyName);
	static std::string GetString(std::string keyName,std::string defaultValue);

	
	static void PutValue(std::string keyName, ComplexData& value);
	static void RetrieveValue(std::string keyName, ComplexData& value);

	///set that network tables should be a client This must be called before initialize or GetTable
	static void SetClientMode();
	///set that network tables should be a server This must be called before initialize or GetTable
	static void SetServerMode();
	///set the team the robot is configured for (this will set the ip address that network tables will connect to in client mode)
	///This must be called before initialize or GetTable @param team the team number
	static void SetTeam(int team);
	/// @param address the address that network tables will connect to in client mode
	static void SetIPAddress(const char* address);

	static  bool IsConnected();
private:
	SmartDashboard();
	virtual ~SmartDashboard();
	DISALLOW_COPY_AND_ASSIGN(SmartDashboard);

	/** The {@link NetworkTable} used by {@link SmartDashboard} */
	static ITable* m_table;
	
	/** 
	 * A map linking tables in the SmartDashboard to the {@link SmartDashboardData} objects
	 * they came from.
	 */
	static std::map<ITable *, Sendable *> m_tablesToData;
};

#endif
