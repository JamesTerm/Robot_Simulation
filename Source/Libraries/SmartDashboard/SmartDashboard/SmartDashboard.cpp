#include "stdafx.h"
#include "SmartDashboard.h"
//#include "../NetworkCommunication/UsageReporting.h"
#include "NamedSendable.h"
//#include "WPIErrors.h"
#include "networktables/NetworkTable.h"


ITable* SmartDashboard::m_table = NULL;
std::map<ITable *, Sendable *> SmartDashboard::m_tablesToData;
namespace
{
	bool g_smartdashboard_initialized = false;
	SmartDashboardDirectPublishSink* g_directPublishSink = NULL;
	SmartDashboardDirectQuerySource* g_directQuerySource = NULL;
}

void SmartDashboard::init()
{
	if (g_smartdashboard_initialized && m_table != NULL)
		return;
	m_table = NetworkTable::GetTable("SmartDashboard");
	g_smartdashboard_initialized = (m_table != NULL);
}

bool SmartDashboard::is_initialized()
{
	return g_smartdashboard_initialized && m_table != NULL;
}
void SmartDashboard::shutdown()
{
	NetworkTable::Shutdown();
	m_table = NULL;
	g_smartdashboard_initialized = false;
}

void SmartDashboard::SetDirectPublishSink(SmartDashboardDirectPublishSink* sink)
{
	g_directPublishSink = sink;
}

void SmartDashboard::ClearDirectPublishSink()
{
	g_directPublishSink = NULL;
}

void SmartDashboard::SetDirectQuerySource(SmartDashboardDirectQuerySource* source)
{
	g_directQuerySource = source;
}

void SmartDashboard::ClearDirectQuerySource()
{
	g_directQuerySource = NULL;
}
void SmartDashboard::SetClientMode()
{
	NetworkTable::SetClientMode();
}
void SmartDashboard::SetServerMode()
{
	NetworkTable::SetServerMode();
}
void SmartDashboard::SetTeam(int team)
{
	NetworkTable::SetTeam(team);
}
void SmartDashboard::SetIPAddress(const char* address)
{
	NetworkTable::SetIPAddress(address);
}

//TODO usage reporting

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutData(std::string key, Sendable *data)
{
	if (data == NULL)
	{
		//TODO wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
    ITable* dataTable = m_table->GetSubTable(key);
    dataTable->PutString("~TYPE~", data->GetSmartDashboardType());
    data->InitTable(dataTable);
    m_tablesToData[dataTable] = data;
}

/**
 * Maps the specified key (where the key is the name of the {@link SmartDashboardNamedData}
 * to the specified value in this table.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param value the value
 */
void SmartDashboard::PutData(NamedSendable *value)
{
	if (value == NULL)
	{
		//TODO wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
	PutData(value->GetName(), value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
//TODO Sendable *SmartDashboard::GetData(std::string key)
/*{
	ITable* subtable = m_table->GetSubTable(keyName);
	Sendable *data = m_tablesToData[subtable];
	if (data == NULL)
	{
		wpi_setWPIErrorWithContext(SmartDashboardMissingKey, keyName);
		return NULL;
	}
    return data;
}*/

/**
 * Maps the specified key to the specified complex value (such as an array) in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the RetrieveValue method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutValue(std::string keyName, ComplexData& value)
{
	m_table->PutValue(keyName, value);
}

/**
 * Retrieves the complex value (such as an array) in this table into the complex data object
 * The key can not be NULL.
 * @param keyName the key
 * @param value the object to retrieve the value into
 */
void SmartDashboard::RetrieveValue(std::string keyName, ComplexData& value)
{
	m_table->RetrieveValue(keyName, value);
}

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutBoolean(std::string keyName, bool value)
{
	if (g_directPublishSink != NULL)
		g_directPublishSink->PublishBoolean(keyName, value);

	m_table->PutBoolean(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
bool SmartDashboard::GetBoolean(std::string keyName)
{
	if (g_directQuerySource != NULL)
	{
		bool value = false;
		if (g_directQuerySource->TryGetBoolean(keyName, value))
			return value;
	}

	return m_table->GetBoolean(keyName);
}

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutNumber(std::string keyName, double value){
	if (!is_initialized()) init();
	if (m_table == NULL) return;
	if (g_directPublishSink != NULL)
		g_directPublishSink->PublishNumber(keyName, value);
	m_table->PutNumber(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
double SmartDashboard::GetNumber(std::string keyName)
{
	if (g_directQuerySource != NULL)
	{
		double value = 0.0;
		if (g_directQuerySource->TryGetNumber(keyName, value))
			return value;
	}

	if (!is_initialized()) init();
	if (m_table == NULL) return 0.0;
	return m_table->GetNumber(keyName);
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutString(std::string keyName, std::string value)
{
	if (g_directPublishSink != NULL)
		g_directPublishSink->PublishString(keyName, value);

	m_table->PutString(keyName, value);
}

bool SmartDashboard::IsConnected()
{
	return m_table->IsConnected();
}
/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @param value the buffer to fill with the value
 * @param valueLen the size of the buffer pointed to by value
 * @return the length of the string
 */
int SmartDashboard::GetString(std::string keyName, char *outBuffer, unsigned int bufferLen){
	std::string value = m_table->GetString(keyName);
	unsigned int i;
	for(i = 0; i<bufferLen-1&&i<value.length(); ++i)
		outBuffer[i] = (char)value.at(i);
	outBuffer[i] = '\0';
	return i;
}


/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
std::string SmartDashboard::GetString(std::string keyName)
{
	if (g_directQuerySource != NULL)
	{
		std::string value;
		if (g_directQuerySource->TryGetString(keyName, value))
			return value;
	}

	return m_table->GetString(keyName);
}

//This is the newer calling interface, but the underlying system doesn't support this, so we can implement it by using try..catch technique
std::string SmartDashboard::GetString(std::string keyName, std::string defaultValue)
{
	std::string ret = defaultValue;
	try
	{
		ret = SmartDashboard::GetString(keyName);
	}
	catch (...)
	{
		//I may need to prime the pump here
		SmartDashboard::PutString(keyName, defaultValue);
	}
	return ret;
}
