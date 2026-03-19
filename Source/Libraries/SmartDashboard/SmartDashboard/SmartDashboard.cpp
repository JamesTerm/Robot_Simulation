#include "stdafx.h"
#include "SmartDashboard.h"
//#include "../NetworkCommunication/UsageReporting.h"
#include "NamedSendable.h"
//#include "WPIErrors.h"
#include "networktables/NetworkTable.h"
#include "networktables2/type/StringArray.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include <sstream>


ITable* SmartDashboard::m_table = NULL;
std::map<ITable *, Sendable *> SmartDashboard::m_tablesToData;
namespace
{
	bool g_smartdashboard_initialized = false;
	SmartDashboardConnectionMode g_connectionMode = SmartDashboardConnectionMode::eDirectConnect;
	bool g_hasExplicitConnectionMode = false;
	SmartDashboardDirectPublishSink* g_directPublishSink = NULL;
	SmartDashboardDirectQuerySource* g_directQuerySource = NULL;

	const wchar_t* c_ConnectionSettingsDir = L"RobotSimulation";
	const wchar_t* c_ConnectionSettingsFile = L"DriverStation.ini";
	const wchar_t* c_ConnectionSettingsSection = L"Connection";
	const wchar_t* c_ConnectionSettingsKey = L"Mode";

	SmartDashboardConnectionMode GetDefaultConnectionMode()
	{
		return SmartDashboardConnectionMode::eDirectConnect;
	}

	bool TryGetConnectionSettingsPath(std::wstring& settingsPath)
	{
		DWORD required = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
		if (!required)
			return false;

		std::wstring basePath(required - 1, L'\0');
		if (GetEnvironmentVariableW(L"LOCALAPPDATA", &basePath[0], required) != (required - 1))
			return false;

		std::wstring settingsDir = basePath;
		if (!settingsDir.empty() && settingsDir.back() != L'\\')
			settingsDir += L'\\';
		settingsDir += c_ConnectionSettingsDir;
		CreateDirectoryW(settingsDir.c_str(), nullptr);

		settingsPath = settingsDir;
		if (!settingsPath.empty() && settingsPath.back() != L'\\')
			settingsPath += L'\\';
		settingsPath += c_ConnectionSettingsFile;
		return true;
	}

	SmartDashboardConnectionMode NormalizeConnectionMode(int rawMode)
	{
		switch (rawMode)
		{
		case static_cast<int>(SmartDashboardConnectionMode::eLegacySmartDashboard):
			return SmartDashboardConnectionMode::eLegacySmartDashboard;
		case static_cast<int>(SmartDashboardConnectionMode::eDirectConnect):
			return SmartDashboardConnectionMode::eDirectConnect;
		case static_cast<int>(SmartDashboardConnectionMode::eShuffleboard):
			return SmartDashboardConnectionMode::eShuffleboard;
		case static_cast<int>(SmartDashboardConnectionMode::eNativeLink):
			return SmartDashboardConnectionMode::eNativeLink;
		default:
			return GetDefaultConnectionMode();
		}
	}

	SmartDashboardConnectionMode LoadPersistedConnectionMode()
	{
		std::wstring settingsPath;
		if (!TryGetConnectionSettingsPath(settingsPath))
			return GetDefaultConnectionMode();

		const UINT rawMode = GetPrivateProfileIntW(
			c_ConnectionSettingsSection,
			c_ConnectionSettingsKey,
			static_cast<UINT>(GetDefaultConnectionMode()),
			settingsPath.c_str());
		return NormalizeConnectionMode(static_cast<int>(rawMode));
	}

	bool HasDirectTransport()
	{
		return (g_connectionMode == SmartDashboardConnectionMode::eDirectConnect) ||
			(g_connectionMode == SmartDashboardConnectionMode::eNativeLink);
	}

	bool UsesNetworkTablesTransport()
	{
		return (g_connectionMode != SmartDashboardConnectionMode::eDirectConnect) &&
			(g_connectionMode != SmartDashboardConnectionMode::eNativeLink);
	}

	void EnsureConnectionModeLoaded()
	{
		if (!g_hasExplicitConnectionMode)
			g_connectionMode = LoadPersistedConnectionMode();
	}

	std::string StripSmartDashboardPrefix(const std::string& keyName)
	{
		static const std::string kPrefix = "SmartDashboard/";
		if (keyName.compare(0, kPrefix.size(), kPrefix) == 0)
			return keyName.substr(kPrefix.size());
		return keyName;
	}

	bool TryGetDirectBoolean(const std::string& keyName, bool& value)
	{
		if (g_directQuerySource == NULL)
			return false;

		if (g_directQuerySource->TryGetBoolean(keyName, value))
			return true;

		const std::string normalized = StripSmartDashboardPrefix(keyName);
		if (normalized != keyName && g_directQuerySource->TryGetBoolean(normalized, value))
			return true;

		return false;
	}

	bool TryGetDirectNumber(const std::string& keyName, double& value)
	{
		if (g_directQuerySource == NULL)
			return false;

		if (g_directQuerySource->TryGetNumber(keyName, value))
			return true;

		const std::string normalized = StripSmartDashboardPrefix(keyName);
		if (normalized != keyName && g_directQuerySource->TryGetNumber(normalized, value))
			return true;

		return false;
	}

	bool TryGetDirectString(const std::string& keyName, std::string& value)
	{
		if (g_directQuerySource == NULL)
			return false;

		if (g_directQuerySource->TryGetString(keyName, value))
			return true;

		const std::string normalized = StripSmartDashboardPrefix(keyName);
		if (normalized != keyName && g_directQuerySource->TryGetString(normalized, value))
			return true;

		return false;
	}

}

void SmartDashboard::init()
{
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return;
	if (g_smartdashboard_initialized && m_table != NULL)
		return;
	m_table = NetworkTable::GetTable("SmartDashboard");
	g_smartdashboard_initialized = (m_table != NULL);
}

bool SmartDashboard::is_initialized()
{
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return true;
	return g_smartdashboard_initialized && m_table != NULL;
}
void SmartDashboard::shutdown()
{
	NetworkTable::Shutdown();
	m_table = NULL;
	g_smartdashboard_initialized = false;
}

void SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode mode)
{
	const bool wasUsingNetworkTables = UsesNetworkTablesTransport();
	g_connectionMode = mode;
	g_hasExplicitConnectionMode = true;
	if (wasUsingNetworkTables && !UsesNetworkTablesTransport())
	{
		NetworkTable::Shutdown();
		m_table = NULL;
		g_smartdashboard_initialized = false;
	}
}

SmartDashboardConnectionMode SmartDashboard::GetConnectionMode()
{
	return g_connectionMode;
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
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return;
	NetworkTable::SetClientMode();
}
void SmartDashboard::SetServerMode()
{
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return;
	NetworkTable::SetServerMode();
}
void SmartDashboard::SetTeam(int team)
{
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return;
	NetworkTable::SetTeam(team);
}
void SmartDashboard::SetIPAddress(const char* address)
{
	EnsureConnectionModeLoaded();
	if (!UsesNetworkTablesTransport())
		return;
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
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

	if (data == NULL)
	{
		//TODO wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
    ITable* dataTable = m_table->GetSubTable(key);
    dataTable->PutString("~TYPE~", data->GetSmartDashboardType());
	dataTable->PutString(".type", data->GetSmartDashboardType());
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
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

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
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

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

	if (HasDirectTransport())
		return;

	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

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
		if (TryGetDirectBoolean(keyName, value))
			return value;
		if (HasDirectTransport())
			return false;
	}
	else if (HasDirectTransport())
		return false;

	if (!is_initialized())
		init();
	if (m_table == NULL)
		return false;

	return m_table->GetBoolean(keyName);
}

bool SmartDashboard::TryGetBoolean(std::string keyName, bool& value)
{
	if (TryGetDirectBoolean(keyName, value))
		return true;
	if (HasDirectTransport())
		return false;
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return false;
	if (!m_table->ContainsKey(keyName))
		return false;
	value = m_table->GetBoolean(keyName);
	return true;
}

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutNumber(std::string keyName, double value){
	if (keyName == "AutonTest")
	{
		char dbg[256] = {};
		sprintf_s(dbg, "[SmartDashboard::PutNumber] key=AutonTest value=%g\n", value);
		OutputDebugStringA(dbg);
	}
	if (g_directPublishSink != NULL)
		g_directPublishSink->PublishNumber(keyName, value);
	if (HasDirectTransport())
		return;
	if (!is_initialized()) init();
	if (m_table == NULL) return;
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
		if (TryGetDirectNumber(keyName, value))
		{
			if (keyName == "AutonTest")
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[SmartDashboard::GetNumber] key=AutonTest source=direct value=%g\n", value);
				OutputDebugStringA(dbg);
			}
			return value;
		}
		else if (keyName == "AutonTest")
		{
			OutputDebugStringA("[SmartDashboard::GetNumber] key=AutonTest source=direct miss\n");
		}
		if (HasDirectTransport())
			return 0.0;
	}
	else if (HasDirectTransport())
		return 0.0;

	if (!is_initialized()) init();
	if (m_table == NULL) return 0.0;
	double value = m_table->GetNumber(keyName);
	if (keyName == "AutonTest")
	{
		char dbg[256] = {};
		sprintf_s(dbg, "[SmartDashboard::GetNumber] key=AutonTest source=nt value=%g\n", value);
		OutputDebugStringA(dbg);
	}
	return value;
}

bool SmartDashboard::TryGetNumber(std::string keyName, double& value)
{
	if (TryGetDirectNumber(keyName, value))
		return true;
	if (HasDirectTransport())
		return false;
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return false;
	if (!m_table->ContainsKey(keyName))
		return false;
	value = m_table->GetNumber(keyName);
	return true;
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

	if (HasDirectTransport())
		return;

	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

	m_table->PutString(keyName, value);
}

void SmartDashboard::PutStringArray(std::string keyName, const std::vector<std::string>& values)
{
	if (g_directPublishSink != NULL)
		g_directPublishSink->PublishStringArray(keyName, values);

	if (HasDirectTransport())
		return;

	if (!is_initialized())
		init();
	if (m_table == NULL)
		return;

	StringArray arrayValue;
	for (size_t i = 0; i < values.size(); ++i)
		arrayValue.add(values[i]);

	m_table->PutValue(keyName, arrayValue);
}

bool SmartDashboard::IsConnected()
{
	if (HasDirectTransport())
		return true;
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return false;

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
	if (HasDirectTransport())
	{
		if (outBuffer == NULL || bufferLen == 0)
			return 0;

		std::string value = GetString(keyName);
		unsigned int i;
		for (i = 0; i<bufferLen - 1 && i<value.length(); ++i)
			outBuffer[i] = (char)value.at(i);
		outBuffer[i] = '\0';
		return i;
	}

	if (!is_initialized())
		init();
	if (m_table == NULL || outBuffer == NULL || bufferLen == 0)
		return 0;

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
		if (TryGetDirectString(keyName, value))
			return value;
		if (HasDirectTransport())
			return std::string();
	}
	else if (HasDirectTransport())
		return std::string();

	if (!is_initialized())
		init();
	if (m_table == NULL)
		return std::string();

	return m_table->GetString(keyName);
}

bool SmartDashboard::TryGetString(std::string keyName, std::string& value)
{
	if (TryGetDirectString(keyName, value))
		return true;
	if (HasDirectTransport())
		return false;
	if (!is_initialized())
		init();
	if (m_table == NULL)
		return false;
	if (!m_table->ContainsKey(keyName))
		return false;
	value = m_table->GetString(keyName);
	return true;
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
