#include "../Base/Base_Includes.h"

#include "script_loader.h"
#include "RegistryV1.h"

namespace properties
{
class script_loader_impl
{
public:
	void load_script(Framework::Base::asset_manager& assets)
	{
		//In this simple example we will simply hard code some of the assets in the registry
		//TODO Add properties that will help test various pieces of code
		// 1. add tweaking of controllers
		// 2. add actual values for legacy simulation
		// 3. add properties for WPI card assignments
		// I may abandon these above and instead get a Lua example... we'll see
		for (size_t i=0;i<4;i++)
		{
			//form our prefix, it will use the naming convention in Vehicle Drive.h
			const char* const prefix_table[2][4] =
			{
				{"sFL_","sFR_","sRL_","sRR_"},
				{"aFL_","aFR_","aRL_","aRR_"}
			};
			const char* const prefix = prefix_table[0][i];
			std::string constructed_name;
			//we'll go down the list
			using namespace properties::registry_v1;
			using namespace Framework::Base;
			//entity 1D
			constructed_name = prefix, constructed_name += csz_Entity1D_StartingPosition;
			assets.put_number(constructed_name.c_str(), 0.01);
			constructed_name = prefix, constructed_name += csz_Entity1D_Mass;
			assets.put_number(constructed_name.c_str(), 1.5);
			//for testing... we can intentionally leave out properties like this
			//constructed_name = prefix, constructed_name += csz_Entity1D_Dimension;
			//assets.put_number(constructed_name.c_str(), .1524);
			constructed_name = prefix, constructed_name += csz_Entity1D_IsAngular;
			assets.put_bool(constructed_name.c_str(), false);
		}
	}
};

script_loader::script_loader()
{
	m_script_loader = std::make_shared<script_loader_impl>();;
}
void script_loader::load_script(Framework::Base::asset_manager& assets)
{
	m_script_loader->load_script(assets);
}

}