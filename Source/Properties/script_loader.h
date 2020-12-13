#pragma once
#include <memory>
#include "../Base/AssetManager.h"

#pragma region _description_
//This is similar to an interface in that the script loader implementation can be whatever, but to keep things simple
//it is wrapped per usual, so that limits to only one script loader to be used (which is good) as there needn't be any
//complexity on interfacing here.  For the sake of testing multiple assemblies, use a dispatch implementation which
//can call the appropriate loader, but for the real robot code none of that will be necessary, and in either case this
//file needn't change
#pragma endregion

namespace properties
{
	class script_loader_impl;

	class script_loader
	{
	public:
		script_loader();
		//The launcher will pass its empty database to be filled through this method
		void load_script(Framework::Base::asset_manager &assets);
	private:
		std::shared_ptr<script_loader_impl> m_script_loader;
	};
}