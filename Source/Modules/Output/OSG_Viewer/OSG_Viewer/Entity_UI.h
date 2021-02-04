#pragma once

#include "ImportExports.h"

namespace Module
{
	namespace Output {
class EntityUI_Internal;

class OSG_View_API Entity_UI
{
public:
	//Use a simple struct to keep methods of return
	struct Vector2D
	{
		double x, y;
	};

	struct Entity_Properties
	{
		//EPI references these
		std::string& entity_name;
		Vector2D& Dimensions;            //x-width, y-length in meters
		Vector2D& Character_Dimensions;  //font dimensions
		std::string text_image;
	};

	union uEntity_State
	{
		struct Entity_State
		{
			Vector2D Pos_m;
			double Att_r; //heading in radians
			double IntendedOrientation;
			//These should remain static:
		} bits;
		//give as an array for ease of updates in a full state loop
		struct AsRaw
		{
			double element[4];
		} raw;
	};
	using Entity_State = uEntity_State::Entity_State;
	Entity_UI();
	//Be sure to set callback before initialize
	void SetEntity_Callback(std::function<Entity_State()> callback);
	//This can be late bound as there is a default implementation when using nullptr
	void SetProperties_Callback(std::function<Entity_Properties()> callback);
	//Call this from viewer's SetSceneCallback() to this for adding; we'll not need to worry about removing for now
	void UpdateScene(void *geode, bool AddOrRemove);
	void Initialize();
	//Call this from viewer's SetUpdateCallback(), also change the state while in this callback
	void TimeChange(double dTime_s);
private:
	std::shared_ptr<EntityUI_Internal> m_Robot;
};
}}