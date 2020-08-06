#pragma once

#include "ImportExports.h"

namespace Robot_Tester
{

class OSG_Viewer_Internal;

class OSG_View_API OSG_Viewer
{
public:
	OSG_Viewer();
	//Set up the hooks, can be dynamically changed if needed
	//Change position and heading etc for each callback of this
	void SetUpdateCallback(std::function<void(double dTime_s)> callback);
	//This simply links to robot's update scene call, anything more will need to be built within OSG building environment
	void SetSceneCallback(std::function<void(void *rootNode, void *geode)> callback);
	void SetKeyboardCallback(std::function<void(int key, bool press)> callback);
	void init();
	//This is optional, usually true to debug, false to get real-time timings
	void SetUseSyntheticTimeDeltas(bool UseSyntheticTimeDeltas);
	void StartStreaming();
	void StopStreaming();
	void Zoom(double size);  //usually set to 100
	void Test(size_t index);
private:
	std::shared_ptr<OSG_Viewer_Internal> m_OSG_Viewer;
};

}