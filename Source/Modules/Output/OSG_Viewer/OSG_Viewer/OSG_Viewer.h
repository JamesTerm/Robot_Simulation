#pragma once

namespace Robot_Tester
{

class OSG_Viewer_Internal;

class OSG_Viewer
{
public:
	OSG_Viewer();
	void Test(size_t index);
private:
	std::shared_ptr<OSG_Viewer_Internal> m_OSG_Viewer;
};

}