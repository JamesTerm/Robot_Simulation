#pragma once
// Ian: ManipulatorUI_Plugin — abstract base class for manipulator visualization in the OSG viewer.
// Each ManipulatorPlugin (physics side) creates its own concrete UI via CreateUI().  This keeps
// the rendering fully encapsulated: swapping the manipulator plugin automatically swaps the
// renderer.  Different year's manipulators (excavator arm, intake, shooter) provide their own
// geometry and update logic by implementing this interface.
//
// Lives in the OSG_Viewer DLL so it has access to OSG types internally (via pImpl), while the
// public interface uses void* for rootNode/geode to avoid leaking OSG headers to consumers.

#include "ImportExports.h"

namespace Module
{
	namespace Output {

class OSG_View_API ManipulatorUI_Plugin
{
public:
	virtual ~ManipulatorUI_Plugin() = default;

	// Initialize any internal state (call after construction and hook setup).
	virtual void Initialize() = 0;

	// Add or remove scene geometry.  rootNode is needed for PositionAttitudeTransform nodes
	// that can't be children of the text geode.  geode is the standard text-layer geode.
	// AddOrRemove: true = add to scene, false = remove from scene.
	virtual void UpdateScene(void* rootNode, void* geode, bool AddOrRemove) = 0;

	// Per-frame update — called from the viewer's update callback.
	virtual void TimeChange(double dTime_s) = 0;
};

}}
