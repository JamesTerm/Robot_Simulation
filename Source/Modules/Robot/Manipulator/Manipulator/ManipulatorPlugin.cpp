// Ian: ManipulatorPlugin.cpp — out-of-line definitions for ManipulatorPlugin virtual methods
// that return types requiring complete definitions (e.g. unique_ptr<ManipulatorUI_Plugin>).
// The CreateUI() default body is here rather than inline in the header so that translation
// units including ManipulatorPlugin.h don't need the complete ManipulatorUI_Plugin type.

#include "ManipulatorPlugin.h"
#include "../../../../Modules/Output/OSG_Viewer/OSG_Viewer/ManipulatorArm_UI.h"

namespace Module {
namespace Robot {

std::unique_ptr<Module::Output::ManipulatorUI_Plugin> ManipulatorPlugin::CreateUI()
{
	return nullptr;
}

}  // namespace Robot
}  // namespace Module
