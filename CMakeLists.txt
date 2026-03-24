cmake_minimum_required(VERSION 3.24)

project(Robot_Simulation LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DRIVER_STATION "Build DriverStation application" ON)
option(BUILD_LEGACY_APPS "Build legacy non-primary applications" OFF)
option(BUILD_TESTER_APPS "Build legacy tester executables" OFF)
option(BUILD_UNIT_TESTS "Build CTest-based unit tests" ON)

if(MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS UNICODE _UNICODE)
endif()

set(RS_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${RS_OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${RS_OUTPUT_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

find_package(OpenGL REQUIRED)
find_package(unofficial-osg CONFIG REQUIRED COMPONENTS plugins)
find_package(ixwebsocket CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

set(SMARTDASHBOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Source/Libraries/SmartDashboard")
file(GLOB_RECURSE SMARTDASHBOARD_SOURCES CONFIGURE_DEPENDS
    "${SMARTDASHBOARD_ROOT}/*.cpp"
)
list(REMOVE_ITEM SMARTDASHBOARD_SOURCES
    "${SMARTDASHBOARD_ROOT}/networktables2/util/System.cpp"
)

add_library(SmartDashboard STATIC ${SMARTDASHBOARD_SOURCES})
target_include_directories(SmartDashboard
    PUBLIC
        "${SMARTDASHBOARD_ROOT}"
)
target_link_libraries(SmartDashboard
    PUBLIC
        ws2_32
        winmm
)

add_library(dx_Joystick_Controller STATIC
    Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.cpp
)
target_include_directories(dx_Joystick_Controller
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller"
)
target_link_libraries(dx_Joystick_Controller
    PUBLIC
        dinput8
        dxguid
)

add_library(OSG_Viewer SHARED
    Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.cpp
    Source/Modules/Output/OSG_Viewer/OSG_Viewer/TextFontData.cpp
)
target_compile_definitions(OSG_Viewer PRIVATE OSGVIEW_EXPORTS)
target_include_directories(OSG_Viewer
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/Source/Modules/Output/OSG_Viewer/OSG_Viewer"
)
target_link_libraries(OSG_Viewer
    PUBLIC
        OpenGL::GL
        unofficial::osg::OpenThreads
        unofficial::osg::osg
        unofficial::osg::osgDB
        unofficial::osg::osgGA
        unofficial::osg::osgText
        unofficial::osg::osgUtil
        unofficial::osg::osgViewer
)

add_library(BaseCore STATIC
    Source/Base/EventMap.cpp
    Source/Base/LUA.cpp
    Source/Base/Misc.cpp
    Source/Base/Script.cpp
    Source/Base/Time_Type.cpp
)

add_library(RobotAssembly_static STATIC
    Source/Application/RobotAssembly/RobotAssembly/TeleAutonV1.cpp
    Source/Application/RobotAssembly/RobotAssembly/TeleAutonV2.cpp
    Source/Application/RobotAssembly/RobotAssembly/TeleOpV1.cpp
    Source/Application/RobotAssembly/RobotAssembly/TeleOpV2.cpp
    Source/Application/RobotAssembly/RobotAssembly/TeleOpV3.cpp
    Source/Modules/Input/AI_Input/AI_Input.cpp
    Source/Modules/Input/AI_Input/AI_Input_Example.cpp
    Source/Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.cpp
    Source/Modules/Robot/Entity2D/Entity2D/Entity2D.cpp
    Source/Modules/Robot/MotionControl2D_physics/MotionControl2D_physics/MotionControl2D_physics.cpp
    Source/Modules/Robot/MotionControl2D_simple/MotionControl2D/MotionControl2D.cpp
    Source/Modules/Robot/SwerveRobot/SwerveRobot/RotarySystem.cpp
    Source/Modules/Robot/SwerveRobot/SwerveRobot/SimulatedOdometry.cpp
    Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot.cpp
    Source/Properties/script_loader_example.cpp
)
target_link_libraries(RobotAssembly_static
    PUBLIC
        BaseCore
        SmartDashboard
        dx_Joystick_Controller
        OSG_Viewer
)

if(BUILD_LEGACY_APPS)
    add_executable(RobotAssembly
        Source/Application/RobotAssembly/RobotAssembly/RobotAssembly.cpp
    )
    target_link_libraries(RobotAssembly
        PRIVATE
            RobotAssembly_static
    )

    add_executable(OutputViewer
        Source/Application/OutputViewer/OutputViewer/OutputViewer.cpp
    )
    target_link_libraries(OutputViewer
        PRIVATE
            BaseCore
            SmartDashboard
            OSG_Viewer
    )
endif()

if(BUILD_TESTER_APPS)
    add_executable(AI_Input_Tester
        Source/Modules/Input/AI_Input/AI_Input_Tester.cpp
        Source/Modules/Input/AI_Input/AI_Input.cpp
        Source/Modules/Input/AI_Input/AI_Input_Example.cpp
    )
    target_link_libraries(AI_Input_Tester
        PRIVATE
            BaseCore
            SmartDashboard
    )

    add_executable(OSG_Viewer_Tester
        Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer_Tester.cpp
    )
    target_link_libraries(OSG_Viewer_Tester
        PRIVATE
            OSG_Viewer
    )

    add_executable(OSG_ViewTester
        Source/Modules/Output/OSG_Viewer/OSG_ViewTester/OSG_ViewTester.cpp
    )
    target_link_libraries(OSG_ViewTester
        PRIVATE
            OSG_Viewer
    )

    add_executable(dx_Joystick_Controller_App
        Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick_Controller.cpp
    )
    target_link_libraries(dx_Joystick_Controller_App
        PRIVATE
            dx_Joystick_Controller
    )

    add_executable(Entity1D_Tester
        Source/Modules/Robot/Entity1D/Entity1D/Entity1D_Tester.cpp
        Source/Modules/Robot/Entity1D/Entity1D/Entity1D.cpp
    )
    target_link_libraries(Entity1D_Tester
        PRIVATE
            BaseCore
            SmartDashboard
    )

    add_executable(Entity2D_Tester
        Source/Modules/Robot/Entity2D/Entity2D/Entity2D_Tester.cpp
        Source/Modules/Robot/Entity2D/Entity2D/Entity2D.cpp
    )
    target_link_libraries(Entity2D_Tester
        PRIVATE
            BaseCore
            SmartDashboard
    )

    add_executable(MotionControl2D_simple_Tester
        Source/Modules/Robot/MotionControl2D_simple/MotionControl2D/MotionControl2D_Tester.cpp
        Source/Modules/Robot/MotionControl2D_simple/MotionControl2D/MotionControl2D.cpp
    )
    target_link_libraries(MotionControl2D_simple_Tester
        PRIVATE
            BaseCore
            SmartDashboard
    )

    add_executable(MotionControl2D_physics_Tester
        Source/Modules/Robot/MotionControl2D_physics/MotionControl2D_physics/MotionControl2D_Tester.cpp
        Source/Modules/Robot/MotionControl2D_physics/MotionControl2D_physics/MotionControl2D_physics.cpp
    )
    target_link_libraries(MotionControl2D_physics_Tester
        PRIVATE
            BaseCore
            SmartDashboard
    )

    add_executable(SwerveRobot_Tester
        Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot_Tester.cpp
    )
    target_link_libraries(SwerveRobot_Tester
        PRIVATE
            RobotAssembly_static
    )
endif()

add_executable(DriverStation_TransportSmoke
    Source/Application/DriverStation/DriverStation/TransportSmoke.cpp
    Source/Application/DriverStation/DriverStation/Robot_Tester.cpp
    Source/Application/DriverStation/DriverStation/Transport.cpp
    Source/Application/DriverStation/DriverStation/NT4Server.cpp
    Source/Application/DriverStation/DriverStation/NativeLinkAuthorityHelpers.cpp
    Source/Application/DriverStation/DriverStation/NativeLink.cpp
    Source/Application/DriverStation/DriverStation/NativeLinkSharedMemory.cpp
    Source/Application/DriverStation/DriverStation/NativeLinkTcp.cpp
)
target_include_directories(DriverStation_TransportSmoke
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/plugins/NativeLinkTransport/include"
)
target_link_libraries(DriverStation_TransportSmoke
    PRIVATE
        RobotAssembly_static
        SmartDashboard
        ixwebsocket::ixwebsocket
        nlohmann_json::nlohmann_json
        ws2_32
        bcrypt
)

if(BUILD_UNIT_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    include(GoogleTest)

    add_executable(robot_unit_tests
        tests/auton_selection_tests.cpp
        tests/native_link_core_tests.cpp
        tests/entity1d_tests.cpp
        tests/entity2d_tests.cpp
        tests/drivekinematics_tests.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkAuthorityHelpers.cpp
        Source/Application/DriverStation/DriverStation/NativeLink.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkSharedMemory.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkTcp.cpp
        Source/Modules/Robot/Entity1D/Entity1D/Entity1D.cpp
        Source/Modules/Robot/Entity2D/Entity2D/Entity2D.cpp
        Source/Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.cpp
    )
    target_include_directories(robot_unit_tests
        PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/Source"
            "${CMAKE_CURRENT_SOURCE_DIR}/Source/Application/DriverStation/DriverStation"
            "${CMAKE_CURRENT_SOURCE_DIR}/plugins/NativeLinkTransport/include"
    )
    target_link_libraries(robot_unit_tests
        PRIVATE
            BaseCore
            SmartDashboard
            GTest::gtest
            GTest::gtest_main
            ws2_32
    )
    gtest_discover_tests(robot_unit_tests)
endif()

if(BUILD_DRIVER_STATION)
    add_executable(DriverStation WIN32
        Source/Application/DriverStation/DriverStation/DriverStation.cpp
        Source/Application/DriverStation/DriverStation/Keyboard.cpp
        Source/Application/DriverStation/DriverStation/Robot_Tester.cpp
        Source/Application/DriverStation/DriverStation/Transport.cpp
        Source/Application/DriverStation/DriverStation/NT4Server.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkAuthorityHelpers.cpp
        Source/Application/DriverStation/DriverStation/NativeLink.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkSharedMemory.cpp
        Source/Application/DriverStation/DriverStation/NativeLinkTcp.cpp
        Source/Application/DriverStation/DriverStation/stdafx.cpp
        Source/Application/DriverStation/DriverStation/DriverStation.rc
    )
    target_include_directories(DriverStation
        PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/plugins/NativeLinkTransport/include"
    )
    target_link_libraries(DriverStation
        PRIVATE
            SmartDashboard
            RobotAssembly_static
            ixwebsocket::ixwebsocket
            nlohmann_json::nlohmann_json
            shlwapi
            winmm
            ws2_32
            bcrypt
    )

    add_custom_command(TARGET DriverStation POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:DriverStation>
            $<TARGET_FILE_DIR:DriverStation>
        COMMAND_EXPAND_LISTS
    )
endif()
