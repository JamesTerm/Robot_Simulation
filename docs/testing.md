# Testing

## GoogleTest and CTest

This repository now uses GoogleTest for unit tests and registers them with CTest.

## Prerequisites

- vcpkg toolchain configured for this repo
- `gtest:x64-windows` installed in vcpkg

Install once:

- `D:/code/vcpkg/vcpkg install gtest:x64-windows`

## Configure

- `cmake -S . -B build-vcpkg -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows -DBUILD_UNIT_TESTS=ON`

## Build tests

- `cmake --build build-vcpkg --config Debug --target robot_unit_tests`

## Run all tests

- `ctest --test-dir build-vcpkg -C Debug --output-on-failure`

## Current suites

- `Entity1DTests`
- `Entity2DTests`
- `DriveKinematicsTests`

## Visual Studio integration

- Open the CMake project in Visual Studio.
- Configure/build Debug (or selected config).
- Tests discovered via `gtest_discover_tests()` appear in Test Explorer through CTest integration.
