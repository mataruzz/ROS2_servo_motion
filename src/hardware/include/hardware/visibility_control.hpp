/*
This code defines a set of macros and logic to control symbol visibility when building a C++ library. 
It is typically used in conjunction with the ROS2 Control framework for creating a hardware interface to control two joints.
The purpose of this code is to properly export and import symbols when building and using the library on different platforms
(Windows and non-Windows) while adhering to symbol visibility rules.
*/

#ifndef HARDWARE__VISIBILITY_CONTROL_HPP_
#define HARDWARE__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HARDWARE_EXPORT __attribute__((dllexport))
#define HARDWARE_IMPORT __attribute__((dllimport))
#else
#define HARDWARE_EXPORT __declspec(dllexport)
#define HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef HARDWARE_BUILDING_DLL
#define HARDWARE_PUBLIC HARDWARE_EXPORT
#else
#define HARDWARE_PUBLIC HARDWARE_IMPORT
#endif
#define HARDWARE_PUBLIC_TYPE HARDWARE_PUBLIC
#define HARDWARE_LOCAL
#else
#define HARDWARE_EXPORT __attribute__((visibility("default")))
#define HARDWARE_IMPORT
#if __GNUC__ >= 4
#define HARDWARE_PUBLIC __attribute__((visibility("default")))
#define HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define HARDWARE_PUBLIC
#define HARDWARE_LOCAL
#endif
#define HARDWARE_PUBLIC_TYPE
#endif

#endif  // HARDWARE__VISIBILITY_CONTROL_HPP_
