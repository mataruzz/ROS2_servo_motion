/*
This code defines a set of macros and logic to control symbol visibility when building a C++ library. 
It is typically used in conjunction with the ROS2 Control framework for creating a MICRO_SERVO_HARDWARE interface to control two joints.
The purpose of this code is to properly export and import symbols when building and using the library on different platforms
(Windows and non-Windows) while adhering to symbol visibility rules.
*/

#ifndef MICRO_SERVO_HARDWARE__VISIBILITY_CONTROL_HPP_
#define MICRO_SERVO_HARDWARE__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MICRO_SERVO_HARDWARE_EXPORT __attribute__((dllexport))
#define MICRO_SERVO_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MICRO_SERVO_HARDWARE_EXPORT __declspec(dllexport)
#define MICRO_SERVO_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MICRO_SERVO_HARDWARE_BUILDING_DLL
#define MICRO_SERVO_HARDWARE_PUBLIC MICRO_SERVO_HARDWARE_EXPORT
#else
#define MICRO_SERVO_HARDWARE_PUBLIC MICRO_SERVO_HARDWARE_IMPORT
#endif
#define MICRO_SERVO_HARDWARE_PUBLIC_TYPE MICRO_SERVO_HARDWARE_PUBLIC
#define MICRO_SERVO_HARDWARE_LOCAL
#else
#define MICRO_SERVO_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MICRO_SERVO_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MICRO_SERVO_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MICRO_SERVO_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MICRO_SERVO_HARDWARE_PUBLIC
#define MICRO_SERVO_HARDWARE_LOCAL
#endif
#define MICRO_SERVO_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MICRO_SERVO_HARDWARE__VISIBILITY_CONTROL_HPP_
