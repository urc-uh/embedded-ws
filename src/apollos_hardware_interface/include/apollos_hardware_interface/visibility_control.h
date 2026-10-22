#ifndef APOLLOS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define APOLLOS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define APOLLOS_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define APOLLOS_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define APOLLOS_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define APOLLOS_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef APOLLOS_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define APOLLOS_HARDWARE_INTERFACE_PUBLIC APOLLOS_HARDWARE_INTERFACE_EXPORT
  #else
    #define APOLLOS_HARDWARE_INTERFACE_PUBLIC APOLLOS_HARDWARE_INTERFACE_IMPORT
  #endif
  #define APOLLOS_HARDWARE_INTERFACE_PUBLIC_TYPE APOLLOS_HARDWARE_INTERFACE_PUBLIC
  #define APOLLOS_HARDWARE_INTERFACE_LOCAL
#else
  #define APOLLOS_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define APOLLOS_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define APOLLOS_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define APOLLOS_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define APOLLOS_HARDWARE_INTERFACE_PUBLIC
    #define APOLLOS_HARDWARE_INTERFACE_LOCAL
  #endif
  #define APOLLOS_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // APOLLOS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
