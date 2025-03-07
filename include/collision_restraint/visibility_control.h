#ifndef COLLISION_RESTRAINT__VISIBILITY_CONTROL_H_
#define COLLISION_RESTRAINT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COLLISION_RESTRAINT_EXPORT __attribute__ ((dllexport))
    #define COLLISION_RESTRAINT_IMPORT __attribute__ ((dllimport))
  #else
    #define COLLISION_RESTRAINT_EXPORT __declspec(dllexport)
    #define COLLISION_RESTRAINT_IMPORT __declspec(dllimport)
  #endif
  #ifdef COLLISION_RESTRAINT_BUILDING_LIBRARY
    #define COLLISION_RESTRAINT_PUBLIC COLLISION_RESTRAINT_EXPORT
  #else
    #define COLLISION_RESTRAINT_PUBLIC COLLISION_RESTRAINT_IMPORT
  #endif
  #define COLLISION_RESTRAINT_PUBLIC_TYPE COLLISION_RESTRAINT_PUBLIC
  #define COLLISION_RESTRAINT_LOCAL
#else
  #define COLLISION_RESTRAINT_EXPORT __attribute__ ((visibility("default")))
  #define COLLISION_RESTRAINT_IMPORT
  #if __GNUC__ >= 4
    #define COLLISION_RESTRAINT_PUBLIC __attribute__ ((visibility("default")))
    #define COLLISION_RESTRAINT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COLLISION_RESTRAINT_PUBLIC
    #define COLLISION_RESTRAINT_LOCAL
  #endif
  #define COLLISION_RESTRAINT_PUBLIC_TYPE
#endif

#endif  // COLLISION_RESTRAINT__VISIBILITY_CONTROL_H_
