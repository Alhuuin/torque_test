#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TorqueTest_DLLIMPORT __declspec(dllimport)
#  define TorqueTest_DLLEXPORT __declspec(dllexport)
#  define TorqueTest_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TorqueTest_DLLIMPORT __attribute__((visibility("default")))
#    define TorqueTest_DLLEXPORT __attribute__((visibility("default")))
#    define TorqueTest_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TorqueTest_DLLIMPORT
#    define TorqueTest_DLLEXPORT
#    define TorqueTest_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TorqueTest_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TorqueTest_DLLAPI
#  define TorqueTest_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TorqueTest_EXPORTS
#    define TorqueTest_DLLAPI TorqueTest_DLLEXPORT
#  else
#    define TorqueTest_DLLAPI TorqueTest_DLLIMPORT
#  endif // TorqueTest_EXPORTS
#  define TorqueTest_LOCAL TorqueTest_DLLLOCAL
#endif // TorqueTest_STATIC