#ifndef CTR_SIM__VISIBILITY_CONTROL_H_
#define CTR_SIM__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

    // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
    //     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CTR_SIM_EXPORT __attribute__((dllexport))
#define CTR_SIM_IMPORT __attribute__((dllimport))
#else
#define CTR_SIM_EXPORT __declspec(dllexport)
#define CTR_SIM_IMPORT __declspec(dllimport)
#endif
#ifdef CTR_SIM_BUILDING_DLL
#define CTR_SIM_PUBLIC CTR_SIM_EXPORT
#else
#define CTR_SIM_PUBLIC CTR_SIM_IMPORT
#endif
#define CTR_SIM_PUBLIC_TYPE CTR_SIM_PUBLIC
#define CTR_SIM_LOCAL
#else
#define CTR_SIM_EXPORT __attribute__((visibility("default")))
#define CTR_SIM_IMPORT
#if __GNUC__ >= 4
#define CTR_SIM_PUBLIC __attribute__((visibility("default")))
#define CTR_SIM_LOCAL __attribute__((visibility("hidden")))
#else
#define CTR_SIM_PUBLIC
#define CTR_SIM_LOCAL
#endif
#define CTR_SIM_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // CTR_SIM__VISIBILITY_CONTROL_H_