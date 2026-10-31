/**
 * @file base.h
 */

#ifndef XYNOVAAPI_BASE_H
#define XYNOVAAPI_BASE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <memory>
#include <chrono>
#include <functional>
#include <system_error>
#include <set>

// *********************           Macros          ************************
// *********************           宏定义           ************************

/// @cond DO_NOT_DOCUMENT

#if defined(_MSC_VER)
 #if defined(XYNOVASDK_DLL_BUILD)
  #define XYNOVA_API __declspec(dllexport)
  #define XYNOVASDK_SUPPRESS_DLL_WARNING
 #elif defined(XYNOVASDK_DLL)
  #define XYNOVA_API __declspec(dllimport)
  #define XYNOVASDK_SUPPRESS_DLL_WARNING
 #else
  #define XYNOVA_API
 #endif
#else
 #define XYNOVA_API __attribute__((visibility("default")))
#endif

namespace xynova {
 template<class T> struct Base { };

}

typedef std::error_code error_code;

#ifndef XYNOVASDK_DECLARE_IMPL
#define XYNOVASDK_DECLARE_IMPL \
protected: \
    class Impl; \
    Impl* impl_;
#endif

#ifndef XYNOVASDK_DECLARE_IMPLD
#define XYNOVASDK_DECLARE_IMPLD \
protected: \
    class Impld; \
    Impld* impld_;
#endif
/// @endcond

#endif //XYNOVAAPI_BASE_H
