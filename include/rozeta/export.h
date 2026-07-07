#pragma once

/*
 * Public import/export policy for Rozeta.
 *
 * - Static builds publish ROZETA_STATIC_DEFINE, so public symbols need no import
 *   or export attribute.
 * - Windows shared-library builds define ROZETA_BUILDING_LIBRARY only while
 *   compiling Rozeta itself, which turns ROZETA_API into dllexport.
 * - Windows shared-library consumers see dllimport through installed CMake
 *   targets/import libraries.
 * - Non-Windows platforms keep default visibility for now; this avoids changing
 *   Linux behavior while keeping the macro ready for explicit visibility later.
 */
#if defined(ROZETA_STATIC_DEFINE)
#  define ROZETA_API
#elif defined(_WIN32) || defined(__CYGWIN__)
#  if defined(ROZETA_BUILDING_LIBRARY)
#    define ROZETA_API __declspec(dllexport)
#  else
#    define ROZETA_API __declspec(dllimport)
#  endif
#else
#  define ROZETA_API
#endif

#define ROZETA_C_API ROZETA_API
