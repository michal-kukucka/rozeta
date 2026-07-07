# Shared compiler warning policy for every Rozeta target.
#
# GNU/Clang and MSVC spell comparable strict warning levels differently. Targets
# should call rozeta_apply_warnings(target) instead of hard-coding compiler flags.

function(rozeta_apply_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()
