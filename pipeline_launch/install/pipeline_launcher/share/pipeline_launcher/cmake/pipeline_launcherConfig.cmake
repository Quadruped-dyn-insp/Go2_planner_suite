# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pipeline_launcher_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pipeline_launcher_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pipeline_launcher_FOUND FALSE)
  elseif(NOT pipeline_launcher_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pipeline_launcher_FOUND FALSE)
  endif()
  return()
endif()
set(_pipeline_launcher_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pipeline_launcher_FIND_QUIETLY)
  message(STATUS "Found pipeline_launcher: 0.0.1 (${pipeline_launcher_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pipeline_launcher' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pipeline_launcher_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pipeline_launcher_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pipeline_launcher_DIR}/${_extra}")
endforeach()
