# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_khemin_omni_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED khemin_omni_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(khemin_omni_FOUND FALSE)
  elseif(NOT khemin_omni_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(khemin_omni_FOUND FALSE)
  endif()
  return()
endif()
set(_khemin_omni_CONFIG_INCLUDED TRUE)

# output package information
if(NOT khemin_omni_FIND_QUIETLY)
  message(STATUS "Found khemin_omni: 0.0.0 (${khemin_omni_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'khemin_omni' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT khemin_omni_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(khemin_omni_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${khemin_omni_DIR}/${_extra}")
endforeach()
