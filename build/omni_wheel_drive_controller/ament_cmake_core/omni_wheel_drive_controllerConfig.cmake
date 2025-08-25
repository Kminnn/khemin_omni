# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_omni_wheel_drive_controller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED omni_wheel_drive_controller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(omni_wheel_drive_controller_FOUND FALSE)
  elseif(NOT omni_wheel_drive_controller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(omni_wheel_drive_controller_FOUND FALSE)
  endif()
  return()
endif()
set(_omni_wheel_drive_controller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT omni_wheel_drive_controller_FIND_QUIETLY)
  message(STATUS "Found omni_wheel_drive_controller: 4.30.1 (${omni_wheel_drive_controller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'omni_wheel_drive_controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT omni_wheel_drive_controller_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(omni_wheel_drive_controller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${omni_wheel_drive_controller_DIR}/${_extra}")
endforeach()
