# SPDX-License-Identifier: Apache-2.0
#
# Copyright 2025 Pepperl+Fuchs SE
#
# Authors:
#   Adrian Krzizok <git@breeze-innovations.com>
#   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
#   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>

add_library(VsxSdk SHARED IMPORTED GLOBAL)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(OS "linux")
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CPU "x64")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86")
  set(CPU "x86")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
  set(CPU "arm")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64" or ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(CPU "arm64")
endif()

set(VSX_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/VsxSdk")
set(VSX_SDK_LIB_PATH "${VSX_SDK_PATH}/C/lib/${OS}-${CPU}/")
set(VSX_SDK_INCLUDE_PATH "${VSX_SDK_PATH}/C/include/")

set_target_properties(VsxSdk PROPERTIES IMPORTED_LOCATION "${VSX_SDK_LIB_PATH}/PF.VsxProtocolDriver.WrapperNE.so")
target_include_directories(VsxSdk INTERFACE "${VSX_SDK_INCLUDE_PATH}")

