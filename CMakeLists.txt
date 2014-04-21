cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(troen)

#################################################
# PROJECT DESCRIPTION
#################################################

set(META_PROJECT_NAME        "Ragdoll")
set(META_VERSION_MAJOR       "0")
set(META_VERSION_MINOR       "2")
set(META_VERSION_PATCH       "0")
set(META_VERSION             "${META_VERSION_MAJOR}.${META_VERSION_MINOR}.${META_VERSION_PATCH}")
set(META_AUTHOR_DOMAIN       "https://github.com/MaxReimann/osgragdoll")

#################################################
# OPTIONS
#################################################

option(OPTION_LIMIT_CONFIGS  "Generate limited configs (Release; Debug)" ON)
option(OPTION_LOCAL_INSTALL  "Install to a local directory instead of the system" OFF)
option(OPTION_ERRORS_AS_EXCEPTION "Throw exceptions" OFF)
option(OPTION_PORTABLE_INSTALL    "Install to a local directory instead of the system" ON)


#################################################
# CMAKE CONFIGURATION
#################################################

set(RAGDOLL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Include cmake modules from ./cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find the WinSDK libs
if (WIN32)
	set(CMAKE_PREFIX_PATH "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64" "C:\\Program Files (x86)\\Windows Kits\\8.1\\Lib\\winv6.3\\um\\x64")
	cmake_policy(SET CMP0020 NEW)
endif (WIN32)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Set configuration types
if(OPTION_LIMIT_CONFIGS)
	set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Limited Configs" FORCE)
endif()

# Project
project(${META_PROJECT_NAME} C CXX)

# Generate folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Include custom cmake functions
include(cmake/Custom.cmake)
include(cmake/GitRevision.cmake)

#################################################
# PLATFORM AND ARCHITECTURE
#################################################

# Architecture (32/64 bit)
set(X64 OFF)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(X64 ON)
endif()

# Check for linux
if(UNIX AND NOT APPLE)
	set(LINUX 1)
endif()

# Setup platform specifics (compile flags, etc., ...)
if(MSVC)
	message(STATUS "Configuring for platform Windows/MSVC.")
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PlatformWindowsMSVC.cmake)
elseif(WIN32 AND CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "Configuring for platform Windows/GCC.")
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PlatformWindowsGCC.cmake)
elseif(LINUX AND CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "Configuring for platform Linux/GCC.")
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PlatformLinuxGCC.cmake)
elseif(APPLE)
	message(STATUS "Configuring for platform MacOS.")
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PlatformMacOS.cmake)
else()
	# Unsupported system/compiler
	message(WARNING "Unsupported platform/compiler combination")
endif()


#################################################
# DEPENDENCIES
#################################################


# OpenGL
find_package(OpenGL REQUIRED)

# OpenSceneGraph
find_package(OpenSceneGraph REQUIRED)

# Bullet
set(BULLET_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/BULLET_PHYSICS")
find_package(BULLET REQUIRED)

#################################################
# SOURCES
#################################################

set(sources source/ragdollmodel.h
	source/ragdollmodel.cpp
	source/ragdollview.cpp
	source/ragdollview.h
	source/conversionutils.h
	source/gldebugdrawer.h
	source/gldebugdrawer.cpp
	source/modelbone.h
	source/modelbone.cpp
	source/constants.h
	source/physicsworld.h
	source/physicsworld.cpp
	source/main.cpp)

source_group_by_path("${CMAKE_CURRENT_SOURCE_DIR}/source"
	"\\\\.h$|\\\\.hpp$|\\\\.cpp$|\\\\.c$|\\\\.ui$|\\\\.qrc$" "Source Files" ${sources})

#################################################
# TARGET
#################################################

# Set target name
set(target ${META_PROJECT_NAME})

add_executable(${target} ${sources})

include_directories(
	${CMAKE_SOURCE_DIR}
	${OPENSCENEGRAPH_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIRS}
	${PROJECT_BINARY_DIR} #for the generated files
)



target_link_libraries( ${target}
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    ${BULLET_LIBRARIES}
)

if (WIN32)
target_link_libraries( ${target}
	${CMAKE_PREFIX_PATH}/Xinput9_1_0.lib
)
endif (WIN32)
