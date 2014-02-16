# - Try to find aitown library
# Once done this will define
#  AITOWN_FOUND - System has AITown
#  AITOWN_INCLUDE_DIRS - The AITown include directories
#  AITOWN_LIBRARIES - The libraries needed to use AITown
#  AITOWN_DEFINITIONS - Compiler switches required for using AITown

find_package(PkgConfig)
pkg_check_modules(PC_AITOWN QUIET aitown)
set(AITOWN_DEFINITIONS ${PC_AITOWN_CFLAGS_OTHER})

find_path(AITOWN_INCLUDE_DIR aitown/aitown_global.h
          HINTS ${PC_AITOWN_INCLUDEDIR} ${PC_AITOWN_INCLUDE_DIRS} 
          PATHS PATHS ENV AITOWN_INC
           )

find_library(AITOWN_LIBRARY_CORE NAMES aitown-core libaitown-core libaitown-core_debug aitown-core_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS} 
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_DSTORAGE NAMES aitown-dstorage libaitown-dstorage libaitown-dstorage_debug aitown-dstorage_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS} 
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_PLUGIN NAMES aitown-plugin libaitown-plugin libaitown-plugin_debug aitown-plugin_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS} 
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_PROTOBUF NAMES aitown-protobuf libaitown-protobuf libaitown-protobuf_debug aitown-protobuf_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS} 
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_UTILS NAMES aitown-utils libaitown-utils libaitown-utils_debug aitown-utils_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS} 
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_CFGPATH NAMES cfgpath libcfgpath libcfgpath_debug cfgpath_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS}
             PATHS ENV AITOWN_LIB )
find_library(AITOWN_LIBRARY_IMAGE NAMES aitown-image libaitown-image libaitown-image_debug aitown-image_debug
             HINTS ${PC_AITOWN_LIBDIR} ${PC_AITOWN_LIBRARY_DIRS}
             PATHS ENV AITOWN_LIB )

set(AITOWN_LIBRARIES
    ${AITOWN_LIBRARY_CORE}
    ${AITOWN_LIBRARY_DSTORAGE}
    ${AITOWN_LIBRARY_PLUGIN}
    ${AITOWN_LIBRARY_PROTOBUF}
    ${AITOWN_LIBRARY_UTILS}
    ${AITOWN_LIBRARY_CFGPATH}
    ${AITOWN_LIBRARY_IMAGE}

)
set(AITOWN_INCLUDE_DIRS ${AITOWN_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set AITOWN_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(AITown  DEFAULT_MSG
    AITOWN_LIBRARY_CORE
    AITOWN_LIBRARY_DSTORAGE
    AITOWN_LIBRARY_PLUGIN
    AITOWN_LIBRARY_PROTOBUF
    AITOWN_LIBRARY_UTILS
    AITOWN_LIBRARY_CFGPATH
    AITOWN_LIBRARY_IMAGE
    AITOWN_INCLUDE_DIR)

mark_as_advanced(
    AITOWN_LIBRARY_CORE
    AITOWN_LIBRARY_DSTORAGE
    AITOWN_LIBRARY_PLUGIN
    AITOWN_LIBRARY_PROTOBUF
    AITOWN_LIBRARY_UTILS
    AITOWN_LIBRARY_CFGPATH
    AITOWN_LIBRARY_IMAGE
    AITOWN_INCLUDE_DIR)
