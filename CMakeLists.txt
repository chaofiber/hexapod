cmake_minimum_required(VERSION 3.5)

# set name of the project
project(final-project)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

set(CRL_DATA_FOLDER "\"${CMAKE_CURRENT_LIST_DIR}/data\"" CACHE STRING "")

set(CRL_FETCHCONTENT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/ext/cache CACHE STRING "Base dir for thirdparty libs" )

# options
option(CRL_BUILD_GUI "Build CRL gui" ON)
option(CRL_EXPORT_LIBS "Export CRLCore libraries" OFF)

# list for targets to export
define_property(GLOBAL PROPERTY TARGET_TO_EXPORT
    BRIEF_DOCS  "Global list of targets to export"
    FULL_DOCS   "Global list of targets to export"
)
set_property(GLOBAL PROPERTY TARGET_TO_EXPORT "")

# thirdparty code
add_subdirectory(ext)

# definitions
if(CRL_BUILD_GUI)
    add_definitions(-DCRL_USE_GUI)
endif(CRL_BUILD_GUI)

# our code
add_subdirectory(src)

# export targets
if(CRL_EXPORT_LIBS)
    get_property(TARGET_TO_EXPORT GLOBAL
        PROPERTY TARGET_TO_EXPORT
    )

    message(STATUS "Targets to export: "
        "${TARGET_TO_EXPORT}"
    )

    export(TARGETS
        ${TARGET_TO_EXPORT}
        NAMESPACE crl::
        FILE CRLCoreTargets.cmake
    )

    export(PACKAGE CRLCore)
endif(CRL_EXPORT_LIBS)
