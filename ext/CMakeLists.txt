# often different packages have same custom target name
# better to allow same target name but specify them by namespace when we use it
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS OFF)

include(ExternalProject)
include(FetchContent)

# set(FETCHCONTENT_QUIET OFF)

# Eigen library
# Download as zip, so it's a bit smaller than downloading the whole repo
FetchContent_Declare(
    eigen
    URL                 https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    URL_HASH            MD5=888aab45512cc0c734b3e8f60280daba
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/eigen-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/eigen-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${eigen_SOURCE_DIR})

# glad
set(OpenGL_GL_PREFERENCE GLVND)
add_subdirectory(glad)

# glfw
FetchContent_Declare(
    glfw
    URL                 https://github.com/glfw/glfw/archive/3.3.2.zip
    URL_HASH            MD5=69d0aa90a9278c0dd8be975d1ae4aaf2
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/glfw-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/glfw-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(glfw)
if(NOT glfw_POPULATED)
    FetchContent_Populate(glfw)
endif()

set( GLFW_BUILD_EXAMPLES    OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_TESTS       OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_DOCS        OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_INSTALL           OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../ext/glfw/include)
# file(GLOB sources ${GLFW_SRC_DIR}/*.h ${GLFW_SRC_DIR}/*.c)
# add_library(glfw ${sources})
add_subdirectory(${glfw_SOURCE_DIR})

# glm
FetchContent_Declare(
    glm
    URL                 https://github.com/g-truc/glm/releases/download/0.9.9.7/glm-0.9.9.7.zip
    URL_HASH            MD5=da4740b5b41a1e2ccb2955af74e7c298
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/glm-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/glm-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(glm)
if(NOT glm_POPULATED)
    FetchContent_Populate(glm)
endif()
set( GLM_TEST_ENABLE OFF CACHE BOOL "" )
add_subdirectory(${glm_SOURCE_DIR})
target_include_directories(glm INTERFACE ${glm_SOURCE_DIR}/glm>)

# imgui
add_subdirectory(imgui)
set( CRL_IMGUI_FONT_FOLDER "\"${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/fonts\"" CACHE STRING "")

# stb_image
add_subdirectory(stb_image)

# tinyobjloader
FetchContent_Declare(
    tinyobjloader
    URL                 https://github.com/tinyobjloader/tinyobjloader/archive/v2.0-rc1.zip
    URL_HASH            MD5=f660a9f91fc2ea5c4f33d7530f6a69bc
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/tinyobjloader-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/tinyobjloader-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(tinyobjloader)
if(NOT tinyobjloader_POPULATED)
    FetchContent_Populate(tinyobjloader)
endif()
add_subdirectory(${tinyobjloader_SOURCE_DIR})

# nanovg
FetchContent_Declare(
    nanovg
    GIT_REPOSITORY      https://github.com/memononen/nanovg.git
    GIT_TAG             1f9c8864fc556a1be4d4bf1d6bfe20cde25734b4
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/nanovg-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/nanovg-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(nanovg)
if(NOT nanovg_POPULATED)
    FetchContent_Populate(nanovg)
endif()
file(GLOB sources ${nanovg_SOURCE_DIR}/src/*.h ${nanovg_SOURCE_DIR}/src/*.c)
add_library(nanovg ${sources})
target_include_directories(nanovg PUBLIC ${nanovg_SOURCE_DIR})

# portable file dialog
add_library(portable_file_dialogs INTERFACE)
target_include_directories(portable_file_dialogs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/portable-file-dialog)

# export targets
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	glm
	glad
	glfw
	imgui
	stb_image
	nanovg
	portable_file_dialogs
	tinyobjloader
)

# lbfgspp
FetchContent_Declare(
    lbfgspp
    GIT_REPOSITORY      https://github.com/yixuan/LBFGSpp
    GIT_TAG             f047ef4586869855f00e72312e7b4d78d11694b1
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/lbfgspp-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/lbfgspp-subbuild
    UPDATE_COMMAND      ""
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
)
FetchContent_GetProperties(lbfgspp)
if(NOT lbfgspp_POPULATED)
    FetchContent_Populate(lbfgspp)
endif()
add_library(lbfgspp INTERFACE)
target_include_directories(lbfgspp INTERFACE $<BUILD_INTERFACE:${lbfgspp_SOURCE_DIR}/include>)

# nlohmann json
FetchContent_Declare(
    json
    URL                 https://github.com/nlohmann/json/releases/download/v3.7.3/include.zip
    URL_HASH            MD5=fb96f95cdf609143e998db401ca4f324
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/json-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/json-subbuild
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
endif()
add_library(nlohmann_json INTERFACE)
add_library(nlohmann::json ALIAS nlohmann_json)
target_include_directories(nlohmann_json INTERFACE $<BUILD_INTERFACE:${json_SOURCE_DIR}/include>)

set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	lbfgspp
	nlohmann_json
)

# ode
FetchContent_Declare(
    ode
    URL                 https://bitbucket.org/odedevs/ode/downloads/ode-0.16.1.tar.gz
    # URL_HASH            MD5=b79ceb3bba47770c4ff17fed63e1bab9
    URL_HASH            MD5=0af3efe98f19163c04083c554afff629
    SOURCE_DIR          ${CRL_FETCHCONTENT_SOURCE_DIR}/ode-src
    SUBBUILD_DIR        ${CRL_FETCHCONTENT_SOURCE_DIR}/ode-subbuild
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
FetchContent_GetProperties(ode)
if(NOT ode_POPULATED)
    FetchContent_Populate(ode)
endif()
message(STATUS "Build ODE for crl::simLib")
set(ODE_WITH_DEMOS 	OFF CACHE BOOL  "ODE lib only" )
set(ODE_WITH_OPCODE OFF CACHE BOOL  "ODE lib only" )
set(ODE_WITH_TESTS	OFF CACHE BOOL  "ODE lib only" )
add_subdirectory(${ode_SOURCE_DIR})
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	ODE
)
