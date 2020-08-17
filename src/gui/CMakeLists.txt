cmake_minimum_required(VERSION 3.5)

project(gui)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(GLOB CRL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui_impl/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui_impl/*.cpp
)

add_library(${CRL_TARGET_NAME} STATIC
    ${CRL_SOURCES}
)
add_library(crl::${CRL_TARGET_NAME} ALIAS
    ${CRL_TARGET_NAME}
)

# Target link libraries
list(APPEND CRL_TARGET_LINK_LIBS
    glad
    ${OPENGL_LIBRARIES}
    glfw
    glm
    imgui
    stb_image
    nanovg
    portable_file_dialogs
    nlohmann::json
    tinyobjloader
    utils
    eigen
)

if(CRL_BUILD_ASSIMP)
    list(APPEND CRL_TARGET_LINK_LIBS
        assimp
    )
endif(CRL_BUILD_ASSIMP)

add_dependencies(${CRL_TARGET_NAME}
    ${CRL_TARGET_LINK_LIBS}
)

target_link_libraries(${CRL_TARGET_NAME} PUBLIC
    ${CRL_TARGET_LINK_LIBS}
)
target_include_directories(${CRL_TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(CRL_SHADER_FOLDER "\"${CMAKE_CURRENT_SOURCE_DIR}/shaders\"" CACHE STRING "")
target_compile_definitions(${CRL_TARGET_NAME} PUBLIC CRL_SHADER_FOLDER=${CRL_SHADER_FOLDER})
target_compile_definitions(${CRL_TARGET_NAME} PUBLIC CRL_DATA_FOLDER=${CRL_DATA_FOLDER})
target_compile_definitions(${CRL_TARGET_NAME} PUBLIC CRL_IMGUI_FONT_FOLDER=${CRL_IMGUI_FONT_FOLDER})
target_compile_definitions(${CRL_TARGET_NAME} PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)

set_property(TARGET ${CRL_TARGET_NAME} PROPERTY FOLDER "Libs") #For solution explorer in visual studios

# add to target to export
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	${CRL_TARGET_NAME}
)
