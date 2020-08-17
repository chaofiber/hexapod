cmake_minimum_required(VERSION 3.5)

project(robotControlLib)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(GLOB CRL_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/robot/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/robot/*.cpp
)

# dependencies
list(APPEND CRL_TARGET_DEPENDENCIES
    crl::optLib
    crl::utils
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# target link libs
list(APPEND CRL_TARGET_LINK_LIBS
    crl::optLib
    crl::utils
)

# add gui as dependency and target link library
if(CRL_BUILD_GUI)
    list(APPEND CRL_TARGET_DEPENDENCIES
        gui
        portable_file_dialogs
    )
    list(APPEND CRL_TARGET_LINK_LIBS
        gui
        portable_file_dialogs
    )
else(CRL_BUILD_GUI)
    # exclude GUI related if CRL_BUILD_GUI is OFF
    list (REMOVE_ITEM CRL_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/RBRenderer.h
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/RBRenderer.cpp
    )
endif(CRL_BUILD_GUI)

# add urdfdom as dependency and target link library
if(CRL_BUILD_URDFDOM)
    list(APPEND CRL_TARGET_DEPENDENCIES
        urdfdom_model
        tinyXML
    )

    list(APPEND CRL_TARGET_LINK_LIBS
        urdfdom_model
        tinyXML
    )
endif(CRL_BUILD_URDFDOM)

# add Assimp as dependency and target link library
if(CRL_BUILD_ASSIMP)
    list(APPEND CRL_TARGET_DEPENDENCIES
        assimp
    )

    list(APPEND CRL_TARGET_LINK_LIBS
        assimp
    )
endif(CRL_BUILD_ASSIMP)

# target
add_library(${CRL_TARGET_NAME} STATIC ${CRL_SOURCES})
add_library(crl::${CRL_TARGET_NAME} ALIAS
    ${CRL_TARGET_NAME}
)

add_dependencies(${CRL_TARGET_NAME}
    ${CRL_TARGET_DEPENDENCIES}
)

target_link_libraries(${CRL_TARGET_NAME} PUBLIC
    ${CRL_TARGET_LINK_LIBS}
)

target_include_directories(${CRL_TARGET_NAME} PUBLIC
    ${CRL_TARGET_INCLUDE_DIRS}
)

if(CRL_BUILD_GUI)
    target_compile_definitions(${CRL_TARGET_NAME}
        PUBLIC
        -DCRL_USE_GUI
    )
endif(CRL_BUILD_GUI)

set_property(TARGET ${CRL_TARGET_NAME} PROPERTY FOLDER "Libs") #For solution explorer in visual studios

# add to target to export
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	${CRL_TARGET_NAME}
)
