cmake_minimum_required(VERSION 3.5)

project(simLib)

set(CRL_TARGET_NAME ${PROJECT_NAME})

FILE(GLOB CRL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/odeLib/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(${CRL_TARGET_NAME} STATIC
    ${CRL_SOURCES}
)
add_library(crl::${CRL_TARGET_NAME} ALIAS
    ${CRL_TARGET_NAME}
)

add_dependencies(${CRL_TARGET_NAME}
    crl::robotControlLib
    ODE
)

target_include_directories(${CRL_TARGET_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${CRL_TARGET_NAME}
    PUBLIC
    crl::robotControlLib
    ODE
)

# add to target to export
set_property(GLOBAL APPEND
    PROPERTY TARGET_TO_EXPORT
    ${CRL_TARGET_NAME}
)
