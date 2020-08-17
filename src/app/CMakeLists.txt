cmake_minimum_required(VERSION 3.5)

project(rbSim_app)

add_executable(${PROJECT_NAME}
    main.cpp
    app.h
)

list(APPEND CRL_TARGET_DEPENDENCIES
    utils
    crl::gui
    crl::optLib
    crl::robotControlLib
)

list(APPEND CRL_TARGET_LINK_LIBS
    utils
    crl::gui
    crl::optLib
    crl::robotControlLib
    eigen
)

# add ODE simulation
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    -DCRL_APP_USE_ODE
)
list(APPEND CRL_TARGET_DEPENDENCIES crl::simLib)
list(APPEND CRL_TARGET_LINK_LIBS    crl::simLib)

add_dependencies(
    ${PROJECT_NAME}
    ${CRL_TARGET_DEPENDENCIES}
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${CRL_TARGET_LINK_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE
    CRL_DATA_FOLDER=${CRL_DATA_FOLDER}
)
