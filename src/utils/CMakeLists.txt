cmake_minimum_required(VERSION 2.6)

project(utils CXX)

set (CMAKE_CXX_STANDARD 11)
set(CRL_TARGET_NAME ${PROJECT_NAME})

FILE(GLOB CRL_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(${CRL_TARGET_NAME} STATIC
    ${CRL_SOURCES}
)
add_library(crl::${CRL_TARGET_NAME} ALIAS
    ${CRL_TARGET_NAME}
)

set_target_properties(${CRL_TARGET_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
)

add_dependencies(${CRL_TARGET_NAME}
    nlohmann::json
)

target_include_directories(${CRL_TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${CRL_TARGET_NAME} PUBLIC
    nlohmann::json
    eigen
)

target_compile_definitions(${CRL_TARGET_NAME} PUBLIC
    CRL_DATA_FOLDER=${CRL_DATA_FOLDER}
)

set_property(TARGET ${CRL_TARGET_NAME} PROPERTY FOLDER "Libs") #For solution explorer in visual studios

# add to target to export
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	${CRL_TARGET_NAME}
)
