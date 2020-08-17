cmake_minimum_required(VERSION 2.6)

project(optLib CXX)

set(CMAKE_CXX_STANDARD 11)
set(CRL_TARGET_NAME ${PROJECT_NAME})

FILE(GLOB CRL_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# dependencies
list(APPEND CRL_TARGET_DEPENDENCIES
	lbfgspp
	crl::utils
	eigen
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

# target link libs
list(APPEND CRL_TARGET_LINK_LIBS
	utils
	eigen
)

# target compile definition
set(CRL_TARGET_COMPILE_DEFINITION "")

# do not use ooqp
list (REMOVE_ITEM CRL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OoqpEigenInterface.cpp
)

add_library(${CRL_TARGET_NAME} STATIC ${CRL_SOURCES})
add_library(crl::${CRL_TARGET_NAME} ALIAS
    ${CRL_TARGET_NAME}
)

set_target_properties(${CRL_TARGET_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
)

add_dependencies(${CRL_TARGET_NAME}
    ${CRL_TARGET_DEPENDENCIES}
)

target_include_directories(${CRL_TARGET_NAME} PUBLIC
    ${CRL_TARGET_INCLUDE_DIRS}
)

target_link_libraries(${CRL_TARGET_NAME} PUBLIC
    ${CRL_TARGET_LINK_LIBS}
)

target_compile_definitions(${CRL_TARGET_NAME}
	PUBLIC
	${CRL_TARGET_COMPILE_DEFINITION}
)

set_property(TARGET ${CRL_TARGET_NAME} PROPERTY FOLDER "Libs") #For solution explorer in visual studios

# add to target to export
set_property(GLOBAL APPEND
	PROPERTY TARGET_TO_EXPORT
	${CRL_TARGET_NAME}
)
