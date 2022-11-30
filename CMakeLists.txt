# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                                     #
# The ACME project                                                    #
#                                                                     #
# Copyright (c) 2020, Davide Stocco and Enrico Bertolazzi.            #
#                                                                     #
# The ACME project and its components are supplied under the terms of #
# the open source BSD 2-Clause License. The contents of the ACME      #
# project and its components may not be copied or disclosed except in #
# accordance with the terms of the BSD 2-Clause License.              #
#                                                                     #
# URL: https://opensource.org/licenses/BSD-2-Clause                   #
#                                                                     #
#    Davide Stocco                                                    #
#    Department of Industrial Engineering                             #
#    University of Trento                                             #
#    e-mail: davide.stocco@unitn.it                                   #
#                                                                     #
#    Enrico Bertolazzi                                                #
#    Department of Industrial Engineering                             #
#    University of Trento                                             #
#    e-mail: enrico.bertolazzi@unitn.it                               #
#                                                                     #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

cmake_minimum_required(VERSION 3.16.3)
project(acme)

# -----------------------------------------------------------------------------
# Create the library
# -----------------------------------------------------------------------------

# Create base sets for the library
set(LIB_SOURCES)
set(LIB_HEADERS_INCLUDE)
set(LIB_HEADERS_PRIVATE)

# Iterate through each project src folder
# Find sources
file(GLOB_RECURSE S
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
		)
foreach (F ${S}) # Build relative path respect to source directory
	file(RELATIVE_PATH RF ${CMAKE_CURRENT_SOURCE_DIR} "${F}")
	list(APPEND LIB_SOURCES ${RF})
endforeach ()

# Now, find headers in include/src
file(GLOB_RECURSE S
		"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/*.hxx"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/*.inl"
		)
foreach (F ${S}) # Build relative path respect to source directory
	file(RELATIVE_PATH RF ${CMAKE_CURRENT_SOURCE_DIR} "${F}")
	list(APPEND LIB_HEADERS_INCLUDE ${RF})
endforeach ()
file(GLOB_RECURSE S
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.hh"
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.hxx"
		"${CMAKE_CURRENT_SOURCE_DIR}/src/*.inl"
		)
foreach (F ${S}) # Build relative path respect to source directory
	file(RELATIVE_PATH RF ${CMAKE_CURRENT_SOURCE_DIR} "${F}")
	list(APPEND LIB_HEADERS_PRIVATE ${RF})
endforeach ()

# Finally, create the library
add_library(${PROJECT_NAME} STATIC)
target_sources(${PROJECT_NAME} PRIVATE
		${LIB_SOURCES}
		${LIB_HEADERS_INCLUDE} # Cannot be in PUBLIC or INTERFACE
		${LIB_HEADERS_PRIVATE}
		)
target_include_directories(${PROJECT_NAME}
		PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
		)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
set(EXELISTCPP
		acme_test_00
		acme_test_01
		acme_test_02
		acme_test_03
		acme_test_04
		acme_test_05
		acme_test_06
		acme_test_07
		acme_test_08
		acme_test_09
		acme_test_10
		acme_test_11
		acme_test_12
		acme_test_13
		acme_test_14
		acme_test_15
		acme_test_16
		acme_test_17
		acme_test_18
		)
foreach (S ${EXELISTCPP})
	add_executable(${S} ${CMAKE_CURRENT_SOURCE_DIR}/test/${S}.cc)
	target_link_libraries(${S} ${PROJECT_NAME})
endforeach ()

# Google tests
enable_testing()
add_subdirectory(test/googletest)
include(GoogleTest)

add_executable(${PROJECT_NAME}_TEST
		test/tests.cpp
		)
target_link_libraries(${PROJECT_NAME}_TEST gtest_main acme)
gtest_discover_tests(${PROJECT_NAME}_TEST)
