project( rgraph )
cmake_minimum_required(VERSION 3.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Boost and its components
set( BOOST_ROOT boost_1_73_0 )
set( Boost_NO_BOOST_CMAKE TRUE )

find_package( Boost COMPONENTS system python27 REQUIRED )
if ( NOT Boost_FOUND )
	message(STATUS "This project requires the Boost library, and will not be compiled.")
	return()
endif()
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

# CGAL and its components
set( CGAL_DIR CGAL-5.0.2 )
find_package( CGAL REQUIRED  )
if ( NOT CGAL_FOUND )
	message(STATUS "This project requires the CGAL library, and will not be compiled.")
	return()
endif()
find_package( Boost COMPONENTS python27 REQUIRED )

message(STATUS "CGAL_LIBRARIES = ${CGAL_LIBRARIES}")
message(STATUS "CGAL_INCLUDE_DIRS = ${CGAL_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

FIND_PACKAGE(PythonInterp)
if (PYTHONINTERP_FOUND)
	if (UNIX AND NOT APPLE)
		if (PYTHON_VERSION_MAJOR EQUAL 3)
			FIND_PACKAGE(PythonInterp 3)
			FIND_PACKAGE(PythonLibs 3 REQUIRED)
		else()
			FIND_PACKAGE(PythonInterp)
			FIND_PACKAGE(PythonLibs REQUIRED)
		endif()
	else()
		if (PYTHON_VERSION_MAJOR EQUAL 3)
			FIND_PACKAGE(PythonInterp 3)
			FIND_PACKAGE(PythonLibs 3 REQUIRED)
		else()
			FIND_PACKAGE(PythonInterp)
			FIND_PACKAGE(PythonLibs REQUIRED)
		endif()
	endif()
else()
	message("Python not found")
endif()

message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES(${CGAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${CGAL_LIBRARIES}) # Deprecated but so convenient!

# TARGETS
# create_single_source_cgal_program( "src/region.cpp" )
PYTHON_ADD_MODULE( region src/region.cpp )
