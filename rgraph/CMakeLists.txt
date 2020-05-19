project( rgraph )
cmake_minimum_required(VERSION 3.1)

# BEGIN CGAL GENERATED
# CGAL and its components
find_package( CGAL REQUIRED  )

if ( NOT CGAL_FOUND )

	message(STATUS "This project requires the CGAL library, and will not be compiled.")
	return()

endif()

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

	message(STATUS "This project requires the Boost library, and will not be compiled.")

	return()

endif()
# END CGAL GENERATED

# BEGIN PYTHON BINDINGS
set(CMAKE_CXX_FLAGS "-lboost_python -lmpfr -lgmpxx -lgmp")

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

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES}) # Deprecated but so convenient!
# END PYTHON BINDINGS

# TARGETS
# create_single_source_cgal_program( "src/rgraph.cpp" )
PYTHON_ADD_MODULE( rgraph src/rgraph.cpp )
