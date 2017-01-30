project(intersect)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "")
cmake_minimum_required(VERSION 2.8.11)
include(FindPkgConfig)

include_directories(
	src
)

set(SOURCES 
	src/IntersectLib.cpp
)

set(HEADERS
	src/IntersectLib.h
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-DPYTHON_SUPPORT)
add_definitions(-DUSE_MEMORYVIEWS)
if (APPLE)
  # here we assume to search for the Python libraries provided by macports
  set(PYTHON_LIBRARY "/opt/local/Library/Frameworks/Python.framework/Versions/2.7/Python")
  set(PYTHON_INCLUDE_DIR "/opt/local/Library/Frameworks/Python.framework/Versions/2.7/include/python2.7" "/opt/local/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/numpy/core/include")
  find_package(PythonLibs REQUIRED)
else (APPLE)
  find_package(PythonLibs REQUIRED)
  #pkg_check_modules(PYTHON "python")
  #find_package(Numpy REQUIRED)
endif (APPLE)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${NUMPY_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED ${SOURCES})

set_target_properties(${PROJECT_NAME}
                      PROPERTIES SUFFIX ".so" PREFIX "")

target_link_libraries(${PROJECT_NAME}
                      ${PYTHON_LIBRARIES}
)

set(LIB_INSTALL_DIR python)

set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib
)


# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

