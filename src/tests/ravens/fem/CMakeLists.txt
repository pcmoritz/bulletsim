cmake_minimum_required(VERSION 2.8)

find_package(DOLFIN)
include_directories(${DOLFIN_INCLUDE_DIRS})

find_package(CGAL)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

execute_process(COMMAND ffc -l dolfin model.ufl
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  
link_directories("/usr/lib/")
include_directories("${DOLFIN_INCLUDE_DIRS}")

find_package(Boost COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

add_library(fem SceneGeometry.cpp FEMRegistration.cpp)
target_link_libraries(fem  ${CGAL_LIBRARIES} ${DOLFIN_LIBRARIES})