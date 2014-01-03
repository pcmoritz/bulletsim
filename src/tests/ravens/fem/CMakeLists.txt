project(fem)

cmake_minimum_required(VERSION 2.8)

find_package(DOLFIN)
include_directories(${DOLFIN_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/modules/")

find_package(CGAL)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

execute_process(COMMAND ffc -l dolfin model.ufl
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  
configure_file(cube.off cube.off COPYONLY)
configure_file(standard-geometry-cloth-1.off standard-geometry-cloth-1.off COPYONLY)
configure_file(standard-geometry-cloth-2.off standard-geometry-cloth-2.off COPYONLY)
  
link_directories("/usr/lib/")
include_directories("${DOLFIN_INCLUDE_DIRS}")

# find_package(Boost COMPONENTS python)
# include_directories (${Boost_INCLUDE_DIRS})
# find_package(PythonLibs)
# include_directories (${PYTHON_INCLUDE_DIRS})

add_executable(test_cgal mesh.cpp main.cpp)
target_link_libraries(test_cgal  ${CGAL_LIBRARIES} ${DOLFIN_LIBRARIES})

add_library(fem SceneGeometry.cpp FEMRegistration.cpp mesh.cpp)
target_link_libraries(fem  ${CGAL_LIBRARIES} ${DOLFIN_LIBRARIES})

