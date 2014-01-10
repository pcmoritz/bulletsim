project(fem)

cmake_minimum_required(VERSION 2.8)

find_package(DOLFIN)
include_directories(${DOLFIN_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/modules/")

execute_process(COMMAND ffc -l dolfin model.ufl
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  
# fix this:
# configure_file(cube.off cube.off COPYONLY)
# configure_file(standard-geometry-cloth-1.off standard-geometry-cloth-1.off COPYONLY)
# configure_file(standard-geometry-cloth-2.off standard-geometry-cloth-2.off COPYONLY)


include_directories("${DOLFIN_INCLUDE_DIRS}")


# find_package(Boost COMPONENTS python)
# include_directories (${Boost_INCLUDE_DIRS})
# find_package(PythonLibs)
# include_directories (${PYTHON_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)

# link_directories("/usr/lib/")

add_subdirectory(TetWrap)

find_package(Bullet)

include_directories (${BULLET_INCLUDE_DIRS})

add_executable(main main.cpp SceneGeometry.cpp FEMRegistration.cpp)
target_link_libraries(main  ${BULLET_LIBRARIES} ${DOLFIN_LIBRARIES} ${Boost_LIBRARIES} tetwrap)

add_library(fem SceneGeometry.cpp FEMRegistration.cpp)
target_link_libraries(fem  ${BULLET_LIBRARIES} ${DOLFIN_LIBRARIES} ${Boost_LIBRARIES} tetwrap)

