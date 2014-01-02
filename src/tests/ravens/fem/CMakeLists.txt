#BOOST PYTHON
find_package(Boost COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

add_library(fem SceneGeometry.cpp)