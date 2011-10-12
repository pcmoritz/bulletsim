cmake_minimum_required(VERSION 2.6)
project(SURGICAL_PHYSICS)

# bullet directory (packaged in this tree)
set(BULLET_DIR ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/bullet-2.77)
set(BULLET_COLLADA_DIR ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/bullet-collada)

# hack
set(CMAKE_SKIP_RPATH TRUE)

# eigen2 directory (external)
# taken from https://raw.github.com/cryos/avogadro/master/cmake/modules/FindEigen2.cmake
if (EIGEN2_INCLUDE_DIR)
    # in cache already
    set(EIGEN2_FOUND TRUE)
else (EIGEN2_INCLUDE_DIR)
    find_package(PkgConfig)
    pkg_check_modules(PC_EIGEN2 QUIET eigen2)
    find_path(EIGEN2_INCLUDE_DIR NAMES Eigen/Core
        HINTS ${PC_EIGEN2_INCLUDEDIR} ${PC_EIGEN2_INCLUDE_DIRS}
        PATH_SUFFIXES eigen2
    )
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Eigen2 DEFAULT_MSG EIGEN2_INCLUDE_DIR)
    mark_as_advanced(EIGEN2_INCLUDE_DIR)
endif(EIGEN2_INCLUDE_DIR)

find_package(GLUT)
find_package(OpenGL)

add_subdirectory(lib)
add_subdirectory(src)