ADD_LIBRARY(TetGen 
            predicates.cxx
            tetgen.cxx 
            )

#-----------------------------------------------------------------------------
# TetGen got a main-function in it, this pre-processor
# directive is used to cut that main-function declaration out of TetGen.
#
# If this is not done then every other target that tries to link with
# the tetgen static library will result in a linker error due to multiple
# defined main-functions.
SET_TARGET_PROPERTIES(TetGen PROPERTIES COMPILE_FLAGS "-D\"TETLIBRARY\"")
