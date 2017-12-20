
aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.


add_executable(tinyxml2_test ${SRC_LIST} )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
