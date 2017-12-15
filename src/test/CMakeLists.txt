aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.



add_executable(vector vector.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(vector native rtdm xenomai ethercat_rtdm)

