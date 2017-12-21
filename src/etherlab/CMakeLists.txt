aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.


add_executable(master master.cpp master.h ecat_motor.cpp ecat_motor.h "${PROJECT_SOURCE_DIR}/src/tinyxml2/tinyxml2.cpp")
add_executable(lan9252_IO lan9252_IO.c )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(master native rtdm xenomai ethercat_rtdm)
target_link_libraries(lan9252_IO native rtdm xenomai ethercat_rtdm)

