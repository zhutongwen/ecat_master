aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.


add_executable(master master.cpp slave.cpp motor.cpp "${PROJECT_SOURCE_DIR}/src/tinyxml2/tinyxml2.cpp")
add_executable(lan9252_IO lan9252_IO.c )
add_executable(etherlab etherlab.cpp )

add_executable(lan9252_402add lan9252_402add.c )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(master native rtdm xenomai ethercat_rtdm)
target_link_libraries(lan9252_IO native rtdm xenomai ethercat_rtdm)
target_link_libraries(etherlab native rtdm xenomai ethercat_rtdm)
target_link_libraries(lan9252_402add native rtdm xenomai ethercat_rtdm)

