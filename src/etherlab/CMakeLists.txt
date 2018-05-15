aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.


INCLUDE_DIRECTORIES("./glog")
INCLUDE_DIRECTORIES("./glog/include")

add_executable(master
                        ecat_master.cpp
                        ecat_imu.cpp
                        ecat_force.cpp
                        ecat_motor.cpp
                        ecat_task.cpp
                        "${PROJECT_SOURCE_DIR}/src/tinyxml2/tinyxml2.cpp")

#add_executable(lan9252_IO lan9252_IO.c )
#add_executable(etherlab etherlab.cpp )
#add_executable(lan9252_402add lan9252_402add.c )
#add_executable(sensor_force sensor_force.cpp )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


target_link_libraries(master            ${CMAKE_SOURCE_DIR}/src/etherlab/glog/lib/libglog.a)
target_link_libraries(master            native rtdm xenomai ethercat_rtdm)
#target_link_libraries(lan9252_IO        native rtdm xenomai ethercat_rtdm)
#target_link_libraries(etherlab          native rtdm xenomai ethercat_rtdm)
#target_link_libraries(lan9252_402add    native rtdm xenomai ethercat_rtdm)
#target_link_libraries(sensor_force      native rtdm xenomai ethercat_rtdm)

