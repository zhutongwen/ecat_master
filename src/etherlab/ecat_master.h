#ifndef ECAT_MASTER_H
#define ECAT_MASTER_H

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>

#include "iostream"

#include "ecrt.h"


#include <unistd.h>
#include <libgen.h>
#include <vector>
#include "tinyxml2.h"


#include "ecat_imu.h"
#include "ecat_motor.h"
#include "ecat_force.h"


#define PI  3.141592654

//#define IMU_Pos_0       0, 0
//#define FORCE_Pos_0     0, 1

//#define IMU_Pos_1       0, 1
#define MOTOR_Pos_0     0, 0


typedef struct
{

#ifdef IMU_Pos_0
    EcatImu imu_0;
#endif

#ifdef FORCE_Pos_0
    EcatForce force_0;
#endif



#ifdef IMU_Pos_1
    EcrtImu imu_1;
#endif

#ifdef MOTOR_Pos_0
    EcatMotor motor_0;
#endif

}slaves_t;





#endif
