#ifndef MASTER_H
#define MASTER_H

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
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

typedef struct
{
    unsigned int gx;
    unsigned int gy;
    unsigned int gz;
    unsigned int ax;
    unsigned int ay;
    unsigned int az;
    unsigned int counter;
    unsigned int led;
}off_imu_t;

typedef struct
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    uint32_t counter;
    uint16_t led;
}imu_data_t;



typedef struct
{
    unsigned int analog_data;
    unsigned int keys;
    unsigned int leds;
}off_lan9252_io_t;

typedef struct
{
    uint16_t analog_data;
    uint8_t key0_1;
    uint8_t led0_7;
}wmlan9252_io_data_t;

#endif
