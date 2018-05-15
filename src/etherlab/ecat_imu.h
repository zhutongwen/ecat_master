#ifndef ETHERCAT_IMU_H
#define ETHERCAT_IMU_H

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

class EcatImu
{
public:

    typedef struct
    {
        unsigned int gx;
        unsigned int gy;
        unsigned int gz;
        unsigned int ax;
        unsigned int ay;
        unsigned int az;
        unsigned int counter{0};
        unsigned int led;
    }offset_t;

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
    }data_t;

    offset_t offset;
    data_t data;


    //TxPdo
    ec_pdo_entry_info_t txpdo_entries[7] = {
        {0x6000, 0x01, 32}, /* gx */
        {0x6000, 0x02, 32}, /* gy */
        {0x6000, 0x03, 32}, /* gz */
        {0x6000, 0x04, 32}, /* ax */
        {0x6000, 0x05, 32}, /* ay */
        {0x6000, 0x06, 32}, /* az */
        {0x6000, 0x07, 32}, /* counter */
    };
    ec_pdo_info_t txpdos[1] = {
        {0x1a00, 7, txpdo_entries + 0}, /* TxPdo Channel 1 */
    };
    //RxPdo
    ec_pdo_entry_info_t rxpdo_entries[1] = {
        {0x7011, 0x01, 16}, /* led0_8 */
    };
    ec_pdo_info_t rxpdos[1] = {
        {0x1601, 1, rxpdo_entries + 0}, /* RxPdo Channel 1 */
    };
    ec_sync_info_t syncs[3] = {
        {2, EC_DIR_OUTPUT, 1, rxpdos, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  1, txpdos, EC_WD_ENABLE},
        {0xff}
    };

    int Init(ec_master_t *master_,
             uint16_t alias_, /**< Slave alias. */
             uint16_t position_/**< Slave position. */);

    void DataRead(uint8_t *domain1_pd_);
    void DataWrite(uint8_t *domain1_pd_);
    void DataPlay(void);

    EcatImu();
    ~EcatImu();

private:

    #define IMU             0xE0000005, 0x26483052

//    volatile unsigned int test;
};



#endif
