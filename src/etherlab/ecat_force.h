#ifndef ETHERCAT_FORCE_H
#define ETHERCAT_FORCE_H

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

class EcatForce
{
public:

    typedef struct
    {
        unsigned int f32data;
        unsigned int s32data;
    }offset_t;

    typedef struct
    {
        float   f32data;
        int32_t s32data;
    }data_t;

    offset_t offset;
    data_t data;


    //TxEntries
    ec_pdo_entry_info_t txpdo_entries[7] = {
        {0x3101, 0x01, 32}, /* analog_Input */
    };

    //TxPdos
    ec_pdo_info_t txpdos[1] = {
        {0x1a00, 1, txpdo_entries + 0}, /* TxPdo Channel 1 */
    };

    ec_sync_info_t syncs[3] = {
//        {2, EC_DIR_OUTPUT, 1, rxpdos, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  1, txpdos, EC_WD_ENABLE},
        {0xff}
    };

    int Init(ec_master_t *master_,
             uint16_t alias_, /**< Slave alias. */
             uint16_t position_/**< Slave position. */);

    void DataRead(uint8_t *domain1_pd_);
    void DataPlay(void);

    EcatForce();
    ~EcatForce();

private:

    #define FORCE   0x00000002, 0x15232452

};



#endif
