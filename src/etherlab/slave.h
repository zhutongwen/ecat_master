#ifndef SLAVE_H
#define SLAVE_H

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

class EcrtSlave
{
public:
    EcrtSlave() {}
//    void Init(ec_master* master);


private:
    std::vector<std::vector<ec_pdo_entry_info_t>> rxpdo_entries;
    std::vector<std::vector<ec_pdo_entry_info_t>> txpdo_entries;

    std::vector<ec_pdo_info_t> rxpdos;
    std::vector<ec_pdo_info_t> txpdos;

    std::vector<ec_sync_info_t> syncs;
};



#endif
