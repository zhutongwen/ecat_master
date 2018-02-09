

#ifndef ECAT_ETHERLAB_H
#define ECAT_ETHERLAB_H

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


class EtherLab
{
public:



    EtherLab();
    ~EtherLab();

    static std::vector<ec_pdo_entry_reg_t> domain1_regs;



private:

};

#endif
