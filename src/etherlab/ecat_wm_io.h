#ifndef WM_IO_H
#define WM_IO_H

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

//class EcrtImu
//{
//public:

//    typedef struct
//    {
//        unsigned int analog_data;
//        unsigned int keys;
//        unsigned int leds;
//    }off_lan9252_io_t;

//    typedef struct
//    {
//        uint16_t analog_data;
//        uint8_t key0_1;
//        uint8_t led0_7;
//    }wmlan9252_io_data_t;


//    offset_t offset;
//    data_t data;

//    //TxPdo
//    ec_pdo_entry_info_t wmlan9252_io_txpdo_entries[] = {
//        {0x6000, 0x01, 8}, /* key0_2 */
//        {0, 0, 8}, //reserve
//        {0x6020, 0x01, 16}, /* analog_Input */
//    };
//    ec_pdo_info_t wmlan9252_io_txpdos[] = {
//        {0x1A02, 1, wmlan9252_io_txpdo_entries + 2}, /* TxPdo Channel 2 */
//        {0x1A00, 2, wmlan9252_io_txpdo_entries + 0}, /* TxPdo Channel 1 */
//    };
//    //RxPdo
//    ec_pdo_entry_info_t wmlan9252_io_rxpdo_entries[] = {
//        {0x7000, 0x01, 8}, /* led0_8 */
//        {0, 0, 8}, //reserve
//    };
//    ec_pdo_info_t wmlan9252_io_rxpdos[] = {
//        {0x1601, 2, wmlan9252_io_rxpdo_entries + 0}, /* RxPdo Channel 1 */
//    };
//    ec_sync_info_t wmlan9252_io_syncs[] = {
//        {2, EC_DIR_OUTPUT, 1, wmlan9252_io_rxpdos, EC_WD_ENABLE},
//        {3, EC_DIR_INPUT,  2, wmlan9252_io_txpdos, EC_WD_ENABLE},
//        {0xff}
//    };
//private:

//};



#endif
