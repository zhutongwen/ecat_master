/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include "etherlab.h"

EtherLab::EtherLab()
{

}

EtherLab::~EtherLab()
{

}


static int run = 1;

using namespace std;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_imu_01 = NULL;
static ec_slave_config_t *sc_lan9252_01 = NULL;
static ec_slave_config_t *sc_motor_01 = NULL;

/****************************************************************************/
// process data
/****************************************************************************/
//lan9252
#define WMLAN9252_IO_POS    0, 0
#define WMLAN9252_IO        0xE0000002, 0x92521000

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

static off_lan9252_io_t     off_lan9252_io;
static wmlan9252_io_data_t  wmlan9252_io_data;

// process data
const static ec_pdo_entry_reg_t domain1_regs[] =
{
#ifdef WMLAN9252_IO_POS
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x7000, 0x01, &off_lan9252_io.leds, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6000, 0x01, &off_lan9252_io.keys, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6020, 0x01, &off_lan9252_io.analog_data, NULL},
#endif

    {}
};

/*****************************************************************************/
//wmlan9252_io pdo
//TxPdo
ec_pdo_entry_info_t wmlan9252_io_txpdo_entries[] = {
    {0x6000, 0x01, 8}, /* key0_2 */
    {0, 0, 8}, //reserve
    {0x6020, 0x01, 16}, /* analog_Input */
};
ec_pdo_info_t wmlan9252_io_txpdos[] = {
    {0x1A02, 1, wmlan9252_io_txpdo_entries + 2}, /* TxPdo Channel 2 */
    {0x1A00, 2, wmlan9252_io_txpdo_entries + 0}, /* TxPdo Channel 1 */
};
//RxPdo
ec_pdo_entry_info_t wmlan9252_io_rxpdo_entries[] = {
    {0x7000, 0x01, 8}, /* led0_8 */
    {0, 0, 8}, //reserve
};
ec_pdo_info_t wmlan9252_io_rxpdos[] = {
    {0x1601, 2, wmlan9252_io_rxpdo_entries + 0}, /* RxPdo Channel 1 */
};
ec_sync_info_t wmlan9252_io_syncs[] = {
    {2, EC_DIR_OUTPUT, 1, wmlan9252_io_rxpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  2, wmlan9252_io_txpdos, EC_WD_ENABLE},
    {0xff}
};

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    static ec_domain_state_t domain1_state = {};
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    static ec_master_state_t master_state = {};
    ec_master_state_t ms;


    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void my_task_proc(void *arg)
{
    int cycle_counter = 0;
    unsigned int blink = 0;

    rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns

    while (run)
    {
        rt_task_wait_period(NULL);

        cycle_counter++;

        // receive EtherCAT frames
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        rt_check_domain_state();

//        if (!(cycle_counter % 1000))
        {
            rt_check_master_state();
        }

//        if (!(cycle_counter % 200))
        {
            blink = !blink;
        }
        /****************************************************************************/
        // lan9252
        #ifdef WMLAN9252_IO_POS
        {
            static int counter_txrx = 0;
            //if((++counter_txrx) >= 100)
            {
                counter_txrx = 0;
                #if 1
                    // read process data
                    wmlan9252_io_data.analog_data = EC_READ_S16(domain1_pd + off_lan9252_io.analog_data);
                    wmlan9252_io_data.key0_1 =  EC_READ_U8(domain1_pd + off_lan9252_io.keys);
                    // write process data
                    EC_WRITE_U8(domain1_pd + off_lan9252_io.leds, ++wmlan9252_io_data.led0_7);
                #endif
                 printf("send leds value: 0x%-4x receive ADC value: %-6d receive keys: 0x%-4x   \n\n",
                        wmlan9252_io_data.led0_7,
                        wmlan9252_io_data.analog_data,
                        wmlan9252_io_data.key0_1);
            }
        }
        #endif

        /****************************************************************************/
        // IMU
        #ifdef IMU_0_Pos
        {
            #if 1
                // read process data
                imu_data.gx = EC_READ_FLOAT(domain1_pd + off_imu_0.gx);
                imu_data.gy = EC_READ_FLOAT(domain1_pd + off_imu_0.gy);
                imu_data.gz = EC_READ_FLOAT(domain1_pd + off_imu_0.gz);
                imu_data.ax = EC_READ_FLOAT(domain1_pd + off_imu_0.ax);
                imu_data.ay = EC_READ_FLOAT(domain1_pd + off_imu_0.ay);
                imu_data.az = EC_READ_FLOAT(domain1_pd + off_imu_0.az);
                imu_data.counter = EC_READ_U32(domain1_pd + off_imu_0.counter);
                // write process data
                EC_WRITE_U16(domain1_pd + off_imu_0.led, 0xaa55);
            #endif
//                std::cout << "gx:" << imu_data.gx << endl;
//                std::cout << "gy:" << imu_data.gy << endl;
//                std::cout << "gz:" << imu_data.gz << endl;
            std::cout << "ax:" << imu_data.ax << endl;
            std::cout << "ay:" << imu_data.ay << endl;
            std::cout << "az:" << imu_data.az << endl;
            std::cout << "counter:" << dec << imu_data.counter << endl;
        }
        #endif

        /****************************************************************************/

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/
int main(int argc, char *argv[])
{
    RT_TASK my_task;
    chdir(dirname(argv[0])); //设置当前目录为应用程序所在的目录。


//    exit(0);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)
    {
        std::cout << "ecrt_request_master error" << std::endl;
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        std::cout << "ecrt_master_create_domain error" << std::endl;
        return -1;
    }

    printf("Creating slave configurations...\n");

#ifdef WMLAN9252_IO_POS
    {
        sc_lan9252_01 = ecrt_master_slave_config(master, WMLAN9252_IO_POS, WMLAN9252_IO);
        if (!sc_lan9252_01)
        {
            return -1;
        }

        if (ecrt_slave_config_pdos(sc_lan9252_01, EC_END, wmlan9252_io_syncs))
        {
            fprintf(stderr, "Failed to configure PDOs.\n");
            return -1;
        }
    }
#endif

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

/////////////////////////
/// SDO
#if MOTOR
    {

    }
#endif
    int ret = rt_task_create(&my_task, "my_task", 0, 80, T_FPU);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting my_task...\n");
    ret = rt_task_start(&my_task, &my_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    while (run) {
        sched_yield();
    }

    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}




/***************************************************************************/
