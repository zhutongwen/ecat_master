/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define WMLAN9252_IO_POS 0, 0

#define WMLAN9252_IO 0xE0000002, 0x92522010


typedef struct
{
    unsigned int target_pos;
    unsigned int target_vel;
    unsigned int control_word;
    unsigned int mode;

    unsigned int actual_pos;
    unsigned int actual_vel;
    unsigned int status_word;
    unsigned int mode_display;

    unsigned int out_dig;
    unsigned int in_dig;
} off_motor_t;

typedef struct
{
    uint16_t control_word;
    int32_t  target_pos;
    int32_t  target_vel;
    int32_t  out_dig;
    uint8_t  mode;

    uint16_t status_word;
    int32_t  actual_pos;
    int32_t  actual_vel;
    int32_t  in_dig;
    uint8_t  mode_display;
}motor_data_t;


off_motor_t      off_motor;
motor_data_t     motor_data;

//// offsets for PDO entries
//static unsigned int off_analog_data;
//static unsigned int off_keys;
//static unsigned int off_leds;

//struct wmlan9252_io_data_struct
//{
//  uint16_t analog_data;
//  uint8_t key0_1;
//  uint8_t led0_7;
//};
//static struct wmlan9252_io_data_struct wmlan9252_io_data;

const static ec_pdo_entry_reg_t domain1_regs[] = {

#if 1
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6040, 0x00, &off_motor.control_word, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x607a, 0x00, &off_motor.target_pos, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x60fe, 0x00, &off_motor.out_dig, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x60ff, 0x00, &off_motor.target_vel, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6060, 0x00, &off_motor.mode, NULL},

    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6041, 0x00, &off_motor.status_word, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6064, 0x00, &off_motor.actual_pos, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x606c, 0x00, &off_motor.actual_vel, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x60fd, 0x00, &off_motor.in_dig, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6061, 0x00, &off_motor.mode_display, NULL},
#endif
    {}
};

/*****************************************************************************/
//wmlan9252_402add
//TxPdo
ec_pdo_entry_info_t wmlan9252_402add_txpdo_entries[] = {
    {0x6041, 0x00, 16}, /* status word */
    {0x6064, 0x00, 32}, /* actual position */
    {0x606C, 0x00, 32}, /* actual velocity */
    {0x60FD, 0x00, 32}, /* input dig */
    {0x6061, 0x00, 8}, /* mode of operation display */
    {0, 0, 8}, //reserve
};

ec_pdo_info_t wmlan9252_402add_txpdos[] = {
    {0x1A00, 6, wmlan9252_402add_txpdo_entries + 0}, /* TxPdo Channel 1 */
};

//RxPdo
ec_pdo_entry_info_t wmlan9252_402add_rxpdo_entries[] = {
    {0x6040, 0x00, 16}, /* control word */
    {0x607A, 0x00, 32}, /* target position */
    {0x60FE, 0x00, 32}, /* out dig */
    {0x60FF, 0x00, 32}, /* target velocity */
    {0x6060, 0x00, 8}, /* mode of operation                                                  */
    {0, 0, 8}, //reserve
};

ec_pdo_info_t wmlan9252_402add_rxpdos[] = {
    {0x1600, 6, wmlan9252_402add_rxpdo_entries + 0}, /* RxPdo Channel 1 */
};

ec_sync_info_t wmlan9252_io_syncs[] = {
    {2, EC_DIR_OUTPUT, 1, wmlan9252_402add_rxpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, wmlan9252_402add_txpdos, EC_WD_ENABLE},
    {0xff}
};


/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);

    if (s.al_state != sc_ana_in_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_ana_in_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_ana_in_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_ana_in_state = s;
}

/*****************************************************************************/

void cyclic_task()
{
	static int counter_check = 0;

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
	check_domain1_state();

    if ((++counter_check) >= FREQUENCY) // do this at 1 Hz
	{
		counter_check = 0;

        // check for master state (optional)
        check_master_state();

        // check for slave configuration state(s) (optional)
        check_slave_config_states();
    }

	static int counter_txrx = 0;
    if((++counter_txrx) >= 100)
	{
		counter_txrx = 0;
        #if 1
            // read process data
            motor_data.actual_pos = EC_READ_S32(domain1_pd + off_motor.actual_pos);
            motor_data.actual_vel = EC_READ_S32(domain1_pd + off_motor.actual_vel);
            motor_data.in_dig = EC_READ_S32(domain1_pd + off_motor.in_dig);
            motor_data.status_word = EC_READ_U16(domain1_pd + off_motor.status_word);
            motor_data.mode_display = EC_READ_U8(domain1_pd + off_motor.mode_display);
            // write process data
            EC_WRITE_S32(domain1_pd + off_motor.out_dig, motor_data.out_dig++);

            printf("actual_pos: 0x%x\n", motor_data.actual_pos);
            printf("actual_vel: 0x%x\n", motor_data.actual_vel);
            printf("in_dig: 0x%x\n", motor_data.in_dig);
            printf("status_word: 0x%x\n", motor_data.status_word);
            printf("mode_display: 0x%x\n\n", motor_data.mode_display);

         #endif
	}
	// send process data
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    struct timespec wakeup_time;
    int ret = 0;
printf("0\n\n");
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }
printf("1\n\n");
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

printf("2\n\n");
/////////////////////////////////
#if 1
    sc_ana_in = ecrt_master_slave_config(master, WMLAN9252_IO_POS, WMLAN9252_IO);
    if (!sc_ana_in)
    {
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_ana_in, EC_END, wmlan9252_io_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
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
        return -1;
    }

    /* Set priority */

    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19)) {
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}

/****************************************************************************/
