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

#include "master.h"
#include "motor.h"

RT_TASK my_task;

static int run = 1;

using namespace std;
using namespace tinyxml2;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_imu_01 = NULL;
static ec_slave_config_t *sc_lan9252_01 = NULL;
static ec_slave_config_t *sc_motor_01 = NULL;

/****************************************************************************/
// process data
/****************************************************************************/
/*****************************************************************************/
// imu
//#define IMU_0_Pos       0, 0
//#define IMU             0xE0000005, 0x26483052
//static off_imu_t        off_imu_0;
//static imu_data_t       imu_data;

/*****************************************************************************/
// motor

EcatMotor motor_01;

/*****************************************************************************/
//lan9252
//#define WMLAN9252_IO_POS    0, 0
//#define WMLAN9252_IO        0xE0000002, 0x92521000
//static off_lan9252_io_t     off_lan9252_io;
//static wmlan9252_io_data_t  wmlan9252_io_data;

// process data
const static ec_pdo_entry_reg_t domain1_regs[] =
{
#ifdef WMLAN9252_IO_POS
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x7000, 0x01, &off_lan9252_io.leds, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6000, 0x01, &off_lan9252_io.keys, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6020, 0x01, &off_lan9252_io.analog_data, NULL},
#endif

#ifdef IMU_0_Pos
    {IMU_0_Pos,  IMU, 0x6000, 0x01, &off_imu_0.gx, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x02, &off_imu_0.gy, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x03, &off_imu_0.gz, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x04, &off_imu_0.ax, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x05, &off_imu_0.ay, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x06, &off_imu_0.az, NULL},
    {IMU_0_Pos,  IMU, 0x6000, 0x07, &off_imu_0.counter, NULL},
    {IMU_0_Pos,  IMU, 0x7011, 0x01, &off_imu_0.led, NULL},
#endif

#ifdef MOTOR_0_Pos
    {MOTOR_0_Pos,  MOTOR, 0x607a, 0x00, &motor_01.off_motor_0.target_pos, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x60ff, 0x00, &motor_01.off_motor_0.target_vel, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6071, 0x00, &motor_01.off_motor_0.target_tor, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6072, 0x00, &motor_01.off_motor_0.max_tor, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6040, 0x00, &motor_01.off_motor_0.control_word, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6060, 0x00, &motor_01.off_motor_0.mode, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x60b1, 0x00, &motor_01.off_motor_0.vel_offset, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x60b2, 0x00, &motor_01.off_motor_0.tor_offset, NULL},

    {MOTOR_0_Pos,  MOTOR, 0x6064, 0x00, &motor_01.off_motor_0.actual_pos, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x606c, 0x00, &motor_01.off_motor_0.actual_vel, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6078, 0x00, &motor_01.off_motor_0.actual_cur, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6077, 0x00, &motor_01.off_motor_0.actual_tor, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6041, 0x00, &motor_01.off_motor_0.status_word, NULL},
    {MOTOR_0_Pos,  MOTOR, 0x6061, 0x00, &motor_01.off_motor_0.mode_display, NULL},
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

/*****************************************************************************/
//IMU pdo
//TxPdo
ec_pdo_entry_info_t imu_txpdo_entries[] = {
    {0x6000, 0x01, 32}, /* gx */
    {0x6000, 0x02, 32}, /* gy */
    {0x6000, 0x03, 32}, /* gz */
    {0x6000, 0x04, 32}, /* ax */
    {0x6000, 0x05, 32}, /* ay */
    {0x6000, 0x06, 32}, /* az */
    {0x6000, 0x07, 32}, /* counter */
};
ec_pdo_info_t imu_txpdos[] = {
    {0x1a00, 7, imu_txpdo_entries + 0}, /* TxPdo Channel 1 */
};
//RxPdo
ec_pdo_entry_info_t imu_rxpdo_entries[] = {
    {0x7011, 0x01, 16}, /* led0_8 */
};
ec_pdo_info_t imu_rxpdos[] = {
    {0x1601, 1, imu_rxpdo_entries + 0}, /* RxPdo Channel 1 */
};
ec_sync_info_t imu_syncs[] = {
    {2, EC_DIR_OUTPUT, 1, imu_rxpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, imu_txpdos, EC_WD_ENABLE},
    {0xff}
};

/*****************************************************************************
 * Realtime task
 ****************************************************************************/
void rt_check_domain_state(void)
{
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
        // motor
        #ifdef MOTOR_0_Pos
        {
            #if 1
            {
                // read process data
                motor_01.motor_data.actual_pos = EC_READ_S32(domain1_pd + motor_01.off_motor_0.actual_pos);
                motor_01.motor_data.actual_vel = EC_READ_S32(domain1_pd + motor_01.off_motor_0.actual_vel);

                motor_01.motor_data.actual_cur = EC_READ_S16(domain1_pd + motor_01.off_motor_0.actual_cur);
                motor_01.motor_data.actual_tor = EC_READ_S16(domain1_pd + motor_01.off_motor_0.actual_tor);
                motor_01.motor_data.status_word = EC_READ_U16(domain1_pd + motor_01.off_motor_0.status_word);
                motor_01.motor_data.mode_display = EC_READ_U8(domain1_pd + motor_01.off_motor_0.mode_display);


                std::cout << "actual_pos:" << dec << motor_01.motor_data.actual_pos << endl;
                std::cout << "actual_vel:" << dec << motor_01.motor_data.actual_vel << endl;
                std::cout << "actual_cur:" << dec << motor_01.motor_data.actual_cur << endl;
                std::cout << "actual_tor:" << dec << motor_01.motor_data.actual_tor << endl;
                std::cout << "status_word: 0x" << hex << motor_01.motor_data.status_word << endl;
                std::cout << "mode_display: 0x" << hex << (uint16_t)motor_01.motor_data.mode_display << endl;
                std::cout << endl;
            }
            #endif

            int motor_state = motor_01.motor_data.status_word & 0x006f;

            if (motor_state != 0x0027)
            {
//				switch (mode)
                {
//				case Mode::POSITION:
                    //targetposition should be equal to actualposition
//                    readEntry(PdoEntry::ACTUALPOSITION, 0x00, actual_position_);
//                    writeEntry(PdoEntry::TARGETPOSITION, 0x00, actual_position_);
//					break;
//				case Mode::VELOCITY:
                    //velocity loop to set velocity of 0
//                    writeEntry(PdoEntry::TARGETVELOCITY, 0x00, static_cast<std::int32_t>(0));
//                    std::cout << "--------target_vel = 0 --------" << std::endl;
//                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_vel, static_cast<std::int32_t>(0));
//					break;
//				case Mode::TORQUE:
//                    writeEntry(PdoEntry::TARGETTORQUE, 0x00, static_cast<std::int16_t>(0));
//                    writeEntry(PdoEntry::MAXTORQUE, 0x00, static_cast<std::int16_t>(1500));
//					break;
                }
            }

            #if 1
            if(motor_01.motor_state != motor_01.enabled)
            {
                if(motor_01.motor_data.mode_display != 0x06)
                {
                    std::cout << "--------mode_display--------" << std::endl;
    //                motor_state = init;
//                    EC_WRITE_U8(domain1_pd +motor_01.off_motor_0.mode, 0x00);//init
//                    EC_WRITE_U8(domain1_pd +motor_01.off_motor_0.mode, 0x01);//position mode
//                    EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x03);//velocity mode
    //                EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x04);//torque mode
                    EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x06);//homing  mode
//                     EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x09);//csv
                }
                else
                {
                    //motor enable
                    if(motor_state == 0x0040 || motor_state == 0x0600)// switch on disable
                    {
    //                    motor_state = enable_ing;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, 0x0006); //shut down
                        std::cout << "--------shut down--------" << std::endl;
                    }
                    else if (motor_state == 0x0021) //read to switch on
                    {
    //                    motor_state = enable_ing;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, 0x0007); //switch on
                        std::cout << "-------switch on---------" << std::endl;
                    }
                    else if (motor_state == 0x0023) //switch on
                    {
    //                    motor_state = enable_ing;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, 0x000f); //Enable Operation
                        std::cout << "-------Enable Operation---------" << std::endl;
                    }
                    else if(motor_state == 0x0027)//operation enable
                    {
                        //successfull, but still need to wait for 10 more cycles to make it stable
                        static int enable_period_ = 0;
                        if(++enable_period_ > 10)
                        {
                            enable_period_ = 0;
                            motor_01.motor_state = motor_01.enabled;
                            std::cout << "motor has been enabled" << std::endl;
                            std::cout << "--------operation enable--------" << std::endl;
                        }

                    }
                    else //falt
                    {
                        std::cout << "---------falt restet-------" << std::endl;
                        motor_01.motor_state = motor_01.enable_ing;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, 0x0080); //falt restet

                    }
                }

                //homing
                #if 0
                if((motor_01.motor_data.status_word & 0x006f) != 0x0027)//motor not enabled
                {
                    std::cout << "homing error, please enable motor first!" << std::endl;
                }
                else //motor has enabled
                {
                    static int homing_counter = 0;
                    if((motor_01.motor_data.status_word & 0x3400) == 0x0400)//homing procedure is interrupted or not started
                    {
                        std::cout << "to start homing procedure" << std::endl;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, static_cast<uint16_t>(0x1f));

                    }
                    else if((motor_01.motor_data.status_word & 0x3400) == 0x0000) //homing ing
                    {
                        std::cout << "homing procedure is running................."<< std::endl;
                        homing_counter = 0;
                        EC_WRITE_U16(domain1_pd + motor_01.off_motor_0.control_word, static_cast<uint16_t>(0x1f));
                        EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_pos, static_cast<int32_t>(35));
                    }
                    else if((motor_01.motor_data.status_word & 0x3400) == 0x1400) //homing ing
                    {
                        std::cout << "homing successfully"<< std::endl;
                        motor_01.motor_state =motor_01.homed;
                    }
                }
                #endif
            }


            //position mode
//            if(motor_01.motor_state == motor_01.homed)
//            if(motor_01.motor_state == motor_01.enabled)
//            {
//                std::cout << "running in position mode"<< std::endl;
//                if(motor_01.motor_data.mode_display != 0x08)
//                {
//                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_pos, static_cast<int32_t>(0));
//                    EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x08);//position mode
//                }
//                else
//                {
//                    static int32_t s32position = 0;
//                    s32position +=10;
//                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_pos, static_cast<int32_t>(s32position));
//                }
//            }

            //velocity mode
//            if(motor_01.motor_state == motor_01.homed)
            if(motor_01.motor_state == motor_01.enabled)
            {
                if(motor_01.motor_data.mode_display != 0x09)
                {
                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_vel, 0);
                    EC_WRITE_U8(domain1_pd + motor_01.off_motor_0.mode, 0x09);//position mode
                }
                else
                {
                    static int32_t s32position = 0;
                    s32position =10000;
//                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_vel, 655350*imu_data.ay);
                    EC_WRITE_S32(domain1_pd + motor_01.off_motor_0.target_vel, 65535);
                    std::cout << "running in velocity mode"<< std::endl;
                }
            }

            #endif
        }
        #endif

        //distribute clock
        ecrt_master_application_time(master, rt_timer_read());
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);

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
//    chdir(dirname(argv[0])); //设置当前目录为应用程序所在的目录。
//    LoadXML();


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

    #ifdef IMU_0_Pos
    {
        sc_imu_01 = ecrt_master_slave_config(master, IMU_0_Pos, IMU);
        if(!sc_imu_01)
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }
        if (ecrt_slave_config_pdos(sc_imu_01, EC_END, imu_syncs))
        {
            fprintf(stderr, "Failed to configure PDOs.\n");
            return -1;
        }
    }
    #endif

    #ifdef MOTOR_0_Pos
    {
        sc_motor_01 = ecrt_master_slave_config(master, MOTOR_0_Pos, MOTOR);
        if(!sc_motor_01)
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }



        #if(1)
        {
            //homing config
            {
                //home mode
                if(ecrt_slave_config_sdo8(sc_motor_01,0x6098,0x00,35) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
                //home acc
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6099,0x01,200000) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
                //home high velocity
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6099,0x02,10000) < 0)
                {
                    std::cout << "config homing acc error" << std::endl;
                    return -1;
                }
                //home low velocity
                if(ecrt_slave_config_sdo32(sc_motor_01,0x609a,0x00,100000) < 0)
                {
                    std::cout << "config low velocity error" << std::endl;
                    return -1;
                }
                //home offset
                if(ecrt_slave_config_sdo32(sc_motor_01,0x607c,0x00,0) < 0)
                {
                    std::cout << "config homing offset error" << std::endl;
                    return -1;
                }
            }
            #if 0
            //position mode config
            {
                //profile velocity
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6081,0x00,10000) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
                //end velocity
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6082,0x00,10000) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
                //profile acceleration
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6083,0x00,10000) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
                //profile deceleration
                if(ecrt_slave_config_sdo32(sc_motor_01,0x6084,0x00,10000) < 0)
                {
                    std::cout << "config homing mode error" << std::endl;
                    return -1;
                }
            }
            #endif



            if (ecrt_slave_config_pdos(sc_motor_01, EC_END, motor_01.motor_syncs))
    //        if (ecrt_slave_config_pdos(sc_motor_01, EC_END, xml_motor_syncs.data()))
            {
                fprintf(stderr, "Failed to configure PDOs.\n");
                return -1;
            }


        }
        #endif
    }
    #endif

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

ecrt_slave_config_dc(sc_motor_01,0x0300, 1000000, 440000, 0, 0);////////////////////////////////////////////////////
  //ecrt_slave_config_dc(slave_config_, distributed_clock_, 1000000, 4400000, 0, 0);

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
    int ret = rt_task_create(&my_task, "my_task", 0, 99, T_FPU);
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
