#include "ecat_task.h"
#include <math.h>



void ControlTask(uint8_t *domain1_pd_, slaves_t &slaves_)
{
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
    //imu
    #ifdef IMU_Pos_0
    {
        std::cout<< "IMU_0---------"<< std::endl;
        slaves_.imu_0.DataRead(domain1_pd_);
        slaves_.imu_0.DataPlay();

    }
    #endif

    #ifdef IMU_Pos_1
    {
        std::cout<< "IMU_1---------"<< std::endl;
        slaves_.imu_1.DataRead(domain1_pd_);
        slaves_.imu_1.DataWrite(domain1_pd_);
        slaves_.imu_1.DataPlay();
    }
    #endif

    /***************************************************************************/
    //force
    #ifdef FORCE_Pos_0
        std::cout<< "force_0---------"<< std::endl;
        slaves_.force_0.DataRead(domain1_pd_);
        slaves_.force_0.DataPlay();
    #endif

    /****************************************************************************/
    // motor
    #ifdef MOTOR_Pos_0
    {
        std::cout << slaves_.motor_0.motor_state << std::endl;
        slaves_.motor_0.Display(domain1_pd_);

        if(slaves_.motor_0.motor_state == slaves_.motor_0.STATE_CSV)
        {
            static float f32angle = 0;
            int32_t s32velocity = 0;

            f32angle += 0.0002;
            if(f32angle >= 1) f32angle = 0;

            s32velocity = 655350.0*sin(f32angle*(2.0*PI));
            slaves_.motor_0.SetTargtVelocity(domain1_pd_, static_cast<int32_t>(s32velocity));
        }

        if(slaves_.motor_0.motor_state == slaves_.motor_0.STATE_INIT)
        {
            slaves_.motor_0.Enable(domain1_pd_);
        }
        else if(slaves_.motor_0.motor_state == slaves_.motor_0.STATE_ENABLED)
        {
            slaves_.motor_0.Homing(domain1_pd_);
        }
        else if(slaves_.motor_0.motor_state == slaves_.motor_0.STATE_HOMED)
        {
            slaves_.motor_0.SetMode(domain1_pd_,static_cast<int8_t>(slaves_.motor_0.MODE_CSV));
        }
    }
    #endif
}
