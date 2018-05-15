#include "ecat_imu.h"

EcatImu::EcatImu()
{

}

EcatImu::~EcatImu()
{

}

int EcatImu::Init(ec_master_t *master_,
         uint16_t alias_, /**< Slave alias. */
         uint16_t position_/**< Slave position. */)
{
    ec_slave_config_t *sc_imu;
    sc_imu = ecrt_master_slave_config(master_, alias_, position_, IMU);
    if(!sc_imu)
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_imu, EC_END, syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    return 0;
}

void EcatImu::DataRead(uint8_t *domain1_pd_)
{
    // read process data
    data.gx = EC_READ_FLOAT(domain1_pd_ + offset.gx);
    data.gy = EC_READ_FLOAT(domain1_pd_ + offset.gy);
    data.gz = EC_READ_FLOAT(domain1_pd_ + offset.gz);
    data.ax = EC_READ_FLOAT(domain1_pd_ + offset.ax);
    data.ay = EC_READ_FLOAT(domain1_pd_ + offset.ay);
    data.az = EC_READ_FLOAT(domain1_pd_ + offset.az);
    data.counter = EC_READ_U32(domain1_pd_ + offset.counter);


}
void EcatImu::DataWrite(uint8_t *domain1_pd_)
{
    // write process data
    EC_WRITE_U16(domain1_pd_ + offset.led, 0xaa55);
}
void EcatImu::DataPlay(void)
{
//    std::cout << "gx:" << data.gx << std::endl;
//    std::cout << "gy:" << data.gy << std::endl;
//    std::cout << "gz:" << data.gz << std::endl;
    std::cout << "ax:" << data.ax << std::endl;
    std::cout << "ay:" << data.ay << std::endl;
    std::cout << "az:" << data.az << std::endl;
    std::cout << "counter:" << std::dec << data.counter << std::endl;

    std::cout << std::endl;


}
