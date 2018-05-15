#include "ecat_force.h"

EcatForce::EcatForce()
{

}

EcatForce::~EcatForce()
{

}

int EcatForce::Init(ec_master_t *master_,
         uint16_t alias_, /**< Slave alias. */
         uint16_t position_/**< Slave position. */)
{
    ec_slave_config_t *sc_force;
    sc_force = ecrt_master_slave_config(master_, alias_, position_, FORCE);
    if(!sc_force)
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_force, EC_END, syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    return 0;
}

void EcatForce::DataRead(uint8_t *domain1_pd_)
{
    // read process data
    data.f32data = EC_READ_FLOAT(domain1_pd_ + offset.f32data);
}


void EcatForce::DataPlay(void)
{
    std::cout << "force:" << data.f32data << std::endl;

    std::cout << std::endl;
}
