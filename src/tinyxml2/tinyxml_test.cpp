/*

*/


#include <iostream>
#include <vector>
#include "tinyxml2.h"
#include <unistd.h>
#include <libgen.h>

#include "ecrt.h"

using namespace std;
using namespace tinyxml2;

vector<ec_pdo_entry_info_t> motor_rxpdo_entries;
vector<ec_pdo_entry_info_t> motor_txpdo_entries;
vector<ec_pdo_info_t> motor_rxpdos;
vector<ec_pdo_info_t> motor_txpdos;

void example1()
{
    XMLDocument doc;
    if(doc.LoadFile("../../resource/test.xml") != XML_SUCCESS)
    {
       std::cout << "load text.xml file fault!" << doc.ErrorName() << std::endl;
       exit(1);
    }

    const XMLElement *root=doc.RootElement();

    ec_sync_info_t temp_sync;
    vector<ec_sync_info_t> motor_syncs;

    const XMLElement *elmo = root->FirstChildElement("elmo");
    if(elmo)
    {
        std::cout << elmo->Name() << " ";
        const XMLAttribute *vendor_id = elmo->FindAttribute("vender_id");
        const XMLAttribute *product_code = elmo->FindAttribute("product_code");
        std::cout << vendor_id->Name() << ":" << vendor_id->Value() << " ";
        std::cout << product_code->Name() << ":" << product_code->Value() << std::endl;


#if(1)
        const XMLElement *rxpdo = elmo->FirstChildElement("rxpdo");
        if(rxpdo)
        {
            ec_pdo_info_t temp_pdo;
            while(rxpdo)
            {
                const XMLAttribute *pdo_index = rxpdo->FindAttribute("index");
                std::cout << rxpdo->Name() << " " << pdo_index->Name() << ":" << pdo_index->Value() << std::endl;

                ec_pdo_entry_info_t temp_entry;

                const XMLElement *entry = rxpdo->FirstChildElement();
                unsigned int  n_entries = 0;
                while(entry)
                {
                    n_entries ++;
                    {
                        temp_entry.index = std::strtoul(entry->FindAttribute("index")->Value(),0,0);
                        temp_entry.subindex = std::strtoul(entry->FindAttribute("subindex")->Value(),0,0);
                        temp_entry.bit_length = std::strtoul(entry->FindAttribute("bitlength")->Value(),0,0);
                        motor_rxpdo_entries.push_back(temp_entry);
                    }
                    entry = entry->NextSiblingElement();
                }
                {
                    static unsigned int sum_n_entries = 0;
                    temp_pdo.index = std::strtoul((rxpdo->FindAttribute("index")->Value()),0,0);
                    temp_pdo.n_entries = n_entries;
                    temp_pdo.entries = &(motor_rxpdo_entries[sum_n_entries]);
                    motor_rxpdos.push_back(temp_pdo);
                    sum_n_entries += n_entries;
                    n_entries=0;
                }
                std::cout << std::endl;
                rxpdo = rxpdo->NextSiblingElement("rxpdo");
            }
            temp_sync.index = 2;
            temp_sync.dir = EC_DIR_OUTPUT;
            temp_sync.n_pdos = motor_rxpdos.size();
            temp_sync.pdos = &(motor_rxpdos[0]);
            temp_sync.watchdog_mode = EC_WD_ENABLE;

            motor_syncs.push_back(temp_sync);
            std::cout << dec << motor_rxpdo_entries.size() << std::endl;
            std::cout << dec << temp_sync.n_pdos << std::endl;
            std::cout << dec << motor_syncs.size() << std::endl;
        }
#endif
#if(1)
        const XMLElement *txpdo = elmo->FirstChildElement("txpdo");
        if(txpdo)
        {
            ec_pdo_info_t temp_pdo;
            while(txpdo)
            {
                const XMLAttribute *pdo_index = txpdo->FindAttribute("index");
                std::cout << txpdo->Name() << " " << pdo_index->Name() << ":" << pdo_index->Value() << std::endl;

                ec_pdo_entry_info_t temp_entry;

                const XMLElement *entry = txpdo->FirstChildElement();
                unsigned int  n_entries = 0;
                while(entry)
                {
                    n_entries ++;
                    {
                        temp_entry.index = std::strtoul(entry->FindAttribute("index")->Value(),0,0);
                        temp_entry.subindex = std::strtoul(entry->FindAttribute("subindex")->Value(),0,0);
                        temp_entry.bit_length = std::strtoul(entry->FindAttribute("bitlength")->Value(),0,0);
                        motor_txpdo_entries.push_back(temp_entry);
                    }
                    entry = entry->NextSiblingElement();
                }
                {
                    static unsigned int sum_n_entries = 0;
                    temp_pdo.index = std::strtoul((txpdo->FindAttribute("index")->Value()),0,0);
                    temp_pdo.n_entries = n_entries;
                    temp_pdo.entries = &(motor_txpdo_entries[sum_n_entries]);
                    motor_txpdos.push_back(temp_pdo);
                    sum_n_entries += n_entries;
                    n_entries=0;
                }
                std::cout << std::endl;
                txpdo = txpdo->NextSiblingElement("txpdo");
            }
            temp_sync.index = 3;
            temp_sync.dir = EC_DIR_INPUT;
            temp_sync.n_pdos = motor_txpdos.size();
            temp_sync.pdos = &(motor_txpdos[0]);
            temp_sync.watchdog_mode = EC_WD_ENABLE;

            motor_syncs.push_back(temp_sync);
            std::cout << dec << motor_txpdo_entries.size() << std::endl;
            std::cout << dec << temp_sync.n_pdos << std::endl;
            std::cout << dec << motor_syncs.size() << std::endl;

            temp_sync.index = 0xff; //end of the list
            motor_syncs.push_back(ec_sync_info_t{
                                      0xff,//end of the list
                                  });
            std::cout << dec << motor_syncs.size() << std::endl;

        }
#endif
#if(0)
        XMLElement *sdo = elmo->FirstChildElement("sdo");
        while(sdo)
        {
             std::cout << "sdo:" << std::endl;
            XMLElement *entry = sdo->FirstChildElement();
            while(entry)
            {
                const XMLAttribute *entry_index = entry->FindAttribute("index");
                const XMLAttribute *entry_subindex = entry->FindAttribute("subindex");
                const XMLAttribute *entry_datatype = entry->FindAttribute("datatype");
                std::cout << "  " << entry->Name() << "  ";
                std::cout << "  " << entry_index->Name() <<":"<< entry_index->Value() << "  ";
                std::cout << "  " << entry_subindex->Name() << ":" << entry_subindex->Value() << "  ";
                std::cout << "  " << entry_datatype->Name() << ":" << entry_datatype->Value() << std::endl;

                entry = entry->NextSiblingElement();
            }
            std::cout << std::endl;
            sdo = sdo->NextSiblingElement("sdo");
        }
#endif
    }
    cout << "...................." << endl;
    for (auto val : motor_syncs)
    {
        cout << (uint16_t)val.index << endl;
        cout << (uint16_t)val.dir << endl;
        cout << (uint16_t)val.n_pdos << endl;
        for(int i=0; i<val.n_pdos; i++)
        {
            cout << "   " << hex << (uint16_t)((val.pdos+i)->index) << "  ";
            cout << "   " << hex << (uint16_t)((val.pdos+i)->n_entries) << endl;
            for(int j=0; j<(val.pdos+i)->n_entries; j++)
            {
                cout << "      " << hex << (uint16_t)(((val.pdos+i)->entries+j)->index) << " ";
                cout << "      " << hex << (uint16_t)(((val.pdos+i)->entries+j)->subindex) << " ";
                cout << "      " << dec << (uint16_t)(((val.pdos+i)->entries+j)->bit_length) << endl;
            }
        }
    }

    std::cout << std::endl;
}
int main(int argc, char** argv)
{
    chdir(dirname(argv[0])); //设置当前目录为应用程序所在的目录。
    example1();
    return 0;
}
