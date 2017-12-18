/*

*/


#include <iostream>
#include <string>
#include "tinyxml2.h"
#include <unistd.h>
#include <libgen.h>

using namespace std;
using namespace tinyxml2;
void example1()
{
    XMLDocument doc;
    if(doc.LoadFile("../../resource/test.xml") != XML_SUCCESS)
    {
       std::cout << "load text.xml file fault!" << doc.LoadFile("test.xml") << std::endl;
       exit(1);
    }

    XMLElement *root=doc.RootElement();
    std::cout << std::endl;
    std::cout << root->FirstAttribute()->Name() << ":" ;
    std::cout << root->FirstAttribute()->Value() << std::endl;

    std::cout << std::endl;

    XMLElement *elmo = root->FirstChildElement("elmo");
    if(elmo)
    {
        std::cout << elmo->Name() << " ";
        const XMLAttribute *vendor_id = elmo->FindAttribute("vender_id");
        const XMLAttribute *product_code = elmo->FindAttribute("product_code");
        std::cout << vendor_id->Name() << ":" << vendor_id->Value() << " ";
        std::cout << product_code->Name() << ":" << product_code->Value() << std::endl;

        XMLElement *rxpdo = elmo->FirstChildElement("rxpdo");
        while(rxpdo)
        {
            const XMLAttribute *rxpdo_index = rxpdo->FindAttribute("index");
            std::cout << rxpdo->Name() << " ";
            std::cout << rxpdo_index->Name() << ":" << rxpdo_index->Value() << std::endl;

            XMLElement *entry = rxpdo->FirstChildElement();
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
            rxpdo = rxpdo->NextSiblingElement("rxpdo");
        }

        XMLElement *txpdo = elmo->FirstChildElement("txpdo");
        while(txpdo)
        {
            const XMLAttribute *txpdo_index = txpdo->FindAttribute("index");
            std::cout << txpdo->Name() << " ";
            std::cout << txpdo_index->Name() << ":" << txpdo_index->Value() << std::endl;

            XMLElement *entry = txpdo->FirstChildElement();
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
            txpdo = txpdo->NextSiblingElement("txpdo");
        }

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
    }
    std::cout << std::endl;
}
int main(int argc, char** argv)
{
    chdir(dirname(argv[0])); //设置当前目录为应用程序所在的目录。
    example1();
    return 0;
}
