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

   XMLElement *elmo = root->FirstChildElement("elmo");
   if(elmo)
   {
       const XMLElement *elmo->FindAttribute("vender_id");
   }

   std::cout << std::endl;
   XMLElement *surface=root->FirstChildElement("node");
   while (surface)
   {
       XMLElement *surfaceChild=surface->FirstChildElement();
       const char* content;
       const XMLAttribute *attributeOfSurface = surface->FirstAttribute();
       std::cout<< attributeOfSurface->Name() << ":" << attributeOfSurface->Value() << std::endl;
       while(surfaceChild)
       {
           content=surfaceChild->GetText();
           surfaceChild=surfaceChild->NextSiblingElement();
           std::cout<<content<<std::endl;
       }
       surface=surface->NextSiblingElement();
   }
   std::cout << std::endl;
}
int main(int argc, char** argv)
{
    chdir(dirname(argv[0])); //设置当前目录为应用程序所在的目录。
    example1();
    return 0;
}
