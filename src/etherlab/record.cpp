#include "record.h"

void RecordThread(void)
{
    std::fstream examplefile;
    examplefile.open("example.txt");

    if (examplefile.is_open())
    {
        LOG(INFO) << "record start!\n";
        examplefile << "This is a line.\n";
        examplefile << "This is another line.\n";
        examplefile.close();
    }
    else
    {
        LOG(ERROR) << "record error!\n";
    }
    while (1);
}
