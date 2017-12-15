
#include <iostream>
#include <vector>

using namespace std;

void ShowVec1(const vector<int>& valList)
{
    int count = valList.size();
    for (int i = 0; i < count;i++)
    {
        cout << valList[i] << endl;
    }
}

void ShowVec2(const vector<int>& valList)
{
    for (vector<int>::const_iterator iter = valList.cbegin(); iter != valList.cend(); iter++)
    {
        cout << (*iter) << endl;
    }
}

void ShowVec3(const vector<int>& valList)
{
    for (auto val : valList)
    {
        cout << val << endl;
    }
}
int main(int argc, char* argv[])
{
    vector<int> valList = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
//    ShowVec1(valList);
//    ShowVec2(valList);
    ShowVec3(valList);
    return 0;
}
