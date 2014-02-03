#include <iostream>

#include "sensors_drivers/logparser.h"

int main()
{
    std::cout << "Hi" << std::endl;

    std::vector<nav::Frame> framesVector;

    nav::parseFile("/home/mpp/DODICH/Registrazioni/01051511_ls_sick.csv", framesVector);

    std::cout << framesVector.size() << std::endl;
}
