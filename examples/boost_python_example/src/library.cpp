#include "../include/library.h"

#include <iostream>

std::string hello(std::string name) {
    std::cout << "Hello, " + name << std::endl;
    return "Hello, " + name;
}
