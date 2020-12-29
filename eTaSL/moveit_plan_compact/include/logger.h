//
// Created by tamp on 20. 11. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_LOGGER_H
#define MOVEIT_PLAN_COMPACT_LOGGER_H

#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>

#define LOG_FRAME_LINE "==================================================\n"
#define LOG_FRAME_LEN 50
#define TEXT_RED(STR) ("\x1B[31m" STR "\033[0m")
#define TEXT_GREEN(STR) ("\x1B[32m" STR "\033[0m")
#define TEXT_YELLOW(STR) ("\x1B[33m" STR "\033[0m")
#define TEXT_BLUE(STR) ("\x1B[34m" STR "\033[0m")
#define TEXT_CYAN(STR) ("\x1B[36m" STR "\033[0m")

namespace RNB{
    std::string WRAP_LOG_FRAME(const char* msg);

    std::string WRAP_LOG_FRAME(std::string& msg);

    void PRINT_ERROR(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg);

    void PRINT_ERROR(std::string& msg);

    void PRINT_FRAMED_LOG(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg, bool endl=false);

    void PRINT_FRAMED_LOG(std::string& msg, bool endl=false);
}

#endif //MOVEIT_PLAN_COMPACT_LOGGER_H
