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
    std::string WRAP_LOG_FRAME(const char* msg){
        int msg_len = strlen(msg);
        float margin = fmax(2, LOG_FRAME_LEN-msg_len);
        std::string prefix(floor(margin/2)-1, '=');
        std::string postfix(ceil(margin/2)-1, '=');
        std::string msg_line = prefix + " " + msg + " " + postfix + "\n";
        return msg_line;
    }

    std::string WRAP_LOG_FRAME(std::string& msg) {
        return WRAP_LOG_FRAME(msg.c_str());
    }

    void PRINT_ERROR(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg){
        std::string msg_line = "\x1B[31m" + WRAP_LOG_FRAME(msg) + "\033[0m";
        std::cout<<TEXT_RED(LOG_FRAME_LINE);
        std::cout<<msg_line.c_str();
        std::cout<<TEXT_RED(LOG_FRAME_LINE "\n");
    }

    void PRINT_ERROR(std::string& msg) {
        PRINT_ERROR(msg.c_str());
    }

    void PRINT_FRAMED_LOG(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg, bool endl=false){
        std::cout<<WRAP_LOG_FRAME(msg);
        if(endl) std::cout<<std::endl;
    }

    void PRINT_FRAMED_LOG(std::string& msg, bool endl=false){
        PRINT_FRAMED_LOG(msg.c_str(), endl);
    }
}

#endif //MOVEIT_PLAN_COMPACT_LOGGER_H
